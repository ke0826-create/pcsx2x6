// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "Rollback.h"

#include "Config.h"
#include "DEV9/ACJV.h"
#include "MemoryTypes.h"
#include "VMManager.h"

#include "common/Console.h"
#include "common/Error.h"
#include "common/Path.h"
#include "common/Timer.h"

#include <algorithm>
#include <cstring>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

// Offline rollback self-test
// --------------------------
// Trigger: hotkey "RollbackSelfTest" (default F10) while a game is running, ideally in the
// middle of a CPU battle with nobody touching the pad.
//
//   frame boundary k   : press a scripted JVS input, save a state (file), n = 0
//   frames n = 1..N    : scripted JVS input as a pure function of n, normal speed
//   n == N             : copy EE RAM -> reference
//   restore            : load the state, n = 0, limiter = Unlimited (fast re-simulation)
//   frames n = 1..N    : the SAME scripted input
//   n == N             : copy EE RAM -> compare with the reference
//   repeat the restore/re-simulate/compare kRepeats times
//
// The log (emulog.txt / console) gets lines starting with "RollbackTest:". PASS means the EE RAM
// was identical (outside the known volatile I/O-board counter) after every re-simulation.
//
// Notes
//  * Rendering is NOT skipped during the re-simulation; the measured speed is therefore a
//    conservative number for "how fast can we re-simulate".
//  * The state goes through the normal zip save state path (includes GS), so the save/load times
//    are an upper bound for an in-memory snapshot.
namespace Rollback
{
	namespace
	{
		enum class Phase
		{
			Idle,
			Reference, // running the first (reference) pass
			Resim,     // running a re-simulation pass after a state load
		};

		constexpr u32 kFrames = 120; // frames per pass (N)
		constexpr u32 kRepeats = 3;  // number of restore + re-simulate passes

		// 64-byte line holding the USB I/O board status counter (".;RAYS PCB;"), measured to differ
		// between a straight run and a restored run without affecting the game logic (2026-10-19).
		constexpr u32 kVolatileBegin = 0x4ADE80;
		constexpr u32 kVolatileEnd = 0x4ADEC0;

		// JVS bits driven by the input script (no START/SERVICE/coin, so menus and test mode stay out).
		constexpr u16 kScriptBits[] = {JVS_BTN_UP, JVS_BTN_DOWN, JVS_BTN_LEFT, JVS_BTN_RIGHT,
			JVS_BTN_1, JVS_BTN_2, JVS_BTN_3, JVS_BTN_4};

		Phase s_phase = Phase::Idle;
		bool s_requested = false;
		u32 s_n = 0;      // frames since the snapshot
		u32 s_repeat = 0; // completed re-simulations
		u32 s_pass = 0;
		u32 s_fail = 0;
		u16 s_mask = 0; // script bits currently pressed
		LimiterModeType s_prev_limiter = LimiterModeType::Nominal;
		std::string s_state_path;
		std::vector<u8> s_ref;
		std::vector<u8> s_cur;
		Common::Timer s_timer;
		double s_ms_save = 0.0;
		double s_ms_load = 0.0;
		double s_ms_ref = 0.0;

		u32 RamSize()
		{
			u32 size = Ps2MemSize::ExposedRam;
			if (size == 0 || size > sizeof(eeMem->Main))
				size = Ps2MemSize::MainRam;
			return size;
		}

		void CopyRam(std::vector<u8>& out)
		{
			const u32 size = RamSize();
			out.resize(size);
			std::memcpy(out.data(), eeMem->Main, size);
		}

		// Scripted input: a pure function of the frame index n (so the reference pass and every
		// re-simulation see exactly the same input at exactly the same frame).
		u16 ScriptMask(u32 n)
		{
			if (n == 0 || n >= kFrames)
				return 0;
			const u32 h = (n / 6 + 1) * 2654435761u;
			u16 mask = 0;
			for (u32 i = 0; i < static_cast<u32>(std::size(kScriptBits)); i++)
			{
				if ((h >> (i * 3 + 5)) & 1u)
					mask |= kScriptBits[i];
			}
			return mask;
		}

		void ApplyMask(u16 mask)
		{
			const u16 changed = static_cast<u16>(mask ^ s_mask);
			for (const u16 bit : kScriptBits)
			{
				if (changed & bit)
					ACJV::SetButtonState(0, bit, (mask & bit) != 0);
			}
			s_mask = mask;
		}

		struct DiffResult
		{
			u32 lines = 0;   // differing 64-byte lines (volatile line excluded)
			u32 ignored = 0; // differing lines inside the known volatile range
			std::vector<std::pair<u32, u32>> ranges;
		};

		DiffResult Diff(const std::vector<u8>& a, const std::vector<u8>& b)
		{
			DiffResult r;
			const size_t n = std::min(a.size(), b.size());
			for (size_t o = 0; o + 64 <= n; o += 64)
			{
				if (std::memcmp(&a[o], &b[o], 64) == 0)
					continue;
				const u32 begin = static_cast<u32>(o);
				if (begin >= kVolatileBegin && begin < kVolatileEnd)
				{
					r.ignored++;
					continue;
				}
				r.lines++;
				if (!r.ranges.empty() && begin - r.ranges.back().second <= 256)
					r.ranges.back().second = begin + 64;
				else
					r.ranges.emplace_back(begin, begin + 64);
			}
			return r;
		}

		void EndTest(const char* why)
		{
			if (VMManager::HasValidVM())
				VMManager::SetLimiterMode(s_prev_limiter);
			ApplyMask(0);
			s_phase = Phase::Idle;
			s_requested = false;
			s_ref.clear();
			s_ref.shrink_to_fit();
			s_cur.clear();
			s_cur.shrink_to_fit();
			Console.WriteLn("RollbackTest: finished (%s): %u PASS / %u FAIL", why, s_pass, s_fail);
		}

		void StartResim()
		{
			Error error;
			Common::Timer timer;
			if (!VMManager::LoadState(s_state_path.c_str(), &error))
			{
				Console.Error("RollbackTest: LoadState failed: %s", error.GetDescription().c_str());
				EndTest("load failed");
				return;
			}
			s_ms_load = timer.GetTimeMilliseconds();

			// We are back at the moment of the snapshot: restart the frame index and put the
			// scripted input back to its initial (released) state.
			s_n = 0;
			ApplyMask(0);
			s_phase = Phase::Resim;
			VMManager::SetLimiterMode(LimiterModeType::Unlimited);
			s_timer.Reset();
		}

		void BeginTest()
		{
			s_requested = false;
			s_n = 0;
			s_repeat = 0;
			s_pass = 0;
			s_fail = 0;
			s_prev_limiter = VMManager::GetLimiterMode();
			s_state_path = Path::Combine(EmuFolders::Savestates, "rollback_selftest.p2s");
			ApplyMask(0);

			Console.WriteLn("RollbackTest: begin. frames=%u repeats=%u ram=%u MB state=%s",
				kFrames, kRepeats, RamSize() >> 20, s_state_path.c_str());

			bool save_ok = true;
			Common::Timer timer;
			VMManager::SaveState(s_state_path.c_str(), false, false,
				[&save_ok](const std::string& message) {
					save_ok = false;
					Console.Error("RollbackTest: SaveState failed: %s", message.c_str());
				});
			s_ms_save = timer.GetTimeMilliseconds();
			if (!save_ok)
			{
				EndTest("save failed");
				return;
			}

			Console.WriteLn("RollbackTest: snapshot saved in %.1f ms; running reference pass", s_ms_save);
			s_phase = Phase::Reference;
			s_timer.Reset();
		}

		void ReportResim(double resim_ms)
		{
			const DiffResult d = Diff(s_ref, s_cur);
			const bool ok = (d.lines == 0);
			if (ok)
				s_pass++;
			else
				s_fail++;

			Console.WriteLn("RollbackTest: pass %u/%u %s: differing 64B lines=%u (known-volatile ignored=%u); "
							"load=%.1f ms, resim %u frames in %.1f ms = %.0f frames/s (reference pass %.1f ms)",
				s_repeat + 1, kRepeats, ok ? "PASS" : "FAIL", d.lines, d.ignored, s_ms_load,
				kFrames, resim_ms, (resim_ms > 0.0) ? (kFrames * 1000.0 / resim_ms) : 0.0, s_ms_ref);

			if (!ok)
			{
				const size_t shown = std::min<size_t>(d.ranges.size(), 24);
				for (size_t i = 0; i < shown; i++)
				{
					Console.Error("RollbackTest:   diff 0x%06X-0x%06X (%u bytes)",
						d.ranges[i].first, d.ranges[i].second, d.ranges[i].second - d.ranges[i].first);
				}
				if (d.ranges.size() > shown)
					Console.Error("RollbackTest:   ... and %u more ranges", static_cast<u32>(d.ranges.size() - shown));
			}
		}
	} // namespace

	void RequestSelfTest()
	{
		if (s_phase == Phase::Idle)
		{
			s_requested = true;
			Console.WriteLn("RollbackTest: requested; starts at the next frame boundary");
		}
	}

	bool IsSelfTestRunning()
	{
		return s_phase != Phase::Idle;
	}

	void OnVSync()
	{
		if (s_phase == Phase::Idle)
		{
			if (s_requested)
			{
				if (VMManager::HasValidVM())
					BeginTest();
				else
					s_requested = false;
			}
			return;
		}

		if (!VMManager::HasValidVM())
		{
			EndTest("VM went away");
			return;
		}

		s_n++;
		ApplyMask(ScriptMask(s_n));
		if (s_n < kFrames)
			return;

		if (s_phase == Phase::Reference)
		{
			CopyRam(s_ref);
			s_ms_ref = s_timer.GetTimeMilliseconds();
			StartResim();
			return;
		}

		// Phase::Resim
		const double resim_ms = s_timer.GetTimeMilliseconds();
		CopyRam(s_cur);
		ReportResim(resim_ms);
		s_repeat++;
		if (s_repeat < kRepeats)
			StartResim();
		else
			EndTest("done");
	}
} // namespace Rollback
