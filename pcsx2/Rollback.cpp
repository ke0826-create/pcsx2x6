// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#define _PC_ // disables the MIPS opcode macros (R3000A.h is included for the SIF structures)

#include "Rollback.h"
#include "RollbackNet.h"

#include "Common.h"
#include "Memory.h"
#include "R3000A.h"
#include "Sif.h"
#include "Config.h"
#include "DEV9/ACJV.h"
#include "MemoryTypes.h"
#include "R5900.h"
#include "SaveState.h"
#include "VMManager.h"

#include "common/Console.h"
#include "common/Error.h"
#include "common/Path.h"
#include "common/Timer.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <deque>
#include <iterator>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// Offline rollback self-test
// --------------------------
// Trigger: hotkey "RollbackSelfTest" (default F10) while a game is running, ideally in the
// middle of a CPU battle with nobody touching the pad.
//
// 1. Measures which EE RAM the in-memory snapshot has to cover (the first 32 MB plus every non-zero
//    64 KB block above it) and benchmarks the snapshot: saves, and loads in the three modes
//    (full / keep-JIT / diff), with the per-phase breakdown of the last iteration.
// 2. A small matrix of configurations (restore mode x scripted input x re-simulation speed).
//    For each configuration:
//
//   frame boundary k   : request "leave the CPU loop"; right after the loop is left (a clean point,
//                        no event handler in flight) take a snapshot, n = 0
//   frames n = 1..N    : (optional) scripted JVS input as a pure function of n; normal speed
//   n == N             : copy ALL EE RAM -> reference
//   restore            : request "leave the CPU loop"; at the clean point restore the snapshot, n = 0
//   frames n = 1..N    : the SAME scripted input; normal or unlimited speed
//   n == N             : copy ALL EE RAM, compare with the reference and with the first re-simulation
//   repeat the restore/re-simulate/compare kRepeats times
//
// The comparison covers the whole EE RAM, so a region the snapshot does not cover but the game writes
// shows up as a difference. A hash of the first 32 MB is taken at every frame boundary to report the
// first frame at which two runs diverge (0 = right after the restore).
//
// The log (emulog.txt / console) gets lines starting with "RollbackTest:" and "SaveStateProfile:".
//  * Rendering is NOT skipped during the re-simulation; the measured speed is therefore a
//    conservative number for "how fast can we re-simulate".
//  * Saving/loading only at the clean point matters: 1447cc1/fa60cbb saved and loaded from inside the
//    VSyncStart handler and every restored run diverged from the live run in the first frame.
namespace Rollback
{
	namespace
	{
		enum class Phase
		{
			Idle,
			WaitBench, // asked the CPU loop to exit; the benchmark and the first snapshot run at the clean point
			WaitSave,  // asked the CPU loop to exit; the snapshot is taken at the clean point
			Reference, // running the first (reference) pass
			WaitLoad,  // asked the CPU loop to exit; the snapshot is restored at the clean point
			Resim,     // running a re-simulation pass after a restore
		};

		enum class Pending
		{
			None,
			Bench,
			Save,
			Load,
		};

		struct Cfg
		{
			bool input;     // drive the JVS buttons from the script
			bool unlimited; // re-simulate with the limiter off
			FastLoadMode mode;
			const char* name;
		};

		// The "full" configuration is the control (what the first version did): it is last.
		constexpr Cfg kMatrix[] = {
			{true, true, FastLoadMode::Diff, "diff/input/unlimited"},
			{false, false, FastLoadMode::Diff, "diff/no-input/normal-speed"},
			{true, true, FastLoadMode::Full, "full/input/unlimited"},
		};

		constexpr u32 kFrames = 120; // frames per pass (N)
		constexpr u32 kRepeats = 3;  // restore + re-simulate passes per configuration
		constexpr u32 kHashBytes = 32u * 1024u * 1024u; // per-frame hash covers the first 32 MB
		constexpr u32 kBaseRegionBytes = 32u * 1024u * 1024u;
		constexpr u32 kBlockBytes = 64u * 1024u; // granularity of the "non-zero above 32 MB" scan
		constexpr u32 kBenchSaves = 20;
		constexpr u32 kBenchDiffLoads = 20;
		constexpr u32 kBenchSlowLoads = 3; // full / keep-JIT loads are slow, a few are enough
		constexpr u32 kFirstFrames = 5;    // number of frames after a restore whose time is logged

		// 64-byte line holding the USB I/O board status counter (".;RAYS PCB;"), measured to differ
		// between a straight run and a restored run without affecting the game logic (2026-10-19).
		constexpr u32 kVolatileBegin = 0x4ADE80;
		constexpr u32 kVolatileEnd = 0x4ADEC0;

		// JVS bits driven by the input script (no START/SERVICE/coin, so menus and test mode stay out).
		constexpr u16 kScriptBits[] = {JVS_BTN_UP, JVS_BTN_DOWN, JVS_BTN_LEFT, JVS_BTN_RIGHT,
			JVS_BTN_1, JVS_BTN_2, JVS_BTN_3, JVS_BTN_4};

		Phase s_phase = Phase::Idle;
		Pending s_pending = Pending::None;
		bool s_requested = false;
		u32 s_cfg = 0;    // index into kMatrix
		u32 s_n = 0;      // frames since the snapshot
		u32 s_repeat = 0; // completed re-simulations of the current configuration
		u32 s_pass = 0;
		u32 s_fail = 0;
		u16 s_mask = 0; // script bits currently pressed
		LimiterModeType s_prev_limiter = LimiterModeType::Nominal;
		FastSnapshot s_snap;
		std::vector<FastRegion> s_regions; // EE RAM covered by the snapshot
		std::vector<u8> s_ref;
		std::vector<u8> s_cur;
		std::vector<u8> s_first; // first re-simulation of the current configuration
		std::vector<u64> s_hash_ref;
		std::vector<u64> s_hash_cur;
		std::vector<u64> s_hash_first;
		double s_first_frame_ms[kFirstFrames + 1] = {};
		Common::Timer s_timer;
		double s_hash_ms = 0.0; // time spent hashing in the current pass
		double s_ms_load = 0.0;
		double s_ms_save = 0.0;
		double s_ms_ref = 0.0;
		u32 s_pages_restored = 0;

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

		// FNV-1a style 64-bit hash over the first 32 MB, skipping the known volatile line.
		u64 HashRam()
		{
			const u64* p = reinterpret_cast<const u64*>(eeMem->Main);
			const size_t total = std::min<size_t>(RamSize(), kHashBytes) / 8;
			const size_t vb = kVolatileBegin / 8;
			const size_t ve = kVolatileEnd / 8;
			u64 h = 1469598103934665603ull;
			for (size_t i = 0; i < vb && i < total; i++)
				h = (h ^ p[i]) * 1099511628211ull;
			for (size_t i = ve; i < total; i++)
				h = (h ^ p[i]) * 1099511628211ull;
			return h;
		}

		void RecordHash(std::vector<u64>& dst, u32 index)
		{
			Common::Timer t;
			if (dst.size() < static_cast<size_t>(kFrames) + 1)
				dst.resize(static_cast<size_t>(kFrames) + 1, 0);
			dst[index] = HashRam();
			s_hash_ms += t.GetTimeMilliseconds();
		}

		// First frame index at which the two hash sequences differ, or -1.
		int FirstDivergence(const std::vector<u64>& a, const std::vector<u64>& b)
		{
			const size_t n = std::min(a.size(), b.size());
			for (size_t i = 0; i < n && i <= kFrames; i++)
			{
				if (a[i] != b[i])
					return static_cast<int>(i);
			}
			return -1;
		}

		// Scripted input: a pure function of the frame index n (so the reference pass and every
		// re-simulation see exactly the same input at exactly the same frame).
		u16 ScriptMask(u32 n)
		{
			if (!kMatrix[s_cfg].input || n == 0 || n >= kFrames)
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
			u32 kernel = 0;  // of `lines`: those below the ELF load address 0x100000 (EE kernel / BIOS area)
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
				if (begin < 0x100000)
					r.kernel++;
				if (!r.ranges.empty() && begin - r.ranges.back().second <= 256)
					r.ranges.back().second = begin + 64;
				else
					r.ranges.emplace_back(begin, begin + 64);
			}
			return r;
		}

		std::string HexPreview(const std::vector<u8>& v, u32 off)
		{
			std::string s;
			char tmp[4];
			for (u32 i = 0; i < 16 && off + i < v.size(); i++)
			{
				std::snprintf(tmp, sizeof(tmp), "%02x", v[off + i]);
				s += tmp;
			}
			return s;
		}

		// Prints one comparison; returns true when the two RAM images match.
		bool LogCompare(const char* tag, const std::vector<u8>& a, const std::vector<u8>& b, int first_div)
		{
			const DiffResult d = Diff(a, b);
			const bool ok = (d.lines == 0);
			Console.WriteLn("RollbackTest:   %s: %s differing 64B lines=%u (known-volatile ignored=%u) first-divergent-frame=%d",
				tag, ok ? "MATCH" : "DIFFER", d.lines, d.ignored, first_div);
			if (!ok)
			{
				const size_t shown = std::min<size_t>(d.ranges.size(), 12);
				for (size_t i = 0; i < shown; i++)
				{
					const u32 begin = d.ranges[i].first;
					Console.WriteLn("RollbackTest:     0x%06X-0x%06X (%u B) A=%s B=%s", begin, d.ranges[i].second,
						d.ranges[i].second - begin, HexPreview(a, begin).c_str(), HexPreview(b, begin).c_str());
				}
				if (d.ranges.size() > shown)
					Console.WriteLn("RollbackTest:     ... and %u more ranges", static_cast<u32>(d.ranges.size() - shown));
			}
			return ok;
		}

		void EndTest(const char* why)
		{
			g_SaveStateProfile = false;
			if (VMManager::HasValidVM())
				VMManager::SetLimiterMode(s_prev_limiter);
			ApplyMask(0);
			s_phase = Phase::Idle;
			s_pending = Pending::None;
			s_requested = false;
			s_ref.clear();
			s_ref.shrink_to_fit();
			s_cur.clear();
			s_cur.shrink_to_fit();
			s_first.clear();
			s_first.shrink_to_fit();
			s_snap = FastSnapshot();
			Console.WriteLn("RollbackTest: finished (%s): %u PASS / %u FAIL (vs reference)", why, s_pass, s_fail);
		}

		// Asks the CPU loop to exit; the action runs in OnExecutionExit().
		void RequestAction(Pending what, Phase wait_phase)
		{
			s_pending = what;
			s_phase = wait_phase;
			Cpu->ExitExecution();
		}

		// Resets the per-configuration counters (no state change, safe anywhere).
		void PrepareConfig()
		{
			s_n = 0;
			s_repeat = 0;
			s_hash_ms = 0.0;
			VMManager::SetLimiterMode(s_prev_limiter);
			ApplyMask(0);
		}

		void BeginConfig()
		{
			PrepareConfig();
			RequestAction(Pending::Save, Phase::WaitSave);
		}

		void NextConfig()
		{
			s_cfg++;
			if (s_cfg < static_cast<u32>(std::size(kMatrix)))
				BeginConfig();
			else
				EndTest("done");
		}

		// Runs at the clean point after the CPU loop was left.
		void DoSave()
		{
			Error error;
			Common::Timer timer;
			if (!SaveState_FastSave(s_snap, true, &error))
			{
				Console.Error("RollbackTest: SaveState_FastSave failed: %s", error.GetDescription().c_str());
				EndTest("save failed");
				return;
			}
			s_ms_save = timer.GetTimeMilliseconds();

			s_hash_ms = 0.0;
			RecordHash(s_hash_ref, 0);
			Console.WriteLn("RollbackTest: [%u/%u] config '%s': snapshot saved in %.2f ms; running reference pass",
				s_cfg + 1, static_cast<u32>(std::size(kMatrix)), kMatrix[s_cfg].name, s_ms_save);
			s_n = 0;
			s_phase = Phase::Reference;
			s_hash_ms = 0.0;
			s_timer.Reset();
		}

		// Runs at the clean point after the CPU loop was left.
		void DoLoad()
		{
			Error error;
			Common::Timer timer;
			if (!SaveState_FastLoad(s_snap, kMatrix[s_cfg].mode, s_repeat == 0, &s_pages_restored, &error))
			{
				Console.Error("RollbackTest: restore failed: %s", error.GetDescription().c_str());
				EndTest("load failed");
				return;
			}
			s_ms_load = timer.GetTimeMilliseconds();

			// We are back at the moment of the snapshot: restart the frame index and put the
			// scripted input back to its initial (released) state.
			s_n = 0;
			ApplyMask(0);
			s_hash_ms = 0.0;
			RecordHash(s_hash_cur, 0);
			s_hash_ms = 0.0;
			std::fill(std::begin(s_first_frame_ms), std::end(s_first_frame_ms), 0.0);
			s_phase = Phase::Resim;
			VMManager::SetLimiterMode(kMatrix[s_cfg].unlimited ? LimiterModeType::Unlimited : s_prev_limiter);
			s_timer.Reset();
		}

		// The snapshot covers the first 32 MB plus every non-zero 64 KB block above it (coalesced).
		void MeasureRamUse()
		{
			const u32 ram = RamSize();
			const u32 base_end = std::min<u32>(kBaseRegionBytes, ram);
			s_regions.clear();
			s_regions.push_back({0, base_end});

			u32 extra_blocks = 0;
			for (u32 off = base_end; off + kBlockBytes <= ram; off += kBlockBytes)
			{
				const u64* w = reinterpret_cast<const u64*>(eeMem->Main + off);
				bool nonzero = false;
				for (u32 i = 0; i < kBlockBytes / 8; i++)
				{
					if (w[i])
					{
						nonzero = true;
						break;
					}
				}
				if (!nonzero)
					continue;

				extra_blocks++;
				FastRegion& last = s_regions.back();
				if (last.offset + last.size == off)
					last.size += kBlockBytes;
				else
					s_regions.push_back({off, kBlockBytes});
			}

			u32 total = 0;
			for (const FastRegion& r : s_regions)
				total += r.size;
			Console.WriteLn("RollbackTest: EE RAM %u MB exposed: base %u MB + %u non-zero 64 KB blocks above it in %u regions; snapshot covers %u KB",
				ram >> 20, base_end >> 20, extra_blocks, static_cast<u32>(s_regions.size()), total >> 10);
			const size_t shown = std::min<size_t>(s_regions.size(), 10);
			for (size_t i = 0; i < shown; i++)
				Console.WriteLn("RollbackTest:   region %u: 0x%07X + %u KB", static_cast<u32>(i), s_regions[i].offset, s_regions[i].size >> 10);
		}

		// Times the in-memory snapshot on its own (no frames run in between; the state does not change).
		void RunBench()
		{
			MeasureRamUse();

			FastSnapshot snap;
			snap.ee_regions = s_regions;
			Error error;
			Common::Timer timer;
			if (!SaveState_FastSave(snap, true, &error)) // first call: allocation + page faults
			{
				Console.Error("RollbackTest: bench: first save failed: %s", error.GetDescription().c_str());
				return;
			}
			const double first_ms = timer.GetTimeMillisecondsAndReset();

			for (u32 i = 0; i < kBenchSaves; i++)
			{
				if (!SaveState_FastSave(snap, i + 1 == kBenchSaves, &error))
				{
					Console.Error("RollbackTest: bench: save failed: %s", error.GetDescription().c_str());
					return;
				}
			}
			const double save_ms = timer.GetTimeMillisecondsAndReset() / kBenchSaves;

			Console.WriteLn("RollbackTest: bench: in-memory snapshot %u KB (internals %u KB, EE RAM %u KB, other %u KB): first save %.2f ms, save avg %.2f ms",
				static_cast<u32>((snap.internals.size() + snap.ee.size() + snap.other.size()) >> 10),
				static_cast<u32>(snap.internals.size() >> 10), static_cast<u32>(snap.ee.size() >> 10),
				static_cast<u32>(snap.other.size() >> 10), first_ms, save_ms);

			struct Item
			{
				FastLoadMode mode;
				u32 iterations;
				const char* name;
			};
			const Item items[] = {
				{FastLoadMode::Diff, kBenchDiffLoads, "diff (no ResetBlockTracking, JIT kept)"},
				{FastLoadMode::KeepJit, kBenchSlowLoads, "keep-JIT (ResetBlockTracking, JIT kept)"},
				{FastLoadMode::Full, kBenchSlowLoads, "full (ResetBlockTracking + JIT cleared)"},
			};
			for (const Item& it : items)
			{
				u32 pages = 0;
				timer.Reset();
				for (u32 i = 0; i < it.iterations; i++)
				{
					if (!SaveState_FastLoad(snap, it.mode, i + 1 == it.iterations, &pages, &error))
					{
						Console.Error("RollbackTest: bench: load failed: %s", error.GetDescription().c_str());
						return;
					}
				}
				Console.WriteLn("RollbackTest: bench: load %s: avg %.2f ms over %u loads (pages written in the last one: %u)",
					it.name, timer.GetTimeMilliseconds() / it.iterations, it.iterations, pages);
			}
		}

		void BeginTest()
		{
			s_requested = false;
			s_cfg = 0;
			s_pass = 0;
			s_fail = 0;
			s_prev_limiter = VMManager::GetLimiterMode();
			s_snap = FastSnapshot();
			g_SaveStateProfile = false;

			Console.WriteLn("RollbackTest: begin. frames=%u repeats=%u configs=%u ram=%u MB",
				kFrames, kRepeats, static_cast<u32>(std::size(kMatrix)), RamSize() >> 20);
			// The benchmark and the first snapshot are taken at the clean point after the CPU loop exits.
			RequestAction(Pending::Bench, Phase::WaitBench);
		}

		void ReportResim(double resim_total_ms)
		{
			const double net_ms = resim_total_ms - s_hash_ms;
			Console.WriteLn("RollbackTest: [%u/%u] '%s' resim %u/%u: restore=%.2f ms (pages written %u), %u frames in %.1f ms (excluding hashing) = %.0f frames/s; "
							"reference pass %.1f ms",
				s_cfg + 1, static_cast<u32>(std::size(kMatrix)), kMatrix[s_cfg].name, s_repeat + 1, kRepeats,
				s_ms_load, s_pages_restored, kFrames, net_ms, (net_ms > 0.0) ? (kFrames * 1000.0 / net_ms) : 0.0, s_ms_ref);
			Console.WriteLn("RollbackTest:   first re-simulated frames, cumulative ms: %.1f %.1f %.1f %.1f %.1f",
				s_first_frame_ms[1], s_first_frame_ms[2], s_first_frame_ms[3], s_first_frame_ms[4], s_first_frame_ms[5]);

			const int load_div = (s_hash_ref[0] == s_hash_cur[0]) ? -1 : 0;
			const bool ok = LogCompare("vs reference", s_ref, s_cur, (load_div == 0) ? 0 : FirstDivergence(s_hash_ref, s_hash_cur));
			if (ok)
				s_pass++;
			else
				s_fail++;

			if (s_repeat == 0)
			{
				s_first = s_cur;
				s_hash_first = s_hash_cur;
			}
			else
			{
				LogCompare("vs first re-simulation", s_first, s_cur, FirstDivergence(s_hash_first, s_hash_cur));
			}
		}
		// ======================================================================================
		// Rollback session test (late inputs)
		// ======================================================================================
		// Models what netplay needs, in one emulator: the input of every frame is KNOWN only some frames
		// later ("delivered"); until then the loop runs with a PREDICTED input (hold the last known one).
		// When a delivered input differs from what was used, the loop restores the newest snapshot at or
		// before that frame and re-simulates up to the present with the corrected inputs.
		//
		//   ground truth : S0 -> kN frames with the true inputs on time -> EE RAM
		//   session      : S0 -> kN frames with late inputs + rollbacks   -> EE RAM   (must be identical)
		//
		// Two ways to give the game its input:
		//   JVS (F11)     : ACJV::SetButtonState, as a human on the pad would.
		//   mailbox (F12) : the guest's human-unit input reader is patched to read its pad word from an
		//                   emulator-owned table in EE RAM (kMailbox + slot * 0x40) instead of the guest's
		//                   pad array; the emulator writes the word for frame n at hook n. This is the entry
		//                   point for the inputs of the other human slots in netplay (design memo, chapter 14).
		//                   The words are the ones the guest's own pad driver produced in the JVS ground truth
		//                   run (*(0x5E8B70) read at the next hook), so no bit mapping has to be reproduced.
		//
		// Snapshots are taken every kK frames into a ring (a snapshot is valid until a misprediction at an
		// earlier frame is found; then it is dropped). Hook n applies the input of "frame n"; the snapshot of
		// frame n is taken at the clean point right after hook n (start of frame n, input n applied).
		// The clean point rule is the same as in the other test: only OnExecutionExit() saves, loads or
		// patches guest code.
		namespace sess
		{
			constexpr u32 kN = 720;   // frames per run
			constexpr u32 kK = 4;     // snapshot interval
			constexpr u32 kRing = 5;  // snapshots kept (covers about kK * kRing frames)
			constexpr u32 kNone = 0xFFFFFFFFu;

			constexpr u32 kMailbox = 0x2000000; // EE RAM address of the input mailbox (above the game's 32 MB)
			constexpr u32 kMailboxStride = 0x40;
			constexpr u32 kDevicePad = 0x5E8B70; // the guest pad driver's output for pad 0 (cur word at +0)

			// Guest patch (all 4 words are in the input reader 0x282540, at the two places where it builds the
			// base address of the pad array: lui v0,0x5f / addiu v0,v0,-0x7590 = 0x5E8A70):
			// the base becomes 0x2000000 (the mailbox). CPU units (unit+0xA != 0) do not read the pad.
			struct PatchWord
			{
				u32 addr;
				u32 original;
				u32 patched;
			};
			constexpr PatchWord kPatch[] = {
				{0x28259C, 0x3C02005F, 0x3C020200}, // lui   v0,0x5f       -> lui   v0,0x200   (mode == 1 path)
				{0x2825A0, 0x24428A70, 0x24420000}, // addiu v0,v0,-0x7590 -> addiu v0,v0,0
				{0x2825C0, 0x3C02005F, 0x3C020200}, // lui   v0,0x5f       -> lui   v0,0x200   (normal path)
				{0x2825C4, 0x24428A70, 0x24420000}, // addiu v0,v0,-0x7590 -> addiu v0,v0,0
			};

			enum class Pat
			{
				Fixed,  // every input is delivered `param` frames late
				Jitter, // 0..param frames late, different per frame (out of order delivery)
				Burst,  // on time, except frames 40..53 of every 60 are all delivered at frame 54
			};

			struct Pattern
			{
				Pat kind;
				u32 param;
				const char* name;
			};

			constexpr Pattern kPatterns[] = {
				{Pat::Fixed, 3, "fixed delay 3"},
				{Pat::Fixed, 8, "fixed delay 8"},
				{Pat::Jitter, 10, "jitter 0-10"},
				{Pat::Burst, 14, "bursts (up to 14 late)"},
			};

			enum class Step
			{
				Idle,
				Gt,   // ground-truth run, JVS inputs (records the pad words in mailbox mode)
				GtMb, // ground-truth run, mailbox inputs, on time (mailbox mode only)
				Run,  // session run (live or re-simulating)
			};

			enum class Act
			{
				None,
				Init,       // take S0
				RestoreGt2, // restore S0, patch the guest, start the mailbox ground truth
				Restore0,   // restore S0 (re-patch in mailbox mode) and start the next pattern
				Save,       // take a ring snapshot
				Rollback,   // restore a ring snapshot
				Cleanup,    // remove the guest patch, finish
			};

			struct Slot
			{
				FastSnapshot snap;
				u32 frame = 0;
				bool valid = false;
			};

			bool requested = false;
			bool want_mailbox = false; // the pending request is the mailbox test (F12)
			bool mb_mode = false;      // inputs go through the mailbox
			bool patched = false;      // the guest patch is in place
			Step step = Step::Idle;
			Act act = Act::None;
			u32 pat = 0;
			u32 n = 0;      // current hook index
			u32 target = 0; // hook index to reach again after a rollback
			bool resim = false;
			u32 save_slot = 0;
			u32 save_frame = 0;
			u32 roll_slot = 0;
			FastSnapshot s0;
			Slot ring[kRing];
			std::vector<u8> gt_ram;  // ground truth the patterns are compared with
			std::vector<u8> jvs_ram; // JVS ground truth (mailbox mode: for the equivalence check)
			std::vector<u8> rb_ram;
			u16 jvs_truth[kN + 1] = {};
			u32 devw[kN + 1] = {};    // pad words the guest produced for frame f in the JVS run
			u32 truth[kN + 1] = {};   // the input words the session delivers
			bool delivered[kN + 1] = {};
			u32 deliver_at[kN + 1] = {};
			u32 used[kN + 1] = {};
			// statistics of the current pattern
			u32 rollbacks = 0, resim_frames = 0, max_depth = 0, saves = 0, pred_ok = 0, pred_bad = 0;
			double ms_restore = 0.0, ms_save = 0.0, gt_ms = 0.0;
			Common::Timer wall;

			u16 HashMask(u32 f)
			{
				const u32 h = (f / 6 + 1) * 2654435761u;
				u16 mask = 0;
				for (u32 i = 0; i < static_cast<u32>(std::size(kScriptBits)); i++)
				{
					if ((h >> (i * 3 + 5)) & 1u)
						mask |= kScriptBits[i];
				}
				return mask;
			}

			u32 Delay(const Pattern& p, u32 f)
			{
				switch (p.kind)
				{
					case Pat::Fixed:
						return p.param;
					case Pat::Jitter:
						return ((f * 2654435761u) >> 7) % (p.param + 1);
					case Pat::Burst:
					{
						const u32 c = f % 60;
						return (c >= 40 && c < 54) ? (54 - c) : 0;
					}
				}
				return 0;
			}

			u32 ReadDevice()
			{
				return *reinterpret_cast<const u32*>(eeMem->Main + kDevicePad);
			}

			void WriteMailbox(u32 slot, u32 word)
			{
				*reinterpret_cast<u32*>(eeMem->Main + kMailbox + slot * kMailboxStride) = word;
			}

			// Writes the guest patch (or restores the original words). Only at the clean point.
			bool ApplyGuestPatch(bool on)
			{
				for (const PatchWord& p : kPatch)
				{
					const u32 now = *reinterpret_cast<const u32*>(eeMem->Main + p.addr);
					const u32 expect = on ? p.original : p.patched;
					if (now != expect)
					{
						Console.Error("RollbackSession: guest word at 0x%06X is 0x%08X, expected 0x%08X: not patching", p.addr, now, expect);
						return false;
					}
				}
				for (const PatchWord& p : kPatch)
					*reinterpret_cast<u32*>(eeMem->Main + p.addr) = on ? p.patched : p.original;
				patched = on;
				return true;
			}

			// The input to use for frame f: the true one if it was delivered, else "hold the last known".
			u32 InputFor(u32 f)
			{
				if (delivered[f])
					return truth[f];
				for (u32 g = f; g > 0; g--)
				{
					if (delivered[g - 1])
						return truth[g - 1];
				}
				return 0;
			}

			void ApplyInput(u32 word)
			{
				if (mb_mode)
					WriteMailbox(0, word);
				else
					ApplyMask(static_cast<u16>(word));
			}

			void ApplyUse(u32 f)
			{
				used[f] = InputFor(f);
				ApplyInput(used[f]);
			}

			bool Active()
			{
				return step != Step::Idle || act != Act::None;
			}

			void Finish(const char* why)
			{
				ApplyMask(0);
				if (VMManager::HasValidVM())
					VMManager::SetLimiterMode(s_prev_limiter);
				step = Step::Idle;
				act = Act::None;
				requested = false;
				resim = false;
				gt_ram.clear();
				gt_ram.shrink_to_fit();
				jvs_ram.clear();
				jvs_ram.shrink_to_fit();
				rb_ram.clear();
				rb_ram.shrink_to_fit();
				s0 = FastSnapshot();
				for (Slot& s : ring)
				{
					s.snap = FastSnapshot();
					s.valid = false;
				}
				Console.WriteLn("RollbackSession: finished (%s): %u PASS / %u FAIL (session vs ground truth)%s", why, s_pass, s_fail,
					patched ? " [WARNING: the guest patch is still applied]" : "");
			}

			void RequestAct(Act a)
			{
				act = a;
				Cpu->ExitExecution();
			}

			void StartPattern()
			{
				RequestAct(Act::Restore0);
			}

			// Delivers the inputs that are due at network time n (all of them when `all`). Returns the
			// earliest frame whose used input was wrong, or kNone.
			u32 Deliver(bool all)
			{
				u32 earliest = kNone;
				const u32 first = (n > 40) ? n - 40 : 0;
				const u32 last = std::min<u32>(all ? kN - 1 : n, kN - 1);
				for (u32 f = all ? 0 : first; f <= last; f++)
				{
					if (delivered[f] || (!all && deliver_at[f] > n))
						continue;
					delivered[f] = true;
					if (f < n)
					{
						if (used[f] != truth[f])
						{
							pred_bad++;
							earliest = std::min(earliest, f);
						}
						else
						{
							pred_ok++;
						}
					}
				}
				return earliest;
			}

			void RequestRollback(u32 mismatch_frame)
			{
				int best = -1;
				for (u32 i = 0; i < kRing; i++)
				{
					if (ring[i].valid && ring[i].frame <= mismatch_frame && (best < 0 || ring[i].frame > ring[best].frame))
						best = static_cast<int>(i);
				}
				if (best < 0)
				{
					Console.Error("RollbackSession: no snapshot at or before frame %u (hook %u): the window is too short", mismatch_frame, n);
					Finish("no snapshot");
					return;
				}
				for (u32 i = 0; i < kRing; i++)
				{
					if (ring[i].frame > ring[best].frame)
						ring[i].valid = false;
				}
				roll_slot = static_cast<u32>(best);
				target = n;
				rollbacks++;
				const u32 depth = target - ring[best].frame;
				resim_frames += depth;
				max_depth = std::max(max_depth, depth);
				RequestAct(Act::Rollback);
			}

			void MaybeSave()
			{
				if (n > 0 && n % kK == 0)
				{
					save_slot = (n / kK) % kRing;
					save_frame = n;
					RequestAct(Act::Save);
				}
			}

			void ComparePattern()
			{
				CopyRam(rb_ram);
				const DiffResult d = Diff(gt_ram, rb_ram);
				const double session_ms = wall.GetTimeMilliseconds();
				const u32 game_lines = d.lines - d.kernel;
				if (game_lines == 0)
					s_pass++;
				else
					s_fail++;
				Console.WriteLn("RollbackSession: [%u/%u] '%s': %s (differing 64B lines: game area %u, EE kernel area <0x100000 %u, volatile ignored=%u) | predictions right %u / wrong %u | "
								"rollbacks=%u, frames re-simulated=%u (deepest %u), snapshots taken=%u | %u frames in %.0f ms (ground truth %.0f ms), restores %.1f ms, saves %.1f ms",
					pat + 1, static_cast<u32>(std::size(kPatterns)), kPatterns[pat].name, (game_lines == 0) ? "MATCH" : "DIFFER", game_lines, d.kernel, d.ignored,
					pred_ok, pred_bad, rollbacks, resim_frames, max_depth, saves, kN, session_ms, gt_ms, ms_restore, ms_save);
				if (d.lines != 0)
					LogCompare("session vs ground truth", gt_ram, rb_ram, -1);
				pat++;
				if (pat < static_cast<u32>(std::size(kPatterns)))
					StartPattern();
				else if (mb_mode)
					RequestAct(Act::Cleanup);
				else
					Finish("done");
			}

			// The mailbox ground truth is done: compare it with the JVS ground truth.
			void CompareMailboxGt()
			{
				CopyRam(gt_ram);
				const DiffResult d = Diff(jvs_ram, gt_ram);
				u32 pad_lines = 0; // differing lines inside the guest's pad arrays / pad driver structures
				for (const auto& r : d.ranges)
				{
					for (u32 a = r.first; a < r.second; a += 64)
					{
						if (a >= 0x5E8A00 && a < 0x5E8D00)
							pad_lines++;
					}
				}
				Console.WriteLn("RollbackSession: mailbox equivalence: JVS-driven run vs mailbox-driven run (the mailbox run had NO JVS input): "
								"differing 64B lines: game area %u (of which in the pad structures 0x5E8A00-0x5E8D00: at most %u), EE kernel area %u, volatile ignored=%u",
					d.lines - d.kernel, pad_lines, d.kernel, d.ignored);
				if (d.lines != 0)
					LogCompare("mailbox run vs JVS run", jvs_ram, gt_ram, -1);
				pat = 0;
				StartPattern();
			}

			void RunHook()
			{
				n++;
				if (resim)
				{
					if (n < target)
					{
						ApplyUse(n);
						MaybeSave();
						return;
					}
					resim = false; // back at the hook where the rollback was requested: handle it as a live hook
				}

				const bool settle = (n >= kN);
				const u32 m = Deliver(settle);
				if (m != kNone)
				{
					RequestRollback(m);
					return;
				}
				if (settle)
				{
					ComparePattern();
					return;
				}
				ApplyUse(n);
				MaybeSave();
			}

			void OnHook()
			{
				if (act != Act::None)
					return; // the CPU loop is about to exit

				if (step == Step::Gt)
				{
					n++;
					// The pad word the guest used for frame n-1 is in its pad driver structure now.
					if (want_mailbox && n >= 1)
						devw[n - 1] = ReadDevice();
					ApplyMask((n < kN) ? jvs_truth[n] : 0);
					if (n < kN)
						return;
					gt_ms = wall.GetTimeMilliseconds();
					if (want_mailbox)
					{
						CopyRam(jvs_ram);
						RequestAct(Act::RestoreGt2);
						return;
					}
					CopyRam(gt_ram);
					pat = 0;
					StartPattern();
					return;
				}
				if (step == Step::GtMb)
				{
					n++;
					if (n < kN)
					{
						WriteMailbox(0, truth[n]);
						return;
					}
					CompareMailboxGt();
					return;
				}
				if (step == Step::Run)
					RunHook();
			}

			void ResetPatternState()
			{
				for (u32 f = 0; f <= kN; f++)
				{
					delivered[f] = false;
					deliver_at[f] = f + Delay(kPatterns[pat], f);
					used[f] = 0;
				}
				for (Slot& s : ring)
					s.valid = false;
				rollbacks = resim_frames = max_depth = saves = pred_ok = pred_bad = 0;
				ms_restore = ms_save = 0.0;
				n = 0;
				resim = false;
			}

			// Runs at the clean point. Returns true when a session action was handled.
			bool OnExit()
			{
				if (act == Act::None)
					return false;

				const Act a = act;
				act = Act::None;
				if (!VMManager::HasValidVM() || VMManager::GetState() != VMState::Running)
				{
					Finish("execution interrupted");
					return true;
				}

				Error error;
				Common::Timer t;
				switch (a)
				{
					case Act::Init:
					{
						MeasureRamUse();
						if (want_mailbox && RamSize() < kMailbox + 4 * kMailboxStride)
						{
							Console.Error("RollbackSession: the mailbox address 0x%X is outside the exposed EE RAM (%u MB)", kMailbox, RamSize() >> 20);
							Finish("no mailbox");
							return true;
						}
						mb_mode = false;
						for (u32 f = 0; f <= kN; f++)
						{
							jvs_truth[f] = (f == 0 || f >= kN) ? 0 : HashMask(f);
							truth[f] = jvs_truth[f];
							devw[f] = 0;
							delivered[f] = false;
							deliver_at[f] = 0;
							used[f] = 0;
						}
						ApplyMask(0);
						s0 = FastSnapshot();
						s0.ee_regions = s_regions;
						if (!SaveState_FastSave(s0, false, &error))
						{
							Console.Error("RollbackSession: S0 save failed: %s", error.GetDescription().c_str());
							Finish("save failed");
							return true;
						}
						s_prev_limiter = VMManager::GetLimiterMode();
						VMManager::SetLimiterMode(LimiterModeType::Unlimited);
						n = 0;
						step = Step::Gt;
						wall.Reset();
						Console.WriteLn("RollbackSession: begin (%s). %u frames per run, snapshot every %u frames (ring %u), %u delay patterns",
							want_mailbox ? "mailbox inputs" : "JVS inputs", kN, kK, kRing, static_cast<u32>(std::size(kPatterns)));
						break;
					}
					case Act::RestoreGt2:
					{
						if (!SaveState_FastLoad(s0, FastLoadMode::Diff, false, nullptr, &error))
						{
							Console.Error("RollbackSession: S0 restore failed: %s", error.GetDescription().c_str());
							Finish("restore failed");
							return true;
						}
						// Log the word format the guest's pad driver produced, as a reference for the design memo.
						{
							u32 shown = 0;
							u32 last = 0xFFFFFFFFu;
							for (u32 f = 0; f < kN && shown < 8; f++)
							{
								if (devw[f] != last)
								{
									Console.WriteLn("RollbackSession:   pad word of frame %u: 0x%08X (JVS mask 0x%04X)", f, devw[f], jvs_truth[f]);
									last = devw[f];
									shown++;
								}
							}
						}
						if (!ApplyGuestPatch(true))
						{
							Finish("patch failed");
							return true;
						}
						mb_mode = true;
						for (u32 f = 0; f <= kN; f++)
						{
							truth[f] = devw[f];
							used[f] = 0;
						}
						ApplyMask(0); // the JVS stays idle: the units read the mailbox
						WriteMailbox(0, truth[0]);
						n = 0;
						step = Step::GtMb;
						wall.Reset();
						break;
					}
					case Act::Restore0:
					{
						if (!SaveState_FastLoad(s0, FastLoadMode::Diff, false, nullptr, &error))
						{
							Console.Error("RollbackSession: S0 restore failed: %s", error.GetDescription().c_str());
							Finish("restore failed");
							return true;
						}
						ResetPatternState();
						if (mb_mode)
						{
							// S0 holds the original guest code: patch again, and take the frame-0 snapshot after that.
							if (!ApplyGuestPatch(true))
							{
								Finish("patch failed");
								return true;
							}
							ring[0].snap.ee_regions = s_regions;
							if (!SaveState_FastSave(ring[0].snap, false, &error))
							{
								Console.Error("RollbackSession: frame-0 snapshot failed: %s", error.GetDescription().c_str());
								Finish("save failed");
								return true;
							}
						}
						else
						{
							ring[0].snap = s0; // frame 0 is the first snapshot
						}
						ring[0].frame = 0;
						ring[0].valid = true;
						step = Step::Run;
						ApplyUse(0);
						wall.Reset();
						break;
					}
					case Act::Save:
					{
						Slot& s = ring[save_slot];
						s.snap.ee_regions = s_regions;
						if (!SaveState_FastSave(s.snap, false, &error))
						{
							Console.Error("RollbackSession: snapshot save failed: %s", error.GetDescription().c_str());
							Finish("save failed");
							return true;
						}
						s.frame = save_frame;
						s.valid = true;
						saves++;
						ms_save += t.GetTimeMilliseconds();
						break;
					}
					case Act::Rollback:
					{
						Slot& s = ring[roll_slot];
						if (!SaveState_FastLoad(s.snap, FastLoadMode::Diff, false, nullptr, &error))
						{
							Console.Error("RollbackSession: restore failed: %s", error.GetDescription().c_str());
							Finish("restore failed");
							return true;
						}
						ms_restore += t.GetTimeMilliseconds();
						n = s.frame;
						resim = true;
						ApplyUse(n); // start of frame n with its (possibly corrected) input
						break;
					}
					case Act::Cleanup:
					{
						ApplyGuestPatch(false);
						mb_mode = false;
						Finish("done");
						break;
					}
					case Act::None:
						break;
				}
				return true;
			}
		} // namespace sess

		// ======================================================================================
		// Net battle test: mailbox inputs + stopping the game's own comm module during a battle
		// ======================================================================================
		// For the multi-emulator (nettest) setup. It is driven from outside through the file nb_ctrl.txt in the
		// emulator's working directory ("<cmd> <targetG1> <targetG2>", status in nb_status.txt), so that every
		// emulator can be switched at the SAME logical frame: the script writes absolute battle frame numbers (BF).
		//
		//   G1 (cmd bit 0): from BF >= TARGET_G1 the human units read the mailbox (the 4-word guest patch of the
		//                   F12 test). Every hook writes the words of all slots for the frame about to run; the
		//                   words are a pure function of (slot, BF), so every emulator writes the same ones.
		//   G2 (cmd bit 1): from BF >= TARGET_G2 the per-frame call of the comm step in the battle main loop
		//                   (0x1DCF84: jal 0x2F9A10) is replaced by "addiu v0,zero,1", i.e. the game believes the
		//                   inputs of all machines are always there and never waits for the network.
		//   cmd = 0       : revert both (at the clean point).
		//
		// Every hook after G1 also logs "NetBattle: BF=<frame> h=<hash of the unit structures> ..." so that the
		// logs of the emulators can be compared frame by frame afterwards. The first G1 decision logs a census of
		// the units and of the game context (human/CPU flags, slot numbers) for the design memo.
		namespace nb
		{
			constexpr u32 kGpBase = 0x4A8370;
			constexpr u32 kUnitPtrs = 0x5E8F30;
			constexpr u32 kUnitSize = 0x4A90;
			constexpr u32 kG2Addr = 0x1DCF84;
			constexpr u32 kG2Original = 0x0C0BE684; // jal 0x2F9A10
			constexpr u32 kG2Patched = 0x24020001;   // addiu v0,zero,1
			constexpr u32 kLogFrames = 2400;
			constexpr u32 kSlotsWritten = 6;

			// words of the control block (kept in the emulator, not in guest RAM: PINE cannot address the RAM above 32 MB)
			constexpr u32 CMD = 0, TARGET_G1 = 1, TARGET_G2 = 2, STATUS = 3, BF_G1 = 4, BF_G2 = 5, LOGGED = 6, HEARTBEAT = 7;
			// STATUS: 0 idle, 1 waiting for a battle, 2 G1 applied, 3 G1+G2 applied, 0x10 patch refused, 0x11 RAM too small

			u32 ctl[8] = {};
			bool dirty = false;
			u32 polls = 0;

			void SetCtl(u32 k, u32 v)
			{
				if (ctl[k] != v)
				{
					ctl[k] = v;
					dirty = true;
				}
			}

			// "nb_ctrl.txt" in the working directory: "<cmd> <targetG1> <targetG2>", polled every 15 frames.
			// "nb_status.txt" is rewritten whenever the status words change.
			void PollControlFile()
			{
				if (polls++ % 15)
					return;
				std::FILE* f = std::fopen("nb_ctrl.txt", "r");
				if (!f)
					return;
				unsigned c = 0, t1 = 0, t2 = 0;
				if (std::fscanf(f, "%u %u %u", &c, &t1, &t2) == 3)
				{
					SetCtl(CMD, c);
					SetCtl(TARGET_G1, t1);
					SetCtl(TARGET_G2, t2);
				}
				std::fclose(f);
			}

			void WriteStatusFile()
			{
				dirty = false;
				std::FILE* f = std::fopen("nb_status.txt", "w");
				if (!f)
					return;
				std::fprintf(f, "%u %u %u %u %u %u\n", ctl[STATUS], ctl[BF_G1], ctl[BF_G2], ctl[LOGGED], ctl[CMD], ctl[HEARTBEAT]);
				std::fclose(f);
			}

			bool g1 = false;
			bool g2 = false;
			u32 pend = 0; // 1 = apply G1, 2 = apply G2, 3 = revert
			u32 logged = 0;

			u32 R32(u32 a)
			{
				return *reinterpret_cast<const u32*>(eeMem->Main + a);
			}
			u8 R8(u32 a)
			{
				return eeMem->Main[a];
			}
			float RF(u32 a)
			{
				float f;
				std::memcpy(&f, eeMem->Main + a, sizeof(f));
				return f;
			}
			bool Plausible(u32 p)
			{
				return p >= 0x100000 && p < 0x2000000;
			}

			// Battle frame counter BF = *(*(gp-0x7D10)+0xC); false while the pointer chain is not valid.
			bool ReadBf(u32* bf)
			{
				const u32 p = R32(kGpBase - 0x7D10);
				if (!Plausible(p))
					return false;
				*bf = R32(p + 0xC);
				return true;
			}

			// Active unit structures (their attack table pointer at +0x2E8 is set).
			u32 Units(u32* ptrs)
			{
				u32 n = 0;
				for (u32 k = 0; k < 8; k++)
				{
					const u32 p = R32(kUnitPtrs + 4 * k);
					ptrs[k] = 0;
					if (Plausible(p) && R32(p + 0x2E8) != 0)
					{
						ptrs[k] = p;
						n++;
					}
				}
				return n;
			}

			u32 NbWord(u32 slot, u32 bf)
			{
				static constexpr u32 kWords[8] = {0x00, 0xBF, 0x9D, 0x57, 0x61, 0xA9, 0x8B, 0x45};
				return kWords[(bf / 6 + slot * 5) % 8];
			}

			u64 HashUnits(const u32* ptrs)
			{
				u64 h = 1469598103934665603ull;
				for (u32 k = 0; k < 8; k++)
				{
					if (!ptrs[k])
						continue;
					const u64* w = reinterpret_cast<const u64*>(eeMem->Main + ptrs[k]);
					for (u32 i = 0; i < kUnitSize / 8; i++)
						h = (h ^ w[i]) * 1099511628211ull;
				}
				return h;
			}

			// Raw copies of the unit structures ("nb_dump_bf<N>.bin": per active unit u32 index + kUnitSize bytes), so that the
			// emulators' copies can be compared word by word offline (which fields differ between machines).
			void DumpUnits(u32 bf, const u32* ptrs)
			{
				char name[64];
				std::snprintf(name, sizeof(name), "nb_dump_bf%u.bin", bf);
				std::FILE* f = std::fopen(name, "wb");
				if (!f)
					return;
				for (u32 k = 0; k < 8; k++)
				{
					if (!ptrs[k])
						continue;
					std::fwrite(&k, sizeof(k), 1, f);
					std::fwrite(eeMem->Main + ptrs[k], 1, kUnitSize, f);
				}
				std::fclose(f);
			}

			void LogCensus(u32 bf, const u32* ptrs)
			{
				const u32 ctx = R32(kGpBase - 0x7D20);
				Console.WriteLn("NetBattle: census ctx pointer *(gp-0x7D20)=0x%X", ctx);
				if (Plausible(ctx))
				{
					Console.WriteLn("NetBattle: census at BF=%u: ctx=0x%X (+0x1D=%u +0x1F=%u +0x27=%u +0xCA=%u); comm: my slot(0x6302BE)=%u, C=%u, D=%u",
						bf, ctx, R8(ctx + 0x1D), R8(ctx + 0x1F), R8(ctx + 0x27), R8(ctx + 0xCA), R8(0x6302BE), R8(0x6302BF), R8(0x6302C2));
				}
				for (u32 k = 0; k < 8; k++)
				{
					const u32 p = R32(kUnitPtrs + 4 * k);
					if (!Plausible(p))
						continue;
					Console.WriteLn("NetBattle:   unit[%u] @0x%X %s slot(+6)=%u +5=%u +9=%u +0xA=%u +0xD=%u id=0x%X atk=0x%X +0x4D4=0x%X hp=%.0f",
						k, p, ptrs[k] ? "active" : "inactive", R8(p + 6), R8(p + 5), R8(p + 9), R8(p + 0xA), R8(p + 0xD),
						R32(p + 8) & 0xFFFF, R32(p + 0x2E8), R8(p + 0x4D4), RF(p + 0x1F4));
				}
				for (u32 s = 0; s < 6; s++)
				{
					const u32 b = 0x5E8A70 + s * 0x40;
					Console.WriteLn("NetBattle:   pad array[%u]: cur=0x%X prev=0x%X pressed=0x%X", s, R32(b), R32(b + 4), R32(b + 8));
				}
			}

			bool ApplyG2(bool on)
			{
				u32* w = reinterpret_cast<u32*>(eeMem->Main + kG2Addr);
				const u32 expect = on ? kG2Original : kG2Patched;
				if (*w != expect)
				{
					Console.Error("NetBattle: guest word at 0x%06X is 0x%08X, expected 0x%08X: not patching", kG2Addr, *w, expect);
					return false;
				}
				*w = on ? kG2Patched : kG2Original;
				return true;
			}

			void RequestExit(u32 what)
			{
				pend = what;
				Cpu->ExitExecution();
			}

			void OnHook()
			{
				PollControlFile();
				if (dirty)
					WriteStatusFile();
				const u32 cmd = ctl[CMD];
				if (cmd == 0 && !g1 && !g2)
					return; // nothing armed
				if (RamSize() < sess::kMailbox + kSlotsWritten * sess::kMailboxStride)
				{
					SetCtl(STATUS, 0x11);
					return;
				}
				ctl[HEARTBEAT]++;

				if (cmd == 0)
				{
					RequestExit(3);
					return;
				}

				u32 ptrs[8];
				const u32 nu = Units(ptrs);
				u32 bf = 0;
				const bool battle = ReadBf(&bf) && nu >= 2;
				if (!battle)
				{
					if (!g1)
						SetCtl(STATUS, 1);
					return;
				}

				if (cmd & 1)
				{
					// The words for the frame about to run; harmless while the guest is not patched yet.
					for (u32 s = 0; s < kSlotsWritten; s++)
						sess::WriteMailbox(s, NbWord(s, bf));
					if (!g1 && bf >= ctl[TARGET_G1])
					{
						LogCensus(bf, ptrs);
						RequestExit(1);
						return;
					}
				}
				if ((cmd & 2) && g1 && !g2 && bf >= ctl[TARGET_G2])
				{
					RequestExit(2);
					return;
				}

				if (g1 && logged < kLogFrames)
				{
					u32 first = 0;
					for (u32 k = 0; k < 8 && !first; k++)
						first = ptrs[k];
					Console.WriteLn("NetBattle: BF=%u h=%016llx p0=%.2f,%.2f,%.2f hp0=%.0f g2=%u", bf,
						static_cast<unsigned long long>(HashUnits(ptrs)), first ? RF(first + 0x60) : 0.0f, first ? RF(first + 0x64) : 0.0f,
						first ? RF(first + 0x68) : 0.0f, first ? RF(first + 0x1F4) : 0.0f, g2 ? 1u : 0u);
					// every active unit: slot, position, HP (shows that each slot's mailbox word drives its own unit)
					for (u32 k = 0; k < 8; k++)
					{
						if (ptrs[k])
							Console.WriteLn("NetBattle: BF=%u u%u slot=%u pos=%.2f,%.2f,%.2f hp=%.0f", bf, k, R8(ptrs[k] + 6),
								RF(ptrs[k] + 0x60), RF(ptrs[k] + 0x64), RF(ptrs[k] + 0x68), RF(ptrs[k] + 0x1F4));
					}
					if (logged == 120 || logged == 600)
						DumpUnits(bf, ptrs);
					logged++;
					ctl[LOGGED] = logged;
				}
			}

			// Runs at the clean point. Returns true when a net battle action was handled.
			bool OnExit()
			{
				if (pend == 0)
					return false;
				const u32 what = pend;
				pend = 0;
				if (!VMManager::HasValidVM() || VMManager::GetState() != VMState::Running)
					return true;

				u32 bf = 0;
				ReadBf(&bf);
				if (what == 1)
				{
					if (sess::ApplyGuestPatch(true))
					{
						g1 = true;
						SetCtl(BF_G1, bf);
						SetCtl(STATUS, 2);
						Console.WriteLn("NetBattle: G1 applied (human units read the mailbox) at BF=%u", bf);
					}
					else
					{
						SetCtl(STATUS, 0x10);
					}
				}
				else if (what == 2)
				{
					if (ApplyG2(true))
					{
						g2 = true;
						SetCtl(BF_G2, bf);
						SetCtl(STATUS, 3);
						Console.WriteLn("NetBattle: G2 applied (the battle loop no longer calls the comm step) at BF=%u", bf);
					}
					else
					{
						SetCtl(STATUS, 0x10);
					}
				}
				else
				{
					if (g2)
						ApplyG2(false);
					if (g1)
						sess::ApplyGuestPatch(false);
					g1 = g2 = false;
					logged = 0;
					SetCtl(STATUS, 0);
					Console.WriteLn("NetBattle: reverted at BF=%u", bf);
				}
				return true;
			}
		} // namespace nb

		// ======================================================================================
		// Rollback netplay: input exchange between emulators + the rollback loop (rn)
		// ======================================================================================
		// Every emulator is one human slot of a networked battle. Once armed (file rb_net.txt, see Cfg) all emulators switch
		// at the same battle frame `start` to: guest inputs from the mailbox (nb/sess patch G1), the game's own comm step
		// stopped (G2). From then on this module runs the loop on every vsync hook, with F = the battle frame counter BF:
		//   - the local input for frame F+L (L = input delay, in frames) is read now and sent to all peers over UDP
		//     (every packet repeats the last kPacketFrames local inputs, so a lost datagram is covered by the next one);
		//   - the mailbox word of every slot for frame F is the true input if it has arrived, else "hold the last known";
		//   - when a true input arrives that differs from the word that was used, the state is restored from the newest
		//     snapshot at or before that frame and the frames are re-simulated with the corrected inputs;
		//   - if a peer is more than kMaxDepth frames behind, the hook waits for it (the only place that blocks).
		// With `synthetic` set, the local input is a pure function of (slot, frame): the whole run is then reproducible and
		// the per-frame state hashes ("RbNet: BF=.. s=..") of all emulators must be identical and independent of timing.
		namespace rn
		{
			constexpr u32 kSlots = 4;
			constexpr u32 kMaxFrames = 60000; // frames after the start frame that can be handled (1000 s)
			constexpr u32 kK = 4;             // snapshot interval
			constexpr u32 kRing = 5;          // snapshots kept
			constexpr u32 kMaxDepth = 10;     // wait when more than this many frames of a peer are predicted
			constexpr u32 kPacketFrames = 12; // local inputs repeated in every packet
			constexpr u32 kMagic = 0x31424E52u; // "RNB1"
			constexpr u32 kPollEvery = 30;
			constexpr u32 kNoFrame = 0xFFFFFFFFu;
			constexpr u32 kMaxArtDelay = 8;
			constexpr const char* kPeerIp = "127.0.0.1";

			struct Packet
			{
				u32 magic;
				u32 slot;
				u32 first; // battle frame of words[0]
				u32 count;
				u32 words[kPacketFrames];
			};

			enum class St
			{
				Idle,
				Armed,
				Run,
			};

			enum class Act
			{
				None,
				Begin,
				Save,
				Rollback,
				Cleanup, // after the battle: put the two guest patches back
			};

			struct Cfg
			{
				u32 enable = 0;
				u32 slot = 0;
				u32 port = 29100;  // peer of slot s listens on port + s
				u32 delay = 2;     // L
				int pattern = -1;  // artificial lateness of the peers' inputs (sess::kPatterns index), -1 = none
				u32 synthetic = 1;
				u32 start = 0;     // battle frame at which the rollback mode starts (same on all emulators)
				u32 debug = 0;     // bit 0: take no snapshots, bit 1: never restore (mispredictions are ignored), bit 2: EE-only snapshots (IOP is not rolled back),
				                   // bit 3: keep the game's comm module data (EE RAM) live: a restore does not rewind it,
				                   // bit 4: comm-module trace, bit 5: also stop the other call sites of the comm master step (G3);
				                   // bits 6/7/8: only the G3 site 0x156918 / 0x1DD558 / 0x1DDE98,
				                   // bit 9: log the IOP->EE SIF0 deliveries and replay them after a restore, bit 10: keep the SIF DMA state live
				u32 net_delay = 0;  // simulated one-way network delay of every outgoing packet, ms (can be changed while running)
				u32 net_jitter = 0; // + a random 0..net_jitter ms
				u32 net_drop = 0;   // simulated packet loss, percent
			};

			St st = St::Idle;
			Act act = Act::None;
			Cfg cfg;
			u32 polls = 0;
			u32 consumed_start = kNoFrame;
			u32 base = 0;
			u32 me = 0;
			u32 L = 2;
			u32 expect = 0;
			u32 target = 0;
			bool resim = false;
			u32 last_logged = kNoFrame;
			u32 session = 0;      // counts the rollback sessions of this run (a new one starts for every battle)
			u32 dump_bf = 0;      // when this battle frame is final, write the first 32 MB of EE RAM to rb_ram_dump_bf<N>.bin (file rb_dump_at.txt)
			u32 last_state = 0x100; // last logged value of this slot's game state byte
			u32 save_slot = 0, save_frame = 0, roll_slot = 0;
			sess::Slot ring[kRing];
			std::vector<u32> truth[kSlots];
			std::vector<u32> recv_word[kSlots];
			std::vector<u32> used[kSlots];
			std::vector<u8> have[kSlots];
			std::vector<u8> recvd[kSlots];
			s32 kn[kSlots] = {};
			// statistics
			u32 rollbacks = 0, resim_frames = 0, max_depth = 0, saves = 0, pred_ok = 0, pred_bad = 0, stalls = 0, anomalies = 0;
			u32 pkts_in = 0, pkts_bad = 0, live_frames = 0, pkts_out = 0, pkts_dropped = 0;

			// Outgoing packets wait here until their simulated delivery time.
			struct Outgoing
			{
				std::chrono::steady_clock::time_point due;
				u16 port;
				u32 len;
				Packet p;
			};
			std::deque<Outgoing> outq;
			u32 rng = 12345u;

			u32 Rand()
			{
				rng ^= rng << 13;
				rng ^= rng >> 17;
				rng ^= rng << 5;
				return rng;
			}

			// Sends what is due. Cheap when the queue is empty.
			void FlushSends()
			{
				if (outq.empty())
					return;
				const auto now = std::chrono::steady_clock::now();
				for (auto it = outq.begin(); it != outq.end();)
				{
					if (it->due <= now)
					{
						RollbackNet::SendTo(kPeerIp, it->port, &it->p, static_cast<int>(it->len));
						it = outq.erase(it);
					}
					else
					{
						++it;
					}
				}
			}
			double ms_restore = 0.0, ms_save = 0.0, ms_stall = 0.0;

			// ---- Communication-module trace (debug bit 4, or the file rb_trace.txt in a battle WITHOUT rollback): which words of
			// the comm module's control data change, when, and what the restores do to them. When the error code (0x630358) appears
			// (or the file rb_trace_dump_now.txt exists) the recent history is written to rb_trace_dump.txt.
			struct TraceRange
			{
				u32 lo, hi;
			};
			constexpr TraceRange kTraceRanges[] = {{0x630000, 0x630400}, {0x4A08E0, 0x4A08F0}, {0x4A0A30, 0x4A0BE0}, {0x4A6F00, 0x4A7000}};
			constexpr u32 kNoiseLo = 0x628000, kNoiseHi = 0x636000; // the comm module's buffers: only counted
			constexpr u32 kTraceMax = 8000;
			struct TraceRec
			{
				u32 tick, bf, a, b, c;
				char kind;
			};
			std::deque<TraceRec> trace;
			std::vector<u32> trace_prev, noise_prev;
			bool trace_active = false;
			u32 trace_tick = 0, trace_bf = 0xFFFFFFFFu, trace_err_tick = 0, trace_noise = 0, trace_last358 = 0xFFFFFFFFu, trace_slot = 0, trace_last_state = 0x100;

			// ---- SIF0 replay (debug bit 9 = 512). The IOP is not rolled back, so what it delivered to the EE between the snapshot
			// and the rollback (SIF0 DMA data into EE RAM + the DMAC interrupt that makes the EE's SIF handler look at it) is lost
			// with the restore and would never come again: RPC end notifications, loaded resources, ... Every delivery is logged with
			// the battle frame and a sequence number; after a restore the deliveries newer than the snapshot are delivered again
			// at the same frames of the re-simulation (the DMA data is copied back, the DMAC interrupt is raised again).
			// debug bit 10 (1024): in addition the SIF DMA state itself (sif0/sif1 structures, DMAC channels 5/6, SBUS registers)
			// is left as it is by a restore, like the IOP side.
			struct SifEvt
			{
				u64 seq;
				u32 frame;
				u32 addr;
				char kind; // 'W' data, 'I' interrupt
				bool replay = true; // false = logged only (statistics), never replayed (outside the include / inside an exclude range)
				std::vector<u8> data;
			};
			std::deque<SifEvt> sif_log;
			std::vector<SifEvt> sif_pending;
			size_t sif_pending_pos = 0;
			u64 sif_seq = 0;
			u64 ring_seq[5] = {};
			u32 cur_frame = 0;
			bool sif_replaying = false;
			// EE write watch (see Rollback.h)
			std::vector<std::pair<u32, u32>> watch_ranges;
			std::FILE* watch_file = nullptr;
			u32 watch_lines = 0;
			// rb_sif.txt (read when the session begins): lines "i <lo> <hi>" / "e <lo> <hi>" (hex EE addresses). With include ranges only the
			// deliveries into them are replayed, otherwise all except those into an exclude range. The interrupt of a transfer follows the
			// decision of the last data chunk before it.
			std::vector<std::pair<u32, u32>> sif_inc, sif_exc;
			bool sif_last_w_replay = true;
			std::map<u32, std::pair<u32, u64>> sif_hist; // 64 KB region -> (chunks, bytes) of the logged deliveries
			u32 sif_logged_w = 0, sif_logged_i = 0, sif_replayed_w = 0, sif_replayed_i = 0;
			bool g_sif0_log_flag = false;

			// G3 (debug bit 5): the other three call sites of the comm master step 0x2F9A10 (0x1DCF84 is G2). The comm module's
			// state words kept counting in a battle with only G2, so something else still runs it.
			constexpr u32 kG3Sites[] = {0x156918, 0x1DD558, 0x1DDE98};
			u32 g3_mask = 0; // bit i = kG3Sites[i] is patched at the moment

			// debug bit 5 (32) = all three sites; otherwise bits 6, 7, 8 (64, 128, 256) pick the sites one by one.
			u32 G3MaskFromDebug(u32 debug)
			{
				return (debug & 32) ? 7u : ((debug >> 6) & 7u);
			}

			bool ApplyG3(bool on, u32 mask)
			{
				for (u32 i = 0; i < 3; i++)
				{
					if (!(mask & (1u << i)))
						continue;
					const u32 now = nb::R32(kG3Sites[i]);
					const u32 expect = on ? nb::kG2Original : nb::kG2Patched;
					if (now != expect)
					{
						Console.Error("RbNet: G3 site 0x%06X is 0x%08X, expected 0x%08X", kG3Sites[i], now, expect);
						return false;
					}
				}
				for (u32 i = 0; i < 3; i++)
				{
					if (mask & (1u << i))
						*reinterpret_cast<u32*>(eeMem->Main + kG3Sites[i]) = on ? nb::kG2Patched : nb::kG2Original;
				}
				g3_mask = on ? mask : 0;
				return true;
			}

			bool Active()
			{
				return st != St::Idle || act != Act::None || trace_active;
			}

			void RequestAct(Act a)
			{
				act = a;
				Cpu->ExitExecution();
			}

			bool SifRangeReplay(u32 a)
			{
				if (!sif_inc.empty())
				{
					for (const auto& r : sif_inc)
					{
						if (a >= r.first && a < r.second)
							return true;
					}
					return false;
				}
				for (const auto& r : sif_exc)
				{
					if (a >= r.first && a < r.second)
						return false;
				}
				return true;
			}

			void WatchStop()
			{
				if (Rollback::g_watch_jit)
				{
					Rollback::g_watch_jit = false;
					Cpu->Clear(0x160000, 0x40000 / 4); // compile the HUD code normally again
				}
				Rollback::g_watch_on = false;
				if (watch_file)
				{
					std::fclose(watch_file);
					watch_file = nullptr;
				}
				watch_ranges.clear();
			}

			void WatchReadConfig()
			{
				WatchStop();
				std::FILE* f = std::fopen("rb_watch.txt", "r");
				if (!f)
					return;
				unsigned lo = 0, hi = 0;
				while (std::fscanf(f, " %x %x", &lo, &hi) == 2)
					watch_ranges.emplace_back(lo, hi);
				std::fclose(f);
				if (watch_ranges.empty())
					return;
				watch_file = std::fopen("rb_watch_log.txt", "a");
				watch_lines = 0;
				Rollback::g_watch_on = watch_file != nullptr;
				if (Rollback::g_watch_on)
				{
					Rollback::g_watch_jit = true;
					Cpu->Clear(0x160000, 0x40000 / 4); // recompile the HUD code with the store hooks
				}
				Console.WriteLn("RbNet: EE write watch on: %u ranges", static_cast<u32>(watch_ranges.size()));
			}

			void WatchWrite(u32 addr, u32 size, u64 val, u32 pc)
			{
				addr &= 0x1FFFFFFFu;
				if (!watch_file || addr >= 0x2000000u)
					return;
				bool hit = false;
				for (const auto& r : watch_ranges)
				{
					if (addr < r.second && addr + size > r.first)
					{
						hit = true;
						break;
					}
				}
				if (!hit)
					return;
				u64 old = 0;
				std::memcpy(&old, eeMem->Main + addr, size > 8 ? 8 : size);
				u64 nv = val;
				if (size < 8)
					nv &= ((1ull << (size * 8)) - 1);
				if (size <= 8 && old == nv)
					return;
				if (watch_lines++ > 800000u)
					return;
				u32 bf = 0;
				u32 p = 0;
				std::memcpy(&p, eeMem->Main + (0x4A8370u - 0x7D10u), 4);
				if (p > 0x100000u && p < 0x2000000u)
					std::memcpy(&bf, eeMem->Main + p + 0xC, 4);
				const double t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
				std::fprintf(watch_file, "%.3f BF=%u PC=%08X A=%06X S=%u V=%llX O=%llX\n", t, bf, pc, addr, size,
					static_cast<unsigned long long>(nv), static_cast<unsigned long long>(old));
				if ((watch_lines & 63u) == 0)
					std::fflush(watch_file);
			}

			void SifReadConfig()
			{
				sif_inc.clear();
				sif_exc.clear();
				std::FILE* f = std::fopen("rb_sif.txt", "r");
				if (!f)
					return;
				char c = 0;
				unsigned lo = 0, hi = 0;
				while (std::fscanf(f, " %c %x %x", &c, &lo, &hi) == 3)
				{
					if (c == 'i' || c == 'I')
						sif_inc.emplace_back(lo, hi);
					else if (c == 'e' || c == 'E')
						sif_exc.emplace_back(lo, hi);
				}
				std::fclose(f);
				Console.WriteLn("RbNet: rb_sif.txt: %u include and %u exclude ranges", static_cast<u32>(sif_inc.size()), static_cast<u32>(sif_exc.size()));
			}

			void SifLogWrite(u32 ee_addr, const void* data, u32 bytes)
			{
				if (st != St::Run || sif_replaying || (ee_addr & 0x80000000u) || bytes == 0 || bytes > 4u * 1024u * 1024u)
					return;
				const u32 a = ee_addr & 0x1FFFFFFFu;
				if (a + bytes > 32u * 1024u * 1024u)
					return;
				SifEvt e;
				e.seq = ++sif_seq;
				e.frame = cur_frame;
				e.addr = a;
				e.kind = 'W';
				e.replay = SifRangeReplay(a);
				sif_last_w_replay = e.replay;
				auto& hh = sif_hist[a >> 16];
				hh.first++;
				hh.second += bytes;
				e.data.assign(static_cast<const u8*>(data), static_cast<const u8*>(data) + bytes);
				sif_log.push_back(std::move(e));
				sif_logged_w++;
			}

			void SifLogIrq()
			{
				if (st != St::Run || sif_replaying)
					return;
				SifEvt e;
				e.seq = ++sif_seq;
				e.frame = cur_frame;
				e.addr = 0;
				e.kind = 'I';
				e.replay = sif_last_w_replay;
				sif_log.push_back(std::move(e));
				sif_logged_i++;
			}

			void SifApply(const SifEvt& e)
			{
				sif_replaying = true;
				if (e.kind == 'W')
				{
					std::memcpy(eeMem->Main + e.addr, e.data.data(), e.data.size());
					sif_replayed_w++;
				}
				else
				{
					EEsif0Interrupt(); // what completing the transfer does: raise the DMAC interrupt, stop the channel
					sif_replayed_i++;
				}
				sif_replaying = false;
			}

			// Delivers the pending replays that belong to frames up to F (all of them for kNoFrame).
			void SifApplyUpTo(u32 F)
			{
				while (sif_pending_pos < sif_pending.size() && (F == 0xFFFFFFFFu || sif_pending[sif_pending_pos].frame <= F))
					SifApply(sif_pending[sif_pending_pos++]);
				if (sif_pending_pos >= sif_pending.size())
				{
					sif_pending.clear();
					sif_pending_pos = 0;
				}
			}

			void SifPrune(u32 ring_slots_valid_mask)
			{
				// events older than the oldest valid snapshot are never needed again
				u64 oldest = ~0ull;
				for (u32 i = 0; i < 5; i++)
				{
					if (ring_slots_valid_mask & (1u << i))
						oldest = std::min(oldest, ring_seq[i]);
				}
				if (oldest == ~0ull)
					return;
				while (!sif_log.empty() && sif_log.front().seq <= oldest)
					sif_log.pop_front();
			}

			void TracePush(char kind, u32 a, u32 b, u32 c)
			{
				trace.push_back({trace_tick, trace_bf, a, b, c, kind});
				while (trace.size() > kTraceMax)
					trace.pop_front();
			}

			void TraceWords(std::vector<u32>& out)
			{
				out.clear();
				for (const TraceRange& r : kTraceRanges)
				{
					for (u32 a = r.lo; a < r.hi; a += 4)
						out.push_back(nb::R32(a));
				}
			}

			void TraceArm(u32 slot)
			{
				if (trace_active)
					return;
				trace_active = true;
				trace.clear();
				trace_tick = 0;
				trace_err_tick = 0;
				trace_noise = 0;
				trace_slot = slot;
				trace_last_state = 0x100;
				TraceWords(trace_prev);
				noise_prev.clear();
				for (u32 a = kNoiseLo; a < kNoiseHi; a += 4)
					noise_prev.push_back(nb::R32(a));
				trace_last358 = nb::R32(0x630358);
				TracePush('A', slot, 0, 0);
				Console.WriteLn("RbTrace: armed (slot %u): %u control words + %u buffer words watched", slot, static_cast<u32>(trace_prev.size()),
					static_cast<u32>(noise_prev.size()));
			}

			void TraceDump(const char* reason)
			{
				std::FILE* f = std::fopen("rb_trace_dump.txt", "w");
				if (f)
				{
					std::fprintf(f, "# comm trace dump (%s): tick bf kind a b c\n", reason);
					for (const TraceRec& r : trace)
						std::fprintf(f, "%u %u %c %08X %08X %08X\n", r.tick, r.bf, r.kind, r.a, r.b, r.c);
					std::fclose(f);
				}
				Console.WriteLn("RbTrace: dumped %u records to rb_trace_dump.txt (%s)", static_cast<u32>(trace.size()), reason);
				trace_active = false;
			}

			// Once per hook (also during re-simulation: those changes are logged in lower case).
			void TraceStep(bool in_resim)
			{
				trace_tick++;
				u32 F = 0;
				trace_bf = nb::ReadBf(&F) ? F : 0xFFFFFFFFu;
				size_t idx = 0;
				for (const TraceRange& r : kTraceRanges)
				{
					for (u32 a = r.lo; a < r.hi; a += 4, idx++)
					{
						const u32 v = nb::R32(a);
						if (v != trace_prev[idx])
						{
							TracePush(in_resim ? 'c' : 'C', a, trace_prev[idx], v);
							trace_prev[idx] = v;
						}
					}
				}
				u32 changed = 0;
				idx = 0;
				for (u32 a = kNoiseLo; a < kNoiseHi; a += 4, idx++)
				{
					const u32 v = nb::R32(a);
					if (v != noise_prev[idx])
					{
						noise_prev[idx] = v;
						changed++;
					}
				}
				trace_noise += changed;
				if (trace_tick % 30 == 0)
				{
					TracePush('N', trace_noise, 0, 0);
					trace_noise = 0;
				}
				const u32 gs = nb::R8(0x5ED060 + 0x50 * trace_slot);
				if (gs != trace_last_state)
				{
					TracePush('G', gs, trace_last_state & 0xFF, 0);
					trace_last_state = gs;
				}
				const u32 e = nb::R32(0x630358);
				if (e != trace_last358)
				{
					TracePush('E', 0x630358, trace_last358, e);
					if (trace_last358 == 0xFFFFFFFFu && e != 0xFFFFFFFFu && trace_err_tick == 0)
						trace_err_tick = trace_tick;
					trace_last358 = e;
				}
				if (trace_err_tick && trace_tick >= trace_err_tick + 120)
				{
					TraceDump("error code appeared");
					return;
				}
				if (trace_tick % 60 == 0)
				{
					std::FILE* f = std::fopen("rb_trace_dump_now.txt", "r");
					if (f)
					{
						std::fclose(f);
						std::remove("rb_trace_dump_now.txt");
						TraceDump("requested");
					}
				}
			}

			void ReadConfig()
			{
				std::FILE* f = std::fopen("rb_net.txt", "r");
				if (!f)
					return;
				unsigned enable = 0, slot = 0, port = 0, delay = 0, synthetic = 0, start = 0, debug = 0, nd = 0, nj = 0, np = 0;
				int pattern = -1;
				const int n = std::fscanf(f, "%u %u %u %u %d %u %u %u %u %u %u", &enable, &slot, &port, &delay, &pattern, &synthetic, &start, &debug, &nd, &nj, &np);
				std::fclose(f);
				if (n < 7)
					return;
				cfg.enable = enable;
				cfg.slot = slot;
				cfg.port = port;
				cfg.delay = delay;
				cfg.pattern = pattern;
				cfg.synthetic = synthetic;
				cfg.start = start;
				cfg.debug = (n >= 8) ? debug : 0;
				cfg.net_delay = (n >= 9) ? nd : 0;
				cfg.net_jitter = (n >= 10) ? nj : 0;
				cfg.net_drop = (n >= 11) ? np : 0;
			}

			// Idle: look for the config file every kPollEvery vsyncs.
			void Poll()
			{
				if (st != St::Idle || (polls++ % kPollEvery))
					return;
				if (!trace_active)
				{
					std::FILE* tf = std::fopen("rb_trace.txt", "r");
					if (tf)
					{
						unsigned tslot = 0;
						if (std::fscanf(tf, "%u", &tslot) != 1)
							tslot = 0;
						std::fclose(tf);
						std::remove("rb_trace.txt");
						TraceArm(tslot);
						return;
					}
				}
				ReadConfig();
				if (cfg.enable == 0 || cfg.start == consumed_start || cfg.slot >= kSlots || cfg.port == 0 || cfg.port > 65000 - kSlots)
					return;
				if (!RollbackNet::Open(static_cast<u16>(cfg.port + cfg.slot)))
					return;
				consumed_start = cfg.start;
				me = cfg.slot;
				L = std::max<u32>(1, std::min<u32>(cfg.delay, 8));
				st = St::Armed;
				Console.WriteLn("RbNet: armed: slot %u, UDP port %u, input delay %u, artificial pattern %d, synthetic inputs %u, start at BF %u",
					me, cfg.port + me, L, cfg.pattern, cfg.synthetic, cfg.start);
			}

			void LogStats(const char* tag)
			{
				Console.WriteLn("RbNet: %s: live frames %u, rollbacks %u (frames re-simulated %u, deepest %u), predictions right %u / wrong %u, "
								"stalls %u (%.0f ms), snapshots %u, restore %.1f ms total, save %.1f ms total, packets in %u (bad %u) out %u (dropped %u), BF anomalies %u",
					tag, live_frames, rollbacks, resim_frames, max_depth, pred_ok, pred_bad, stalls, ms_stall, saves, ms_restore, ms_save, pkts_in, pkts_bad, pkts_out,
					pkts_dropped, anomalies);
				if (sif_logged_w || sif_logged_i)
				{
					Console.WriteLn("RbNet: SIF0 deliveries logged: %u data + %u interrupts; replayed %u data + %u interrupts", sif_logged_w, sif_logged_i,
						sif_replayed_w, sif_replayed_i);
					// the 12 busiest 64 KB regions of the logged deliveries (chunks, KB)
					std::vector<std::pair<u32, u32>> top; // chunks, region
					for (const auto& kv : sif_hist)
						top.emplace_back(kv.second.first, kv.first);
					std::sort(top.begin(), top.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
					for (size_t i = 0; i < top.size() && i < 12; i++)
						Console.WriteLn("RbNet:   SIF0 into 0x%06X-0x%06X: %u chunks, %u KB", top[i].second << 16, (top[i].second << 16) + 0xFFFF, top[i].first,
							static_cast<u32>(sif_hist[top[i].second].second >> 10));
				}
			}

			void Stop(const char* why, bool revert = false)
			{
				if (st == St::Idle)
					return;
				if (trace_active)
					TracePush('S', revert ? 1 : 0, 0, 0);
				if (resim && VMManager::HasValidVM())
					VMManager::SetLimiterMode(s_prev_limiter);
				resim = false;
				Rollback::g_sif1_drop = false;
				WatchStop();
				LogStats(why);
				RollbackNet::Close();
				outq.clear();
				g_FastSkipIop = false;
				g_FastKeepSif = false;
				Rollback::g_sif0_log = false;
				sif_log.clear();
				sif_pending.clear();
				sif_pending_pos = 0;
				st = St::Idle;
				act = Act::None;
				for (sess::Slot& s : ring)
				{
					s.snap = FastSnapshot();
					s.valid = false;
				}
				if (revert && VMManager::HasValidVM())
				{
					// The battle is over: the game's own comm is needed again (results, rejoin, the next battle).
					Console.WriteLn("RbNet: stopped (%s). Reverting the mailbox/comm-stop guest patches at the next clean point.", why);
					RequestAct(Act::Cleanup);
				}
				else
				{
					// In the middle of a battle the game cannot go back to its own comm: the patches stay.
					Console.WriteLn("RbNet: stopped (%s). The mailbox/comm-stop guest patches are still applied.", why);
				}
			}

			u32 ArtDelay(u32 slot, u32 f)
			{
				if (cfg.pattern < 0 || cfg.pattern >= static_cast<int>(std::size(sess::kPatterns)))
					return 0;
				return std::min<u32>(sess::Delay(sess::kPatterns[cfg.pattern], f + slot * 7), kMaxArtDelay);
			}

			u32 Predict(u32 slot, u32 i)
			{
				for (u32 g = i; g > 0; g--)
				{
					if (have[slot][g - 1])
						return truth[slot][g - 1];
				}
				return 0;
			}

			u32 InputFor(u32 slot, u32 i)
			{
				if (have[slot][i])
					return truth[slot][i];
				return (slot == me) ? 0 : Predict(slot, i);
			}

			// Writes the mailbox words of frame F (all slots) and remembers them.
			void WriteAll(u32 F)
			{
				const u32 i = F - base;
				for (u32 s = 0; s < kSlots; s++)
				{
					const u32 w = InputFor(s, i);
					used[s][i] = w;
					sess::WriteMailbox(s, w);
				}
				sess::WriteMailbox(4, 0);
				sess::WriteMailbox(5, 0);
			}

			void PollNet()
			{
				u8 buf[256];
				for (u32 guard = 0; guard < 256; guard++)
				{
					const int n = RollbackNet::Recv(buf, sizeof(buf));
					if (n <= 0)
						return;
					Packet p;
					if (n < 16)
					{
						pkts_bad++;
						continue;
					}
					std::memcpy(&p, buf, std::min<size_t>(static_cast<size_t>(n), sizeof(p)));
					if (p.magic != kMagic || p.slot >= kSlots || p.slot == me || p.count == 0 || p.count > kPacketFrames ||
						static_cast<size_t>(n) < 16 + 4 * static_cast<size_t>(p.count))
					{
						pkts_bad++;
						continue;
					}
					pkts_in++;
					for (u32 k = 0; k < p.count; k++)
					{
						const u32 f = p.first + k;
						if (f < base + L || f - base >= kMaxFrames)
							continue;
						const u32 i = f - base;
						if (!recvd[p.slot][i])
						{
							recvd[p.slot][i] = 1;
							recv_word[p.slot][i] = p.words[k];
						}
					}
				}
			}

			// Makes the received inputs that are visible at frame F true. Returns the earliest frame (already run) whose used
			// word was wrong, or kNoFrame. `force` ignores the artificial lateness (used while waiting).
			u32 Deliver(u32 F, bool force)
			{
				u32 earliest = kNoFrame;
				const u32 iF = F - base;
				for (u32 s = 0; s < kSlots; s++)
				{
					if (s == me)
						continue;
					const u32 hi = std::min<u32>(iF + L + kPacketFrames, kMaxFrames - 1);
					for (u32 i = static_cast<u32>(kn[s] + 1); i <= hi; i++)
					{
						if (have[s][i] || !recvd[s][i])
							continue;
						const u32 f = base + i;
						if (!force && F + L < f + ArtDelay(s, f))
							continue;
						have[s][i] = 1;
						truth[s][i] = recv_word[s][i];
						if (f < F)
						{
							if (used[s][i] != truth[s][i])
							{
								pred_bad++;
								earliest = std::min(earliest, f);
							}
							else
							{
								pred_ok++;
							}
						}
					}
					while (static_cast<u32>(kn[s] + 1) < kMaxFrames && have[s][kn[s] + 1])
						kn[s]++;
				}
				return earliest;
			}

			u32 Depth(u32 F)
			{
				const s32 iF = static_cast<s32>(F - base);
				s32 d = 0;
				for (u32 s = 0; s < kSlots; s++)
				{
					if (s != me)
						d = std::max(d, iF - kn[s]);
				}
				return static_cast<u32>(d);
			}

			// Reads the local input for frame F+L (once) and sends the last kPacketFrames local inputs to all peers.
			void LocalStep(u32 F)
			{
				const u32 iL = F - base + L;
				if (iL >= kMaxFrames)
					return;
				if (!have[me][iL])
				{
					truth[me][iL] = cfg.synthetic ? nb::NbWord(me, F + L) : sess::ReadDevice();
					have[me][iL] = 1;
				}
				kn[me] = static_cast<s32>(iL);
				Packet p{};
				p.magic = kMagic;
				p.slot = me;
				s32 first = static_cast<s32>(iL) - static_cast<s32>(kPacketFrames) + 1;
				if (first < static_cast<s32>(L))
					first = static_cast<s32>(L);
				p.count = static_cast<u32>(static_cast<s32>(iL) - first + 1);
				p.first = base + static_cast<u32>(first);
				for (u32 k = 0; k < p.count; k++)
					p.words[k] = truth[me][static_cast<u32>(first) + k];
				const bool simulated = cfg.net_delay != 0 || cfg.net_jitter != 0 || cfg.net_drop != 0;
				for (u32 s = 0; s < kSlots; s++)
				{
					if (s == me)
						continue;
					pkts_out++;
					if (cfg.net_drop && (Rand() % 100) < cfg.net_drop)
					{
						pkts_dropped++;
						continue;
					}
					if (!simulated)
					{
						RollbackNet::SendTo(kPeerIp, static_cast<u16>(cfg.port + s), &p, static_cast<int>(16 + 4 * p.count));
						continue;
					}
					Outgoing o;
					o.due = std::chrono::steady_clock::now() + std::chrono::microseconds(1000ull * (cfg.net_delay + (cfg.net_jitter ? Rand() % (cfg.net_jitter + 1) : 0)));
					o.port = static_cast<u16>(cfg.port + s);
					o.len = 16 + 4 * p.count;
					o.p = p;
					outq.push_back(o);
				}
				FlushSends();
			}

			// The state at hook F is final when every used word of the frames before F was a true one.
			void LogVerified(u32 F, const u32* ptrs)
			{
				if (last_logged != kNoFrame && F <= last_logged)
					return;
				const s32 iPrev = static_cast<s32>(F - base) - 1;
				for (u32 s = 0; s < kSlots; s++)
				{
					if (s != me && kn[s] < iPrev)
						return;
				}
				u64 h = 1469598103934665603ull;
				u32 first = 0;
				for (u32 k = 0; k < 8; k++)
				{
					if (!ptrs[k])
						continue;
					if (!first)
						first = ptrs[k];
					static constexpr u32 kFields[4] = {0x60, 0x64, 0x68, 0x1F4};
					for (const u32 off : kFields)
						h = (h ^ nb::R32(ptrs[k] + off)) * 1099511628211ull;
				}
				Console.WriteLn("RbNet: BF=%u s=%016llx u0=%.2f,%.2f,%.2f n=%u", F, static_cast<unsigned long long>(h),
					first ? nb::RF(first + 0x60) : 0.0f, first ? nb::RF(first + 0x64) : 0.0f, first ? nb::RF(first + 0x68) : 0.0f, session);
				last_logged = F;
				if (dump_bf && F == dump_bf)
				{
					char name[64];
					std::snprintf(name, sizeof(name), "rb_ram_dump_bf%u.bin", F);
					std::FILE* df = std::fopen(name, "wb");
					if (df)
					{
						std::fwrite(eeMem->Main, 1, 32u * 1024u * 1024u, df);
						std::fclose(df);
					}
					Console.WriteLn("RbNet: EE RAM (32 MB) dumped at the final state of BF %u to %s", F, name);
					dump_bf = 0;
				}
			}

			void MaybeSave(u32 F)
			{
				if (!(cfg.debug & 1) && F > base && (F - base) % kK == 0)
				{
					save_slot = ((F - base) / kK) % kRing;
					save_frame = F;
					RequestAct(Act::Save);
				}
			}

			void RequestRollback(u32 F, u32 mismatch_frame)
			{
				int best = -1;
				for (u32 i = 0; i < kRing; i++)
				{
					if (ring[i].valid && ring[i].frame <= mismatch_frame && (best < 0 || ring[i].frame > ring[best].frame))
						best = static_cast<int>(i);
				}
				if (best < 0)
				{
					Console.Error("RbNet: no snapshot at or before frame %u (now %u)", mismatch_frame, F);
					Stop("no snapshot");
					return;
				}
				for (u32 i = 0; i < kRing; i++)
				{
					if (ring[i].frame > ring[best].frame)
						ring[i].valid = false;
				}
				roll_slot = static_cast<u32>(best);
				target = F;
				rollbacks++;
				const u32 depth = F - ring[best].frame;
				resim_frames += depth;
				max_depth = std::max(max_depth, depth);
				RequestAct(Act::Rollback);
			}

			void RunHook()
			{
				u32 F = 0;
				u32 ptrs[8];
				if (!nb::ReadBf(&F) || nb::Units(ptrs) < 2)
				{
					Stop("battle ended", true);
					return;
				}
				if (F != expect)
				{
					if (anomalies < 8)
						Console.Warning("RbNet: BF %u at this hook, expected %u", F, expect);
					anomalies++;
				}
				expect = F + 1;
				cur_frame = F;

				if (resim)
				{
					if (F < target)
					{
						SifApplyUpTo(F);
						FlushSends();
						WriteAll(F);
						LogVerified(F, ptrs);
						MaybeSave(F);
						return;
					}
					resim = false; // back at the frame where the rollback was requested: this is a live hook again
					Rollback::g_sif1_drop = false;
					SifApplyUpTo(0xFFFFFFFFu);
					VMManager::SetLimiterMode(s_prev_limiter);
				}

				if (F + L + kPacketFrames + 2 >= base + kMaxFrames)
				{
					Stop("buffer full");
					return;
				}

				FlushSends();
				PollNet();
				u32 m = Deliver(F, false);
				if (m == kNoFrame && Depth(F) > kMaxDepth)
				{
					Common::Timer t;
					stalls++;
					while (m == kNoFrame && Depth(F) > kMaxDepth)
					{
						if (t.GetTimeMilliseconds() > 3000.0)
						{
							Stop("a peer was silent for 3 s");
							return;
						}
						std::this_thread::sleep_for(std::chrono::milliseconds(1));
						FlushSends();
						PollNet();
						m = Deliver(F, true);
					}
					ms_stall += t.GetTimeMilliseconds();
				}
				if (m != kNoFrame && (cfg.debug & 2))
					m = kNoFrame; // debug: ignore the misprediction
				if (m != kNoFrame)
				{
					RequestRollback(F, m);
					return;
				}

				LogVerified(F, ptrs);
				{
					// the guest's game-state byte of this slot (0x5ED060 + 0x50 * slot): log its changes
					const u32 gs = nb::R8(0x5ED060 + 0x50 * me);
					if (gs != last_state)
					{
						Console.WriteLn("RbNet: game state byte of slot %u: 0x%02X -> 0x%02X at BF %u", me, last_state & 0xFF, gs, F);
						last_state = gs;
					}
				}
				LocalStep(F);
				WriteAll(F);
				live_frames++;
				if (live_frames % 600 == 0)
					LogStats("progress");
				if (live_frames % 30 == 0)
				{
					// rb_net.txt is re-read while running: "0 ..." stops the session (the final statistics are logged); the
					// artificial-lateness pattern and the simulated network (delay, jitter, loss) can be changed live, because
					// they only change WHEN inputs become visible, never what the true inputs are.
					if (std::FILE* df = std::fopen("rb_dump_at.txt", "r"))
					{
						unsigned x = 0;
						if (std::fscanf(df, "%u", &x) == 1)
							dump_bf = x;
						std::fclose(df);
						std::remove("rb_dump_at.txt");
						Console.WriteLn("RbNet: EE RAM dump requested at BF %u", dump_bf);
					}
					const int old_pattern = cfg.pattern;
					const u32 od = cfg.net_delay, oj = cfg.net_jitter, op = cfg.net_drop;
					Cfg saved = cfg;
					ReadConfig();
					const bool enabled = cfg.enable != 0;
					const Cfg now = cfg;
					cfg = saved;
					if (!enabled)
					{
						Stop("stopped by rb_net.txt");
						return;
					}
					cfg.pattern = now.pattern;
					cfg.net_delay = now.net_delay;
					cfg.net_jitter = now.net_jitter;
					cfg.net_drop = now.net_drop;
					if (cfg.pattern != old_pattern || cfg.net_delay != od || cfg.net_jitter != oj || cfg.net_drop != op)
					{
						Console.WriteLn("RbNet: settings changed at BF %u: artificial pattern %d, network delay %u ms + jitter %u ms, loss %u%%", F, cfg.pattern,
							cfg.net_delay, cfg.net_jitter, cfg.net_drop);
						LogStats("settings");
					}
				}
				MaybeSave(F);
			}

			void OnHook()
			{
				if (act != Act::None)
					return; // the CPU loop is about to exit
				if (trace_active)
					TraceStep(resim);
				if (st == St::Armed)
				{
					u32 F = 0;
					u32 ptrs[8];
					if (!nb::ReadBf(&F) || nb::Units(ptrs) < 2)
						return;
					if (F > cfg.start)
					{
						Console.Error("RbNet: the start frame %u was missed (BF is already %u)", cfg.start, F);
						RollbackNet::Close();
						st = St::Idle;
						return;
					}
					if (F == cfg.start)
						RequestAct(Act::Begin);
					return;
				}
				if (st == St::Run)
					RunHook();
			}

			// Runs at the clean point. Returns true when a rollback-net action was handled.
			bool OnExit()
			{
				if (act == Act::None)
					return false;
				const Act a = act;
				act = Act::None;
				if (!VMManager::HasValidVM() || VMManager::GetState() != VMState::Running)
				{
					Stop("execution interrupted");
					return true;
				}
				Error error;
				Common::Timer t;
				switch (a)
				{
					case Act::Begin:
					{
						u32 F = 0;
						nb::ReadBf(&F);
						if (F != cfg.start || RamSize() < sess::kMailbox + 6 * sess::kMailboxStride)
						{
							Console.Error("RbNet: cannot begin (BF %u, start %u, RAM %u MB)", F, cfg.start, RamSize() >> 20);
							RollbackNet::Close();
							st = St::Idle;
							return true;
						}
						MeasureRamUse();
						base = F;
						expect = F + 1;
						session++;
						sif_log.clear();
						sif_pending.clear();
						sif_pending_pos = 0;
						sif_seq = 0;
						for (u64& q : ring_seq)
							q = 0;
						sif_logged_w = sif_logged_i = sif_replayed_w = sif_replayed_i = 0;
						sif_hist.clear();
						sif_last_w_replay = true;
						SifReadConfig();
						WatchReadConfig();
						g_sif0_log_flag = (cfg.debug & 512) != 0;
						g_FastKeepSif = (cfg.debug & 1024) != 0;
						Rollback::g_sif1_drop = false;
						Rollback::g_sif0_log = g_sif0_log_flag;
						if (cfg.debug & 16)
							TraceArm(me);
						last_state = 0x100;
						for (u32 s = 0; s < kSlots; s++)
						{
							truth[s].assign(kMaxFrames, 0);
							recv_word[s].assign(kMaxFrames, 0);
							used[s].assign(kMaxFrames, 0);
							have[s].assign(kMaxFrames, 0);
							recvd[s].assign(kMaxFrames, 0);
							for (u32 i = 0; i < L; i++)
								have[s][i] = recvd[s][i] = 1; // the first L frames are defined as "no input" for everybody
							kn[s] = static_cast<s32>(L) - 1;
						}
						rollbacks = resim_frames = max_depth = saves = pred_ok = pred_bad = stalls = anomalies = 0;
						pkts_in = pkts_bad = live_frames = pkts_out = pkts_dropped = 0;
						outq.clear();
						rng = 12345u + me * 7919u;
						ms_restore = ms_save = ms_stall = 0.0;
						last_logged = kNoFrame;
						resim = false;
						for (sess::Slot& s : ring)
							s.valid = false;
						if (!sess::ApplyGuestPatch(true))
						{
							Stop("input patch failed");
							return true;
						}
						if (!nb::ApplyG2(true))
						{
							Stop("comm-stop patch failed");
							return true;
						}
						if (G3MaskFromDebug(cfg.debug) && !ApplyG3(true, G3MaskFromDebug(cfg.debug)))
						{
							Stop("G3 patch failed");
							return true;
						}
						Console.WriteLn("RbNet: comm-step call sites stopped: G2 (0x1DCF84) + G3 mask %u", g3_mask);
						s_prev_limiter = VMManager::GetLimiterMode();
						g_FastSkipIop = (cfg.debug & 4) != 0;
						LocalStep(F);
						WriteAll(F);
						if (!(cfg.debug & 1))
						{
							ring[0].snap.ee_regions = s_regions;
							if (!SaveState_FastSave(ring[0].snap, false, &error))
							{
								Console.Error("RbNet: first snapshot failed: %s", error.GetDescription().c_str());
								Stop("save failed");
								return true;
							}
							ring[0].frame = F;
							ring[0].valid = true;
						}
						st = St::Run;
						const u32 ctx = nb::R32(nb::kGpBase - 0x7D20);
						Console.WriteLn("RbNet: running from BF %u (guest ctx local slot %u, configured slot %u)", F,
							nb::Plausible(ctx) ? nb::R8(ctx + 0x1D) : 0xFFu, me);
						break;
					}
					case Act::Save:
					{
						sess::Slot& s = ring[save_slot];
						s.snap.ee_regions = s_regions;
						if (!SaveState_FastSave(s.snap, false, &error))
						{
							Console.Error("RbNet: snapshot save failed: %s", error.GetDescription().c_str());
							Stop("save failed");
							return true;
						}
						s.frame = save_frame;
						s.valid = true;
						ring_seq[save_slot] = sif_seq;
						if (g_sif0_log_flag)
						{
							u32 mask = 0;
							for (u32 i = 0; i < kRing; i++)
								mask |= ring[i].valid ? (1u << i) : 0u;
							SifPrune(mask);
						}
						saves++;
						ms_save += t.GetTimeMilliseconds();
						break;
					}
					case Act::Rollback:
					{
						sess::Slot& s = ring[roll_slot];
						// The comm module's EE-side data is the other half of what the (not rewound) IOP network stack and the peers
						// know: its receive buffers/connection state are written by the IOP asynchronously. A rewind of it makes the
						// game report a communication error as soon as it uses the comm again (end of the battle). Keep it live.
						struct KeepRange
						{
							u32 lo, hi;
						};
						static constexpr KeepRange kKeep[] = {
							{0x628000, 0x636000}, // the comm module's data/buffers (lui 0x62/0x63 references of the module at 0x2F0000-0x300000)
							{0x4A08E0, 0x4A08F0}, // its gp-relative variables
							{0x4A0A30, 0x4A0BE0},
							{0x4A6F00, 0x4A7000}, // SIF RPC bookkeeping seen changing at the connection loss
						};
						std::vector<u32> pre_words;
						if (trace_active)
							TraceWords(pre_words);
						// debug bit 10: the SIF DMA state stays as it is (copy the live bytes aside, put them back after the restore)
						struct HwRange
						{
							u32 lo, hi;
						};
						static constexpr HwRange kSifHw[] = {{0xC000, 0xC100}, {0xC400, 0xC500}, {0xF200, 0xF300}};
						static std::vector<u8> sif_keep;
						static std::vector<u8> sif_state_keep;
						sif_keep.clear();
						sif_state_keep.clear();
						if (cfg.debug & 1024)
						{
							for (const HwRange& r : kSifHw)
								sif_keep.insert(sif_keep.end(), eeHw + r.lo, eeHw + r.hi);
							sif_state_keep.assign(reinterpret_cast<const u8*>(&sif0), reinterpret_cast<const u8*>(&sif0) + sizeof(sif0));
							sif_state_keep.insert(sif_state_keep.end(), reinterpret_cast<const u8*>(&sif1), reinterpret_cast<const u8*>(&sif1) + sizeof(sif1));
						}
						static std::vector<u8> keep;
						keep.clear();
						if (cfg.debug & 8)
						{
							for (const KeepRange& r : kKeep)
								keep.insert(keep.end(), eeMem->Main + r.lo, eeMem->Main + r.hi);
						}
						if (!SaveState_FastLoad(s.snap, FastLoadMode::Diff, false, nullptr, &error))
						{
							Console.Error("RbNet: restore failed: %s", error.GetDescription().c_str());
							Stop("restore failed");
							return true;
						}
						if (!keep.empty())
						{
							size_t pos = 0;
							for (const KeepRange& r : kKeep)
							{
								std::memcpy(eeMem->Main + r.lo, keep.data() + pos, r.hi - r.lo);
								pos += r.hi - r.lo;
							}
						}
						if (!sif_keep.empty())
						{
							size_t pos = 0;
							for (const HwRange& r : kSifHw)
							{
								std::memcpy(eeHw + r.lo, sif_keep.data() + pos, r.hi - r.lo);
								pos += r.hi - r.lo;
							}
							std::memcpy(&sif0, sif_state_keep.data(), sizeof(sif0));
							std::memcpy(&sif1, sif_state_keep.data() + sizeof(sif0), sizeof(sif1));
						}
						if (g_sif0_log_flag)
						{
							// deliveries newer than the snapshot: those of the snapshot's own frame period come back now,
							// the others at the hooks of their frames
							const u64 from = ring_seq[roll_slot];
							sif_pending.clear();
							sif_pending_pos = 0;
							for (const SifEvt& e : sif_log)
							{
								if (e.seq > from && e.replay)
									sif_pending.push_back(e);
							}
							SifApplyUpTo(s.frame);
						}
						if (trace_active)
						{
							// what the restore did to the comm control words (live value before -> value after)
							u32 changed = 0;
							size_t idx = 0;
							for (const TraceRange& r : kTraceRanges)
							{
								for (u32 a = r.lo; a < r.hi; a += 4, idx++)
								{
									const u32 v = nb::R32(a);
									if (v != pre_words[idx])
									{
										TracePush('D', a, pre_words[idx], v);
										changed++;
									}
								}
							}
							TracePush('R', s.frame, target - s.frame, changed);
							TraceWords(trace_prev);
						}
						ms_restore += t.GetTimeMilliseconds();
						expect = s.frame + 1;
						resim = true;
						Rollback::g_sif1_drop = (cfg.debug & 2048) != 0;
						VMManager::SetLimiterMode(LimiterModeType::Unlimited);
						WriteAll(s.frame); // the word of the snapshot's own frame, now with the corrected input
						break;
					}
					case Act::Cleanup:
					{
						const bool ok1 = nb::ApplyG2(false);
						if (g3_mask)
							ApplyG3(false, g3_mask);
						const bool ok2 = sess::ApplyGuestPatch(false);
						Console.WriteLn("RbNet: guest patches reverted after the battle (comm stop %s, input mailbox %s)", ok1 ? "ok" : "FAILED", ok2 ? "ok" : "FAILED");
						break;
					}
					case Act::None:
						break;
				}
				return true;
			}
		} // namespace rn
	} // namespace

	void RequestSelfTest()
	{
		if (s_phase == Phase::Idle && !sess::Active())
		{
			s_requested = true;
			Console.WriteLn("RollbackTest: requested; starts at the next frame boundary");
		}
	}

	void RequestSessionTest()
	{
		if (s_phase == Phase::Idle && !sess::Active())
		{
			sess::requested = true;
			sess::want_mailbox = false;
			Console.WriteLn("RollbackSession: requested (JVS inputs); starts at the next frame boundary");
		}
	}

	void RequestMailboxTest()
	{
		if (s_phase == Phase::Idle && !sess::Active())
		{
			sess::requested = true;
			sess::want_mailbox = true;
			Console.WriteLn("RollbackSession: requested (mailbox inputs); starts at the next frame boundary");
		}
	}

	bool IsSelfTestRunning()
	{
		return s_phase != Phase::Idle || sess::Active();
	}

	bool g_sif0_log = false;
	bool g_sif1_drop = false;
	bool g_watch_on = false;

	bool g_watch_jit = false;

	void WatchWrite(u32 addr, u32 size, u64 value)
	{
		rn::WatchWrite(addr, size, value, cpuRegs.pc - 4);
	}

	void WatchJit(u32 op, u32 pc)
	{
		const u32 rs = (op >> 21) & 31, rt = (op >> 16) & 31;
		const u32 addr = cpuRegs.GPR.r[rs].UL[0] + static_cast<u32>(static_cast<s32>(static_cast<s16>(op & 0xFFFF)));
		u32 size = 0;
		u64 v = 0;
		switch (op >> 26)
		{
			case 0x28: size = 1; v = cpuRegs.GPR.r[rt].UD[0]; break; // SB
			case 0x29: size = 2; v = cpuRegs.GPR.r[rt].UD[0]; break; // SH
			case 0x2B: size = 4; v = cpuRegs.GPR.r[rt].UD[0]; break; // SW
			case 0x3F: size = 8; v = cpuRegs.GPR.r[rt].UD[0]; break; // SD
			case 0x1F: size = 16; v = cpuRegs.GPR.r[rt].UD[0]; break; // SQ
			case 0x39: size = 4; v = fpuRegs.fpr[rt].UL; break; // SWC1
			default: return;
		}
		rn::WatchWrite(addr, size, v, pc);
	}

	void LogSif0Write(u32 ee_addr, const void* data, u32 bytes)
	{
		rn::SifLogWrite(ee_addr, data, bytes);
	}

	void LogSif0Irq()
	{
		rn::SifLogIrq();
	}

	void OnVSync()
	{
		if (sess::Active())
		{
			if (!VMManager::HasValidVM())
				sess::Finish("VM went away");
			else
				sess::OnHook();
			return;
		}
		if (rn::Active())
		{
			if (!VMManager::HasValidVM())
				rn::Stop("VM went away");
			else
				rn::OnHook();
			return;
		}
		if (s_phase == Phase::Idle && sess::requested)
		{
			sess::requested = false;
			if (VMManager::HasValidVM())
			{
				s_pass = 0;
				s_fail = 0;
				sess::RequestAct(sess::Act::Init);
			}
			return;
		}

		if (s_phase == Phase::Idle)
		{
			nb::OnHook();
			rn::Poll();
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

		if (s_phase == Phase::WaitBench || s_phase == Phase::WaitSave || s_phase == Phase::WaitLoad)
			return; // the CPU loop is about to exit; nothing to count

		s_n++;
		RecordHash((s_phase == Phase::Reference) ? s_hash_ref : s_hash_cur, s_n);
		ApplyMask(ScriptMask(s_n));
		if (s_phase == Phase::Resim && s_n <= kFirstFrames)
			s_first_frame_ms[s_n] = s_timer.GetTimeMilliseconds() - s_hash_ms;
		if (s_n < kFrames)
			return;

		if (s_phase == Phase::Reference)
		{
			CopyRam(s_ref);
			s_ms_ref = s_timer.GetTimeMilliseconds();
			RequestAction(Pending::Load, Phase::WaitLoad);
			return;
		}

		// Phase::Resim
		const double resim_ms = s_timer.GetTimeMilliseconds();
		CopyRam(s_cur);
		ReportResim(resim_ms);
		s_repeat++;
		if (s_repeat < kRepeats)
			RequestAction(Pending::Load, Phase::WaitLoad);
		else
			NextConfig();
	}

	void OnExecutionExit()
	{
		if (sess::OnExit())
			return;
		if (rn::OnExit())
			return;
		if (nb::OnExit())
			return;
		if (s_pending == Pending::None)
			return;

		const Pending what = s_pending;
		s_pending = Pending::None;
		if (!VMManager::HasValidVM() || VMManager::GetState() != VMState::Running)
		{
			EndTest("execution interrupted");
			return;
		}

		switch (what)
		{
			case Pending::Bench:
				RunBench();
				// We are already at the clean point: take the first snapshot right here.
				s_snap.ee_regions = s_regions;
				PrepareConfig();
				DoSave();
				break;
			case Pending::Save:
				DoSave();
				break;
			case Pending::Load:
				DoLoad();
				break;
			case Pending::None:
				break;
		}
	}
} // namespace Rollback
