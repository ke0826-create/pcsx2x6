// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "Rollback.h"

#include "Common.h"
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
#include <cstdio>
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
		// Snapshots are taken every kK frames into a ring (a snapshot is valid until a misprediction at an
		// earlier frame is found; then it is dropped). Hook n applies the input of "frame n"; the snapshot of
		// frame n is taken at the clean point right after hook n (start of frame n, input n applied).
		// The clean point rule is the same as in the other test: only OnExecutionExit() saves or loads.
		namespace sess
		{
			constexpr u32 kN = 360;   // frames per run
			constexpr u32 kK = 4;     // snapshot interval
			constexpr u32 kRing = 5;  // snapshots kept (covers about kK * kRing frames)
			constexpr u32 kNone = 0xFFFFFFFFu;

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
				Gt,  // ground-truth run
				Run, // session run (live or re-simulating)
			};

			enum class Act
			{
				None,
				Init,     // take S0
				Restore0, // restore S0 and start the next pattern
				Save,     // take a ring snapshot
				Rollback, // restore a ring snapshot
			};

			struct Slot
			{
				FastSnapshot snap;
				u32 frame = 0;
				bool valid = false;
			};

			bool requested = false;
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
			std::vector<u8> gt_ram;
			std::vector<u8> rb_ram;
			u16 truth[kN + 1] = {};
			bool delivered[kN + 1] = {};
			u32 deliver_at[kN + 1] = {};
			u16 used[kN + 1] = {};
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

			// The input to use for frame f: the true one if it was delivered, else "hold the last known".
			u16 InputFor(u32 f)
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

			void ApplyUse(u32 f)
			{
				used[f] = InputFor(f);
				ApplyMask(used[f]);
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
				rb_ram.clear();
				rb_ram.shrink_to_fit();
				s0 = FastSnapshot();
				for (Slot& s : ring)
				{
					s.snap = FastSnapshot();
					s.valid = false;
				}
				Console.WriteLn("RollbackSession: finished (%s): %u PASS / %u FAIL (session vs ground truth)", why, s_pass, s_fail);
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
				if (d.lines == 0)
					s_pass++;
				else
					s_fail++;
				Console.WriteLn("RollbackSession: [%u/%u] '%s': %s (differing 64B lines=%u, volatile ignored=%u) | predictions right %u / wrong %u | "
								"rollbacks=%u, frames re-simulated=%u (deepest %u), snapshots taken=%u | %u frames in %.0f ms (ground truth %.0f ms), restores %.1f ms, saves %.1f ms",
					pat + 1, static_cast<u32>(std::size(kPatterns)), kPatterns[pat].name, (d.lines == 0) ? "MATCH" : "DIFFER", d.lines, d.ignored,
					pred_ok, pred_bad, rollbacks, resim_frames, max_depth, saves, kN, session_ms, gt_ms, ms_restore, ms_save);
				if (d.lines != 0)
					LogCompare("session vs ground truth", gt_ram, rb_ram, -1);
				pat++;
				if (pat < static_cast<u32>(std::size(kPatterns)))
					StartPattern();
				else
					Finish("done");
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
					ApplyMask((n < kN) ? truth[n] : 0);
					if (n < kN)
						return;
					gt_ms = wall.GetTimeMilliseconds();
					CopyRam(gt_ram);
					pat = 0;
					StartPattern();
					return;
				}
				if (step == Step::Run)
					RunHook();
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
						for (u32 f = 0; f <= kN; f++)
						{
							truth[f] = (f == 0 || f >= kN) ? 0 : HashMask(f);
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
						Console.WriteLn("RollbackSession: begin. %u frames per run, snapshot every %u frames (ring %u), %u delay patterns",
							kN, kK, kRing, static_cast<u32>(std::size(kPatterns)));
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
						for (u32 f = 0; f <= kN; f++)
						{
							delivered[f] = false;
							deliver_at[f] = f + Delay(kPatterns[pat], f);
							used[f] = 0;
						}
						for (Slot& s : ring)
							s.valid = false;
						ring[0].snap = s0; // frame 0 is the first snapshot
						ring[0].frame = 0;
						ring[0].valid = true;
						rollbacks = resim_frames = max_depth = saves = pred_ok = pred_bad = 0;
						ms_restore = ms_save = 0.0;
						n = 0;
						resim = false;
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
					case Act::None:
						break;
				}
				return true;
			}
		} // namespace sess
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
			Console.WriteLn("RollbackSession: requested; starts at the next frame boundary");
		}
	}

	bool IsSelfTestRunning()
	{
		return s_phase != Phase::Idle || sess::Active();
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
