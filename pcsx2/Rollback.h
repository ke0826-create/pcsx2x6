// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

// Rollback netplay groundwork for PCSX2x6.
//
// Stage 1 (this file): an OFFLINE self-test. It answers "does restoring a save state and
// re-simulating the same frames with the same inputs reproduce the same EE RAM?" without any
// networking. See Rollback.cpp for the procedure and the log lines it prints.
//
// Everything here runs on the CPU thread, from the frame boundary (VSyncStart).
namespace Rollback
{
	/// Called once per frame from VSyncStart on the CPU thread, right after input polling.
	/// Cheap (one branch) when no test is running.
	void OnVSync();

	/// Called from VMManager::Execute() right after the CPU execution loop has been left (a clean point:
	/// no event handler is in flight). Performs the pending state save/load requested from OnVSync().
	void OnExecutionExit();

	/// Arms the self-test; it starts at the next frame boundary. Ignored while a test is running.
	void RequestSelfTest();

	/// Arms the late-input rollback session test (F11); it starts at the next frame boundary.
	void RequestSessionTest();

	/// Arms the mailbox-input rollback session test (F12): the guest's human-unit input reader is patched to read
	/// its pad word from an emulator-owned mailbox; the test then runs the late-input rollback loop through it.
	void RequestMailboxTest();

	/// True while a self-test is in progress.
	bool IsSelfTestRunning();

	/// SIF0 (IOP -> EE DMA) delivery log for the rollback session (see Rollback.cpp, "SIF0 replay"): the IOP is not rolled
	/// back, so what it delivered to the EE between a snapshot and the rollback would be lost; it is logged here and
	/// delivered again while the frames are re-simulated.
	extern bool g_sif0_log;
	/// A SIF0 chunk was written to EE RAM at ee_addr (physical).
	void LogSif0Write(u32 ee_addr, const void* data, u32 bytes);
	/// A SIF0 transfer completed (the EE DMAC interrupt is raised).
	void LogSif0Irq();
} // namespace Rollback
