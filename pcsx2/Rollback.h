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

	/// Arms the self-test; it starts at the next frame boundary. Ignored while a test is running.
	void RequestSelfTest();

	/// True while a self-test is in progress.
	bool IsSelfTestRunning();
} // namespace Rollback
