// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "common/Pcsx2Types.h"

// A tiny non-blocking UDP socket wrapper for the rollback input exchange (Rollback.cpp).
// Kept in its own translation unit so that no socket header is pulled into Rollback.cpp.
namespace RollbackNet
{
	/// Binds a non-blocking UDP socket to 0.0.0.0:port. Closes a previously opened socket first.
	bool Open(u16 port);

	void Close();

	bool IsOpen();

	/// Sends one datagram to an IPv4 address given as text ("127.0.0.1"). Never blocks; failures are ignored.
	void SendTo(const char* ipv4, u16 port, const void* data, int len);

	/// Receives one pending datagram. Returns its size, or 0 when nothing is waiting.
	int Recv(void* buf, int capacity);
} // namespace RollbackNet
