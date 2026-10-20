// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "RollbackNet.h"

#include "common/Console.h"

#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
static constexpr socket_t kInvalidSocket = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
static constexpr socket_t kInvalidSocket = -1;
#endif

namespace RollbackNet
{
	static socket_t s_sock = kInvalidSocket;

	static void CloseSocket(socket_t s)
	{
#ifdef _WIN32
		closesocket(s);
#else
		close(s);
#endif
	}

	bool Open(u16 port)
	{
		Close();
#ifdef _WIN32
		WSADATA wsa;
		WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
		s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (s_sock == kInvalidSocket)
		{
			Console.Error("RollbackNet: socket() failed");
			return false;
		}
		int one = 1;
		setsockopt(s_sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
#ifdef _WIN32
		u_long nonblock = 1;
		ioctlsocket(s_sock, FIONBIO, &nonblock);
		// Sending to a port that has no listener yet makes Windows fail the NEXT recvfrom() with WSAECONNRESET: turn that off.
		DWORD bytes_returned = 0;
		BOOL new_behavior = FALSE;
		WSAIoctl(s_sock, _WSAIOW(IOC_VENDOR, 12) /* SIO_UDP_CONNRESET */, &new_behavior, sizeof(new_behavior), nullptr, 0, &bytes_returned, nullptr, nullptr);
#else
		fcntl(s_sock, F_SETFL, fcntl(s_sock, F_GETFL, 0) | O_NONBLOCK);
#endif
		sockaddr_in addr;
		std::memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		addr.sin_port = htons(port);
		if (bind(s_sock, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0)
		{
			Console.Error("RollbackNet: bind(UDP %u) failed", static_cast<unsigned>(port));
			CloseSocket(s_sock);
			s_sock = kInvalidSocket;
			return false;
		}
		Console.WriteLn("RollbackNet: listening on UDP port %u", static_cast<unsigned>(port));
		return true;
	}

	void Close()
	{
		if (s_sock != kInvalidSocket)
		{
			CloseSocket(s_sock);
			s_sock = kInvalidSocket;
		}
	}

	bool IsOpen()
	{
		return s_sock != kInvalidSocket;
	}

	void SendTo(const char* ipv4, u16 port, const void* data, int len)
	{
		if (s_sock == kInvalidSocket)
			return;
		sockaddr_in addr;
		std::memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);
		if (inet_pton(AF_INET, ipv4, &addr.sin_addr) != 1)
			return;
		sendto(s_sock, static_cast<const char*>(data), len, 0, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
	}

	int Recv(void* buf, int capacity)
	{
		if (s_sock == kInvalidSocket)
			return 0;
		const int n = static_cast<int>(recvfrom(s_sock, static_cast<char*>(buf), capacity, 0, nullptr, nullptr));
		return (n > 0) ? n : 0;
	}
} // namespace RollbackNet
