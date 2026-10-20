/* Na Linuxu s čistim C11 POSIX funkcije treba eksplicitno "uključiti" */
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif

#include "osal.h"

#include <string.h>

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    typedef SOCKET native_socket_t;
#else
    #include <arpa/inet.h>
    #include <netinet/in.h>
    #include <sys/select.h>
    #include <sys/socket.h>
    #include <time.h>
    #include <unistd.h>
    typedef int native_socket_t;
#endif

#define OSAL_INVALID_HANDLE UINTPTR_MAX

/* ======================= Pomoćne funkcije ======================= */

static native_socket_t to_native(const OsalUdpSocket *sock)
{
    return (native_socket_t)sock->handle;
}

static void close_native(native_socket_t s)
{
#ifdef _WIN32
    (void)closesocket(s);
#else
    (void)close(s);
#endif
}

/* ============================ Mreža ============================ */

bool osal_net_init(void)
{
#ifdef _WIN32
    WSADATA wsa_data;
    return WSAStartup(MAKEWORD(2, 2), &wsa_data) == 0;
#else
    return true;
#endif
}

void osal_net_cleanup(void)
{
#ifdef _WIN32
    (void)WSACleanup();
#endif
}

bool osal_udp_open(OsalUdpSocket *sock, uint16_t bind_port)
{
    sock->handle = OSAL_INVALID_HANDLE;

    const native_socket_t s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#ifdef _WIN32
    if (s == INVALID_SOCKET) {
        return false;
    }
#else
    if (s < 0) {
        return false;
    }
#endif

    if (bind_port != 0u) {
        struct sockaddr_in addr;
        (void)memset(&addr, 0, sizeof(addr));
        addr.sin_family      = AF_INET;
        addr.sin_port        = htons(bind_port);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* samo 127.0.0.1 */

        if (bind(s, (const struct sockaddr *)&addr, (int)sizeof(addr)) != 0) {
            close_native(s);
            return false;
        }
    }

    sock->handle = (uintptr_t)s;
    return true;
}

void osal_udp_close(OsalUdpSocket *sock)
{
    if (sock->handle != OSAL_INVALID_HANDLE) {
        close_native(to_native(sock));
        sock->handle = OSAL_INVALID_HANDLE;
    }
}

bool osal_udp_send(const OsalUdpSocket *sock, const char *ip, uint16_t port,
                   const void *data, size_t len)
{
    struct sockaddr_in addr;
    (void)memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
        return false;   /* neispravna IP adresa */
    }

#ifdef _WIN32
    const int sent = sendto(to_native(sock), (const char *)data, (int)len, 0,
                            (const struct sockaddr *)&addr, (int)sizeof(addr));
#else
    const ssize_t sent = sendto(to_native(sock), data, len, 0,
                                (const struct sockaddr *)&addr, sizeof(addr));
#endif
    return (sent >= 0) && ((size_t)sent == len);
}

int osal_udp_recv(const OsalUdpSocket *sock, void *buffer, size_t buffer_len,
                  uint32_t timeout_ms)
{
    const native_socket_t s = to_native(sock);

    /* 1) Čekaj da paket stigne, najviše timeout_ms */
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(s, &read_set);

    struct timeval tv;
    tv.tv_sec  = (long)(timeout_ms / 1000u);
    tv.tv_usec = (long)((timeout_ms % 1000u) * 1000u);

#ifdef _WIN32
    const int ready = select(0, &read_set, NULL, NULL, &tv);   /* 1. parametar Windows ignorira */
#else
    const int ready = select(s + 1, &read_set, NULL, NULL, &tv);
#endif
    if (ready < 0) {
        return -1;   /* greška */
    }
    if (ready == 0) {
        return 0;    /* timeout: ništa nije stiglo */
    }

    /* 2) Paket je tu, pročitaj ga */
#ifdef _WIN32
    const int received = recvfrom(s, (char *)buffer, (int)buffer_len, 0, NULL, NULL);
#else
    const ssize_t received = recvfrom(s, buffer, buffer_len, 0, NULL, NULL);
#endif
    return (received < 0) ? -1 : (int)received;
}

/* ============================ Vrijeme ============================ */

uint64_t osal_time_ms(void)
{
#ifdef _WIN32
    return (uint64_t)GetTickCount64();
#else
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u);
#endif
}

void osal_sleep_ms(uint32_t ms)
{
#ifdef _WIN32
    Sleep((DWORD)ms);
#else
    struct timespec ts;
    ts.tv_sec  = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)((ms % 1000u) * 1000000u);
    (void)nanosleep(&ts, NULL);
#endif
}