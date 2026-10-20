#ifndef OSAL_H
#define OSAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OSAL - Operating System Abstraction Layer
 * Jedinstveno sučelje za mrežu i vrijeme na Windowsu (Winsock) i POSIX sustavima.
 * Ovaj header namjerno NE uključuje nikakve sistemske headere (windows.h i sl.).
 */

/* UDP socket; sadržaj je neproziran (opaque), koristi se samo kroz osal_* funkcije */
typedef struct {
    uintptr_t handle;
} OsalUdpSocket;

/* ---------- Mreža ---------- */

/* Mora se pozvati jednom na početku programa (Windows: WSAStartup; POSIX: ništa) */
bool osal_net_init(void);

/* Poziva se jednom na kraju programa */
void osal_net_cleanup(void);

/* Otvori UDP socket. bind_port != 0: slušaj na 127.0.0.1:bind_port.
   bind_port == 0: socket samo za slanje. */
bool osal_udp_open(OsalUdpSocket *sock, uint16_t bind_port);

/* Zatvori socket */
void osal_udp_close(OsalUdpSocket *sock);

/* Pošalji len bajtova na ip:port (npr. "127.0.0.1", 5000) */
bool osal_udp_send(const OsalUdpSocket *sock, const char *ip, uint16_t port,
                   const void *data, size_t len);

/* Čekaj paket najviše timeout_ms milisekundi.
   Vraća: broj primljenih bajtova (> 0), 0 = timeout, -1 = greška */
int osal_udp_recv(const OsalUdpSocket *sock, void *buffer, size_t buffer_len,
                  uint32_t timeout_ms);

/* ---------- Vrijeme ---------- */

/* Monotono vrijeme u milisekundama (nikad ne ide unatrag) */
uint64_t osal_time_ms(void);

/* Spavaj ms milisekundi */
void osal_sleep_ms(uint32_t ms);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* OSAL_H */