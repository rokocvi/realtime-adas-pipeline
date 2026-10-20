#include "adas_msg.h"
#include "osal.h"

#include <stdio.h>

#define ECU_LISTEN_PORT   5000u
#define ECU_RECV_TIMEOUT  200u   /* ms: ako ništa ne stigne, to je "nema poruke" */

static const char *level_name(AdasLevel level)
{
    switch (level) {
        case ADAS_LEVEL_NONE:    return "NONE";
        case ADAS_LEVEL_CAUTION: return "CAUTION";
        case ADAS_LEVEL_WARNING: return "WARNING";
    }
    return "?";
}

int main(void)
{
    if (!osal_net_init()) {
        fprintf(stderr, "[BRAKE ECU] ne mogu inicijalizirati mrežu\n");
        return 1;
    }

    OsalUdpSocket sock;
    if (!osal_udp_open(&sock, ECU_LISTEN_PORT)) {
        fprintf(stderr, "[BRAKE ECU] ne mogu otvoriti port %u (zauzet?)\n",
                (unsigned)ECU_LISTEN_PORT);
        osal_net_cleanup();
        return 1;
    }
    printf("[BRAKE ECU] listening on 127.0.0.1:%u\n", (unsigned)ECU_LISTEN_PORT);

    uint32_t received = 0u;
    uint32_t rejected = 0u;

    for (;;) {
        uint8_t buf[64];
        const int n = osal_udp_recv(&sock, buf, sizeof(buf), ECU_RECV_TIMEOUT);

        if (n < 0) {
            fprintf(stderr, "[BRAKE ECU] greška na socketu\n");
            break;
        }
        if (n == 0) {
            printf("[BRAKE ECU] !!! nema poruke %u ms\n", (unsigned)ECU_RECV_TIMEOUT);
            continue;
        }

        /* Paket mora biti točno veličine CAN framea: 8 bajtova */
        CanFrame frame;
        frame.id  = ADAS_MSG_ID;   /* ID ne šaljemo u paketu, pretpostavka je naš protokol */
        frame.dlc = (uint8_t)n;
        if (n > (int)CAN_MAX_DLC) {
            rejected++;
            continue;
        }
        for (int i = 0; i < n; ++i) {
            frame.data[i] = buf[i];
        }

        AdasWarningMsg msg;
        const AdasUnpackResult res = adas_msg_unpack(&frame, &msg);
        if (res != ADAS_UNPACK_OK) {
            rejected++;
            printf("[BRAKE ECU] odbačena poruka (rezultat %d)\n", (int)res);
            continue;
        }

        received++;
        if (msg.ttc_valid) {
            printf("[BRAKE ECU] #%03u alive %2u | %-7s | TTC %5.2f s | track %3u | x %+5.1f m\n",
                   (unsigned)received, (unsigned)msg.alive_counter,
                   level_name(msg.level), (double)msg.ttc_s,
                   (unsigned)msg.track_id, (double)msg.lateral_offset_m);
        } else {
            printf("[BRAKE ECU] #%03u alive %2u | %-7s | TTC   -- | track %3u | x %+5.1f m\n",
                   (unsigned)received, (unsigned)msg.alive_counter,
                   level_name(msg.level), (unsigned)msg.track_id,
                   (double)msg.lateral_offset_m);
        }
    }

    printf("[BRAKE ECU] primljeno %u, odbačeno %u\n",
           (unsigned)received, (unsigned)rejected);
    osal_udp_close(&sock);
    osal_net_cleanup();
    return 0;
}