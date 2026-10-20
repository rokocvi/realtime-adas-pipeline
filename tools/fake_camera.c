#include "adas_msg.h"
#include "osal.h"

#include <stdio.h>

#define ECU_PORT     5000u
#define SEND_PERIOD  50u     /* ms: 20 poruka u sekundi, kao pravi ADAS */

int main(void)
{
    if (!osal_net_init()) {
        fprintf(stderr, "[FAKE CAM] ne mogu inicijalizirati mrežu\n");
        return 1;
    }

    OsalUdpSocket sock;
    if (!osal_udp_open(&sock, 0u)) {   /* 0 = samo za slanje */
        fprintf(stderr, "[FAKE CAM] ne mogu otvoriti socket\n");
        osal_net_cleanup();
        return 1;
    }

    /* Scenarij: auto ispred nas, TTC pada s 4.0 s na 0.5 s */
    float ttc = 4.0f;
    uint8_t alive = 0u;

    for (int i = 0; i < 80; ++i) {          /* 80 x 50 ms = 4 s */
        AdasWarningMsg msg;
        msg.alive_counter    = alive;
        msg.ttc_valid        = true;
        msg.ttc_s            = ttc;
        msg.object_class     = ADAS_OBJ_CAR;
        msg.track_id         = 12u;
        msg.lateral_offset_m = 0.1f;
        msg.level = (ttc < 2.5f) ? ADAS_LEVEL_WARNING
                  : (ttc < 4.0f) ? ADAS_LEVEL_CAUTION
                                 : ADAS_LEVEL_NONE;

        CanFrame frame;
        adas_msg_pack(&msg, &frame);

        /* Šaljemo samo 8 bajtova podataka (bez ID-a i DLC-a, to je dogovor protokola) */
        if (!osal_udp_send(&sock, "127.0.0.1", (uint16_t)ECU_PORT,
                           frame.data, frame.dlc)) {
            fprintf(stderr, "[FAKE CAM] slanje nije uspjelo\n");
        }

        printf("[FAKE CAM] alive %2u | TTC %4.2f s | level %d\n",
               (unsigned)alive, (double)ttc, (int)msg.level);

        alive = (uint8_t)((alive + 1u) & ADAS_ALIVE_MASK);  /* 0..15, pa opet 0 */
        ttc  -= 0.05f;                                      /* TTC pada 1 s po sekundi */
        osal_sleep_ms(SEND_PERIOD);
    }

    osal_udp_close(&sock);
    osal_net_cleanup();
    return 0;
}