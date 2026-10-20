#include "adas_msg.h"
#include "brake_logic.h"
#include "osal.h"

#include <stdio.h>

#define ECU_LISTEN_PORT   5000u
#define ECU_POLL_MS       20u      /* koliko često provjeravamo socket */

static const char *command_name(BrakeCommand c)
{
    switch (c) {
        case BRAKE_NONE:    return "none";
        case BRAKE_WARN:    return "DRIVER WARNING";
        case BRAKE_PARTIAL: return "PARTIAL BRAKING";
        case BRAKE_FULL:    return "FULL BRAKING";
    }
    return "?";
}

static const char *state_name(EcuState s)
{
    switch (s) {
        case ECU_STATE_OK:       return "OK";
        case ECU_STATE_DEGRADED: return "DEGRADED";
        case ECU_STATE_FAULT:    return "FAULT";
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
        fprintf(stderr, "[BRAKE ECU] ne mogu otvoriti port %u\n", (unsigned)ECU_LISTEN_PORT);
        osal_net_cleanup();
        return 1;
    }
    printf("[BRAKE ECU] listening on 127.0.0.1:%u\n", (unsigned)ECU_LISTEN_PORT);

    uint64_t last_rx_ms = osal_time_ms();     /* kad je zadnji put nešto stiglo */
    bool     have_msg   = false;              /* je li ikad stigla poruka */
    AdasWarningMsg last;
    EcuState prev_state = ECU_STATE_OK;

    for (;;) {
        uint8_t buf[64];
        const int n = osal_udp_recv(&sock, buf, sizeof(buf), ECU_POLL_MS);
        if (n < 0) {
            fprintf(stderr, "[BRAKE ECU] greška na socketu\n");
            break;
        }

        if (n > 0 && n == (int)CAN_MAX_DLC) {
            CanFrame frame;
            frame.id  = ADAS_MSG_ID;
            frame.dlc = CAN_MAX_DLC;
            for (int i = 0; i < CAN_MAX_DLC; ++i) {
                frame.data[i] = buf[i];
            }

            AdasWarningMsg msg;
            if (adas_msg_unpack(&frame, &msg) == ADAS_UNPACK_OK) {
                last        = msg;
                have_msg    = true;
                last_rx_ms  = osal_time_ms();
            }
            /* odbačene poruke: ne resetiramo vrijeme, pa tiha kamera i dalje vodi do FAULT-a */
        }

        /* Koliko je staro zadnje valjano stanje */
        const uint64_t now = osal_time_ms();
        const uint32_t age = (uint32_t)(now - last_rx_ms);
        const EcuState state = have_msg ? brake_state_from_age(age) : ECU_STATE_FAULT;

        if (have_msg && state == ECU_STATE_FAULT) {
            /* FAULT: sigurna reakcija, ne vjerujemo staroj poruci */
        }

        const BrakeCommand cmd = have_msg
            ? brake_decide_safe(state, &last)
            : BRAKE_WARN;

        /* Ispiši samo kad se stanje ili naredba promijene, da terminal ostane čitljiv */
        if (state != prev_state || (n > 0)) {
            if (have_msg) {
                printf("[BRAKE ECU] state %-8s | age %3u ms | TTC %5.2f s | cmd %s\n",
                       state_name(state), (unsigned)age,
                       (double)last.ttc_s, command_name(cmd));
            } else {
                printf("[BRAKE ECU] state %-8s | čekam prvu poruku | cmd %s\n",
                       state_name(state), command_name(cmd));
            }
        }
        prev_state = state;
    }

    osal_udp_close(&sock);
    osal_net_cleanup();
    return 0;
}