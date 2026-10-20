#include "brake_logic.h"

BrakeCommand brake_decide(const AdasWarningMsg *msg)
{
    /* Bez valjanog TTC-a ne znamo koliko je opasno: samo upozorenje ako je razina upozorenja */
    if (!msg->ttc_valid) {
        return (msg->level == ADAS_LEVEL_WARNING) ? BRAKE_WARN : BRAKE_NONE;
    }

    /* Odluka po TTC-u: od najhitnijeg prema manje hitnom */
    if (msg->ttc_s < BRAKE_TTC_FULL_S) {
        return BRAKE_FULL;
    }
    if (msg->ttc_s < BRAKE_TTC_PARTIAL_S) {
        return BRAKE_PARTIAL;
    }
    if (msg->ttc_s < BRAKE_TTC_WARN_S) {
        return BRAKE_WARN;
    }
    return BRAKE_NONE;
}

BrakeCommand brake_decide_safe(EcuState state, const AdasWarningMsg *msg)
{
    switch (state) {
        case ECU_STATE_OK:
            return brake_decide(msg);

        case ECU_STATE_DEGRADED:
            /* kamera tiha kratko: ne vjerujemo novoj odluci više nego upozorenju */
            return (brake_decide(msg) == BRAKE_NONE) ? BRAKE_NONE : BRAKE_WARN;

        case ECU_STATE_FAULT:
            /* kamera ne radi: sigurna reakcija, ne vjerujemo staroj poruci */
            return BRAKE_WARN;
    }
    return BRAKE_WARN;   /* nepoznato stanje = sigurna reakcija */
}

EcuState brake_state_from_age(uint32_t age_ms)
{
    if (age_ms >= BRAKE_FAULT_MS) {
        return ECU_STATE_FAULT;
    }
    if (age_ms >= BRAKE_DEGRADED_MS) {
        return ECU_STATE_DEGRADED;
    }
    return ECU_STATE_OK;
}