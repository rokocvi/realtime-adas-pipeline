#ifndef BRAKE_LOGIC_H
#define BRAKE_LOGIC_H

#include "adas_msg.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BRAKE_NONE = 0,
    BRAKE_WARN = 1,
    BRAKE_PARTIAL = 2,
    BRAKE_FULL = 3
} BrakeCommand;


typedef enum {
    ECU_STATE_OK = 0,
    ECU_STATE_DEGRADED = 1,
    ECU_STATE_FAULT = 2
} EcuState;

/* Pragovi odluke (u sekundama TTC-a) */
#define BRAKE_TTC_WARN_S     2.5f
#define BRAKE_TTC_PARTIAL_S  1.5f
#define BRAKE_TTC_FULL_S     0.8f

/* Nadzor poruka (u milisekundama) */
#define BRAKE_DEGRADED_MS    100u   /* do ovoga tolerišemo, zaostajanje od jednog ciklusa */
#define BRAKE_FAULT_MS       200u   /* nakon ovoga kamera se smatra neispravnom */

/* Čista odluka: samo iz TTC-a i razine, bez stanja i bez mreže */
BrakeCommand brake_decide(const AdasWarningMsg *msg);

/* Odluka uz stanje sustava: kad FAULT, uvijek sigurna reakcija */
BrakeCommand brake_decide_safe(EcuState state, const AdasWarningMsg *msg);

/* Izračunaj stanje iz "koliko je prošlo od zadnje poruke" */
EcuState brake_state_from_age(uint32_t age_ms);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* BRAKE_LOGIC_H */