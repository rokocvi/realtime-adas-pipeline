#ifndef ADAS_MSG_H
#define ADAS_MSG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== CAN frame ===================== */

#define CAN_MAX_DLC 8u

typedef struct {
    uint32_t id;                 /* CAN identifikator (11-bitni) */
    uint8_t  dlc;                /* broj bajtova podataka, 0-8 */
    uint8_t  data[CAN_MAX_DLC];  /* podaci */
} CanFrame;

/* ============ Poruka ADAS_CollisionWarning ============ */

#define ADAS_MSG_ID           0x120u
#define ADAS_MSG_DLC          8u

#define ADAS_TTC_FACTOR       0.01f     /* 1 bit = 0.01 s */
#define ADAS_TTC_INVALID      0xFFFFu   /* "nema TTC-a" (beskonačno) */
#define ADAS_TTC_MAX_RAW      0xFFFEu   /* najveća valjana vrijednost: 655.34 s */

#define ADAS_LATERAL_FACTOR   0.1f      /* 1 bit = 0.1 m */

#define ADAS_ALIVE_MASK       0x0Fu     /* alive counter: 4 bita, 0-15 */

typedef enum {
    ADAS_LEVEL_NONE    = 0,
    ADAS_LEVEL_CAUTION = 1,
    ADAS_LEVEL_WARNING = 2
} AdasLevel;

typedef enum {
    ADAS_OBJ_PERSON     = 0,
    ADAS_OBJ_BICYCLE    = 1,
    ADAS_OBJ_CAR        = 2,
    ADAS_OBJ_MOTORCYCLE = 3,
    ADAS_OBJ_BUS        = 4,
    ADAS_OBJ_TRUCK      = 5,
    ADAS_OBJ_NONE       = 15        /* nema relevantnog objekta */
} AdasObjectClass;

/* Sadržaj poruke u "ljudskom" obliku (prije pakiranja / nakon raspakiranja) */
typedef struct {
    uint8_t         alive_counter;     /* 0-15 */
    AdasLevel       level;
    bool            ttc_valid;         /* false = nema TTC-a */
    float           ttc_s;             /* vrijeme do sudara [s] */
    AdasObjectClass object_class;
    uint8_t         track_id;
    float           lateral_offset_m;  /* bočni pomak [m], + = desno */
} AdasWarningMsg;

/* Rezultat raspakiranja */
typedef enum {
    ADAS_UNPACK_OK = 0,
    ADAS_UNPACK_WRONG_ID,
    ADAS_UNPACK_WRONG_DLC,
    ADAS_UNPACK_CRC_ERROR
} AdasUnpackResult;

/* ===================== Funkcije ===================== */

/* CRC-8 SAE J1850 (koristi se u AUTOSAR E2E zaštiti) */
uint8_t crc8_sae_j1850(const uint8_t *data, size_t len);

/* Poruka -> CAN frame (8 bajtova) */
void adas_msg_pack(const AdasWarningMsg *msg, CanFrame *frame);

/* CAN frame -> poruka; provjerava ID, DLC i CRC */
AdasUnpackResult adas_msg_unpack(const CanFrame *frame, AdasWarningMsg *msg);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* ADAS_MSG_H */