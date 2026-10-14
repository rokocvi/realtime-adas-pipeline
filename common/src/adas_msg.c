#include "adas_msg.h"

#include <string.h>

/* ============ Pomoćne funkcije (vidljive samo u ovoj datoteci) ============ */

/* TTC u sekundama -> sirova 16-bitna vrijednost (1 bit = 0.01 s) */
static uint16_t encode_ttc(bool valid, float ttc_s)
{
    if ((!valid) || (ttc_s < 0.0f)) {
        return (uint16_t)ADAS_TTC_INVALID;
    }

    const float scaled = (ttc_s / ADAS_TTC_FACTOR) + 0.5f;   /* +0.5 = zaokruživanje */
    if (scaled >= (float)ADAS_TTC_MAX_RAW) {
        return (uint16_t)ADAS_TTC_MAX_RAW;                    /* zasićenje na maksimum */
    }
    return (uint16_t)scaled;
}

/* Bočni pomak u metrima -> sirova 8-bitna vrijednost s predznakom (1 bit = 0.1 m) */
static int8_t encode_lateral(float offset_m)
{
    float scaled = offset_m / ADAS_LATERAL_FACTOR;

    /* zaokruži na najbliži cijeli broj (i za negativne brojeve) */
    scaled = (scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f);

    if (scaled > 127.0f) {
        return (int8_t)127;
    }
    if (scaled < -128.0f) {
        return (int8_t)-128;
    }
    return (int8_t)scaled;
}

/* ============================ CRC-8 SAE J1850 ============================ */

uint8_t crc8_sae_j1850(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFFu;                       /* početna vrijednost */

    for (size_t i = 0u; i < len; ++i) {
        crc ^= data[i];                        /* "ubaci" sljedeći bajt */

        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            if ((crc & 0x80u) != 0u) {         /* je li najviši bit 1? */
                crc = (uint8_t)((uint8_t)(crc << 1u) ^ 0x1Du);
            } else {
                crc = (uint8_t)(crc << 1u);
            }
        }
    }
    return (uint8_t)(crc ^ 0xFFu);             /* završni XOR */
}

/* ========================= Pakiranje: msg -> frame ========================= */

void adas_msg_pack(const AdasWarningMsg *msg, CanFrame *frame)
{
    const uint16_t ttc_raw     = encode_ttc(msg->ttc_valid, msg->ttc_s);
    const int8_t   lateral_raw = encode_lateral(msg->lateral_offset_m);

    (void)memset(frame, 0, sizeof(*frame));    /* sve na nulu (i rezervirani bitovi) */
    frame->id  = ADAS_MSG_ID;
    frame->dlc = (uint8_t)ADAS_MSG_DLC;

    /* bajt 1: bitovi 0-3 = alive counter, bitovi 4-5 = warning level */
    frame->data[1] = (uint8_t)((msg->alive_counter & ADAS_ALIVE_MASK) |
                               (((uint8_t)msg->level & 0x03u) << 4u));

    /* bajtovi 2-3: TTC, little endian (niži bajt prvi) */
    frame->data[2] = (uint8_t)(ttc_raw & 0xFFu);
    frame->data[3] = (uint8_t)(ttc_raw >> 8u);

    /* bajt 4: klasa objekta (donja 4 bita) */
    frame->data[4] = (uint8_t)((uint8_t)msg->object_class & 0x0Fu);

    /* bajt 5: track ID */
    frame->data[5] = msg->track_id;

    /* bajt 6: bočni pomak, int8 u dvojnom komplementu */
    frame->data[6] = (uint8_t)lateral_raw;

    /* bajt 7: rezerva, ostaje 0 */

    /* bajt 0: CRC preko bajtova 1-7, računa se ZADNJI */
    frame->data[0] = crc8_sae_j1850(&frame->data[1], 7u);
}

/* ======================= Raspakiranje: frame -> msg ======================= */

AdasUnpackResult adas_msg_unpack(const CanFrame *frame, AdasWarningMsg *msg)
{
    /* 1) Provjere: je li ovo uopće naša poruka i je li ispravna? */
    if (frame->id != ADAS_MSG_ID) {
        return ADAS_UNPACK_WRONG_ID;
    }
    if (frame->dlc != ADAS_MSG_DLC) {
        return ADAS_UNPACK_WRONG_DLC;
    }
    if (crc8_sae_j1850(&frame->data[1], 7u) != frame->data[0]) {
        return ADAS_UNPACK_CRC_ERROR;
    }

    /* 2) Izvuci signale (obrnuto od pakiranja) */
    msg->alive_counter = (uint8_t)(frame->data[1] & ADAS_ALIVE_MASK);
    msg->level         = (AdasLevel)((frame->data[1] >> 4u) & 0x03u);

    const uint16_t ttc_raw = (uint16_t)((uint16_t)frame->data[2] |
                                        (uint16_t)((uint16_t)frame->data[3] << 8u));
    msg->ttc_valid = (ttc_raw != ADAS_TTC_INVALID);
    msg->ttc_s     = msg->ttc_valid ? ((float)ttc_raw * ADAS_TTC_FACTOR) : 0.0f;

    msg->object_class     = (AdasObjectClass)(frame->data[4] & 0x0Fu);
    msg->track_id         = frame->data[5];
    msg->lateral_offset_m = (float)(int8_t)frame->data[6] * ADAS_LATERAL_FACTOR;

    return ADAS_UNPACK_OK;
}