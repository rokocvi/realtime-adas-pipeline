#include "adas_msg.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* ===================== Mini testni okvir ===================== */

static int g_checks   = 0;
static int g_failures = 0;

/* Provjeri uvjet; ako ne vrijedi, ispiši gdje i što je palo */
#define CHECK(cond)                                                       \
    do {                                                                  \
        ++g_checks;                                                       \
        if (!(cond)) {                                                    \
            ++g_failures;                                                 \
            printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
        }                                                                 \
    } while (0)

static bool near_equal(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

static void print_frame(const CanFrame *f)
{
    printf("  ID 0x%03X  DLC %u  DATA:", (unsigned)f->id, (unsigned)f->dlc);
    for (unsigned i = 0u; i < (unsigned)f->dlc; ++i) {
        printf(" %02X", (unsigned)f->data[i]);
    }
    printf("\n");
}

/* Poruka iz primjera: TTC 1.83 s, WARNING, auto #12, -0.4 m */
static AdasWarningMsg make_example(void)
{
    AdasWarningMsg m;
    (void)memset(&m, 0, sizeof(m));
    m.alive_counter    = 5u;
    m.level            = ADAS_LEVEL_WARNING;
    m.ttc_valid        = true;
    m.ttc_s            = 1.83f;
    m.object_class     = ADAS_OBJ_CAR;
    m.track_id         = 12u;
    m.lateral_offset_m = -0.4f;
    return m;
}

/* ========================== Testovi ========================== */

static void test_crc_check_value(void)
{
    printf("[test] CRC-8 SAE J1850 check value\n");
    const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    const uint8_t crc = crc8_sae_j1850(text, sizeof(text));
    printf("  CRC(\"123456789\") = 0x%02X (expected 0x4B)\n", (unsigned)crc);
    CHECK(crc == 0x4Bu);
}

static void test_pack_example(void)
{
    printf("[test] pack example message\n");
    const AdasWarningMsg msg = make_example();
    CanFrame frame;
    adas_msg_pack(&msg, &frame);
    print_frame(&frame);

    CHECK(frame.id == ADAS_MSG_ID);
    CHECK(frame.dlc == 8u);
    CHECK(frame.data[1] == 0x25u);   /* alive 5, level 2 */
    CHECK(frame.data[2] == 0xB7u);   /* TTC 183, niži bajt */
    CHECK(frame.data[3] == 0x00u);   /* TTC 183, viši bajt */
    CHECK(frame.data[4] == 0x02u);   /* CAR */
    CHECK(frame.data[5] == 0x0Cu);   /* track 12 */
    CHECK(frame.data[6] == 0xFCu);   /* -4 = -0.4 m */
    CHECK(frame.data[7] == 0x00u);   /* rezerva */
    CHECK(frame.data[0] == crc8_sae_j1850(&frame.data[1], 7u));
}

static void test_round_trip(void)
{
    printf("[test] pack -> unpack gives the same message\n");
    const AdasWarningMsg in = make_example();
    CanFrame frame;
    AdasWarningMsg out;

    adas_msg_pack(&in, &frame);
    CHECK(adas_msg_unpack(&frame, &out) == ADAS_UNPACK_OK);

    CHECK(out.alive_counter == in.alive_counter);
    CHECK(out.level == in.level);
    CHECK(out.ttc_valid);
    CHECK(near_equal(out.ttc_s, in.ttc_s, 0.005f));
    CHECK(out.object_class == in.object_class);
    CHECK(out.track_id == in.track_id);
    CHECK(near_equal(out.lateral_offset_m, in.lateral_offset_m, 0.05f));
}

static void test_corrupted_frame_rejected(void)
{
    printf("[test] corrupted / wrong frames are rejected\n");
    const AdasWarningMsg in = make_example();
    CanFrame frame;
    AdasWarningMsg out;

    adas_msg_pack(&in, &frame);
    frame.data[2] ^= 0x01u;                    /* jedan bit se pokvario putem */
    CHECK(adas_msg_unpack(&frame, &out) == ADAS_UNPACK_CRC_ERROR);

    adas_msg_pack(&in, &frame);
    frame.id = 0x121u;                         /* tuđa poruka */
    CHECK(adas_msg_unpack(&frame, &out) == ADAS_UNPACK_WRONG_ID);

    adas_msg_pack(&in, &frame);
    frame.dlc = 6u;                            /* kriva duljina */
    CHECK(adas_msg_unpack(&frame, &out) == ADAS_UNPACK_WRONG_DLC);
}

static void test_special_values(void)
{
    printf("[test] invalid TTC and saturation\n");
    AdasWarningMsg in = make_example();
    CanFrame frame;
    AdasWarningMsg out;

    /* nema TTC-a -> 0xFFFF */
    in.ttc_valid = false;
    adas_msg_pack(&in, &frame);
    CHECK((frame.data[2] == 0xFFu) && (frame.data[3] == 0xFFu));
    CHECK(adas_msg_unpack(&frame, &out) == ADAS_UNPACK_OK);
    CHECK(!out.ttc_valid);

    /* prevelike vrijednosti -> zasićenje na maksimum */
    in.ttc_valid        = true;
    in.ttc_s            = 1000.0f;            /* > 655.34 s */
    in.lateral_offset_m = 20.0f;              /* > 12.7 m */
    adas_msg_pack(&in, &frame);
    CHECK((frame.data[2] == 0xFEu) && (frame.data[3] == 0xFFu));   /* 0xFFFE */
    CHECK(frame.data[6] == 0x7Fu);                                /* +127 */

    in.lateral_offset_m = -20.0f;             /* < -12.8 m */
    adas_msg_pack(&in, &frame);
    CHECK(frame.data[6] == 0x80u);                                /* -128 */
}

/* =========================== main =========================== */

int main(void)
{
    test_crc_check_value();
    test_pack_example();
    test_round_trip();
    test_corrupted_frame_rejected();
    test_special_values();

    printf("\n%d checks, %d failed\n", g_checks, g_failures);
    if (g_failures == 0) {
        printf("ALL TESTS PASSED\n");
    }
    return (g_failures == 0) ? 0 : 1;
}