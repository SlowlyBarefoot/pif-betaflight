#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "platform.h"

#if defined(USE_DSHOT_TELEMETRY)

#include "common/maths.h"
#include "common/utils.h"
#include "drivers/dshot.h"
#include "drivers/dshot_bitbang_decode.h"

#define MIN_VALID_BBSAMPLES ((21 - 2) * 3)
#define MAX_VALID_BBSAMPLES ((21 + 2) * 3)

/* Bit band SRAM definitions */
#define BITBAND_SRAM_REF   0x20000000
#define BITBAND_SRAM_BASE  0x22000000
#define BITBAND_SRAM(a,b) ((BITBAND_SRAM_BASE + (((a)-BITBAND_SRAM_REF)<<5) + ((b)<<2)))  // Convert SRAM address

typedef struct bitBandWord_s {
    uint32_t value;
    uint32_t junk[15];
} bitBandWord_t;

// The same as pifDshot_SamplesToGcr(), with 3 samples per bit, but reading the
// pin through the bit band alias of the sample buffer, a word per sample,
// which saves the masking on Cortex-M3/M4. What it gives is decoded by PIF
// (dshotTelemetryReceive()) like any other GCR frame.
uint32_t decode_bb_bitband( uint16_t buffer[], uint32_t count, uint32_t bit)
{
    uint32_t value = 0;

    bitBandWord_t* p = (bitBandWord_t*)BITBAND_SRAM((uint32_t)buffer, bit);
    bitBandWord_t* b = p;
    bitBandWord_t* endP = p + (count - MIN_VALID_BBSAMPLES);

    // Eliminate leading high signal level by looking for first zero bit in data stream.
    // Manual loop unrolling and branch hinting to produce faster code.
    while (p < endP) {
        if (__builtin_expect((!(p++)->value), 0) ||
            __builtin_expect((!(p++)->value), 0) ||
            __builtin_expect((!(p++)->value), 0) ||
            __builtin_expect((!(p++)->value), 0)) {
            break;
        }
    }

    if (p >= endP) {
        // not returning telemetry is ok if the esc cpu is
        // overburdened.  in that case no edge will be found and
        // PIF_DSHOT_GCR_NONE indicates the condition to caller
        return PIF_DSHOT_GCR_NONE;
    }

    int remaining = MIN(count - (p - b), (unsigned int)MAX_VALID_BBSAMPLES);

    bitBandWord_t* oldP = p;
    uint32_t bits = 0;
    endP = p + remaining;


    while (endP > p) {
        do {
            // Look for next positive edge. Manual loop unrolling and branch hinting to produce faster code.
            if(__builtin_expect((p++)->value, 0) ||
               __builtin_expect((p++)->value, 0) ||
               __builtin_expect((p++)->value, 0) ||
               __builtin_expect((p++)->value, 0)) {
                break;
            }
        } while (endP > p);

        if (endP > p) {

            // A level of length n gets decoded to a sequence of bits of
            // the form 1000 with a length of (n+1) / 3 to account for 3x
            // oversampling.
            const int len = MAX((p - oldP + 1) / 3, 1);
            bits += len;
            value <<= len;
            value |= 1 << (len - 1);
            oldP = p;

            // Look for next zero edge. Manual loop unrolling and branch hinting to produce faster code.
            do {
                if (__builtin_expect(!(p++)->value, 0) ||
                    __builtin_expect(!(p++)->value, 0) ||
                    __builtin_expect(!(p++)->value, 0) ||
                    __builtin_expect(!(p++)->value, 0)) {
                    break;
                }
            } while (endP > p);

            if (endP > p) {

                // A level of length n gets decoded to a sequence of bits of
                // the form 1000 with a length of (n+1) / 3 to account for 3x
                // oversampling.
                const int len = MAX((p - oldP + 1) / 3, 1);
                bits += len;
                value <<= len;
                value |= 1 << (len - 1);
                oldP = p;
            }
        }
    }

    if (bits < 18) {
        return PIF_DSHOT_GCR_NONE;
    }

    // length of last sequence has to be inferred since the last bit with inverted dshot is high
    const int nlen = 21 - bits;
    if (nlen < 0) {
        return PIF_DSHOT_GCR_INVALID;
    }

    if (nlen > 0) {
        value <<= nlen;
        value |= 1 << (nlen - 1);
    }
    return value;
}

#endif
