/*
 * antplus_profiles.h - ANT+ device profile encode/decode.
 *
 * Each ANT+ device profile defines the meaning of the 8-byte broadcast page.
 * This module turns those 8 bytes into structured data (slave/receiver side)
 * and structured data back into 8 bytes (master/sensor side). It is pure data
 * manipulation, no I/O, and fully unit-tested.
 *
 * Implemented profiles:
 *   - Heart Rate Monitor (device type 120)
 *   - Bicycle Power, power-only page 0x10 (device type 11)
 *   - Bicycle Speed & Cadence, combined (device type 121)
 *   - Stride-based Speed and Distance Monitor, pages 1 and 2 (device type 124)
 *
 * References: "ANT+ Device Profile - Heart Rate Monitor",
 * "ANT+ Device Profile - Bicycle Power", "... Bicycle Speed and Cadence",
 * "... Stride Based Speed and Distance Monitor".
 */
#ifndef ANTPLUS_PROFILES_H
#define ANTPLUS_PROFILES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- ANT+ device type numbers. ---- */
#define ANTPLUS_DEVTYPE_HRM           120u
#define ANTPLUS_DEVTYPE_BIKE_POWER    11u
#define ANTPLUS_DEVTYPE_BIKE_SPDCAD   121u
#define ANTPLUS_DEVTYPE_BIKE_SPEED    123u
#define ANTPLUS_DEVTYPE_BIKE_CADENCE  122u
#define ANTPLUS_DEVTYPE_FITNESS_EQUIP 17u
#define ANTPLUS_DEVTYPE_STRIDE        124u

/* ---- Channel periods (1/32768 s counts) for each profile. ---- */
#define ANTPLUS_PERIOD_HRM            8070u   /* ~4.06 Hz */
#define ANTPLUS_PERIOD_BIKE_POWER     8182u   /* ~4.00 Hz */
#define ANTPLUS_PERIOD_BIKE_SPDCAD    8086u   /* ~4.05 Hz */
#define ANTPLUS_PERIOD_FITNESS_EQUIP  8192u   /* 4.00 Hz  */
#define ANTPLUS_PERIOD_STRIDE         8134u   /* ~4.03 Hz */

/* ============================ Heart Rate ============================ */
typedef struct {
    uint8_t  page_number;          /* 0..4 (data page), toggle bit stripped */
    uint8_t  computed_heart_rate;  /* bpm; 0 = invalid */
    uint16_t heart_beat_event_time;/* 1/1024 s, rolls over */
    uint8_t  heart_beat_count;     /* rolls over at 256 */
    bool     toggle;               /* page toggle bit (bit7 of byte0) */
    /* Optional page-specific fields (valid only for matching page_number). */
    uint32_t operating_time_s;     /* page 1: cumulative operating time (s) */
    uint8_t  manufacturer_id;      /* page 2 */
    uint16_t serial_number;        /* page 2 (lower 16 bits) */
    uint8_t  hw_version;           /* page 3 */
    uint8_t  sw_version;           /* page 3 */
    uint8_t  model_number;         /* page 3 */
    uint16_t prev_beat_event_time; /* page 4 */
} antplus_hrm_data_t;

/* Decode an 8-byte HRM page. Returns true on success. */
bool antplus_hrm_decode(const uint8_t page[8], antplus_hrm_data_t *out);

/*
 * Encode HRM page 0 (the common page every sensor broadcasts). `toggle` should
 * alternate every 4 messages per spec, but any value is accepted.
 */
void antplus_hrm_encode_page0(const antplus_hrm_data_t *in, bool toggle,
                              uint8_t page[8]);

/*
 * Compute instantaneous BPM from two consecutive beat-event-time samples.
 * Times are in 1/1024 s. Handles 16-bit rollover. Returns 0 if inputs are equal.
 */
uint16_t antplus_hrm_bpm_from_events(uint16_t t_prev, uint16_t t_now,
                                     uint8_t beats_between);

/* ====================== Stride Speed & Distance ====================== */
/*
 * Page 1 (main data page) carries time/distance/speed/strides/latency.
 * Pages 2-15 (supplemental, e.g. page 2) carry cadence/speed/status and,
 * on page 3 only, calories. A given broadcast is only ever one page, so
 * fields not populated by that page are left at their previous value in
 * *out (out is NOT cleared) - inspect out->page_number to know what's
 * fresh on this call.
 */
typedef struct {
    uint8_t  page_number;        /* 1, or 2..15 for supplemental pages */

    /* Page 1 only. */
    uint8_t  time_fractional;    /* 1/200 s */
    uint8_t  time_integer;       /* s, rolls over at 256 */
    uint8_t  distance_integer;   /* m, rolls over at 256 */
    uint8_t  distance_fractional;/* 1/16 m (0..15) */
    uint8_t  stride_count;       /* rolls over at 256 */
    uint8_t  update_latency;     /* 1/32 s */

    /* Pages 2-15. */
    uint8_t  cadence_integer;    /* strides/min */
    uint8_t  cadence_fractional; /* 1/16 strides/min (0..15) */
    uint8_t  status;             /* bits: location/battery/health/state */
    uint8_t  calories;           /* page 3 only, kcal */

    /* Common to page 1 and pages 2-15. */
    uint8_t  speed_integer;      /* m/s */
    uint8_t  speed_fractional;   /* 1/256 m/s */
} antplus_stride_data_t;

/*
 * Decode an 8-byte STRIDE (device type 124) page. Returns false only for
 * NULL args; unrecognized page numbers (0, 16-255) still populate
 * page_number and return true with the rest of *out left untouched, so
 * callers can filter on page_number themselves.
 */
bool antplus_stride_decode(const uint8_t page[8], antplus_stride_data_t *out);

/* Combine a page's integer/fractional pair into m/s, kph, or /min. */
static inline float antplus_stride_speed_mps(uint8_t speed_integer, uint8_t speed_fractional)
{
    return (float)speed_integer + (float)speed_fractional / 256.0f;
}
static inline float antplus_stride_cadence_spm(uint8_t cadence_integer, uint8_t cadence_fractional)
{
    return (float)cadence_integer + (float)(cadence_fractional & 0x0Fu) / 16.0f;
}

/* ============================ Bicycle Power ============================ */
typedef struct {
    uint8_t  event_count;          /* increments each power event */
    uint8_t  instantaneous_cadence;/* rpm; 0xFF = invalid */
    uint16_t accumulated_power;     /* watts, running sum, rolls over */
    uint16_t instantaneous_power;   /* watts */
    uint8_t  pedal_power;           /* % on right pedal, 0xFF = not used */
} antplus_power_data_t;

/* Decode standard power-only page 0x10. Returns false if not page 0x10. */
bool antplus_power_decode(const uint8_t page[8], antplus_power_data_t *out);

/* Encode standard power-only page 0x10. */
void antplus_power_encode(const antplus_power_data_t *in, uint8_t page[8]);

/*
 * Average power between two power events, per the ANT+ definition:
 *   avg = (accum_now - accum_prev) / (count_now - count_prev)
 * Handles 16-bit / 8-bit rollover. Returns 0 if no events elapsed.
 */
uint16_t antplus_power_average(uint16_t accum_prev, uint16_t accum_now,
                               uint8_t count_prev, uint8_t count_now);

/* ====================== Bicycle Speed & Cadence ====================== */
typedef struct {
    uint16_t cadence_event_time;   /* 1/1024 s */
    uint16_t cumulative_cadence;   /* pedal revolutions, rolls over */
    uint16_t speed_event_time;     /* 1/1024 s */
    uint16_t cumulative_speed;     /* wheel revolutions, rolls over */
} antplus_spdcad_data_t;

/* Decode combined speed & cadence page (no page number; all 8 bytes used). */
bool antplus_spdcad_decode(const uint8_t page[8], antplus_spdcad_data_t *out);
void antplus_spdcad_encode(const antplus_spdcad_data_t *in, uint8_t page[8]);

/* Cadence (rpm) from two cadence samples. wheel/crank event time in 1/1024 s. */
uint16_t antplus_spdcad_rpm(uint16_t t_prev, uint16_t t_now,
                            uint16_t rev_prev, uint16_t rev_now);

/* Speed (mm/s) from two speed samples given wheel circumference in mm. */
uint32_t antplus_spdcad_speed_mm_s(uint16_t t_prev, uint16_t t_now,
                                   uint16_t rev_prev, uint16_t rev_now,
                                   uint32_t wheel_circumference_mm);

#ifdef __cplusplus
}
#endif

#endif /* ANTPLUS_PROFILES_H */
