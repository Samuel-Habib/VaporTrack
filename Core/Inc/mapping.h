/*
 * mapping.h - spatial gas map and gradient estimation
 *
 * The map is a flat array of measurement samples stored in
 * recording order. Each sample ties a gas reading to an (x,y)
 * position so we can compute spatial derivatives later.
 *
 * Grid size is limited by available SRAM -- on the F446RE we
 * have 128K total, and FreeRTOS + drivers already eat a good
 * chunk. 256 samples at ~28 bytes each uses about 7K which
 * leaves plenty of room.
 */

#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>
#include <stdbool.h>

#define MAP_MAX_SAMPLES     256

typedef struct {
    float    x;
    float    y;
    float    heading;
    float    gas;
    float    temperature;
    float    humidity;
    float    pressure;
    uint32_t timestamp_ms;
} map_sample_t;

/* 2D gradient vector */
typedef struct {
    float dg_dx;
    float dg_dy;
    float magnitude;
    float direction_deg;    /* 0=+x, 90=+y, etc */
    bool  valid;
} gas_gradient_t;

typedef struct {
    map_sample_t  samples[MAP_MAX_SAMPLES];
    uint16_t      count;
    gas_gradient_t gradient;
} gas_map_t;

void            map_init(gas_map_t *map);
bool            map_add_sample(gas_map_t *map, const map_sample_t *s);
map_sample_t   *map_nearest(gas_map_t *map, float x, float y);
float           map_peak_gas(const gas_map_t *map, float *out_x, float *out_y);

#endif /* MAPPING_H */
