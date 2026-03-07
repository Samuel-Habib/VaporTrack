/*
 * mapping.c - spatial gas mapping and gradient estimation
 *
 * The gradient calculation uses a weighted least-squares fit
 * of gas values to position within a local neighborhood.
 * This is more robust than a simple two-point finite difference
 * because it uses all nearby samples and gracefully handles
 * irregular spacing (since we don't have precise odometry,
 * the samples won't be on a perfect grid).
 *
 * The math:
 *   For samples within radius R of query point (x0, y0):
 *     dx_i = x_i - x0
 *     dy_i = y_i - y0
 *     g_i  = gas_i (conductance in uS, inversely proportional to MOX resistance)
 *
 *   We fit g = a + b*dx + c*dy using least squares.
 *   Then dG/dx = b, dG/dy = c.
 *
 *   Since MOX sensor resistance drops with higher VOC concentration,
 *   using conductance (1/R_gas) ensures that gradient direction = atan2(c, b)
 *   points directly toward increasing gas concentration (the plume source).
 */

#include "mapping.h"
#include <math.h>
#include <string.h>

void map_init(gas_map_t *map)
{
    memset(map, 0, sizeof(gas_map_t));
}

bool map_add_sample(gas_map_t *map, const map_sample_t *s)
{
    if (map->count >= MAP_MAX_SAMPLES)
        return false;

    map->samples[map->count] = *s;
    map->count++;
    return true;
}

map_sample_t *map_nearest(gas_map_t *map, float x, float y)
{
    if (map->count == 0)
        return NULL;

    float best_dist = 1e9f;
    uint16_t best_idx = 0;

    for (uint16_t i = 0; i < map->count; i++) {
        float dx = map->samples[i].x - x;
        float dy = map->samples[i].y - y;
        float d2 = dx * dx + dy * dy;
        if (d2 < best_dist) {
            best_dist = d2;
            best_idx = i;
        }
    }

    return &map->samples[best_idx];
}

float map_peak_gas(const gas_map_t *map, float *out_x, float *out_y)
{
    if (map->count == 0) {
        if (out_x) *out_x = 0;
        if (out_y) *out_y = 0;
        return 0;
    }

    float best = map->samples[0].gas;
    uint16_t best_idx = 0;

    for (uint16_t i = 1; i < map->count; i++) {
        if (map->samples[i].gas > best) {
            best = map->samples[i].gas;
            best_idx = i;
        }
    }

    if (out_x) *out_x = map->samples[best_idx].x;
    if (out_y) *out_y = map->samples[best_idx].y;

    return best;
}
