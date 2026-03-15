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

gas_gradient_t map_estimate_gradient(gas_map_t *map, float x, float y, float radius)
{
    gas_gradient_t grad = {0};
    grad.valid = false;

    if (map->count < 3)
        return grad;

    float r2 = radius * radius;

    /*
     * accumulate sums for least-squares fit of:
     *   gas = a + b*dx + c*dy
     *
     * normal equations (ignoring the constant term by centering):
     *   [sum_dxdx  sum_dxdy] [b]   [sum_dxg]
     *   [sum_dxdy  sum_dydy] [c] = [sum_dyg]
     */
    float sum_dxdx = 0, sum_dxdy = 0, sum_dydy = 0;
    float sum_dxg = 0, sum_dyg = 0;
    int n = 0;

    for (uint16_t i = 0; i < map->count; i++) {
        float dx = map->samples[i].x - x;
        float dy = map->samples[i].y - y;
        float d2 = dx * dx + dy * dy;

        if (d2 > r2)
            continue;

        float g = map->samples[i].gas;

        sum_dxdx += dx * dx;
        sum_dxdy += dx * dy;
        sum_dydy += dy * dy;
        sum_dxg  += dx * g;
        sum_dyg  += dy * g;
        n++;
    }

    /* need at least 3 samples in the neighborhood */
    if (n < 3)
        return grad;

    /* solve 2x2 system using Cramer's rule */
    float det = sum_dxdx * sum_dydy - sum_dxdy * sum_dxdy;

    /* degenerate -- samples are collinear or too clustered */
    if (fabsf(det) < 1e-6f)
        return grad;

    float b = (sum_dxg * sum_dydy - sum_dyg * sum_dxdy) / det;
    float c = (sum_dxdx * sum_dyg - sum_dxdy * sum_dxg) / det;

    grad.dg_dx = b;
    grad.dg_dy = c;
    grad.magnitude = sqrtf(b * b + c * c);
    grad.direction_deg = atan2f(c, b) * (180.0f / 3.14159265f);

    /* normalize to 0..360 */
    if (grad.direction_deg < 0)
        grad.direction_deg += 360.0f;

    grad.valid = (grad.magnitude > 0.01f);
    return grad;
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
