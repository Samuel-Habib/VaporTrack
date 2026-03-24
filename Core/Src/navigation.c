/*
 * navigation.c - autonomous navigation state machine
 *
 * The state machine progresses through:
 *   BOOT -> INIT_SENSORS -> CALIBRATION -> MAPPING -> LOCALIZATION -> TRACKING
 *
 * At any point, if an obstacle is detected, we transition to
 * OBSTACLE_AVOID, handle it, and return to whatever we were doing.
 *
 * Position estimation is dead-reckoning from heading + time at
 * known speed. Without wheel encoders this is approximate, but
 * it's good enough for spatial gas mapping purposes -- we're
 * looking for gradients over ~30cm scales, not millimeter accuracy.
 */

#include "navigation.h"
#include "motor.h"
#include <math.h>
#include <string.h>

#define DEG2RAD (3.14159265f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265f)

/* rough forward speed in m/s at cruise PWM -- measured empirically */
#define ESTIMATED_SPEED_MS  0.15f

static const char *state_names[] = {
    "BOOT",
    "INIT_SENSORS",
    "CALIBRATION",
    "MAPPING",
    "LOCALIZATION",
    "TRACKING",
    "AVOID",
    "SOURCE_FOUND",
    "ERROR",
};

const char *nav_state_name(nav_state_t s)
{
    if (s > NAV_STATE_ERROR)
        return "???";
    return state_names[s];
}

static void enter_state(nav_ctx_t *ctx, nav_state_t next, uint32_t now_ms)
{
    ctx->prev_state = ctx->state;
    ctx->state = next;
    ctx->state_entry_ms = now_ms;
}

/*
 * angle_diff: shortest signed angular difference in degrees.
 * result is in [-180, +180], positive = counterclockwise.
 */
static float angle_diff(float target, float current)
{
    float d = target - current;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/*
 * update dead-reckoning position based on elapsed time
 * and whether we're driving forward.
 */
static void update_position(nav_ctx_t *ctx, const nav_sensor_input_t *in,
                            float dt_s, bool driving)
{
    if (in->heading_valid)
        ctx->heading_deg = in->heading_deg;

    if (driving && dt_s > 0 && dt_s < 2.0f) {
        float rad = ctx->heading_deg * DEG2RAD;
        ctx->pos_x += ESTIMATED_SPEED_MS * dt_s * cosf(rad);
        ctx->pos_y += ESTIMATED_SPEED_MS * dt_s * sinf(rad);
    }
}

/* ------------------------------------------------------------------ */
/* obstacle check -- returns true if front is blocked                  */
/* ------------------------------------------------------------------ */
static bool front_blocked(const nav_sensor_input_t *in)
{
    return (in->dist_front_cm > 0 && in->dist_front_cm < NAV_OBSTACLE_STOP_CM);
}

static bool front_close(const nav_sensor_input_t *in)
{
    return (in->dist_front_cm > 0 && in->dist_front_cm < NAV_OBSTACLE_SLOW_CM);
}

/* ------------------------------------------------------------------ */
/* per-state handlers                                                  */
/* ------------------------------------------------------------------ */

static motor_cmd_t handle_boot(nav_ctx_t *ctx, const nav_sensor_input_t *in, uint32_t now_ms)
{
    (void)in;
    motor_cmd_t cmd = {0, 0};
    /* boot state does nothing; main.c transitions us to INIT_SENSORS */
    if (now_ms - ctx->state_entry_ms > 100)
        enter_state(ctx, NAV_STATE_INIT_SENSORS, now_ms);
    return cmd;
}

static motor_cmd_t handle_init(nav_ctx_t *ctx, const nav_sensor_input_t *in, uint32_t now_ms)
{
    (void)in;
    motor_cmd_t cmd = {0, 0};
    /* sensor init happens in the sensor tasks -- we just wait here.
     * after 3 seconds assume they're up or errored and move on. */
    if (now_ms - ctx->state_entry_ms > 3000)
        enter_state(ctx, NAV_STATE_CALIBRATION, now_ms);
    return cmd;
}

static motor_cmd_t handle_calibration(nav_ctx_t *ctx, const nav_sensor_input_t *in, uint32_t now_ms)
{
    motor_cmd_t cmd = {0, 0};

    /* wait for BNO055 calibration or timeout after 30 seconds */
    if (in->heading_valid || (now_ms - ctx->state_entry_ms > 30000)) {
        ctx->pos_x = 0;
        ctx->pos_y = 0;
        ctx->heading_deg = in->heading_valid ? in->heading_deg : 0;
        ctx->waypoint_count = 0;
        enter_state(ctx, NAV_STATE_MAPPING, now_ms);
    }

    return cmd;
}

static motor_cmd_t handle_mapping(nav_ctx_t *ctx, const nav_sensor_input_t *in, uint32_t now_ms)
{
    motor_cmd_t cmd = {0, 0};
    uint32_t elapsed = now_ms - ctx->last_measure_ms;

    /* check obstacles first */
    if (front_blocked(in)) {
        enter_state(ctx, NAV_STATE_OBSTACLE_AVOID, now_ms);
        return cmd;
    }

    /* alternate between driving and measuring */
    if (elapsed < NAV_MEASURE_DWELL_MS) {
        /* stopped, taking measurement */
        if (in->gas_valid && elapsed > 500) {
            map_sample_t s;
            s.x = ctx->pos_x;
            s.y = ctx->pos_y;
            s.heading = ctx->heading_deg;
            s.gas = in->gas;
            s.temperature = in->temperature;
            s.humidity = in->humidity;
            s.pressure = in->pressure;
            s.timestamp_ms = now_ms;

            map_add_sample(ctx->map, &s);
            ctx->waypoint_count++;
            ctx->last_measure_ms = now_ms;
        }
    } else if (elapsed < NAV_MEASURE_DWELL_MS + NAV_DRIVE_INTERVAL_MS) {
        /* driving forward */
        int16_t speed = front_close(in) ? NAV_SPEED_SLOW : NAV_SPEED_CRUISE;
        cmd.left  = speed;
        cmd.right = speed;
    } else {
        /* stop and start next measurement cycle */
        ctx->last_measure_ms = now_ms;
    }

    /* check if we've collected enough waypoints */
    if (ctx->waypoint_count >= NAV_MAP_WAYPOINTS) {
        enter_state(ctx, NAV_STATE_LOCALIZATION, now_ms);
    }

    return cmd;
}

static motor_cmd_t handle_localization(nav_ctx_t *ctx, const nav_sensor_input_t *in, uint32_t now_ms)
{
    motor_cmd_t cmd = {0, 0};

    if (front_blocked(in)) {
        enter_state(ctx, NAV_STATE_OBSTACLE_AVOID, now_ms);
        return cmd;
    }

    /* compute gradient at current position */
    gas_gradient_t grad = map_estimate_gradient(ctx->map, ctx->pos_x, ctx->pos_y, 1.0f);
    ctx->map->gradient = grad;

    if (!grad.valid) {
        /* not enough local data -- keep mapping */
        uint32_t elapsed = now_ms - ctx->last_measure_ms;
        if (elapsed > NAV_MEASURE_DWELL_MS + NAV_DRIVE_INTERVAL_MS) {
            ctx->last_measure_ms = now_ms;
        } else if (elapsed > NAV_MEASURE_DWELL_MS) {
            cmd.left  = NAV_SPEED_SLOW;
            cmd.right = NAV_SPEED_SLOW;
        } else if (in->gas_valid) {
            map_sample_t s;
            s.x = ctx->pos_x;
            s.y = ctx->pos_y;
            s.heading = ctx->heading_deg;
            s.gas = in->gas;
            s.temperature = in->temperature;
            s.humidity = in->humidity;
            s.pressure = in->pressure;
            s.timestamp_ms = now_ms;
            map_add_sample(ctx->map, &s);
        }
        return cmd;
    }

    /* gradient is valid -- transition to tracking */
    enter_state(ctx, NAV_STATE_TRACKING, now_ms);
    return cmd;
}

static motor_cmd_t handle_tracking(nav_ctx_t *ctx, const nav_sensor_input_t *in, uint32_t now_ms)
{
    motor_cmd_t cmd = {0, 0};

    if (front_blocked(in)) {
        enter_state(ctx, NAV_STATE_OBSTACLE_AVOID, now_ms);
        return cmd;
    }

    /* periodically re-sample and update gradient */
    uint32_t elapsed = now_ms - ctx->last_measure_ms;

    if (elapsed < NAV_MEASURE_DWELL_MS) {
        /* dwelling -- take a measurement */
        if (in->gas_valid && elapsed > 500) {
            map_sample_t s;
            s.x = ctx->pos_x;
            s.y = ctx->pos_y;
            s.heading = ctx->heading_deg;
            s.gas = in->gas;
            s.temperature = in->temperature;
            s.humidity = in->humidity;
            s.pressure = in->pressure;
            s.timestamp_ms = now_ms;
            map_add_sample(ctx->map, &s);

            /* update gradient */
            ctx->map->gradient = map_estimate_gradient(
                ctx->map, ctx->pos_x, ctx->pos_y, 1.0f);

            ctx->last_measure_ms = now_ms;
        }
        return cmd;
    }

    /* driving phase -- steer toward gradient direction */
    gas_gradient_t *g = &ctx->map->gradient;

    if (!g->valid) {
        /* lost gradient -- go back to localization */
        enter_state(ctx, NAV_STATE_LOCALIZATION, now_ms);
        return cmd;
    }

    /* gradient magnitude very small means we're probably at the source */
    if (g->magnitude < 0.5f && ctx->waypoint_count > NAV_MAP_WAYPOINTS + 10) {
        enter_state(ctx, NAV_STATE_SOURCE_FOUND, now_ms);
        return cmd;
    }

    /* steer toward gradient direction */
    float heading_err = angle_diff(g->direction_deg, ctx->heading_deg);

    if (fabsf(heading_err) > 30.0f) {
        /* need a significant turn */
        int16_t turn_speed = NAV_SPEED_TURN;
        if (heading_err > 0) {
            cmd.left  = -turn_speed;
            cmd.right = turn_speed;
        } else {
            cmd.left  = turn_speed;
            cmd.right = -turn_speed;
        }
    } else {
        /* mostly forward with proportional steering correction */
        int16_t base = front_close(in) ? NAV_SPEED_SLOW : NAV_SPEED_CRUISE;
        int16_t steer = (int16_t)(heading_err * 30.0f); /* proportional gain */
        cmd.left  = base - steer;
        cmd.right = base + steer;

        /* clamp */
        if (cmd.left  < 0) cmd.left  = 0;
        if (cmd.right < 0) cmd.right = 0;
        if (cmd.left  > MOTOR_PWM_MAX) cmd.left  = MOTOR_PWM_MAX;
        if (cmd.right > MOTOR_PWM_MAX) cmd.right = MOTOR_PWM_MAX;
    }

    /* reset dwell timer after driving interval */
    if (elapsed > NAV_MEASURE_DWELL_MS + NAV_DRIVE_INTERVAL_MS)
        ctx->last_measure_ms = now_ms;

    return cmd;
}

static motor_cmd_t handle_obstacle_avoid(nav_ctx_t *ctx, const nav_sensor_input_t *in, uint32_t now_ms)
{
    motor_cmd_t cmd = {0, 0};
    uint32_t elapsed = now_ms - ctx->state_entry_ms;

    if (elapsed < 200) {
        /* brief reverse */
        cmd.left  = -NAV_SPEED_SLOW;
        cmd.right = -NAV_SPEED_SLOW;
        return cmd;
    }

    /* pick the clearer side and rotate toward it */
    if (elapsed < 200 + 800) {
        if (in->dist_left_cm > in->dist_right_cm) {
            /* turn left */
            cmd.left  = -NAV_SPEED_TURN;
            cmd.right = NAV_SPEED_TURN;
        } else {
            /* turn right */
            cmd.left  = NAV_SPEED_TURN;
            cmd.right = -NAV_SPEED_TURN;
        }
        return cmd;
    }

    /* check if front is now clear */
    if (!front_blocked(in) || elapsed > 3000) {
        /* return to previous state */
        nav_state_t return_to = ctx->prev_state;
        if (return_to == NAV_STATE_OBSTACLE_AVOID || return_to == NAV_STATE_BOOT)
            return_to = NAV_STATE_MAPPING;
        enter_state(ctx, return_to, now_ms);
        ctx->last_measure_ms = now_ms;
    } else {
        /* keep turning */
        if (in->dist_left_cm > in->dist_right_cm) {
            cmd.left  = -NAV_SPEED_TURN;
            cmd.right = NAV_SPEED_TURN;
        } else {
            cmd.left  = NAV_SPEED_TURN;
            cmd.right = -NAV_SPEED_TURN;
        }
    }

    return cmd;
}

static motor_cmd_t handle_source_found(nav_ctx_t *ctx, const nav_sensor_input_t *in, uint32_t now_ms)
{
    (void)ctx;
    (void)in;
    (void)now_ms;
    /* stop and celebrate */
    motor_cmd_t cmd = {0, 0};
    return cmd;
}

static motor_cmd_t handle_error(nav_ctx_t *ctx, const nav_sensor_input_t *in, uint32_t now_ms)
{
    (void)ctx;
    (void)in;
    (void)now_ms;
    motor_cmd_t cmd = {0, 0};
    return cmd;
}

/* ------------------------------------------------------------------ */
/* public interface                                                    */
/* ------------------------------------------------------------------ */

void nav_init(nav_ctx_t *ctx, gas_map_t *map)
{
    memset(ctx, 0, sizeof(nav_ctx_t));
    ctx->state = NAV_STATE_BOOT;
    ctx->map = map;
}

motor_cmd_t nav_update(nav_ctx_t *ctx, const nav_sensor_input_t *input, uint32_t now_ms)
{
    /* update dead-reckoning */
    float dt = 0;
    if (ctx->state_entry_ms > 0) {
        uint32_t elapsed = now_ms - ctx->state_entry_ms;
        dt = (float)elapsed / 1000.0f;
        /* cap to avoid huge jumps on first call */
        if (dt > 1.0f) dt = 0;
    }

    bool driving = (ctx->cmd.left > 0 || ctx->cmd.right > 0);
    update_position(ctx, input, dt, driving);

    motor_cmd_t cmd;

    switch (ctx->state) {
    case NAV_STATE_BOOT:           cmd = handle_boot(ctx, input, now_ms);           break;
    case NAV_STATE_INIT_SENSORS:   cmd = handle_init(ctx, input, now_ms);           break;
    case NAV_STATE_CALIBRATION:    cmd = handle_calibration(ctx, input, now_ms);    break;
    case NAV_STATE_MAPPING:        cmd = handle_mapping(ctx, input, now_ms);        break;
    case NAV_STATE_LOCALIZATION:   cmd = handle_localization(ctx, input, now_ms);   break;
    case NAV_STATE_TRACKING:       cmd = handle_tracking(ctx, input, now_ms);       break;
    case NAV_STATE_OBSTACLE_AVOID: cmd = handle_obstacle_avoid(ctx, input, now_ms); break;
    case NAV_STATE_SOURCE_FOUND:   cmd = handle_source_found(ctx, input, now_ms);   break;
    default:                       cmd = handle_error(ctx, input, now_ms);          break;
    }

    ctx->cmd = cmd;
    return cmd;
}
