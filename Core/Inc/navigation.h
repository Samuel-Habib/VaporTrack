/*
 * navigation.h - autonomous navigation state machine
 *
 * This is the brain of the rover. It consumes sensor data,
 * manages the operational state machine (boot -> calibration ->
 * mapping -> localization -> tracking), handles obstacle
 * avoidance, and issues motor commands.
 *
 * Obstacle avoidance always takes priority over gas gradient
 * following -- the rover shouldn't drive into walls regardless
 * of how interesting the gas reading is.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "mapping.h"
#include <stdint.h>
#include <stdbool.h>

/* distance thresholds in cm */
#define NAV_OBSTACLE_STOP_CM    20.0f
#define NAV_OBSTACLE_SLOW_CM    40.0f
#define NAV_CLEAR_CM            60.0f

/* base motor speeds (PWM counts, max 8999) */
#define NAV_SPEED_CRUISE        5400    /* ~60% duty */
#define NAV_SPEED_SLOW          3600    /* ~40% duty */
#define NAV_SPEED_TURN          4500    /* ~50% duty */

/* mapping grid step in estimated meters */
#define NAV_MAP_STEP_M          0.30f

/* how long to drive forward between measurement stops (ms) */
#define NAV_DRIVE_INTERVAL_MS   1500
#define NAV_MEASURE_DWELL_MS    2000

/* maximum number of mapping waypoints before switching to localization */
#define NAV_MAP_WAYPOINTS       64

typedef enum {
    NAV_STATE_BOOT = 0,
    NAV_STATE_INIT_SENSORS,
    NAV_STATE_CALIBRATION,
    NAV_STATE_MAPPING,
    NAV_STATE_LOCALIZATION,
    NAV_STATE_TRACKING,
    NAV_STATE_OBSTACLE_AVOID,
    NAV_STATE_SOURCE_FOUND,
    NAV_STATE_ERROR,
} nav_state_t;

/* motor command output from nav controller */
typedef struct {
    int16_t left;
    int16_t right;
} motor_cmd_t;

/* full rover state snapshot consumed by navigation */
typedef struct {
    float gas;
    float temperature;
    float humidity;
    float pressure;
    bool  gas_valid;

    float heading_deg;
    bool  heading_valid;

    float dist_front_cm;
    float dist_left_cm;
    float dist_right_cm;

    float pos_x;
    float pos_y;
} nav_sensor_input_t;

typedef struct {
    nav_state_t         state;
    nav_state_t         prev_state;     /* for returning after obstacle avoidance */
    gas_map_t          *map;

    /* dead-reckoning position estimate */
    float               pos_x;
    float               pos_y;
    float               heading_deg;

    /* timing */
    uint32_t            state_entry_ms;
    uint32_t            last_measure_ms;
    uint32_t            waypoint_count;

    /* obstacle avoidance memory */
    float               avoid_target_heading;

    /* output */
    motor_cmd_t         cmd;
} nav_ctx_t;

void        nav_init(nav_ctx_t *ctx, gas_map_t *map);
motor_cmd_t nav_update(nav_ctx_t *ctx, const nav_sensor_input_t *input, uint32_t now_ms);
const char *nav_state_name(nav_state_t s);

#endif /* NAVIGATION_H */
