#ifndef SAILING_NAVIGATION_H
#define SAILING_NAVIGATION_H

#include "sailing/infrared_link.h"
#include "sailing/jy61.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SAILING_NAV_DISARMED = 0,
    SAILING_NAV_SEARCH,
    SAILING_NAV_TRACK,
    SAILING_NAV_PASS_THROUGH,
    SAILING_NAV_COMPLETE,
    SAILING_NAV_FAULT,
    SAILING_NAV_EMERGENCY_STOP
} sailing_navigation_state_t;

typedef struct {
    float minimum_total_strength;
    float minimum_confidence;
    float centered_angle_deg;
    float approach_strength;
    float search_turn;
    float track_throttle;
    float turn_throttle;
    float pass_throttle;
    float steering_kp;
    float yaw_rate_kd;
    float maximum_steering;
    float motor_deadband;
    float left_trim;
    float right_trim;
    uint32_t infrared_timeout_ms;
    uint32_t imu_timeout_ms;
    uint32_t pass_loss_ms;
    uint32_t pass_forward_ms;
    uint32_t mission_timeout_ms;
} sailing_navigation_config_t;

typedef struct {
    float angle_deg;
    float confidence;
    float total_strength;
    bool valid;
} sailing_bearing_estimate_t;

typedef struct {
    float left;
    float right;
} sailing_drive_command_t;

typedef struct {
    sailing_navigation_config_t config;
    sailing_navigation_state_t state;
    sailing_bearing_estimate_t bearing;
    uint64_t started_ms;
    uint64_t last_seen_ms;
    uint64_t pass_started_ms;
    uint8_t passed_gates;
    bool approached;
} sailing_navigator_t;

sailing_navigation_config_t sailing_navigation_default_config(void);

bool sailing_estimate_bearing(
    const sailing_infrared_sample_t *sample,
    sailing_bearing_estimate_t *estimate);

bool sailing_navigator_init(
    sailing_navigator_t *navigator,
    const sailing_navigation_config_t *config);

void sailing_navigator_arm(sailing_navigator_t *navigator, uint64_t now_ms);
void sailing_navigator_disarm(sailing_navigator_t *navigator);
void sailing_navigator_emergency_stop(sailing_navigator_t *navigator);

sailing_drive_command_t sailing_navigator_tick(
    sailing_navigator_t *navigator,
    uint64_t now_ms,
    const sailing_infrared_sample_t *infrared,
    const sailing_imu_sample_t *imu);

#endif
