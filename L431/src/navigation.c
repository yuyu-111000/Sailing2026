#include "sailing/navigation.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define SAILING_PI 3.14159265358979323846F
#define SAILING_SENSOR_STEP_DEG 22.5F

static float clamp_float(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static bool config_valid(const sailing_navigation_config_t *config) {
    return config != NULL &&
        isfinite(config->minimum_total_strength) &&
        isfinite(config->minimum_confidence) &&
        isfinite(config->centered_angle_deg) &&
        isfinite(config->approach_strength) &&
        isfinite(config->search_turn) &&
        isfinite(config->track_throttle) &&
        isfinite(config->turn_throttle) &&
        isfinite(config->pass_throttle) &&
        isfinite(config->steering_kp) &&
        isfinite(config->yaw_rate_kd) &&
        isfinite(config->maximum_steering) &&
        isfinite(config->motor_deadband) &&
        isfinite(config->left_trim) &&
        isfinite(config->right_trim) &&
        config->minimum_total_strength > 0.0F &&
        config->minimum_confidence >= 0.0F && config->minimum_confidence <= 1.0F &&
        config->centered_angle_deg >= 0.0F && config->centered_angle_deg <= 180.0F &&
        config->approach_strength >= config->minimum_total_strength &&
        config->search_turn >= -1.0F && config->search_turn <= 1.0F &&
        config->track_throttle >= 0.0F && config->track_throttle <= 1.0F &&
        config->turn_throttle >= 0.0F && config->turn_throttle <= 1.0F &&
        config->pass_throttle >= 0.0F && config->pass_throttle <= 1.0F &&
        config->steering_kp >= 0.0F && config->yaw_rate_kd >= 0.0F &&
        config->maximum_steering > 0.0F && config->maximum_steering <= 1.0F &&
        config->motor_deadband >= 0.0F && config->motor_deadband < 1.0F &&
        config->left_trim >= -1.0F && config->left_trim <= 1.0F &&
        config->right_trim >= -1.0F && config->right_trim <= 1.0F &&
        config->mission_timeout_ms > 0U && config->infrared_timeout_ms > 0U &&
        config->imu_timeout_ms > 0U && config->pass_loss_ms > 0U &&
        config->pass_forward_ms > 0U;
}

static sailing_drive_command_t mix(
    const sailing_navigator_t *navigator,
    float throttle,
    float steering) {
    sailing_drive_command_t command = {0.0F, 0.0F};
    command.left = clamp_float(
        throttle - steering + navigator->config.left_trim, -1.0F, 1.0F);
    command.right = clamp_float(
        throttle + steering + navigator->config.right_trim, -1.0F, 1.0F);
    if (fabsf(command.left) < navigator->config.motor_deadband) {
        command.left = 0.0F;
    }
    if (fabsf(command.right) < navigator->config.motor_deadband) {
        command.right = 0.0F;
    }
    return command;
}

sailing_navigation_config_t sailing_navigation_default_config(void) {
    sailing_navigation_config_t config = {
        20.0F, 0.18F, 18.0F, 180.0F,
        0.22F, 0.42F, 0.20F, 0.34F,
        0.012F, 0.006F, 0.65F, 0.08F,
        0.0F, 0.0F,
        180U, 250U, 120U, 350U, 300000U
    };
    return config;
}

bool sailing_estimate_bearing(
    const sailing_infrared_sample_t *sample,
    sailing_bearing_estimate_t *estimate) {
    float x = 0.0F;
    float y = 0.0F;
    size_t index;
    if (estimate == NULL) {
        return false;
    }
    memset(estimate, 0, sizeof(*estimate));
    if (sample == NULL || !sample->valid) {
        return false;
    }
    for (index = 0U; index < SAILING_INFRARED_CHANNEL_COUNT; ++index) {
        const float weight = (float)sample->strength[index];
        const float angle = (float)index * SAILING_SENSOR_STEP_DEG * SAILING_PI / 180.0F;
        x += weight * cosf(angle);
        y += weight * sinf(angle);
        estimate->total_strength += weight;
    }
    if (estimate->total_strength <= 0.0F) {
        return false;
    }
    estimate->angle_deg = atan2f(y, x) * 180.0F / SAILING_PI;
    estimate->confidence = sqrtf(x * x + y * y) / estimate->total_strength;
    estimate->valid = isfinite(estimate->angle_deg) && isfinite(estimate->confidence);
    return estimate->valid;
}

bool sailing_navigator_init(
    sailing_navigator_t *navigator,
    const sailing_navigation_config_t *config) {
    if (navigator == NULL) {
        return false;
    }
    memset(navigator, 0, sizeof(*navigator));
    navigator->state = SAILING_NAV_FAULT;
    if (!config_valid(config)) {
        return false;
    }
    navigator->config = *config;
    navigator->state = SAILING_NAV_DISARMED;
    return true;
}

void sailing_navigator_arm(sailing_navigator_t *navigator, uint64_t now_ms) {
    if (navigator == NULL || navigator->state == SAILING_NAV_FAULT ||
        navigator->state == SAILING_NAV_EMERGENCY_STOP) {
        return;
    }
    navigator->state = SAILING_NAV_SEARCH;
    navigator->started_ms = now_ms;
    navigator->last_seen_ms = now_ms;
    navigator->pass_started_ms = 0U;
    navigator->passed_gates = 0U;
    navigator->approached = false;
}

void sailing_navigator_disarm(sailing_navigator_t *navigator) {
    if (navigator != NULL && navigator->state != SAILING_NAV_EMERGENCY_STOP) {
        navigator->state = SAILING_NAV_DISARMED;
    }
}

void sailing_navigator_emergency_stop(sailing_navigator_t *navigator) {
    if (navigator != NULL) {
        navigator->state = SAILING_NAV_EMERGENCY_STOP;
    }
}

sailing_drive_command_t sailing_navigator_tick(
    sailing_navigator_t *navigator,
    uint64_t now_ms,
    const sailing_infrared_sample_t *infrared,
    const sailing_imu_sample_t *imu) {
    sailing_drive_command_t neutral = {0.0F, 0.0F};
    bool infrared_fresh;
    bool target_visible;
    bool imu_fresh;
    float damping;
    float steering;
    float throttle;
    if (navigator == NULL || infrared == NULL || imu == NULL) {
        return neutral;
    }
    if (navigator->state == SAILING_NAV_DISARMED ||
        navigator->state == SAILING_NAV_COMPLETE ||
        navigator->state == SAILING_NAV_FAULT ||
        navigator->state == SAILING_NAV_EMERGENCY_STOP) {
        return neutral;
    }
    if (now_ms < navigator->started_ms ||
        now_ms - navigator->started_ms >= navigator->config.mission_timeout_ms) {
        navigator->state = SAILING_NAV_FAULT;
        return neutral;
    }

    (void)sailing_estimate_bearing(infrared, &navigator->bearing);
    infrared_fresh = infrared->valid && now_ms >= infrared->received_ms &&
        now_ms - infrared->received_ms < navigator->config.infrared_timeout_ms;
    target_visible = infrared_fresh && navigator->bearing.valid &&
        navigator->bearing.total_strength >= navigator->config.minimum_total_strength &&
        navigator->bearing.confidence >= navigator->config.minimum_confidence;
    imu_fresh = imu->rate_valid && now_ms >= imu->rate_received_ms &&
        now_ms - imu->rate_received_ms < navigator->config.imu_timeout_ms;

    if (navigator->state == SAILING_NAV_PASS_THROUGH) {
        if (now_ms - navigator->pass_started_ms >= navigator->config.pass_forward_ms) {
            ++navigator->passed_gates;
            if (navigator->passed_gates >= 10U) {
                navigator->state = SAILING_NAV_COMPLETE;
                return neutral;
            }
            navigator->state = SAILING_NAV_SEARCH;
            navigator->approached = false;
        } else {
            return mix(navigator, navigator->config.pass_throttle, 0.0F);
        }
    }

    if (!target_visible) {
        if (navigator->approached && now_ms >= navigator->last_seen_ms &&
            now_ms - navigator->last_seen_ms >= navigator->config.pass_loss_ms) {
            navigator->state = SAILING_NAV_PASS_THROUGH;
            navigator->pass_started_ms = now_ms;
            return mix(navigator, navigator->config.pass_throttle, 0.0F);
        }
        navigator->state = SAILING_NAV_SEARCH;
        return mix(navigator, 0.0F, navigator->config.search_turn);
    }

    navigator->last_seen_ms = now_ms;
    navigator->state = SAILING_NAV_TRACK;
    if (fabsf(navigator->bearing.angle_deg) <= navigator->config.centered_angle_deg &&
        navigator->bearing.total_strength >= navigator->config.approach_strength) {
        navigator->approached = true;
    }
    damping = imu_fresh ? navigator->config.yaw_rate_kd * imu->yaw_rate_dps : 0.0F;
    steering = clamp_float(
        navigator->config.steering_kp * navigator->bearing.angle_deg - damping,
        -navigator->config.maximum_steering,
        navigator->config.maximum_steering);
    throttle = fabsf(navigator->bearing.angle_deg) > 55.0F
        ? navigator->config.turn_throttle
        : navigator->config.track_throttle;
    return mix(navigator, throttle, steering);
}
