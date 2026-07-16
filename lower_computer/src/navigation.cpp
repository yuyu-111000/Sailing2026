#include "sailing/navigation.hpp"

#include <algorithm>
#include <cmath>

namespace sailing {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kSensorStepDeg = 22.5F;

[[nodiscard]] float clamp_unit(float value) noexcept {
    return std::clamp(value, -1.0F, 1.0F);
}

}  // namespace

BearingEstimate estimate_bearing(const InfraredSample& sample) noexcept {
    BearingEstimate result{};
    if (!sample.valid) { return result; }
    float x = 0.0F;
    float y = 0.0F;
    for (std::size_t i = 0U; i < sample.strength.size(); ++i) {
        const float weight = static_cast<float>(sample.strength[i]);
        const float angle = static_cast<float>(i) * kSensorStepDeg * kPi / 180.0F;
        x += weight * std::cos(angle);
        y += weight * std::sin(angle);
        result.total_strength += weight;
    }
    if (result.total_strength <= 0.0F) { return result; }
    result.angle_deg = std::atan2(y, x) * 180.0F / kPi;
    result.confidence = std::sqrt(x * x + y * y) / result.total_strength;
    result.valid = std::isfinite(result.angle_deg) && std::isfinite(result.confidence);
    return result;
}

Navigator::Navigator(NavigationConfig config) noexcept : config_(config) {
    if (!config_valid()) { state_ = NavigationState::Fault; }
}

bool Navigator::config_valid() const noexcept {
    return config_.minimum_total_strength > 0.0F &&
        config_.minimum_confidence >= 0.0F && config_.minimum_confidence <= 1.0F &&
        config_.maximum_steering > 0.0F && config_.maximum_steering <= 1.0F &&
        config_.mission_timeout_ms > 0U && config_.infrared_timeout_ms > 0U &&
        config_.imu_timeout_ms > 0U && config_.pass_forward_ms > 0U;
}

void Navigator::arm(std::uint64_t now_ms) noexcept {
    if (state_ == NavigationState::Fault || state_ == NavigationState::EmergencyStop) { return; }
    state_ = NavigationState::Search;
    started_ms_ = now_ms;
    last_seen_ms_ = now_ms;
    pass_started_ms_ = 0U;
    passed_gates_ = 0U;
    approached_ = false;
}

void Navigator::disarm() noexcept {
    if (state_ != NavigationState::EmergencyStop) { state_ = NavigationState::Disarmed; }
}

void Navigator::emergency_stop() noexcept { state_ = NavigationState::EmergencyStop; }

DriveCommand Navigator::mix(float throttle, float steering) const noexcept {
    float left = clamp_unit(throttle - steering + config_.left_trim);
    float right = clamp_unit(throttle + steering + config_.right_trim);
    if (std::fabs(left) < config_.motor_deadband) { left = 0.0F; }
    if (std::fabs(right) < config_.motor_deadband) { right = 0.0F; }
    return {left, right};
}

DriveCommand Navigator::tick(
    std::uint64_t now_ms, const InfraredSample& infrared, const ImuSample& imu) noexcept {
    if (state_ == NavigationState::Disarmed || state_ == NavigationState::Complete ||
        state_ == NavigationState::Fault || state_ == NavigationState::EmergencyStop) { return {}; }
    if (now_ms - started_ms_ >= config_.mission_timeout_ms) {
        state_ = NavigationState::Fault;
        return {};
    }

    bearing_ = estimate_bearing(infrared);
    const bool infrared_fresh = infrared.valid && now_ms >= infrared.received_ms &&
        now_ms - infrared.received_ms < config_.infrared_timeout_ms;
    const bool target_visible = infrared_fresh && bearing_.valid &&
        bearing_.total_strength >= config_.minimum_total_strength &&
        bearing_.confidence >= config_.minimum_confidence;
    const bool imu_fresh = imu.rate_valid && now_ms >= imu.rate_received_ms &&
        now_ms - imu.rate_received_ms < config_.imu_timeout_ms;

    if (state_ == NavigationState::PassThrough) {
        if (now_ms - pass_started_ms_ >= config_.pass_forward_ms) {
            if (++passed_gates_ >= 10U) { state_ = NavigationState::Complete; return {}; }
            state_ = NavigationState::Search;
            approached_ = false;
        } else {
            return mix(config_.pass_throttle, 0.0F);
        }
    }

    if (!target_visible) {
        if (approached_ && now_ms - last_seen_ms_ >= config_.pass_loss_ms) {
            state_ = NavigationState::PassThrough;
            pass_started_ms_ = now_ms;
            return mix(config_.pass_throttle, 0.0F);
        }
        state_ = NavigationState::Search;
        return mix(0.0F, config_.search_turn);
    }

    last_seen_ms_ = now_ms;
    state_ = NavigationState::Track;
    if (std::fabs(bearing_.angle_deg) <= config_.centered_angle_deg &&
        bearing_.total_strength >= config_.approach_strength) { approached_ = true; }
    const float damping = imu_fresh ? config_.yaw_rate_kd * imu.yaw_rate_dps : 0.0F;
    const float steering = std::clamp(
        config_.steering_kp * bearing_.angle_deg - damping,
        -config_.maximum_steering, config_.maximum_steering);
    const float throttle = std::fabs(bearing_.angle_deg) > 55.0F
        ? config_.turn_throttle : config_.track_throttle;
    return mix(throttle, steering);
}

}  // namespace sailing
