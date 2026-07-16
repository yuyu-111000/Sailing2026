#pragma once

#include "sailing/sensors.hpp"

#include <cstdint>

namespace sailing {

enum class NavigationState : std::uint8_t {
    Disarmed,
    Search,
    Track,
    PassThrough,
    Complete,
    Fault,
    EmergencyStop,
};

struct NavigationConfig {
    float minimum_total_strength{20.0F};
    float minimum_confidence{0.18F};
    float centered_angle_deg{18.0F};
    float approach_strength{180.0F};
    float search_turn{0.22F};
    float track_throttle{0.42F};
    float turn_throttle{0.20F};
    float pass_throttle{0.34F};
    float steering_kp{0.012F};
    float yaw_rate_kd{0.006F};
    float maximum_steering{0.65F};
    float motor_deadband{0.08F};
    float left_trim{0.0F};
    float right_trim{0.0F};
    std::uint32_t infrared_timeout_ms{180U};
    std::uint32_t imu_timeout_ms{250U};
    std::uint32_t pass_loss_ms{120U};
    std::uint32_t pass_forward_ms{350U};
    std::uint32_t mission_timeout_ms{300000U};
};

struct BearingEstimate {
    float angle_deg{0.0F};
    float confidence{0.0F};
    float total_strength{0.0F};
    bool valid{false};
};

struct DriveCommand { float left{0.0F}; float right{0.0F}; };

[[nodiscard]] BearingEstimate estimate_bearing(const InfraredSample& sample) noexcept;

class Navigator {
public:
    explicit Navigator(NavigationConfig config = {}) noexcept;
    void arm(std::uint64_t now_ms) noexcept;
    void disarm() noexcept;
    void emergency_stop() noexcept;
    [[nodiscard]] DriveCommand tick(
        std::uint64_t now_ms, const InfraredSample& infrared, const ImuSample& imu) noexcept;
    [[nodiscard]] NavigationState state() const noexcept { return state_; }
    [[nodiscard]] std::uint8_t passed_gates() const noexcept { return passed_gates_; }
    [[nodiscard]] BearingEstimate bearing() const noexcept { return bearing_; }
private:
    [[nodiscard]] DriveCommand mix(float throttle, float steering) const noexcept;
    [[nodiscard]] bool config_valid() const noexcept;
    NavigationConfig config_{};
    NavigationState state_{NavigationState::Disarmed};
    BearingEstimate bearing_{};
    std::uint64_t started_ms_{0U};
    std::uint64_t last_seen_ms_{0U};
    std::uint64_t pass_started_ms_{0U};
    std::uint8_t passed_gates_{0U};
    bool approached_{false};
};

}  // namespace sailing
