#pragma once

#include "sailing/hal.hpp"
#include "sailing/protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace sailing {

enum class MissionMode : std::uint8_t {
    C1 = 1U,
    C2 = 2U,
};

enum class SafetyState : std::uint8_t {
    Disarmed,
    Armed,
    Running,
    Complete,
    Aborted,
    Fault,
    EStop,
};

enum class RejectionReason : std::uint8_t {
    WrongState = 1U,
    MissingC1Completion = 2U,
    InvalidSession = 3U,
    StaleSequence = 4U,
    InvalidCommand = 5U,
    WrongMode = 6U,
    MotionStale = 7U,
    InterlockOpen = 8U,
    LauncherNotArmed = 9U,
    DuplicateShot = 10U,
    AmmunitionExhausted = 11U,
    OutputFailure = 12U,
    MissionTimeout = 13U,
};

struct SafetyConfig {
    std::uint32_t watchdog_ms{500U};
    std::uint32_t maximum_command_validity_ms{1000U};
    std::uint32_t mission_timeout_ms{300000U};
};

struct FireInterlocks {
    bool in_firing_zone{false};
    bool target_locked{false};
    bool target_stable{false};
    bool cooldown_ready{false};

    [[nodiscard]] bool all_closed() const noexcept {
        return in_firing_zone && target_locked && target_stable && cooldown_ready;
    }
};

class SafetySupervisor {
public:
    static constexpr std::size_t kMaximumShots = 10U;

    SafetySupervisor(
        hal::MonotonicClock& clock,
        hal::MotionOutput& motion,
        hal::LauncherOutput& launcher,
        hal::EventSink& events,
        SafetyConfig config = {}) noexcept;

    [[nodiscard]] bool begin_session(
        MissionMode mode,
        std::uint32_t session_id,
        std::uint32_t sequence,
        bool c1_completed) noexcept;
    [[nodiscard]] bool start(std::uint32_t session_id, std::uint32_t sequence) noexcept;
    [[nodiscard]] bool apply_control(
        std::uint32_t session_id,
        std::uint32_t sequence,
        const protocol::ControlSetpoint& command) noexcept;
    [[nodiscard]] bool set_fire_interlocks(
        std::uint32_t session_id,
        std::uint32_t sequence,
        const FireInterlocks& interlocks) noexcept;
    [[nodiscard]] bool arm_launcher(
        std::uint32_t session_id,
        std::uint32_t sequence) noexcept;
    [[nodiscard]] bool fire_once(
        std::uint32_t session_id,
        std::uint32_t sequence,
        const protocol::FireOnceRequest& request) noexcept;

    void heartbeat() noexcept;
    void tick() noexcept;
    void disarm() noexcept;
    void complete() noexcept;
    void abort() noexcept;
    void fault() noexcept;
    void emergency_stop() noexcept;
    [[nodiscard]] bool reset() noexcept;

    [[nodiscard]] SafetyState state() const noexcept { return state_; }
    [[nodiscard]] MissionMode mode() const noexcept { return mode_; }
    [[nodiscard]] std::uint32_t active_session_id() const noexcept { return session_id_; }
    [[nodiscard]] std::size_t shot_count() const noexcept { return shot_count_; }
    [[nodiscard]] bool launcher_armed() const noexcept { return launcher_armed_; }
    [[nodiscard]] bool control_fresh() const noexcept;
    [[nodiscard]] std::uint32_t watchdog_ms() const noexcept { return config_.watchdog_ms; }
    [[nodiscard]] std::uint32_t mission_timeout_ms() const noexcept {
        return config_.mission_timeout_ms;
    }

private:
    [[nodiscard]] bool validate_active_command(
        std::uint32_t session_id,
        std::uint32_t sequence) noexcept;
    [[nodiscard]] static bool sequence_is_newer(
        std::uint32_t candidate,
        std::uint32_t current) noexcept;
    [[nodiscard]] bool shot_id_seen(std::uint32_t shot_id) const noexcept;
    [[nodiscard]] bool safe_outputs() noexcept;
    void accept_sequence(std::uint32_t sequence) noexcept;
    void transition(SafetyState state) noexcept;
    void reject(RejectionReason reason) noexcept;
    void enter_fault(RejectionReason reason) noexcept;
    void revoke_launcher() noexcept;

    hal::MonotonicClock& clock_;
    hal::MotionOutput& motion_;
    hal::LauncherOutput& launcher_;
    hal::EventSink& events_;
    SafetyConfig config_{};
    SafetyState state_{SafetyState::Disarmed};
    MissionMode mode_{MissionMode::C1};
    FireInterlocks fire_interlocks_{};
    std::uint32_t session_id_{0U};
    std::uint32_t last_sequence_{0U};
    std::uint64_t session_started_ms_{0U};
    std::uint64_t last_control_ms_{0U};
    std::uint16_t control_valid_for_ms_{0U};
    bool has_sequence_{false};
    bool has_control_{false};
    bool launcher_armed_{false};
    std::array<std::uint32_t, kMaximumShots> used_shot_ids_{};
    std::size_t shot_count_{0U};
};

}  // namespace sailing
