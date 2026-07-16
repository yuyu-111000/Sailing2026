#include "sailing/safety_supervisor.hpp"

#include <cmath>

namespace sailing {
namespace {

constexpr std::uint32_t kMaximumReviewedWatchdogMs = 1000U;

[[nodiscard]] bool valid_source(protocol::ShotSource source) noexcept {
    return source == protocol::ShotSource::Auto ||
        source == protocol::ShotSource::ManualFutureRemote;
}

}  // namespace

SafetySupervisor::SafetySupervisor(
    hal::MonotonicClock& clock,
    hal::MotionOutput& motion,
    hal::LauncherOutput& launcher,
    hal::EventSink& events,
    SafetyConfig config) noexcept
    : clock_(clock),
      motion_(motion),
      launcher_(launcher),
      events_(events),
      config_(config) {
    const bool config_valid = config_.watchdog_ms > 0U &&
        config_.watchdog_ms <= kMaximumReviewedWatchdogMs &&
        config_.maximum_command_validity_ms > 0U &&
        config_.maximum_command_validity_ms <= kMaximumReviewedWatchdogMs &&
        config_.mission_timeout_ms > 0U;
    const bool outputs_safe = safe_outputs();
    if (!config_valid || !outputs_safe) {
        state_ = SafetyState::Fault;
        reject(config_valid ? RejectionReason::OutputFailure : RejectionReason::InvalidCommand);
    }
}

bool SafetySupervisor::begin_session(
    MissionMode mode,
    std::uint32_t session_id,
    std::uint32_t sequence,
    bool c1_completed) noexcept {
    if (state_ != SafetyState::Disarmed) {
        reject(RejectionReason::WrongState);
        return false;
    }
    if (session_id == 0U) {
        reject(RejectionReason::InvalidSession);
        return false;
    }
    if (mode == MissionMode::C2 && !c1_completed) {
        reject(RejectionReason::MissingC1Completion);
        return false;
    }
    if (!safe_outputs()) {
        enter_fault(RejectionReason::OutputFailure);
        return false;
    }

    mode_ = mode;
    session_id_ = session_id;
    last_sequence_ = sequence;
    has_sequence_ = true;
    session_started_ms_ = clock_.now_ms();
    last_control_ms_ = session_started_ms_;
    control_valid_for_ms_ = 0U;
    has_control_ = false;
    fire_interlocks_ = {};
    launcher_armed_ = false;
    transition(SafetyState::Armed);
    return true;
}

bool SafetySupervisor::start(std::uint32_t session_id, std::uint32_t sequence) noexcept {
    if (state_ != SafetyState::Armed) {
        reject(RejectionReason::WrongState);
        return false;
    }
    if (!validate_active_command(session_id, sequence)) {
        return false;
    }
    accept_sequence(sequence);
    session_started_ms_ = clock_.now_ms();
    last_control_ms_ = session_started_ms_;
    has_control_ = false;
    transition(SafetyState::Running);
    return true;
}

bool SafetySupervisor::apply_control(
    std::uint32_t session_id,
    std::uint32_t sequence,
    const protocol::ControlSetpoint& command) noexcept {
    if (state_ != SafetyState::Running) {
        reject(RejectionReason::WrongState);
        return false;
    }
    if (!std::isfinite(command.propulsion) || !std::isfinite(command.steering) ||
        command.propulsion < -1.0F || command.propulsion > 1.0F ||
        command.steering < -1.0F || command.steering > 1.0F ||
        command.valid_for_ms == 0U ||
        command.valid_for_ms > config_.maximum_command_validity_ms) {
        reject(RejectionReason::InvalidCommand);
        return false;
    }
    if (!validate_active_command(session_id, sequence)) {
        return false;
    }
    if (!motion_.apply(command.propulsion, command.steering)) {
        enter_fault(RejectionReason::OutputFailure);
        return false;
    }

    accept_sequence(sequence);
    last_control_ms_ = clock_.now_ms();
    control_valid_for_ms_ = command.valid_for_ms;
    has_control_ = true;
    return true;
}

bool SafetySupervisor::set_fire_interlocks(
    std::uint32_t session_id,
    std::uint32_t sequence,
    const FireInterlocks& interlocks) noexcept {
    if (state_ != SafetyState::Running) {
        reject(RejectionReason::WrongState);
        return false;
    }
    if (mode_ != MissionMode::C2) {
        reject(RejectionReason::WrongMode);
        return false;
    }
    if (!validate_active_command(session_id, sequence)) {
        return false;
    }

    accept_sequence(sequence);
    fire_interlocks_ = interlocks;
    if (!fire_interlocks_.all_closed()) {
        revoke_launcher();
    }
    return true;
}

bool SafetySupervisor::arm_launcher(
    std::uint32_t session_id,
    std::uint32_t sequence) noexcept {
    if (state_ != SafetyState::Running) {
        reject(RejectionReason::WrongState);
        return false;
    }
    if (mode_ != MissionMode::C2) {
        reject(RejectionReason::WrongMode);
        return false;
    }
    if (!control_fresh()) {
        reject(RejectionReason::MotionStale);
        return false;
    }
    if (!fire_interlocks_.all_closed()) {
        reject(RejectionReason::InterlockOpen);
        return false;
    }
    if (shot_count_ >= kMaximumShots) {
        reject(RejectionReason::AmmunitionExhausted);
        return false;
    }
    if (launcher_armed_) {
        reject(RejectionReason::WrongState);
        return false;
    }
    if (!validate_active_command(session_id, sequence)) {
        return false;
    }
    if (!launcher_.set_armed(true)) {
        enter_fault(RejectionReason::OutputFailure);
        return false;
    }

    accept_sequence(sequence);
    launcher_armed_ = true;
    events_.record({hal::EventCode::LauncherArmed, 0U});
    return true;
}

bool SafetySupervisor::fire_once(
    std::uint32_t session_id,
    std::uint32_t sequence,
    const protocol::FireOnceRequest& request) noexcept {
    if (state_ != SafetyState::Running) {
        reject(RejectionReason::WrongState);
        return false;
    }
    if (mode_ != MissionMode::C2) {
        reject(RejectionReason::WrongMode);
        return false;
    }
    if (!valid_source(request.source) || request.valid_for_ms == 0U ||
        request.valid_for_ms > config_.maximum_command_validity_ms) {
        reject(RejectionReason::InvalidCommand);
        return false;
    }
    if (!control_fresh()) {
        reject(RejectionReason::MotionStale);
        return false;
    }
    if (!fire_interlocks_.all_closed()) {
        revoke_launcher();
        reject(RejectionReason::InterlockOpen);
        return false;
    }
    if (!launcher_armed_) {
        reject(RejectionReason::LauncherNotArmed);
        return false;
    }
    if (shot_count_ >= kMaximumShots) {
        revoke_launcher();
        reject(RejectionReason::AmmunitionExhausted);
        return false;
    }
    if (shot_id_seen(request.shot_id)) {
        revoke_launcher();
        reject(RejectionReason::DuplicateShot);
        return false;
    }
    if (!validate_active_command(session_id, sequence)) {
        return false;
    }
    if (!launcher_.fire_once(request.shot_id)) {
        enter_fault(RejectionReason::OutputFailure);
        return false;
    }

    used_shot_ids_[shot_count_] = request.shot_id;
    ++shot_count_;
    accept_sequence(sequence);
    events_.record({
        hal::EventCode::ShotFired,
        request.shot_id,
        static_cast<std::uint32_t>(request.source),
    });
    launcher_armed_ = false;
    if (!launcher_.set_armed(false)) {
        enter_fault(RejectionReason::OutputFailure);
        return false;
    }
    return true;
}

void SafetySupervisor::heartbeat() noexcept {
    // HEARTBEAT is deliberately side-effect free and never refreshes motion freshness.
}

void SafetySupervisor::tick() noexcept {
    if (state_ != SafetyState::Running) {
        return;
    }
    const auto now = clock_.now_ms();
    if ((now - session_started_ms_) >= config_.mission_timeout_ms) {
        events_.record({hal::EventCode::MissionTimeout, config_.mission_timeout_ms});
        enter_fault(RejectionReason::MissionTimeout);
        return;
    }
    const bool expired = has_control_
        ? !control_fresh()
        : (now - session_started_ms_) >= config_.watchdog_ms;
    if (expired) {
        events_.record({hal::EventCode::WatchdogExpired, config_.watchdog_ms});
        enter_fault(RejectionReason::MotionStale);
    }
}

void SafetySupervisor::disarm() noexcept {
    if (state_ != SafetyState::Armed && state_ != SafetyState::Running) {
        reject(RejectionReason::WrongState);
        return;
    }
    const bool safe = safe_outputs();
    session_id_ = 0U;
    has_sequence_ = false;
    fire_interlocks_ = {};
    if (safe) {
        transition(SafetyState::Disarmed);
    } else {
        transition(SafetyState::Fault);
    }
}

void SafetySupervisor::complete() noexcept {
    if (state_ != SafetyState::Running) {
        reject(RejectionReason::WrongState);
        return;
    }
    const bool safe = safe_outputs();
    session_id_ = 0U;
    has_sequence_ = false;
    transition(safe ? SafetyState::Complete : SafetyState::Fault);
}

void SafetySupervisor::abort() noexcept {
    if (state_ != SafetyState::Running && state_ != SafetyState::Armed) {
        reject(RejectionReason::WrongState);
        return;
    }
    const bool safe = safe_outputs();
    session_id_ = 0U;
    has_sequence_ = false;
    transition(safe ? SafetyState::Aborted : SafetyState::Fault);
}

void SafetySupervisor::fault() noexcept {
    if (state_ == SafetyState::EStop) {
        reject(RejectionReason::WrongState);
        return;
    }
    enter_fault(RejectionReason::InvalidCommand);
}

void SafetySupervisor::emergency_stop() noexcept {
    (void)safe_outputs();
    session_id_ = 0U;
    has_sequence_ = false;
    fire_interlocks_ = {};
    transition(SafetyState::EStop);
    events_.record({hal::EventCode::EmergencyStopLatched, 0U});
}

bool SafetySupervisor::reset() noexcept {
    if (state_ != SafetyState::Complete && state_ != SafetyState::Aborted &&
        state_ != SafetyState::Fault && state_ != SafetyState::EStop) {
        reject(RejectionReason::WrongState);
        return false;
    }
    if (!safe_outputs()) {
        transition(SafetyState::Fault);
        return false;
    }

    session_id_ = 0U;
    has_sequence_ = false;
    fire_interlocks_ = {};
    transition(SafetyState::Disarmed);
    return true;
}

bool SafetySupervisor::control_fresh() const noexcept {
    if (state_ != SafetyState::Running || !has_control_) {
        return false;
    }
    const auto elapsed = clock_.now_ms() - last_control_ms_;
    return elapsed < config_.watchdog_ms && elapsed < control_valid_for_ms_;
}

bool SafetySupervisor::validate_active_command(
    std::uint32_t session_id,
    std::uint32_t sequence) noexcept {
    if (session_id_ == 0U || session_id != session_id_) {
        reject(RejectionReason::InvalidSession);
        return false;
    }
    if (has_sequence_ && !sequence_is_newer(sequence, last_sequence_)) {
        reject(RejectionReason::StaleSequence);
        return false;
    }
    return true;
}

bool SafetySupervisor::sequence_is_newer(
    std::uint32_t candidate,
    std::uint32_t current) noexcept {
    const auto difference = candidate - current;
    return difference != 0U && difference < 0x80000000U;
}

bool SafetySupervisor::shot_id_seen(std::uint32_t shot_id) const noexcept {
    for (std::size_t index = 0U; index < shot_count_; ++index) {
        if (used_shot_ids_[index] == shot_id) {
            return true;
        }
    }
    return false;
}

bool SafetySupervisor::safe_outputs() noexcept {
    const bool motion_safe = motion_.neutral();
    const bool launcher_safe = launcher_.set_armed(false);
    has_control_ = false;
    control_valid_for_ms_ = 0U;
    launcher_armed_ = false;
    if (!motion_safe || !launcher_safe) {
        events_.record({hal::EventCode::SafeOutputFailed, 0U});
    }
    return motion_safe && launcher_safe;
}

void SafetySupervisor::accept_sequence(std::uint32_t sequence) noexcept {
    last_sequence_ = sequence;
    has_sequence_ = true;
}

void SafetySupervisor::transition(SafetyState state) noexcept {
    state_ = state;
    events_.record({hal::EventCode::StateChanged, static_cast<std::uint32_t>(state)});
}

void SafetySupervisor::reject(RejectionReason reason) noexcept {
    events_.record({hal::EventCode::CommandRejected, static_cast<std::uint32_t>(reason)});
}

void SafetySupervisor::enter_fault(RejectionReason reason) noexcept {
    reject(reason);
    (void)safe_outputs();
    session_id_ = 0U;
    has_sequence_ = false;
    fire_interlocks_ = {};
    transition(SafetyState::Fault);
}

void SafetySupervisor::revoke_launcher() noexcept {
    if (!launcher_armed_) {
        return;
    }
    launcher_armed_ = false;
    if (!launcher_.set_armed(false)) {
        enter_fault(RejectionReason::OutputFailure);
    }
}

}  // namespace sailing
