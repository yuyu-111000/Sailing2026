#pragma once

#include <cstddef>
#include <cstdint>

namespace sailing::hal {

enum class EventCode : std::uint8_t {
    StateChanged,
    CommandRejected,
    WatchdogExpired,
    MissionTimeout,
    EmergencyStopLatched,
    LauncherArmed,
    ShotFired,
    SafeOutputFailed,
};

struct Event {
    EventCode code{EventCode::StateChanged};
    std::uint32_t detail{0U};
    std::uint32_t auxiliary{0U};
};

class MonotonicClock {
public:
    virtual ~MonotonicClock() = default;
    [[nodiscard]] virtual std::uint64_t now_ms() const noexcept = 0;
};

class MotionOutput {
public:
    virtual ~MotionOutput() = default;
    virtual bool apply(float propulsion, float steering) noexcept = 0;
    virtual bool neutral() noexcept = 0;
};

class LauncherOutput {
public:
    virtual ~LauncherOutput() = default;
    virtual bool set_armed(bool armed) noexcept = 0;
    virtual bool fire_once(std::uint32_t shot_id) noexcept = 0;
};

class EventSink {
public:
    virtual ~EventSink() = default;
    virtual void record(const Event& event) noexcept = 0;
};

class ByteTransport {
public:
    virtual ~ByteTransport() = default;
    virtual std::size_t read(std::uint8_t* destination, std::size_t capacity) noexcept = 0;
    virtual bool write(const std::uint8_t* data, std::size_t size) noexcept = 0;
};

}  // namespace sailing::hal
