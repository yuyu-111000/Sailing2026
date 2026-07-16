#pragma once

#include "sailing/hal.hpp"
#include "sailing/navigation.hpp"
#include "sailing/sensors.hpp"

namespace sailing {

class BoatController {
public:
    BoatController(hal::MonotonicClock& clock, hal::ByteInput& infrared_uart,
                   hal::ByteInput& jy61_uart, hal::DifferentialDrive& drive,
                   hal::ArmInput& arm_input, NavigationConfig config = {}) noexcept;
    void tick() noexcept;
    [[nodiscard]] const Navigator& navigator() const noexcept { return navigator_; }
    [[nodiscard]] const InfraredSample& infrared() const noexcept { return infrared_.sample(); }
    [[nodiscard]] const ImuSample& imu() const noexcept { return jy61_.sample(); }
    [[nodiscard]] bool output_fault() const noexcept { return output_fault_; }
private:
    void drain_inputs(std::uint64_t now_ms) noexcept;
    hal::MonotonicClock& clock_;
    hal::ByteInput& infrared_uart_;
    hal::ByteInput& jy61_uart_;
    hal::DifferentialDrive& drive_;
    hal::ArmInput& arm_input_;
    InfraredSerialDecoder infrared_{};
    Jy61Decoder jy61_{};
    Navigator navigator_;
    bool was_armed_{false};
    bool output_fault_{false};
};

}  // namespace sailing
