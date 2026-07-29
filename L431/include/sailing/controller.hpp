#pragma once

#include "sailing/infrared_link.hpp"
#include "sailing/jy61.hpp"
#include "sailing/l431_hal.hpp"
#include "sailing/navigation.hpp"

namespace sailing::l431 {

class BoatController {
public:
    BoatController(MonotonicClock& clock, ByteInput& infrared_uart,
                   ByteInput& jy61_uart, DifferentialDrive& drive,
                   ArmInput& arm_input, NavigationConfig config = {}) noexcept;
    void tick() noexcept;
    [[nodiscard]] const Navigator& navigator() const noexcept { return navigator_; }
    [[nodiscard]] const InfraredSample& infrared() const noexcept { return infrared_.sample(); }
    [[nodiscard]] const ImuSample& imu() const noexcept { return jy61_.sample(); }
    [[nodiscard]] bool output_fault() const noexcept { return output_fault_; }

private:
    void drain_inputs(std::uint64_t now_ms) noexcept;
    MonotonicClock& clock_;
    ByteInput& infrared_uart_;
    ByteInput& jy61_uart_;
    DifferentialDrive& drive_;
    ArmInput& arm_input_;
    InfraredStreamDecoder infrared_{};
    Jy61Decoder jy61_{};
    Navigator navigator_;
    bool was_armed_{false};
    bool output_fault_{false};
};

}  // namespace sailing::l431
