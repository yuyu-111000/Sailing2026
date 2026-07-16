#include "sailing/controller.hpp"

#include <array>

namespace sailing {

BoatController::BoatController(
    hal::MonotonicClock& clock, hal::ByteInput& infrared_uart, hal::ByteInput& jy61_uart,
    hal::DifferentialDrive& drive, hal::ArmInput& arm_input, NavigationConfig config) noexcept
    : clock_(clock), infrared_uart_(infrared_uart), jy61_uart_(jy61_uart), drive_(drive),
      arm_input_(arm_input), navigator_(config) {
    if (!drive_.neutral()) { output_fault_ = true; }
}

void BoatController::drain_inputs(std::uint64_t now_ms) noexcept {
    std::array<std::uint8_t, 64U> bytes{};
    std::size_t count = infrared_uart_.read(bytes.data(), bytes.size());
    for (std::size_t i = 0U; i < count; ++i) { (void)infrared_.feed(bytes[i], now_ms); }
    count = jy61_uart_.read(bytes.data(), bytes.size());
    for (std::size_t i = 0U; i < count; ++i) { (void)jy61_.feed(bytes[i], now_ms); }
}

void BoatController::tick() noexcept {
    const auto now_ms = clock_.now_ms();
    drain_inputs(now_ms);
    if (output_fault_ || arm_input_.emergency_stop()) {
        navigator_.emergency_stop();
        (void)drive_.neutral();
        return;
    }
    const bool armed = arm_input_.armed();
    if (armed && !was_armed_) { navigator_.arm(now_ms); }
    if (!armed) {
        navigator_.disarm();
        if (!drive_.neutral()) { output_fault_ = true; }
        was_armed_ = false;
        return;
    }
    was_armed_ = true;
    const auto command = navigator_.tick(now_ms, infrared_.sample(), jy61_.sample());
    if (!drive_.apply(command.left, command.right)) {
        output_fault_ = true;
        navigator_.emergency_stop();
        (void)drive_.neutral();
    }
}

}  // namespace sailing
