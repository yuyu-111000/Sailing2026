#include "sailing/controller.hpp"
#include "sailing/infrared_link.hpp"
#include "fakes.hpp"
#include "sailing/navigation.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

#define CHECK(condition) do { if (!(condition)) { std::cerr << "CHECK failed at " \
    << __FILE__ << ':' << __LINE__ << ": " #condition << '\n'; return false; } } while (false)

std::array<std::uint8_t, 11U> jy61_frame(std::uint8_t type, std::int16_t z) {
    std::array<std::uint8_t, 11U> frame{};
    frame[0] = 0x55U;
    frame[1] = type;
    frame[6] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(z) & 0xFFU);
    frame[7] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(z) >> 8U);
    for (std::size_t index = 0U; index < 10U; ++index) {
        frame[10] = static_cast<std::uint8_t>(frame[10] + frame[index]);
    }
    return frame;
}

bool test_jy61_decoder() {
    sailing::l431::Jy61Decoder decoder;
    const auto rate = jy61_frame(0x52U, 16384);
    for (const auto byte : rate) { (void)decoder.feed(byte, 10U); }
    CHECK(decoder.sample().rate_valid);
    CHECK(std::fabs(decoder.sample().yaw_rate_dps - 1000.0F) < 0.1F);
    const auto angle = jy61_frame(0x53U, -16384);
    for (const auto byte : angle) { (void)decoder.feed(byte, 20U); }
    CHECK(decoder.sample().angle_valid);
    CHECK(std::fabs(decoder.sample().yaw_deg + 90.0F) < 0.1F);
    CHECK(decoder.sample().rate_received_ms == 10U);
    CHECK(decoder.sample().angle_received_ms == 20U);
    auto damaged = rate;
    damaged[10] ^= 1U;
    sailing::l431::Jy61Decoder rejected;
    for (const auto byte : damaged) { (void)rejected.feed(byte, 30U); }
    CHECK(!rejected.sample().rate_valid);
    return true;
}

bool test_bearing_geometry() {
    sailing::InfraredSample infrared{};
    infrared.valid = true;
    infrared.strength[0] = 100U;
    infrared.strength[1] = 50U;
    infrared.strength[15] = 50U;
    const auto bearing = sailing::l431::estimate_bearing(infrared);
    CHECK(bearing.valid);
    CHECK(std::fabs(bearing.angle_deg) < 0.01F);
    CHECK(bearing.confidence > 0.9F);
    CHECK(!sailing::l431::estimate_bearing({}).valid);
    return true;
}

bool test_navigation_search_track_pass_and_timeout() {
    sailing::l431::NavigationConfig config{};
    config.approach_strength = 80.0F;
    config.pass_loss_ms = 100U;
    config.pass_forward_ms = 200U;
    config.mission_timeout_ms = 1000U;
    sailing::l431::Navigator navigator(config);
    navigator.arm(0U);
    sailing::InfraredSample infrared{};
    sailing::l431::ImuSample imu{};
    auto command = navigator.tick(0U, infrared, imu);
    CHECK(navigator.state() == sailing::l431::NavigationState::Search);
    CHECK(command.left < 0.0F && command.right > 0.0F);

    infrared.valid = true;
    infrared.received_ms = 10U;
    infrared.strength[2] = 100U;
    command = navigator.tick(10U, infrared, imu);
    CHECK(navigator.state() == sailing::l431::NavigationState::Track);
    CHECK(command.right > command.left);

    infrared.strength = {};
    infrared.strength[0] = 100U;
    infrared.received_ms = 20U;
    command = navigator.tick(20U, infrared, imu);
    CHECK(command.left == command.right);
    infrared.valid = false;
    (void)navigator.tick(119U, infrared, imu);
    CHECK(navigator.state() == sailing::l431::NavigationState::Search);
    (void)navigator.tick(120U, infrared, imu);
    CHECK(navigator.state() == sailing::l431::NavigationState::PassThrough);
    (void)navigator.tick(320U, infrared, imu);
    CHECK(navigator.passed_gates() == 1U);
    CHECK(navigator.state() == sailing::l431::NavigationState::Search);
    (void)navigator.tick(1000U, infrared, imu);
    CHECK(navigator.state() == sailing::l431::NavigationState::Fault);

    config.minimum_confidence = 2.0F;
    sailing::l431::Navigator invalid(config);
    CHECK(invalid.state() == sailing::l431::NavigationState::Fault);
    return true;
}

bool test_controller_safety_paths() {
    sailing::l431::FakeClock clock;
    sailing::l431::FakeByteInput infrared_uart;
    sailing::l431::FakeByteInput jy61_uart;
    sailing::l431::FakeDrive drive;
    sailing::l431::FakeArmInput arm;
    sailing::l431::BoatController controller(clock, infrared_uart, jy61_uart, drive, arm);
    controller.tick();
    CHECK(drive.neutral_state);
    arm.armed_value = true;
    std::array<std::uint16_t, sailing::kInfraredChannelCount> strength{};
    strength[0] = 200U;
    const auto infrared = sailing::encode_infrared_frame(strength, 1U);
    const auto imu = jy61_frame(0x52U, 0);
    CHECK(infrared_uart.push(infrared.bytes.data(), infrared.bytes.size()));
    CHECK(jy61_uart.push(imu.data(), imu.size()));
    controller.tick();
    CHECK(controller.navigator().state() == sailing::l431::NavigationState::Track);
    CHECK(!drive.neutral_state);
    arm.estop_value = true;
    controller.tick();
    CHECK(controller.navigator().state() == sailing::l431::NavigationState::EmergencyStop);
    CHECK(drive.neutral_state);

    sailing::l431::FakeDrive failed_drive;
    sailing::l431::FakeArmInput failed_arm;
    failed_drive.fail = true;
    sailing::l431::BoatController failed(
        clock, infrared_uart, jy61_uart, failed_drive, failed_arm);
    CHECK(failed.output_fault());
    return true;
}

}  // namespace

int main() {
    const bool results[]{
        test_jy61_decoder(),
        test_bearing_geometry(),
        test_navigation_search_track_pass_and_timeout(),
        test_controller_safety_paths(),
    };
    std::size_t passed = 0U;
    for (const bool result : results) { if (result) { ++passed; } }
    std::cout << passed << '/' << std::size(results) << " L431 test groups passed\n";
    return passed == std::size(results) ? 0 : 1;
}
