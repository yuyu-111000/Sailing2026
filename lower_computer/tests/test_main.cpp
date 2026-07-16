#include "sailing/controller.hpp"
#include "sailing/fakes.hpp"
#include "sailing/navigation.hpp"
#include "sailing/sensors.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

namespace {

#define CHECK(condition) do { if (!(condition)) { std::cerr << "CHECK failed at " \
    << __FILE__ << ':' << __LINE__ << ": " #condition << '\n'; return false; } } while (false)

std::array<std::uint8_t, sailing::InfraredSerialDecoder::kFrameSize> infrared_frame(
    std::uint8_t sequence, const std::array<std::uint16_t, 16U>& strength) {
    std::array<std::uint8_t, sailing::InfraredSerialDecoder::kFrameSize> frame{};
    frame[0] = 0xA5U; frame[1] = 0x5AU; frame[2] = sequence;
    for (std::size_t i = 0U; i < strength.size(); ++i) {
        frame[3U + i * 2U] = static_cast<std::uint8_t>(strength[i] & 0xFFU);
        frame[4U + i * 2U] = static_cast<std::uint8_t>(strength[i] >> 8U);
    }
    const auto crc = sailing::crc16_ccitt_false(frame.data(), 35U);
    frame[35] = static_cast<std::uint8_t>(crc & 0xFFU);
    frame[36] = static_cast<std::uint8_t>(crc >> 8U);
    return frame;
}

std::array<std::uint8_t, 11U> jy61_frame(std::uint8_t type, std::int16_t z) {
    std::array<std::uint8_t, 11U> frame{};
    frame[0] = 0x55U; frame[1] = type;
    frame[6] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(z) & 0xFFU);
    frame[7] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(z) >> 8U);
    for (std::size_t i = 0U; i < 10U; ++i) { frame[10] = static_cast<std::uint8_t>(frame[10] + frame[i]); }
    return frame;
}

bool test_infrared_decoder_and_geometry() {
    sailing::InfraredSerialDecoder decoder;
    std::array<std::uint16_t, 16U> strength{};
    strength[0] = 100U; strength[1] = 50U; strength[15] = 50U;
    auto frame = infrared_frame(7U, strength);
    for (const auto byte : frame) { (void)decoder.feed(byte, 123U); }
    CHECK(decoder.sample().valid);
    CHECK(decoder.sample().sequence == 7U);
    CHECK(decoder.sample().strength[0] == 100U);
    const auto bearing = sailing::estimate_bearing(decoder.sample());
    CHECK(bearing.valid);
    CHECK(std::fabs(bearing.angle_deg) < 0.01F);
    CHECK(bearing.confidence > 0.9F);

    frame[10] ^= 0x01U;
    sailing::InfraredSerialDecoder damaged;
    for (const auto byte : frame) { (void)damaged.feed(byte, 200U); }
    CHECK(!damaged.sample().valid);
    CHECK(sailing::crc16_ccitt_false(
        reinterpret_cast<const std::uint8_t*>("123456789"), 9U) == 0x29B1U);
    return true;
}

bool test_jy61_decoder() {
    sailing::Jy61Decoder decoder;
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
    return true;
}

bool test_navigation_search_track_and_pass() {
    sailing::NavigationConfig config{};
    config.approach_strength = 80.0F;
    config.pass_loss_ms = 100U;
    config.pass_forward_ms = 200U;
    sailing::Navigator navigator(config);
    navigator.arm(0U);
    sailing::InfraredSample infrared{};
    sailing::ImuSample imu{};
    auto command = navigator.tick(0U, infrared, imu);
    CHECK(navigator.state() == sailing::NavigationState::Search);
    CHECK(command.left < 0.0F && command.right > 0.0F);

    infrared.valid = true; infrared.received_ms = 10U; infrared.strength[2] = 100U;
    command = navigator.tick(10U, infrared, imu);
    CHECK(navigator.state() == sailing::NavigationState::Track);
    CHECK(command.right > command.left);

    infrared.strength = {}; infrared.strength[0] = 100U; infrared.received_ms = 20U;
    command = navigator.tick(20U, infrared, imu);
    CHECK(command.left == command.right);
    infrared.valid = false;
    (void)navigator.tick(119U, infrared, imu);
    CHECK(navigator.state() == sailing::NavigationState::Search);
    command = navigator.tick(120U, infrared, imu);
    CHECK(navigator.state() == sailing::NavigationState::PassThrough);
    CHECK(command.left > 0.0F && command.right > 0.0F);
    (void)navigator.tick(320U, infrared, imu);
    CHECK(navigator.passed_gates() == 1U);
    CHECK(navigator.state() == sailing::NavigationState::Search);
    return true;
}

bool test_controller_and_safety_paths() {
    sailing::hal::FakeClock clock;
    sailing::hal::FakeByteInput infrared_uart;
    sailing::hal::FakeByteInput jy61_uart;
    sailing::hal::FakeDrive drive;
    sailing::hal::FakeArmInput arm;
    sailing::BoatController controller(clock, infrared_uart, jy61_uart, drive, arm);
    controller.tick();
    CHECK(drive.neutral_state);
    arm.armed_value = true;
    std::array<std::uint16_t, 16U> strength{}; strength[0] = 200U;
    const auto ir = infrared_frame(1U, strength);
    const auto imu = jy61_frame(0x52U, 0);
    CHECK(infrared_uart.push(ir.data(), ir.size()));
    CHECK(jy61_uart.push(imu.data(), imu.size()));
    controller.tick();
    CHECK(controller.navigator().state() == sailing::NavigationState::Track);
    CHECK(!drive.neutral_state);
    arm.estop_value = true;
    controller.tick();
    CHECK(controller.navigator().state() == sailing::NavigationState::EmergencyStop);
    CHECK(drive.neutral_state);

    sailing::hal::FakeDrive failed_drive;
    sailing::hal::FakeArmInput failed_arm;
    failed_drive.fail = true;
    sailing::BoatController failed(clock, infrared_uart, jy61_uart, failed_drive, failed_arm);
    CHECK(failed.output_fault());
    return true;
}

bool test_timeout_and_invalid_config() {
    sailing::NavigationConfig config{};
    config.mission_timeout_ms = 100U;
    sailing::Navigator navigator(config);
    navigator.arm(0U);
    (void)navigator.tick(100U, {}, {});
    CHECK(navigator.state() == sailing::NavigationState::Fault);
    config.minimum_confidence = 2.0F;
    sailing::Navigator invalid(config);
    CHECK(invalid.state() == sailing::NavigationState::Fault);
    return true;
}

}  // namespace

int main() {
    const std::vector<std::pair<const char*, bool (*)()>> tests{
        {"infrared_decoder_and_geometry", test_infrared_decoder_and_geometry},
        {"jy61_decoder", test_jy61_decoder},
        {"navigation_search_track_and_pass", test_navigation_search_track_and_pass},
        {"controller_and_safety_paths", test_controller_and_safety_paths},
        {"timeout_and_invalid_config", test_timeout_and_invalid_config},
    };
    std::size_t passed = 0U;
    for (const auto& test : tests) {
        if (test.second()) { ++passed; std::cout << "PASS " << test.first << '\n'; }
    }
    std::cout << passed << '/' << tests.size() << " test groups passed\n";
    return passed == tests.size() ? 0 : 1;
}
