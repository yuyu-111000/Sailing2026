#include "fakes.h"
#include "sailing/controller.h"
#include "sailing/infrared_link.h"
#include "sailing/jy61.h"
#include "sailing/navigation.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    return false; } } while (0)

static void make_jy61_frame(uint8_t type, int16_t z, uint8_t frame[11U]) {
    size_t index;
    memset(frame, 0, 11U);
    frame[0] = 0x55U;
    frame[1] = type;
    frame[6] = (uint8_t)((uint16_t)z & 0xFFU);
    frame[7] = (uint8_t)((uint16_t)z >> 8U);
    for (index = 0U; index < 10U; ++index) {
        frame[10] = (uint8_t)(frame[10] + frame[index]);
    }
}

static bool test_jy61_decoder(void) {
    sailing_jy61_decoder_t decoder;
    uint8_t frame[11U];
    size_t index;
    sailing_jy61_decoder_init(&decoder);
    make_jy61_frame(0x52U, 16384, frame);
    for (index = 0U; index < sizeof(frame); ++index) {
        (void)sailing_jy61_decoder_feed(&decoder, frame[index], 10U);
    }
    CHECK(decoder.sample.rate_valid);
    CHECK(fabsf(decoder.sample.yaw_rate_dps - 1000.0F) < 0.1F);
    make_jy61_frame(0x53U, -16384, frame);
    for (index = 0U; index < sizeof(frame); ++index) {
        (void)sailing_jy61_decoder_feed(&decoder, frame[index], 20U);
    }
    CHECK(decoder.sample.angle_valid);
    CHECK(fabsf(decoder.sample.yaw_deg + 90.0F) < 0.1F);
    CHECK(decoder.sample.rate_received_ms == 10U);
    CHECK(decoder.sample.angle_received_ms == 20U);
    frame[10] ^= 1U;
    sailing_jy61_decoder_init(&decoder);
    for (index = 0U; index < sizeof(frame); ++index) {
        (void)sailing_jy61_decoder_feed(&decoder, frame[index], 30U);
    }
    CHECK(!decoder.sample.angle_valid);
    CHECK(!sailing_jy61_decoder_feed(NULL, 0U, 0U));
    CHECK(sailing_jy61_decoder_sample(NULL) == NULL);
    sailing_jy61_decoder_init(NULL);
    return true;
}

static bool test_bearing_geometry(void) {
    sailing_infrared_sample_t infrared = {0};
    sailing_bearing_estimate_t bearing;
    infrared.valid = true;
    infrared.strength[0] = 100U;
    infrared.strength[1] = 50U;
    infrared.strength[15] = 50U;
    CHECK(sailing_estimate_bearing(&infrared, &bearing));
    CHECK(fabsf(bearing.angle_deg) < 0.01F);
    CHECK(bearing.confidence > 0.9F);
    infrared.valid = false;
    CHECK(!sailing_estimate_bearing(&infrared, &bearing));
    CHECK(!sailing_estimate_bearing(NULL, &bearing));
    CHECK(!sailing_estimate_bearing(&infrared, NULL));
    return true;
}

static bool test_navigation_states(void) {
    sailing_navigation_config_t config = sailing_navigation_default_config();
    sailing_navigator_t navigator;
    sailing_infrared_sample_t infrared = {0};
    sailing_imu_sample_t imu = {0};
    sailing_drive_command_t command;
    config.approach_strength = 80.0F;
    config.pass_loss_ms = 100U;
    config.pass_forward_ms = 200U;
    config.mission_timeout_ms = 1000U;
    CHECK(sailing_navigator_init(&navigator, &config));
    sailing_navigator_arm(&navigator, 0U);
    command = sailing_navigator_tick(&navigator, 0U, &infrared, &imu);
    CHECK(navigator.state == SAILING_NAV_SEARCH);
    CHECK(command.left < 0.0F && command.right > 0.0F);

    infrared.valid = true;
    infrared.received_ms = 10U;
    infrared.strength[2] = 100U;
    command = sailing_navigator_tick(&navigator, 10U, &infrared, &imu);
    CHECK(navigator.state == SAILING_NAV_TRACK);
    CHECK(command.right > command.left);

    memset(infrared.strength, 0, sizeof(infrared.strength));
    infrared.strength[0] = 100U;
    infrared.received_ms = 20U;
    command = sailing_navigator_tick(&navigator, 20U, &infrared, &imu);
    CHECK(command.left == command.right);
    infrared.valid = false;
    (void)sailing_navigator_tick(&navigator, 119U, &infrared, &imu);
    CHECK(navigator.state == SAILING_NAV_SEARCH);
    (void)sailing_navigator_tick(&navigator, 120U, &infrared, &imu);
    CHECK(navigator.state == SAILING_NAV_PASS_THROUGH);
    (void)sailing_navigator_tick(&navigator, 320U, &infrared, &imu);
    CHECK(navigator.passed_gates == 1U);
    CHECK(navigator.state == SAILING_NAV_SEARCH);
    (void)sailing_navigator_tick(&navigator, 1000U, &infrared, &imu);
    CHECK(navigator.state == SAILING_NAV_FAULT);

    config = sailing_navigation_default_config();
    config.pass_forward_ms = 200U;
    CHECK(sailing_navigator_init(&navigator, &config));
    sailing_navigator_arm(&navigator, 0U);
    navigator.state = SAILING_NAV_PASS_THROUGH;
    navigator.pass_started_ms = 0U;
    navigator.passed_gates = 9U;
    command = sailing_navigator_tick(&navigator, 200U, &infrared, &imu);
    CHECK(navigator.state == SAILING_NAV_COMPLETE);
    CHECK(command.left == 0.0F && command.right == 0.0F);

    config.minimum_confidence = 2.0F;
    CHECK(!sailing_navigator_init(&navigator, &config));
    CHECK(navigator.state == SAILING_NAV_FAULT);
    CHECK(!sailing_navigator_init(NULL, &config));
    sailing_navigator_disarm(NULL);
    sailing_navigator_emergency_stop(NULL);
    command = sailing_navigator_tick(NULL, 0U, &infrared, &imu);
    CHECK(command.left == 0.0F && command.right == 0.0F);
    return true;
}

static bool test_controller_safety(void) {
    fake_clock_t clock = {0U};
    fake_byte_input_t infrared_uart = {{0U}, 0U};
    fake_byte_input_t jy61_uart = {{0U}, 0U};
    fake_drive_t drive = {0.0F, 0.0F, 0U, 0U, false, false};
    fake_arm_input_t arm = {false, false};
    sailing_boat_controller_t controller;
    sailing_navigation_config_t config = sailing_navigation_default_config();
    sailing_l431_clock_t clock_hal = {&clock, fake_clock_now};
    sailing_l431_byte_input_t infrared_hal = {&infrared_uart, fake_input_read};
    sailing_l431_byte_input_t jy61_hal = {&jy61_uart, fake_input_read};
    sailing_l431_drive_t drive_hal = {&drive, fake_drive_apply, fake_drive_neutral};
    sailing_l431_arm_input_t arm_hal = {&arm, fake_arm_armed, fake_arm_estop};
    uint16_t strength[SAILING_INFRARED_CHANNEL_COUNT] = {0U};
    uint8_t infrared_frame[SAILING_INFRARED_FRAME_SIZE];
    uint8_t imu_frame[11U];
    CHECK(sailing_boat_controller_init(
        &controller, clock_hal, infrared_hal, jy61_hal, drive_hal, arm_hal, &config));
    sailing_boat_controller_tick(&controller);
    CHECK(drive.neutral_state);
    arm.armed_value = true;
    strength[0] = 200U;
    sailing_infrared_encode(strength, 1U, infrared_frame);
    make_jy61_frame(0x52U, 0, imu_frame);
    CHECK(fake_input_push(&infrared_uart, infrared_frame, sizeof(infrared_frame)));
    CHECK(fake_input_push(&jy61_uart, imu_frame, sizeof(imu_frame)));
    sailing_boat_controller_tick(&controller);
    CHECK(controller.navigator.state == SAILING_NAV_TRACK);
    CHECK(!drive.neutral_state);
    arm.estop_value = true;
    sailing_boat_controller_tick(&controller);
    CHECK(controller.navigator.state == SAILING_NAV_EMERGENCY_STOP);
    CHECK(drive.neutral_state);

    drive.fail = true;
    CHECK(!sailing_boat_controller_init(
        &controller, clock_hal, infrared_hal, jy61_hal, drive_hal, arm_hal, &config));
    CHECK(controller.output_fault);
    CHECK(!sailing_boat_controller_init(
        NULL, clock_hal, infrared_hal, jy61_hal, drive_hal, arm_hal, &config));
    sailing_boat_controller_tick(NULL);
    return true;
}

int main(void) {
    const bool results[] = {
        test_jy61_decoder(),
        test_bearing_geometry(),
        test_navigation_states(),
        test_controller_safety(),
    };
    size_t index;
    size_t passed = 0U;
    for (index = 0U; index < sizeof(results) / sizeof(results[0]); ++index) {
        if (results[index]) {
            ++passed;
        }
    }
    printf("%zu/%zu L431 test groups passed\n", passed, sizeof(results) / sizeof(results[0]));
    return passed == sizeof(results) / sizeof(results[0]) ? 0 : 1;
}
