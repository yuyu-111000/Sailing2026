#include "sailing/controller.h"

#include <stddef.h>
#include <string.h>

static void drain_inputs(sailing_boat_controller_t *controller, uint64_t now_ms) {
    uint8_t bytes[64U];
    size_t count;
    size_t index;
    count = controller->infrared_uart.read(
        controller->infrared_uart.context, bytes, sizeof(bytes));
    for (index = 0U; index < count && index < sizeof(bytes); ++index) {
        (void)sailing_infrared_decoder_feed(
            &controller->infrared_decoder, bytes[index], now_ms);
    }
    count = controller->jy61_uart.read(
        controller->jy61_uart.context, bytes, sizeof(bytes));
    for (index = 0U; index < count && index < sizeof(bytes); ++index) {
        (void)sailing_jy61_decoder_feed(&controller->jy61_decoder, bytes[index], now_ms);
    }
}

bool sailing_boat_controller_init(
    sailing_boat_controller_t *controller,
    sailing_l431_clock_t clock,
    sailing_l431_byte_input_t infrared_uart,
    sailing_l431_byte_input_t jy61_uart,
    sailing_l431_drive_t drive,
    sailing_l431_arm_input_t arm_input,
    const sailing_navigation_config_t *config) {
    if (controller == NULL || clock.now_ms == NULL || infrared_uart.read == NULL ||
        jy61_uart.read == NULL || drive.apply == NULL || drive.neutral == NULL ||
        arm_input.armed == NULL || arm_input.emergency_stop == NULL) {
        return false;
    }
    memset(controller, 0, sizeof(*controller));
    controller->clock = clock;
    controller->infrared_uart = infrared_uart;
    controller->jy61_uart = jy61_uart;
    controller->drive = drive;
    controller->arm_input = arm_input;
    sailing_infrared_decoder_init(&controller->infrared_decoder);
    sailing_jy61_decoder_init(&controller->jy61_decoder);
    if (!controller->drive.neutral(controller->drive.context)) {
        controller->output_fault = true;
        return false;
    }
    if (!sailing_navigator_init(&controller->navigator, config)) {
        return false;
    }
    controller->initialized = true;
    return true;
}

void sailing_boat_controller_tick(sailing_boat_controller_t *controller) {
    uint64_t now_ms;
    bool armed;
    sailing_drive_command_t command;
    if (controller == NULL || !controller->initialized) {
        return;
    }
    now_ms = controller->clock.now_ms(controller->clock.context);
    drain_inputs(controller, now_ms);
    if (controller->output_fault ||
        controller->arm_input.emergency_stop(controller->arm_input.context)) {
        sailing_navigator_emergency_stop(&controller->navigator);
        (void)controller->drive.neutral(controller->drive.context);
        return;
    }
    armed = controller->arm_input.armed(controller->arm_input.context);
    if (armed && !controller->was_armed) {
        sailing_navigator_arm(&controller->navigator, now_ms);
    }
    if (!armed) {
        sailing_navigator_disarm(&controller->navigator);
        if (!controller->drive.neutral(controller->drive.context)) {
            controller->output_fault = true;
        }
        controller->was_armed = false;
        return;
    }
    controller->was_armed = true;
    command = sailing_navigator_tick(
        &controller->navigator,
        now_ms,
        sailing_infrared_decoder_sample(&controller->infrared_decoder),
        sailing_jy61_decoder_sample(&controller->jy61_decoder));
    if (controller->navigator.state == SAILING_NAV_COMPLETE ||
        controller->navigator.state == SAILING_NAV_FAULT ||
        controller->navigator.state == SAILING_NAV_EMERGENCY_STOP) {
        if (!controller->drive.neutral(controller->drive.context)) {
            controller->output_fault = true;
        }
        return;
    }
    if (!controller->drive.apply(controller->drive.context, command.left, command.right)) {
        controller->output_fault = true;
        sailing_navigator_emergency_stop(&controller->navigator);
        (void)controller->drive.neutral(controller->drive.context);
    }
}
