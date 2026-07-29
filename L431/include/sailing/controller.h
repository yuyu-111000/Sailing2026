#ifndef SAILING_CONTROLLER_H
#define SAILING_CONTROLLER_H

#include "sailing/infrared_link.h"
#include "sailing/jy61.h"
#include "sailing/l431_hal.h"
#include "sailing/navigation.h"

#include <stdbool.h>

typedef struct {
    sailing_l431_clock_t clock;
    sailing_l431_byte_input_t infrared_uart;
    sailing_l431_byte_input_t jy61_uart;
    sailing_l431_drive_t drive;
    sailing_l431_arm_input_t arm_input;
    sailing_infrared_decoder_t infrared_decoder;
    sailing_jy61_decoder_t jy61_decoder;
    sailing_navigator_t navigator;
    bool was_armed;
    bool output_fault;
    bool initialized;
} sailing_boat_controller_t;

bool sailing_boat_controller_init(
    sailing_boat_controller_t *controller,
    sailing_l431_clock_t clock,
    sailing_l431_byte_input_t infrared_uart,
    sailing_l431_byte_input_t jy61_uart,
    sailing_l431_drive_t drive,
    sailing_l431_arm_input_t arm_input,
    const sailing_navigation_config_t *config);

void sailing_boat_controller_tick(sailing_boat_controller_t *controller);

#endif
