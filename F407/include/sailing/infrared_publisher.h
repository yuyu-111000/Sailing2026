#ifndef SAILING_INFRARED_PUBLISHER_H
#define SAILING_INFRARED_PUBLISHER_H

#include "sailing/f407_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    sailing_f407_sampler_t sampler;
    sailing_f407_output_t output;
    uint8_t sequence;
    bool initialized;
} sailing_f407_publisher_t;

bool sailing_f407_publisher_init(
    sailing_f407_publisher_t *publisher,
    sailing_f407_sampler_t sampler,
    sailing_f407_output_t output);

bool sailing_f407_publisher_publish(sailing_f407_publisher_t *publisher);

uint8_t sailing_f407_publisher_next_sequence(
    const sailing_f407_publisher_t *publisher);

#endif
