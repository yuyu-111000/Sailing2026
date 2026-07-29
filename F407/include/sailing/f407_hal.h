#ifndef SAILING_F407_HAL_H
#define SAILING_F407_HAL_H

#include "sailing/infrared_link.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef bool (*sailing_f407_snapshot_fn)(
    void *context,
    uint16_t strength[SAILING_INFRARED_CHANNEL_COUNT]);

typedef bool (*sailing_f407_write_fn)(
    void *context,
    const uint8_t *data,
    size_t size);

typedef struct {
    void *context;
    sailing_f407_snapshot_fn snapshot_and_reset;
} sailing_f407_sampler_t;

typedef struct {
    void *context;
    sailing_f407_write_fn write;
} sailing_f407_output_t;

#endif
