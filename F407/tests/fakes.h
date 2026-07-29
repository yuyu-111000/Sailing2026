#ifndef SAILING_F407_TEST_FAKES_H
#define SAILING_F407_TEST_FAKES_H

#include "sailing/infrared_link.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint16_t strength[SAILING_INFRARED_CHANNEL_COUNT];
    size_t snapshot_calls;
    bool fail;
} fake_f407_sampler_t;

typedef struct {
    uint8_t bytes[SAILING_INFRARED_FRAME_SIZE];
    size_t size_written;
    size_t write_calls;
    bool fail;
} fake_f407_output_t;

static bool fake_f407_snapshot(void *context, uint16_t *destination) {
    fake_f407_sampler_t *sampler = (fake_f407_sampler_t *)context;
    ++sampler->snapshot_calls;
    if (sampler->fail) {
        return false;
    }
    memcpy(destination, sampler->strength, sizeof(sampler->strength));
    memset(sampler->strength, 0, sizeof(sampler->strength));
    return true;
}

static bool fake_f407_write(void *context, const uint8_t *data, size_t size) {
    fake_f407_output_t *output = (fake_f407_output_t *)context;
    ++output->write_calls;
    if (output->fail || data == NULL || size != sizeof(output->bytes)) {
        return false;
    }
    memcpy(output->bytes, data, size);
    output->size_written = size;
    return true;
}

#endif
