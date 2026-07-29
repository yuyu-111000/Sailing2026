#ifndef SAILING_L431_TEST_FAKES_H
#define SAILING_L431_TEST_FAKES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define FAKE_INPUT_CAPACITY 512U

typedef struct { uint64_t value; } fake_clock_t;

typedef struct {
    uint8_t buffer[FAKE_INPUT_CAPACITY];
    size_t size;
} fake_byte_input_t;

typedef struct {
    float left;
    float right;
    size_t apply_calls;
    size_t neutral_calls;
    bool neutral_state;
    bool fail;
} fake_drive_t;

typedef struct {
    bool armed_value;
    bool estop_value;
} fake_arm_input_t;

static uint64_t fake_clock_now(void *context) {
    return ((fake_clock_t *)context)->value;
}

static bool fake_input_push(fake_byte_input_t *input, const uint8_t *data, size_t size) {
    if (input == NULL || (data == NULL && size != 0U) || input->size + size > sizeof(input->buffer)) {
        return false;
    }
    memcpy(input->buffer + input->size, data, size);
    input->size += size;
    return true;
}

static size_t fake_input_read(void *context, uint8_t *destination, size_t capacity) {
    fake_byte_input_t *input = (fake_byte_input_t *)context;
    size_t count;
    if (destination == NULL && capacity != 0U) {
        return 0U;
    }
    count = input->size < capacity ? input->size : capacity;
    memcpy(destination, input->buffer, count);
    memmove(input->buffer, input->buffer + count, input->size - count);
    input->size -= count;
    return count;
}

static bool fake_drive_apply(void *context, float left, float right) {
    fake_drive_t *drive = (fake_drive_t *)context;
    ++drive->apply_calls;
    if (drive->fail) {
        return false;
    }
    drive->left = left;
    drive->right = right;
    drive->neutral_state = false;
    return true;
}

static bool fake_drive_neutral(void *context) {
    fake_drive_t *drive = (fake_drive_t *)context;
    ++drive->neutral_calls;
    if (drive->fail) {
        return false;
    }
    drive->left = 0.0F;
    drive->right = 0.0F;
    drive->neutral_state = true;
    return true;
}

static bool fake_arm_armed(void *context) {
    return ((fake_arm_input_t *)context)->armed_value;
}

static bool fake_arm_estop(void *context) {
    return ((fake_arm_input_t *)context)->estop_value;
}

#endif
