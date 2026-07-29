#ifndef SAILING_L431_HAL_H
#define SAILING_L431_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint64_t (*sailing_l431_now_ms_fn)(void *context);
typedef size_t (*sailing_l431_read_fn)(void *context, uint8_t *destination, size_t capacity);
typedef bool (*sailing_l431_drive_apply_fn)(void *context, float left, float right);
typedef bool (*sailing_l431_drive_neutral_fn)(void *context);
typedef bool (*sailing_l431_input_fn)(void *context);

typedef struct {
    void *context;
    sailing_l431_now_ms_fn now_ms;
} sailing_l431_clock_t;

typedef struct {
    void *context;
    sailing_l431_read_fn read;
} sailing_l431_byte_input_t;

typedef struct {
    void *context;
    sailing_l431_drive_apply_fn apply;
    sailing_l431_drive_neutral_fn neutral;
} sailing_l431_drive_t;

typedef struct {
    void *context;
    sailing_l431_input_fn armed;
    sailing_l431_input_fn emergency_stop;
} sailing_l431_arm_input_t;

#endif
