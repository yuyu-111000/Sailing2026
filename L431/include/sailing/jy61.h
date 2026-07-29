#ifndef SAILING_JY61_H
#define SAILING_JY61_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    float yaw_deg;
    float yaw_rate_dps;
    uint64_t angle_received_ms;
    uint64_t rate_received_ms;
    bool angle_valid;
    bool rate_valid;
} sailing_imu_sample_t;

typedef struct {
    uint8_t frame[11U];
    size_t size;
    sailing_imu_sample_t sample;
} sailing_jy61_decoder_t;

void sailing_jy61_decoder_init(sailing_jy61_decoder_t *decoder);

bool sailing_jy61_decoder_feed(
    sailing_jy61_decoder_t *decoder,
    uint8_t byte,
    uint64_t now_ms);

const sailing_imu_sample_t *sailing_jy61_decoder_sample(
    const sailing_jy61_decoder_t *decoder);

#endif
