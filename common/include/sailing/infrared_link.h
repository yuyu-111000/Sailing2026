#ifndef SAILING_INFRARED_LINK_H
#define SAILING_INFRARED_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SAILING_INFRARED_CHANNEL_COUNT 16U
#define SAILING_INFRARED_FRAME_SIZE 37U

typedef struct {
    uint16_t strength[SAILING_INFRARED_CHANNEL_COUNT];
    uint8_t sequence;
    uint64_t received_ms;
    bool valid;
} sailing_infrared_sample_t;

typedef struct {
    uint8_t frame[SAILING_INFRARED_FRAME_SIZE];
    size_t size;
    sailing_infrared_sample_t sample;
    bool has_sequence;
} sailing_infrared_decoder_t;

uint16_t sailing_crc16_ccitt_false(const uint8_t *data, size_t size);

void sailing_infrared_encode(
    const uint16_t strength[SAILING_INFRARED_CHANNEL_COUNT],
    uint8_t sequence,
    uint8_t frame[SAILING_INFRARED_FRAME_SIZE]);

void sailing_infrared_decoder_init(sailing_infrared_decoder_t *decoder);

bool sailing_infrared_decoder_feed(
    sailing_infrared_decoder_t *decoder,
    uint8_t byte,
    uint64_t now_ms);

const sailing_infrared_sample_t *sailing_infrared_decoder_sample(
    const sailing_infrared_decoder_t *decoder);

#endif
