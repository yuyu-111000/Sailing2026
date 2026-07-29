#include "sailing/infrared_link.h"

#include <string.h>

uint16_t sailing_crc16_ccitt_false(const uint8_t *data, size_t size) {
    uint16_t crc = 0xFFFFU;
    size_t index;
    uint8_t bit;
    if (data == NULL && size != 0U) {
        return 0U;
    }
    for (index = 0U; index < size; ++index) {
        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

void sailing_infrared_encode(
    const uint16_t strength[SAILING_INFRARED_CHANNEL_COUNT],
    uint8_t sequence,
    uint8_t frame[SAILING_INFRARED_FRAME_SIZE]) {
    size_t index;
    uint16_t crc;
    if (strength == NULL || frame == NULL) {
        return;
    }
    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = sequence;
    for (index = 0U; index < SAILING_INFRARED_CHANNEL_COUNT; ++index) {
        const size_t offset = 3U + index * 2U;
        frame[offset] = (uint8_t)(strength[index] & 0xFFU);
        frame[offset + 1U] = (uint8_t)(strength[index] >> 8U);
    }
    crc = sailing_crc16_ccitt_false(frame, 35U);
    frame[35] = (uint8_t)(crc & 0xFFU);
    frame[36] = (uint8_t)(crc >> 8U);
}

void sailing_infrared_decoder_init(sailing_infrared_decoder_t *decoder) {
    if (decoder != NULL) {
        memset(decoder, 0, sizeof(*decoder));
    }
}

bool sailing_infrared_decoder_feed(
    sailing_infrared_decoder_t *decoder,
    uint8_t byte,
    uint64_t now_ms) {
    uint16_t expected;
    size_t index;
    if (decoder == NULL) {
        return false;
    }
    if (decoder->size == 0U && byte != 0xA5U) {
        return false;
    }
    if (decoder->size == 1U && byte != 0x5AU) {
        decoder->size = byte == 0xA5U ? 1U : 0U;
        if (decoder->size == 1U) {
            decoder->frame[0] = byte;
        }
        return false;
    }
    decoder->frame[decoder->size++] = byte;
    if (decoder->size != SAILING_INFRARED_FRAME_SIZE) {
        return false;
    }
    decoder->size = 0U;
    expected = (uint16_t)decoder->frame[35] |
        ((uint16_t)decoder->frame[36] << 8U);
    if (sailing_crc16_ccitt_false(decoder->frame, 35U) != expected) {
        return false;
    }
    if (decoder->has_sequence && decoder->frame[2] == decoder->sample.sequence) {
        return false;
    }
    decoder->sample.sequence = decoder->frame[2];
    for (index = 0U; index < SAILING_INFRARED_CHANNEL_COUNT; ++index) {
        const size_t offset = 3U + index * 2U;
        decoder->sample.strength[index] = (uint16_t)decoder->frame[offset] |
            ((uint16_t)decoder->frame[offset + 1U] << 8U);
    }
    decoder->sample.received_ms = now_ms;
    decoder->sample.valid = true;
    decoder->has_sequence = true;
    return true;
}

const sailing_infrared_sample_t *sailing_infrared_decoder_sample(
    const sailing_infrared_decoder_t *decoder) {
    return decoder == NULL ? NULL : &decoder->sample;
}
