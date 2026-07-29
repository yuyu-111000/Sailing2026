#include "sailing/jy61.h"

#include <string.h>

static int16_t signed_word(uint8_t low, uint8_t high) {
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8U));
}

void sailing_jy61_decoder_init(sailing_jy61_decoder_t *decoder) {
    if (decoder != NULL) {
        memset(decoder, 0, sizeof(*decoder));
    }
}

bool sailing_jy61_decoder_feed(
    sailing_jy61_decoder_t *decoder,
    uint8_t byte,
    uint64_t now_ms) {
    uint8_t sum = 0U;
    size_t index;
    if (decoder == NULL) {
        return false;
    }
    if (decoder->size == 0U && byte != 0x55U) {
        return false;
    }
    decoder->frame[decoder->size++] = byte;
    if (decoder->size == 2U && decoder->frame[1] != 0x52U && decoder->frame[1] != 0x53U) {
        decoder->size = byte == 0x55U ? 1U : 0U;
        return false;
    }
    if (decoder->size != sizeof(decoder->frame)) {
        return false;
    }
    decoder->size = 0U;
    for (index = 0U; index < 10U; ++index) {
        sum = (uint8_t)(sum + decoder->frame[index]);
    }
    if (sum != decoder->frame[10]) {
        return false;
    }
    if (decoder->frame[1] == 0x52U) {
        decoder->sample.yaw_rate_dps = (float)signed_word(
            decoder->frame[6], decoder->frame[7]) / 32768.0F * 2000.0F;
        decoder->sample.rate_valid = true;
        decoder->sample.rate_received_ms = now_ms;
    } else {
        decoder->sample.yaw_deg = (float)signed_word(
            decoder->frame[6], decoder->frame[7]) / 32768.0F * 180.0F;
        decoder->sample.angle_valid = true;
        decoder->sample.angle_received_ms = now_ms;
    }
    return true;
}

const sailing_imu_sample_t *sailing_jy61_decoder_sample(
    const sailing_jy61_decoder_t *decoder) {
    return decoder == NULL ? NULL : &decoder->sample;
}
