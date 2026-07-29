#include "fakes.h"
#include "sailing/infrared_link.h"
#include "sailing/infrared_publisher.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    return false; } } while (0)

static bool test_protocol_round_trip(void) {
    uint16_t strength[SAILING_INFRARED_CHANNEL_COUNT] = {0U};
    uint8_t frame[SAILING_INFRARED_FRAME_SIZE] = {0U};
    sailing_infrared_decoder_t decoder;
    const sailing_infrared_sample_t *sample;
    size_t index;
    strength[0] = 10U;
    strength[7] = 0x1234U;
    strength[15] = 65535U;
    sailing_infrared_encode(strength, 42U, frame);
    sailing_infrared_decoder_init(&decoder);
    for (index = 0U; index < sizeof(frame); ++index) {
        (void)sailing_infrared_decoder_feed(&decoder, frame[index], 123U);
    }
    sample = sailing_infrared_decoder_sample(&decoder);
    CHECK(sample != NULL && sample->valid);
    CHECK(sample->sequence == 42U);
    CHECK(memcmp(sample->strength, strength, sizeof(strength)) == 0);
    CHECK(sample->received_ms == 123U);
    for (index = 0U; index < sizeof(frame); ++index) {
        (void)sailing_infrared_decoder_feed(&decoder, frame[index], 999U);
    }
    CHECK(sample->received_ms == 123U);
    CHECK(sailing_crc16_ccitt_false((const uint8_t *)"123456789", 9U) == 0x29B1U);
    CHECK(sailing_crc16_ccitt_false(NULL, 1U) == 0U);
    CHECK(sailing_infrared_decoder_sample(NULL) == NULL);
    sailing_infrared_decoder_init(NULL);

    frame[10] ^= 0x01U;
    sailing_infrared_decoder_init(&decoder);
    for (index = 0U; index < sizeof(frame); ++index) {
        (void)sailing_infrared_decoder_feed(&decoder, frame[index], 200U);
    }
    CHECK(!decoder.sample.valid);
    CHECK(!sailing_infrared_decoder_feed(NULL, 0U, 0U));
    return true;
}

static bool test_publisher_success_and_sequence(void) {
    fake_f407_sampler_t sampler = {0};
    fake_f407_output_t output = {0};
    sailing_f407_publisher_t publisher;
    sailing_infrared_decoder_t decoder;
    sailing_f407_sampler_t sampler_hal = {&sampler, fake_f407_snapshot};
    sailing_f407_output_t output_hal = {&output, fake_f407_write};
    size_t index;
    CHECK(sailing_f407_publisher_init(&publisher, sampler_hal, output_hal));
    sampler.strength[3] = 77U;
    CHECK(sailing_f407_publisher_publish(&publisher));
    CHECK(output.size_written == SAILING_INFRARED_FRAME_SIZE);
    CHECK(sailing_f407_publisher_next_sequence(&publisher) == 1U);
    CHECK(sampler.strength[3] == 0U);
    sailing_infrared_decoder_init(&decoder);
    for (index = 0U; index < output.size_written; ++index) {
        (void)sailing_infrared_decoder_feed(&decoder, output.bytes[index], 50U);
    }
    CHECK(decoder.sample.strength[3] == 77U);
    CHECK(decoder.sample.sequence == 0U);
    CHECK(sailing_f407_publisher_publish(&publisher));
    CHECK(sailing_f407_publisher_next_sequence(&publisher) == 2U);
    return true;
}

static bool test_publisher_failures(void) {
    fake_f407_sampler_t sampler = {0};
    fake_f407_output_t output = {0};
    sailing_f407_publisher_t publisher = {0};
    sailing_f407_sampler_t sampler_hal = {&sampler, fake_f407_snapshot};
    sailing_f407_output_t output_hal = {&output, fake_f407_write};
    sailing_f407_sampler_t bad_sampler = {NULL, NULL};
    CHECK(!sailing_f407_publisher_init(NULL, sampler_hal, output_hal));
    CHECK(!sailing_f407_publisher_init(&publisher, bad_sampler, output_hal));
    CHECK(!sailing_f407_publisher_publish(NULL));
    CHECK(!sailing_f407_publisher_publish(&publisher));
    CHECK(sailing_f407_publisher_init(&publisher, sampler_hal, output_hal));
    sampler.fail = true;
    CHECK(!sailing_f407_publisher_publish(&publisher));
    CHECK(output.write_calls == 0U);
    sampler.fail = false;
    output.fail = true;
    CHECK(!sailing_f407_publisher_publish(&publisher));
    CHECK(sailing_f407_publisher_next_sequence(&publisher) == 0U);
    CHECK(sailing_f407_publisher_next_sequence(NULL) == 0U);
    return true;
}

int main(void) {
    const bool results[] = {
        test_protocol_round_trip(),
        test_publisher_success_and_sequence(),
        test_publisher_failures(),
    };
    size_t index;
    size_t passed = 0U;
    for (index = 0U; index < sizeof(results) / sizeof(results[0]); ++index) {
        if (results[index]) {
            ++passed;
        }
    }
    printf("%zu/%zu F407 test groups passed\n", passed, sizeof(results) / sizeof(results[0]));
    return passed == sizeof(results) / sizeof(results[0]) ? 0 : 1;
}
