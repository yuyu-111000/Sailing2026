#include "sailing/infrared_publisher.h"

#include "sailing/infrared_link.h"

#include <string.h>

bool sailing_f407_publisher_init(
    sailing_f407_publisher_t *publisher,
    sailing_f407_sampler_t sampler,
    sailing_f407_output_t output) {
    if (publisher == NULL || sampler.snapshot_and_reset == NULL || output.write == NULL) {
        return false;
    }
    memset(publisher, 0, sizeof(*publisher));
    publisher->sampler = sampler;
    publisher->output = output;
    publisher->initialized = true;
    return true;
}

bool sailing_f407_publisher_publish(sailing_f407_publisher_t *publisher) {
    uint16_t strength[SAILING_INFRARED_CHANNEL_COUNT] = {0U};
    uint8_t frame[SAILING_INFRARED_FRAME_SIZE] = {0U};
    if (publisher == NULL || !publisher->initialized) {
        return false;
    }
    if (!publisher->sampler.snapshot_and_reset(publisher->sampler.context, strength)) {
        return false;
    }
    sailing_infrared_encode(strength, publisher->sequence, frame);
    if (!publisher->output.write(
            publisher->output.context, frame, SAILING_INFRARED_FRAME_SIZE)) {
        return false;
    }
    ++publisher->sequence;
    return true;
}

uint8_t sailing_f407_publisher_next_sequence(
    const sailing_f407_publisher_t *publisher) {
    return publisher == NULL ? 0U : publisher->sequence;
}
