#include "fakes.hpp"
#include "sailing/infrared_link.hpp"
#include "sailing/infrared_publisher.hpp"

#include <cstdint>
#include <iostream>

namespace {

#define CHECK(condition) do { if (!(condition)) { std::cerr << "CHECK failed at " \
    << __FILE__ << ':' << __LINE__ << ": " #condition << '\n'; return false; } } while (false)

bool test_protocol_round_trip() {
    std::array<std::uint16_t, sailing::kInfraredChannelCount> strength{};
    strength[0] = 10U;
    strength[7] = 0x1234U;
    strength[15] = 65535U;
    auto frame = sailing::encode_infrared_frame(strength, 42U);
    sailing::InfraredStreamDecoder decoder;
    for (const auto byte : frame.bytes) { (void)decoder.feed(byte, 123U); }
    CHECK(decoder.sample().valid);
    CHECK(decoder.sample().sequence == 42U);
    CHECK(decoder.sample().strength == strength);
    CHECK(decoder.sample().received_ms == 123U);
    for (const auto byte : frame.bytes) { (void)decoder.feed(byte, 999U); }
    CHECK(decoder.sample().received_ms == 123U);
    CHECK(sailing::crc16_ccitt_false(
        reinterpret_cast<const std::uint8_t*>("123456789"), 9U) == 0x29B1U);
    CHECK(sailing::crc16_ccitt_false(nullptr, 1U) == 0U);

    frame.bytes[10] ^= 0x01U;
    sailing::InfraredStreamDecoder damaged;
    for (const auto byte : frame.bytes) { (void)damaged.feed(byte, 200U); }
    CHECK(!damaged.sample().valid);
    return true;
}

bool test_publisher_success_and_sequence() {
    sailing::f407::FakeInfraredSampler sampler;
    sailing::f407::FakeByteOutput output;
    sailing::f407::InfraredPublisher publisher(sampler, output);
    sampler.strength[3] = 77U;
    CHECK(publisher.publish());
    CHECK(output.size_written == sailing::kInfraredFrameSize);
    CHECK(publisher.next_sequence() == 1U);
    CHECK(sampler.strength[3] == 0U);

    sailing::InfraredStreamDecoder decoder;
    for (const auto byte : output.bytes) { (void)decoder.feed(byte, 50U); }
    CHECK(decoder.sample().strength[3] == 77U);
    CHECK(decoder.sample().sequence == 0U);
    CHECK(publisher.publish());
    CHECK(publisher.next_sequence() == 2U);
    return true;
}

bool test_publisher_failures_do_not_advance_sequence() {
    sailing::f407::FakeInfraredSampler sampler;
    sailing::f407::FakeByteOutput output;
    sailing::f407::InfraredPublisher publisher(sampler, output);
    sampler.fail = true;
    CHECK(!publisher.publish());
    CHECK(output.write_calls == 0U);
    CHECK(publisher.next_sequence() == 0U);
    sampler.fail = false;
    output.fail = true;
    CHECK(!publisher.publish());
    CHECK(publisher.next_sequence() == 0U);
    return true;
}

}  // namespace

int main() {
    const bool results[]{
        test_protocol_round_trip(),
        test_publisher_success_and_sequence(),
        test_publisher_failures_do_not_advance_sequence(),
    };
    std::size_t passed = 0U;
    for (const bool result : results) { if (result) { ++passed; } }
    std::cout << passed << '/' << std::size(results) << " F407 test groups passed\n";
    return passed == std::size(results) ? 0 : 1;
}
