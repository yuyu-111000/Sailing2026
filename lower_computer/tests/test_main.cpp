#include "sailing/fakes.hpp"
#include "sailing/protocol.hpp"
#include "sailing/safety_supervisor.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct GoldenVector {
    std::string name;
    bool valid{false};
    std::vector<std::uint8_t> bytes;
};

std::vector<GoldenVector> load_vectors(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open shared protocol vectors: " + path);
    }
    std::vector<GoldenVector> vectors;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#') {
            continue;
        }
        std::istringstream fields(line);
        GoldenVector item;
        std::string validity;
        std::string hexadecimal;
        if (!std::getline(fields, item.name, '\t') ||
            !std::getline(fields, validity, '\t') ||
            !std::getline(fields, hexadecimal)) {
            throw std::runtime_error("invalid TSV vector row");
        }
        item.valid = validity == "true";
        std::istringstream bytes(hexadecimal);
        std::string token;
        while (bytes >> token) {
            item.bytes.push_back(static_cast<std::uint8_t>(std::stoul(token, nullptr, 16)));
        }
        vectors.push_back(item);
    }
    return vectors;
}

const GoldenVector& find_vector(
    const std::vector<GoldenVector>& vectors,
    const std::string& name) {
    const auto found = std::find_if(
        vectors.begin(), vectors.end(), [&name](const GoldenVector& item) { return item.name == name; });
    if (found == vectors.end()) {
        throw std::runtime_error("missing vector: " + name);
    }
    return *found;
}

class CollectingSink final : public sailing::protocol::FrameSink {
public:
    void on_frame(const sailing::protocol::Frame& frame) noexcept override {
        frames.push_back(frame);
    }

    void on_parse_error(sailing::protocol::ParseError error) noexcept override {
        errors.push_back(error);
    }

    std::vector<sailing::protocol::Frame> frames;
    std::vector<sailing::protocol::ParseError> errors;
};

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            std::cerr << "CHECK failed at " << __FILE__ << ':' << __LINE__ \
                      << ": " #condition << '\n'; \
            return false; \
        } \
    } while (false)

bool test_protocol_vectors(const std::vector<GoldenVector>& vectors) {
    CHECK(vectors.size() == 4U);
    for (const auto& item : vectors) {
        sailing::protocol::StreamDecoder decoder;
        CollectingSink sink;
        for (std::size_t offset = 0U; offset < item.bytes.size(); offset += 3U) {
            const auto count = std::min<std::size_t>(3U, item.bytes.size() - offset);
            decoder.feed(item.bytes.data() + offset, count, sink);
        }
        CHECK((sink.frames.size() == 1U) == item.valid);
        CHECK((sink.errors.empty()) == item.valid);
        CHECK(decoder.buffered_size() == 0U);
    }

    const auto& expected = find_vector(vectors, "control").bytes;
    sailing::protocol::Frame frame{};
    frame.session_id = 0x11223344U;
    frame.sequence = 0x01020304U;
    frame.sender_time_ms = 123456U;
    CHECK(sailing::protocol::set_control_payload(frame, {0.5F, -0.25F, 500U}));
    sailing::protocol::EncodedFrame encoded{};
    CHECK(sailing::protocol::encode_frame(frame, encoded));
    CHECK(encoded.size == expected.size());
    CHECK(std::equal(expected.begin(), expected.end(), encoded.bytes.begin()));
    sailing::protocol::ControlSetpoint decoded_control{};
    CHECK(sailing::protocol::decode_control_payload(frame, decoded_control));
    CHECK(decoded_control.propulsion == 0.5F);
    CHECK(decoded_control.steering == -0.25F);
    CHECK(decoded_control.valid_for_ms == 500U);
    sailing::protocol::Frame fire_frame{};
    const sailing::protocol::FireOnceRequest fire_request{
        7U,
        sailing::protocol::ShotSource::Auto,
        250U,
    };
    CHECK(sailing::protocol::set_fire_once_payload(fire_frame, fire_request));
    sailing::protocol::FireOnceRequest decoded_fire{};
    CHECK(sailing::protocol::decode_fire_once_payload(fire_frame, decoded_fire));
    CHECK(decoded_fire.shot_id == fire_request.shot_id);
    CHECK(decoded_fire.source == fire_request.source);
    CHECK(decoded_fire.valid_for_ms == fire_request.valid_for_ms);
    CHECK(sailing::protocol::crc16_ccitt_false(
        reinterpret_cast<const std::uint8_t*>("123456789"), 9U) == 0x29B1U);
    CHECK(sailing::protocol::crc16_ccitt_false(nullptr, 1U) == 0U);
    CHECK(!sailing::protocol::is_known_message_type(0xFFU));
    return true;
}

bool test_protocol_resynchronization(const std::vector<GoldenVector>& vectors) {
    const auto& bad = find_vector(vectors, "bad_crc").bytes;
    const auto& estop = find_vector(vectors, "estop").bytes;
    std::vector<std::uint8_t> stream{0x00U, 0x99U, 0x53U};
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), estop.begin(), estop.end());
    sailing::protocol::StreamDecoder decoder;
    CollectingSink sink;
    decoder.feed(stream.data(), stream.size(), sink);
    CHECK(sink.frames.size() == 1U);
    CHECK(sink.frames.front().type == sailing::protocol::MessageType::EStop);
    CHECK(!sink.errors.empty());

    sailing::protocol::Frame invalid{};
    invalid.flags = 1U;
    sailing::protocol::EncodedFrame encoded{};
    CHECK(!sailing::protocol::encode_frame(invalid, encoded));
    CHECK(!sailing::protocol::set_control_payload(invalid, {1.1F, 0.0F, 500U}));
    CHECK(!sailing::protocol::set_fire_once_payload(
        invalid,
        {1U, static_cast<sailing::protocol::ShotSource>(99U), 250U}));
    sailing::protocol::ControlSetpoint decoded_control{};
    sailing::protocol::FireOnceRequest decoded_fire{};
    CHECK(!sailing::protocol::decode_control_payload(invalid, decoded_control));
    CHECK(!sailing::protocol::decode_fire_once_payload(invalid, decoded_fire));

    const auto& control = find_vector(vectors, "control").bytes;
    for (const auto mutation : std::vector<std::pair<std::size_t, std::uint8_t>>{
             {2U, 2U},
             {3U, 0xFFU},
             {4U, 1U},
         }) {
        auto malformed = control;
        malformed[mutation.first] = mutation.second;
        sailing::protocol::StreamDecoder malformed_decoder;
        CollectingSink malformed_sink;
        malformed_decoder.feed(malformed.data(), malformed.size(), malformed_sink);
        CHECK(malformed_sink.frames.empty());
        CHECK(!malformed_sink.errors.empty());
    }
    auto oversized = control;
    oversized[13] = 0x01U;
    oversized[14] = 0x01U;
    sailing::protocol::StreamDecoder oversized_decoder;
    CollectingSink oversized_sink;
    oversized_decoder.feed(oversized.data(), oversized.size(), oversized_sink);
    CHECK(oversized_sink.frames.empty());
    CHECK(!oversized_sink.errors.empty());
    oversized_decoder.feed(nullptr, 1U, oversized_sink);
    CHECK(oversized_sink.errors.size() >= 2U);

    auto length_poisoned = control;
    length_poisoned[13] = 30U;
    length_poisoned[14] = 0U;
    length_poisoned.insert(length_poisoned.end(), estop.begin(), estop.end());
    sailing::protocol::StreamDecoder recovery_decoder;
    CollectingSink recovery_sink;
    recovery_decoder.feed(length_poisoned.data(), length_poisoned.size(), recovery_sink);
    CHECK(recovery_sink.frames.size() == 1U);
    CHECK(recovery_sink.frames.front().type == sailing::protocol::MessageType::EStop);
    return true;
}

bool test_watchdog_and_estop() {
    sailing::hal::FakeClock clock;
    sailing::hal::FakeMotionOutput motion;
    sailing::hal::FakeLauncherOutput launcher;
    sailing::hal::FakeEventSink events;
    sailing::SafetySupervisor safety(clock, motion, launcher, events);
    CHECK(safety.state() == sailing::SafetyState::Disarmed);
    CHECK(motion.neutral_state);
    CHECK(!launcher.armed);
    CHECK(safety.begin_session(sailing::MissionMode::C1, 10U, 1U, false));
    CHECK(safety.start(10U, 2U));
    CHECK(safety.apply_control(10U, 3U, {0.5F, -0.25F, 500U}));
    CHECK(!motion.neutral_state);
    clock.advance_ms(499U);
    safety.heartbeat();
    safety.tick();
    CHECK(safety.state() == sailing::SafetyState::Running);
    clock.advance_ms(1U);
    safety.tick();
    CHECK(safety.state() == sailing::SafetyState::Fault);
    CHECK(motion.neutral_state);
    CHECK(!launcher.armed);
    CHECK(events.contains(sailing::hal::EventCode::WatchdogExpired));
    CHECK(safety.reset());
    CHECK(safety.state() == sailing::SafetyState::Disarmed);

    CHECK(safety.begin_session(sailing::MissionMode::C1, 11U, 1U, false));
    CHECK(safety.start(11U, 2U));
    safety.emergency_stop();
    CHECK(safety.state() == sailing::SafetyState::EStop);
    CHECK(!safety.start(11U, 3U));
    safety.disarm();
    CHECK(safety.state() == sailing::SafetyState::EStop);
    safety.fault();
    CHECK(safety.state() == sailing::SafetyState::EStop);
    CHECK(safety.reset());
    CHECK(safety.state() == sailing::SafetyState::Disarmed);
    return true;
}

bool test_c1_launcher_is_always_inhibited() {
    sailing::hal::FakeClock clock;
    sailing::hal::FakeMotionOutput motion;
    sailing::hal::FakeLauncherOutput launcher;
    sailing::hal::FakeEventSink events;
    sailing::SafetySupervisor safety(clock, motion, launcher, events);
    CHECK(safety.begin_session(sailing::MissionMode::C1, 20U, 1U, false));
    CHECK(safety.start(20U, 2U));
    CHECK(safety.apply_control(20U, 3U, {0.1F, 0.0F, 500U}));
    CHECK(!safety.set_fire_interlocks(20U, 4U, {true, true, true, true}));
    CHECK(!safety.arm_launcher(20U, 4U));
    CHECK(!safety.fire_once(
        20U,
        4U,
        {1U, sailing::protocol::ShotSource::Auto, 250U}));
    CHECK(launcher.shot_count == 0U);
    CHECK(!launcher.armed);
    return true;
}

bool test_300_second_mission_timeout() {
    sailing::hal::FakeClock clock;
    sailing::hal::FakeMotionOutput motion;
    sailing::hal::FakeLauncherOutput launcher;
    sailing::hal::FakeEventSink events;
    sailing::SafetySupervisor safety(clock, motion, launcher, events);
    CHECK(safety.mission_timeout_ms() == 300000U);
    CHECK(safety.begin_session(sailing::MissionMode::C1, 25U, 1U, false));
    CHECK(safety.start(25U, 2U));
    clock.set_ms(299999U);
    CHECK(safety.apply_control(25U, 3U, {0.0F, 0.0F, 500U}));
    safety.tick();
    CHECK(safety.state() == sailing::SafetyState::Running);
    clock.advance_ms(1U);
    safety.tick();
    CHECK(safety.state() == sailing::SafetyState::Fault);
    CHECK(motion.neutral_state);
    CHECK(events.contains(sailing::hal::EventCode::MissionTimeout));
    return true;
}

bool test_lifecycle_rejections_and_output_failures() {
    sailing::hal::FakeClock clock;
    sailing::hal::FakeMotionOutput motion;
    sailing::hal::FakeLauncherOutput launcher;
    sailing::hal::FakeEventSink events;
    sailing::SafetySupervisor safety(clock, motion, launcher, events);
    safety.complete();
    safety.abort();
    CHECK(!safety.reset());
    CHECK(!safety.begin_session(sailing::MissionMode::C1, 0U, 1U, false));
    CHECK(safety.begin_session(sailing::MissionMode::C1, 40U, 1U, false));
    CHECK(!safety.begin_session(sailing::MissionMode::C1, 41U, 1U, false));
    CHECK(!safety.start(99U, 2U));
    CHECK(!safety.start(40U, 1U));
    CHECK(safety.start(40U, 2U));
    CHECK(!safety.apply_control(40U, 3U, {2.0F, 0.0F, 500U}));
    CHECK(!safety.apply_control(99U, 3U, {0.0F, 0.0F, 500U}));
    CHECK(safety.apply_control(40U, 3U, {0.0F, 0.0F, 500U}));
    CHECK(!safety.apply_control(40U, 3U, {0.0F, 0.0F, 500U}));
    safety.complete();
    CHECK(safety.state() == sailing::SafetyState::Complete);
    CHECK(safety.reset());
    CHECK(safety.begin_session(sailing::MissionMode::C1, 42U, 1U, false));
    safety.abort();
    CHECK(safety.state() == sailing::SafetyState::Aborted);
    CHECK(safety.reset());
    CHECK(safety.begin_session(sailing::MissionMode::C1, 43U, 1U, false));
    CHECK(safety.start(43U, 2U));
    safety.disarm();
    CHECK(safety.state() == sailing::SafetyState::Disarmed);
    CHECK(safety.begin_session(sailing::MissionMode::C1, 44U, 1U, false));
    CHECK(safety.start(44U, 2U));
    safety.fault();
    CHECK(safety.state() == sailing::SafetyState::Fault);

    sailing::hal::FakeMotionOutput bad_motion;
    sailing::hal::FakeLauncherOutput good_launcher;
    sailing::hal::FakeEventSink failure_events;
    bad_motion.fail_apply = true;
    sailing::SafetySupervisor failing(clock, bad_motion, good_launcher, failure_events);
    CHECK(failing.begin_session(sailing::MissionMode::C1, 50U, 1U, false));
    CHECK(failing.start(50U, 2U));
    CHECK(!failing.apply_control(50U, 3U, {0.0F, 0.0F, 500U}));
    CHECK(failing.state() == sailing::SafetyState::Fault);

    sailing::hal::FakeMotionOutput unsafe_motion;
    sailing::hal::FakeLauncherOutput unsafe_launcher;
    sailing::hal::FakeEventSink unsafe_events;
    unsafe_motion.fail_neutral = true;
    sailing::SafetySupervisor invalid_config(
        clock,
        unsafe_motion,
        unsafe_launcher,
        unsafe_events,
        {0U, 1000U, 300000U});
    CHECK(invalid_config.state() == sailing::SafetyState::Fault);
    CHECK(unsafe_events.contains(sailing::hal::EventCode::SafeOutputFailed));
    return true;
}

bool test_c2_rejects_open_interlocks_and_launcher_failures() {
    sailing::hal::FakeClock clock;
    sailing::hal::FakeMotionOutput motion;
    sailing::hal::FakeLauncherOutput launcher;
    sailing::hal::FakeEventSink events;
    sailing::SafetySupervisor safety(clock, motion, launcher, events);
    CHECK(safety.begin_session(sailing::MissionMode::C2, 60U, 1U, true));
    CHECK(safety.start(60U, 2U));
    CHECK(!safety.arm_launcher(60U, 3U));
    CHECK(safety.apply_control(60U, 3U, {0.0F, 0.0F, 500U}));
    CHECK(!safety.arm_launcher(60U, 4U));
    CHECK(safety.set_fire_interlocks(60U, 4U, {true, true, true, true}));
    CHECK(!safety.fire_once(
        60U,
        5U,
        {1U, sailing::protocol::ShotSource::Auto, 250U}));
    CHECK(safety.arm_launcher(60U, 5U));
    CHECK(safety.set_fire_interlocks(60U, 6U, {false, true, true, true}));
    CHECK(!safety.launcher_armed());
    CHECK(safety.set_fire_interlocks(60U, 7U, {true, true, true, true}));
    CHECK(safety.arm_launcher(60U, 8U));
    launcher.fail_arm = true;
    CHECK(!safety.fire_once(
        60U,
        9U,
        {2U, sailing::protocol::ShotSource::Auto, 250U}));
    CHECK(safety.state() == sailing::SafetyState::Fault);
    return true;
}

bool test_c2_two_stage_unique_shots_and_budget() {
    sailing::hal::FakeClock clock;
    sailing::hal::FakeMotionOutput motion;
    sailing::hal::FakeLauncherOutput launcher;
    sailing::hal::FakeEventSink events;
    sailing::SafetySupervisor safety(clock, motion, launcher, events);
    CHECK(!safety.begin_session(sailing::MissionMode::C2, 30U, 1U, false));
    CHECK(safety.begin_session(sailing::MissionMode::C2, 30U, 1U, true));
    CHECK(safety.start(30U, 2U));
    std::uint32_t sequence = 3U;
    CHECK(safety.apply_control(30U, sequence++, {0.0F, 0.0F, 500U}));
    CHECK(safety.set_fire_interlocks(30U, sequence++, {true, true, true, true}));
    CHECK(safety.arm_launcher(30U, sequence++));
    CHECK(safety.fire_once(
        30U,
        sequence++,
        {1U, sailing::protocol::ShotSource::Auto, 250U}));
    CHECK(safety.shot_count() == 1U);
    CHECK(!safety.launcher_armed());
    bool source_recorded = false;
    for (std::size_t index = 0U; index < events.size(); ++index) {
        const auto& event = events.at(index);
        if (event.code == sailing::hal::EventCode::ShotFired && event.detail == 1U) {
            CHECK(event.auxiliary ==
                static_cast<std::uint32_t>(sailing::protocol::ShotSource::Auto));
            source_recorded = true;
        }
    }
    CHECK(source_recorded);

    CHECK(safety.apply_control(30U, sequence++, {0.0F, 0.0F, 500U}));
    CHECK(safety.arm_launcher(30U, sequence++));
    CHECK(!safety.fire_once(
        30U,
        sequence++,
        {1U, sailing::protocol::ShotSource::Auto, 250U}));
    CHECK(safety.shot_count() == 1U);
    CHECK(!safety.launcher_armed());

    for (std::uint32_t shot_id = 2U; shot_id <= 10U; ++shot_id) {
        CHECK(safety.apply_control(30U, sequence++, {0.0F, 0.0F, 500U}));
        CHECK(safety.arm_launcher(30U, sequence++));
        CHECK(safety.fire_once(
            30U,
            sequence++,
            {shot_id, sailing::protocol::ShotSource::Auto, 250U}));
    }
    CHECK(safety.shot_count() == sailing::SafetySupervisor::kMaximumShots);
    CHECK(safety.apply_control(30U, sequence++, {0.0F, 0.0F, 500U}));
    CHECK(!safety.arm_launcher(30U, sequence));
    CHECK(launcher.shot_count == 10U);
    safety.disarm();
    CHECK(safety.begin_session(sailing::MissionMode::C2, 31U, 1U, true));
    CHECK(safety.start(31U, 2U));
    CHECK(safety.apply_control(31U, 3U, {0.0F, 0.0F, 500U}));
    CHECK(safety.set_fire_interlocks(31U, 4U, {true, true, true, true}));
    CHECK(!safety.arm_launcher(31U, 5U));
    CHECK(safety.shot_count() == 10U);
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::string vector_path = argc > 1
            ? argv[1]
            : "../shared/test-vectors/protocol-v1.tsv";
        const auto vectors = load_vectors(vector_path);
        const std::vector<std::pair<std::string, bool (*)()>> safety_tests{
            {"watchdog_and_estop", test_watchdog_and_estop},
            {"c1_launcher_inhibited", test_c1_launcher_is_always_inhibited},
            {"mission_timeout", test_300_second_mission_timeout},
            {"lifecycle_and_failures", test_lifecycle_rejections_and_output_failures},
            {"c2_rejections", test_c2_rejects_open_interlocks_and_launcher_failures},
            {"c2_launch_interlocks", test_c2_two_stage_unique_shots_and_budget},
        };
        std::size_t passed = 0U;
        if (test_protocol_vectors(vectors)) {
            ++passed;
            std::cout << "PASS protocol_vectors\n";
        }
        if (test_protocol_resynchronization(vectors)) {
            ++passed;
            std::cout << "PASS protocol_resynchronization\n";
        }
        for (const auto& test : safety_tests) {
            if (test.second()) {
                ++passed;
                std::cout << "PASS " << test.first << '\n';
            }
        }
        constexpr std::size_t total = 8U;
        std::cout << passed << '/' << total << " lower-computer test groups passed\n";
        return passed == total ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
