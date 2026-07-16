#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sailing::protocol {

constexpr std::uint8_t kMagic0 = 0x53U;
constexpr std::uint8_t kMagic1 = 0x41U;
constexpr std::uint8_t kVersion = 0x01U;
constexpr std::size_t kMaxPayloadSize = 256U;
constexpr std::size_t kFixedPrefixSize = 19U;
constexpr std::size_t kCrcSize = 2U;
constexpr std::size_t kMinFrameSize = kFixedPrefixSize + kCrcSize;
constexpr std::size_t kMaxFrameSize = kFixedPrefixSize + kMaxPayloadSize + kCrcSize;

enum class MessageType : std::uint8_t {
    Heartbeat = 0x01U,
    Arm = 0x02U,
    Disarm = 0x03U,
    EStop = 0x04U,
    ControlSetpoint = 0x05U,
    MissionStatus = 0x06U,
    Telemetry = 0x07U,
    LaunchArm = 0x08U,
    FireOnce = 0x09U,
    Ack = 0x0AU,
    Nack = 0x0BU,
    Fault = 0x0CU,
};

enum class ShotSource : std::uint8_t {
    Auto = 0x01U,
    ManualFutureRemote = 0x02U,
};

enum class ParseError : std::uint8_t {
    UnsupportedVersion,
    UnknownMessageType,
    UndefinedFlags,
    PayloadTooLarge,
    CrcMismatch,
    BufferOverflow,
};

struct Frame {
    std::uint8_t version{kVersion};
    MessageType type{MessageType::Heartbeat};
    std::uint8_t flags{0U};
    std::uint32_t session_id{0U};
    std::uint32_t sequence{0U};
    std::uint32_t sender_time_ms{0U};
    std::array<std::uint8_t, kMaxPayloadSize> payload{};
    std::uint16_t payload_size{0U};
};

struct EncodedFrame {
    std::array<std::uint8_t, kMaxFrameSize> bytes{};
    std::size_t size{0U};
};

struct ControlSetpoint {
    float propulsion{0.0F};
    float steering{0.0F};
    std::uint16_t valid_for_ms{0U};
};

struct FireOnceRequest {
    std::uint32_t shot_id{0U};
    ShotSource source{ShotSource::Auto};
    std::uint16_t valid_for_ms{0U};
};

class FrameSink {
public:
    virtual ~FrameSink() = default;
    virtual void on_frame(const Frame& frame) noexcept = 0;
    virtual void on_parse_error(ParseError error) noexcept = 0;
};

[[nodiscard]] bool is_known_message_type(std::uint8_t raw_type) noexcept;
[[nodiscard]] std::uint16_t crc16_ccitt_false(
    const std::uint8_t* data,
    std::size_t size) noexcept;
[[nodiscard]] bool encode_frame(const Frame& frame, EncodedFrame& encoded) noexcept;

[[nodiscard]] bool set_control_payload(
    Frame& frame,
    const ControlSetpoint& command) noexcept;
[[nodiscard]] bool decode_control_payload(
    const Frame& frame,
    ControlSetpoint& command) noexcept;
[[nodiscard]] bool set_fire_once_payload(
    Frame& frame,
    const FireOnceRequest& request) noexcept;
[[nodiscard]] bool decode_fire_once_payload(
    const Frame& frame,
    FireOnceRequest& request) noexcept;

class StreamDecoder {
public:
    void feed(const std::uint8_t* data, std::size_t size, FrameSink& sink) noexcept;
    void reset() noexcept;

    [[nodiscard]] std::size_t buffered_size() const noexcept { return buffered_size_; }

private:
    void append_byte(std::uint8_t byte, FrameSink& sink) noexcept;
    void evaluate(FrameSink& sink) noexcept;
    void resynchronize() noexcept;

    std::array<std::uint8_t, kMaxFrameSize> buffer_{};
    std::size_t buffered_size_{0U};
};

}  // namespace sailing::protocol
