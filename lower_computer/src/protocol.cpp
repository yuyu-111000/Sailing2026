#include "sailing/protocol.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace sailing::protocol {
namespace {

constexpr std::size_t kVersionOffset = 2U;
constexpr std::size_t kTypeOffset = 3U;
constexpr std::size_t kFlagsOffset = 4U;
constexpr std::size_t kSessionOffset = 5U;
constexpr std::size_t kSequenceOffset = 9U;
constexpr std::size_t kPayloadLengthOffset = 13U;
constexpr std::size_t kSenderTimeOffset = 15U;
constexpr std::size_t kPayloadOffset = 19U;
constexpr std::size_t kControlPayloadSize = 10U;
constexpr std::size_t kFirePayloadSize = 7U;

static_assert(sizeof(float) == 4U, "protocol v1 requires 32-bit float");
static_assert(std::numeric_limits<float>::is_iec559, "protocol v1 requires IEEE-754 float");

[[nodiscard]] std::uint16_t read_u16(const std::uint8_t* data) noexcept {
    return static_cast<std::uint16_t>(data[0]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8U);
}

[[nodiscard]] std::uint32_t read_u32(const std::uint8_t* data) noexcept {
    return static_cast<std::uint32_t>(data[0]) |
        (static_cast<std::uint32_t>(data[1]) << 8U) |
        (static_cast<std::uint32_t>(data[2]) << 16U) |
        (static_cast<std::uint32_t>(data[3]) << 24U);
}

void write_u16(std::uint8_t* destination, std::uint16_t value) noexcept {
    destination[0] = static_cast<std::uint8_t>(value & 0xFFU);
    destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

void write_u32(std::uint8_t* destination, std::uint32_t value) noexcept {
    destination[0] = static_cast<std::uint8_t>(value & 0xFFU);
    destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    destination[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    destination[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

[[nodiscard]] float read_float32(const std::uint8_t* data) noexcept {
    const auto raw = read_u32(data);
    float value = 0.0F;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

void write_float32(std::uint8_t* destination, float value) noexcept {
    std::uint32_t raw = 0U;
    std::memcpy(&raw, &value, sizeof(raw));
    write_u32(destination, raw);
}

[[nodiscard]] bool valid_shot_source(ShotSource source) noexcept {
    return source == ShotSource::Auto || source == ShotSource::ManualFutureRemote;
}

}  // namespace

bool is_known_message_type(std::uint8_t raw_type) noexcept {
    switch (static_cast<MessageType>(raw_type)) {
        case MessageType::Heartbeat:
        case MessageType::Arm:
        case MessageType::Disarm:
        case MessageType::EStop:
        case MessageType::ControlSetpoint:
        case MessageType::MissionStatus:
        case MessageType::Telemetry:
        case MessageType::LaunchArm:
        case MessageType::FireOnce:
        case MessageType::Ack:
        case MessageType::Nack:
        case MessageType::Fault:
            return true;
    }
    return false;
}

std::uint16_t crc16_ccitt_false(const std::uint8_t* data, std::size_t size) noexcept {
    if (data == nullptr && size != 0U) {
        return 0U;
    }

    std::uint16_t crc = 0xFFFFU;
    for (std::size_t index = 0U; index < size; ++index) {
        crc ^= static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[index]) << 8U);
        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<std::uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

bool encode_frame(const Frame& frame, EncodedFrame& encoded) noexcept {
    encoded.size = 0U;
    if (frame.version != kVersion || frame.flags != 0U ||
        !is_known_message_type(static_cast<std::uint8_t>(frame.type)) ||
        frame.payload_size > kMaxPayloadSize) {
        return false;
    }

    encoded.bytes[0] = kMagic0;
    encoded.bytes[1] = kMagic1;
    encoded.bytes[kVersionOffset] = frame.version;
    encoded.bytes[kTypeOffset] = static_cast<std::uint8_t>(frame.type);
    encoded.bytes[kFlagsOffset] = frame.flags;
    write_u32(encoded.bytes.data() + kSessionOffset, frame.session_id);
    write_u32(encoded.bytes.data() + kSequenceOffset, frame.sequence);
    write_u16(encoded.bytes.data() + kPayloadLengthOffset, frame.payload_size);
    write_u32(encoded.bytes.data() + kSenderTimeOffset, frame.sender_time_ms);
    std::memcpy(
        encoded.bytes.data() + kPayloadOffset,
        frame.payload.data(),
        frame.payload_size);

    const auto crc_input_size = (kFixedPrefixSize - kVersionOffset) + frame.payload_size;
    const auto crc = crc16_ccitt_false(encoded.bytes.data() + kVersionOffset, crc_input_size);
    const auto crc_offset = kPayloadOffset + frame.payload_size;
    write_u16(encoded.bytes.data() + crc_offset, crc);
    encoded.size = crc_offset + kCrcSize;
    return true;
}

bool set_control_payload(Frame& frame, const ControlSetpoint& command) noexcept {
    if (!std::isfinite(command.propulsion) || !std::isfinite(command.steering) ||
        command.propulsion < -1.0F || command.propulsion > 1.0F ||
        command.steering < -1.0F || command.steering > 1.0F ||
        command.valid_for_ms == 0U) {
        return false;
    }

    frame.type = MessageType::ControlSetpoint;
    frame.payload_size = static_cast<std::uint16_t>(kControlPayloadSize);
    write_float32(frame.payload.data(), command.propulsion);
    write_float32(frame.payload.data() + 4U, command.steering);
    write_u16(frame.payload.data() + 8U, command.valid_for_ms);
    return true;
}

bool decode_control_payload(const Frame& frame, ControlSetpoint& command) noexcept {
    if (frame.type != MessageType::ControlSetpoint || frame.payload_size != kControlPayloadSize) {
        return false;
    }

    const ControlSetpoint decoded{
        read_float32(frame.payload.data()),
        read_float32(frame.payload.data() + 4U),
        read_u16(frame.payload.data() + 8U),
    };
    if (!std::isfinite(decoded.propulsion) || !std::isfinite(decoded.steering) ||
        decoded.propulsion < -1.0F || decoded.propulsion > 1.0F ||
        decoded.steering < -1.0F || decoded.steering > 1.0F ||
        decoded.valid_for_ms == 0U) {
        return false;
    }
    command = decoded;
    return true;
}

bool set_fire_once_payload(Frame& frame, const FireOnceRequest& request) noexcept {
    if (!valid_shot_source(request.source) || request.valid_for_ms == 0U) {
        return false;
    }

    frame.type = MessageType::FireOnce;
    frame.payload_size = static_cast<std::uint16_t>(kFirePayloadSize);
    write_u32(frame.payload.data(), request.shot_id);
    frame.payload[4] = static_cast<std::uint8_t>(request.source);
    write_u16(frame.payload.data() + 5U, request.valid_for_ms);
    return true;
}

bool decode_fire_once_payload(const Frame& frame, FireOnceRequest& request) noexcept {
    if (frame.type != MessageType::FireOnce || frame.payload_size != kFirePayloadSize) {
        return false;
    }

    const auto source = static_cast<ShotSource>(frame.payload[4]);
    const FireOnceRequest decoded{
        read_u32(frame.payload.data()),
        source,
        read_u16(frame.payload.data() + 5U),
    };
    if (!valid_shot_source(decoded.source) || decoded.valid_for_ms == 0U) {
        return false;
    }
    request = decoded;
    return true;
}

void StreamDecoder::feed(
    const std::uint8_t* data,
    std::size_t size,
    FrameSink& sink) noexcept {
    if (data == nullptr && size != 0U) {
        sink.on_parse_error(ParseError::BufferOverflow);
        return;
    }
    for (std::size_t index = 0U; index < size; ++index) {
        append_byte(data[index], sink);
    }
}

void StreamDecoder::reset() noexcept {
    buffered_size_ = 0U;
}

void StreamDecoder::append_byte(std::uint8_t byte, FrameSink& sink) noexcept {
    if (buffered_size_ == 0U) {
        if (byte == kMagic0) {
            buffer_[0] = byte;
            buffered_size_ = 1U;
        }
        return;
    }

    if (buffered_size_ == 1U) {
        if (byte == kMagic1) {
            buffer_[1] = byte;
            buffered_size_ = 2U;
        } else if (byte != kMagic0) {
            buffered_size_ = 0U;
        }
        return;
    }

    if (buffered_size_ >= buffer_.size()) {
        sink.on_parse_error(ParseError::BufferOverflow);
        reset();
        if (byte == kMagic0) {
            buffer_[0] = byte;
            buffered_size_ = 1U;
        }
        return;
    }

    buffer_[buffered_size_] = byte;
    ++buffered_size_;
    evaluate(sink);
}

void StreamDecoder::evaluate(FrameSink& sink) noexcept {
    while (buffered_size_ >= kFixedPrefixSize) {
        if (buffer_[kVersionOffset] != kVersion) {
            sink.on_parse_error(ParseError::UnsupportedVersion);
            resynchronize();
            continue;
        }
        if (!is_known_message_type(buffer_[kTypeOffset])) {
            sink.on_parse_error(ParseError::UnknownMessageType);
            resynchronize();
            continue;
        }
        if (buffer_[kFlagsOffset] != 0U) {
            sink.on_parse_error(ParseError::UndefinedFlags);
            resynchronize();
            continue;
        }

        const auto payload_size = read_u16(buffer_.data() + kPayloadLengthOffset);
        if (payload_size > kMaxPayloadSize) {
            sink.on_parse_error(ParseError::PayloadTooLarge);
            resynchronize();
            continue;
        }

        const auto expected_size = kFixedPrefixSize + payload_size + kCrcSize;
        if (buffered_size_ < expected_size) {
            return;
        }

        const auto expected_crc = read_u16(buffer_.data() + kPayloadOffset + payload_size);
        const auto crc_input_size = (kFixedPrefixSize - kVersionOffset) + payload_size;
        const auto actual_crc = crc16_ccitt_false(buffer_.data() + kVersionOffset, crc_input_size);
        if (actual_crc != expected_crc) {
            sink.on_parse_error(ParseError::CrcMismatch);
            resynchronize();
            continue;
        }

        Frame frame{};
        frame.version = buffer_[kVersionOffset];
        frame.type = static_cast<MessageType>(buffer_[kTypeOffset]);
        frame.flags = buffer_[kFlagsOffset];
        frame.session_id = read_u32(buffer_.data() + kSessionOffset);
        frame.sequence = read_u32(buffer_.data() + kSequenceOffset);
        frame.payload_size = payload_size;
        frame.sender_time_ms = read_u32(buffer_.data() + kSenderTimeOffset);
        std::memcpy(frame.payload.data(), buffer_.data() + kPayloadOffset, payload_size);
        sink.on_frame(frame);
        reset();
    }
}

void StreamDecoder::resynchronize() noexcept {
    for (std::size_t index = 1U; index + 1U < buffered_size_; ++index) {
        if (buffer_[index] == kMagic0 && buffer_[index + 1U] == kMagic1) {
            const auto retained = buffered_size_ - index;
            std::memmove(buffer_.data(), buffer_.data() + index, retained);
            buffered_size_ = retained;
            return;
        }
    }

    if (buffered_size_ != 0U && buffer_[buffered_size_ - 1U] == kMagic0) {
        buffer_[0] = kMagic0;
        buffered_size_ = 1U;
    } else {
        reset();
    }
}

}  // namespace sailing::protocol
