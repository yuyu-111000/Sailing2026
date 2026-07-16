#include "sailing/sensors.hpp"

#include <cstdint>

namespace sailing {
namespace {

[[nodiscard]] std::int16_t signed_word(std::uint8_t low, std::uint8_t high) noexcept {
    return static_cast<std::int16_t>(
        static_cast<std::uint16_t>(low) | (static_cast<std::uint16_t>(high) << 8U));
}

}  // namespace

std::uint16_t crc16_ccitt_false(const std::uint8_t* data, std::size_t size) noexcept {
    if (data == nullptr && size != 0U) { return 0U; }
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t i = 0U; i < size; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i]) << 8U;
        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

bool InfraredSerialDecoder::feed(std::uint8_t byte, std::uint64_t now_ms) noexcept {
    if (size_ == 0U && byte != 0xA5U) { return false; }
    if (size_ == 1U && byte != 0x5AU) {
        size_ = byte == 0xA5U ? 1U : 0U;
        if (size_ == 1U) { frame_[0] = byte; }
        return false;
    }
    frame_[size_++] = byte;
    if (size_ != frame_.size()) { return false; }
    size_ = 0U;
    const auto expected = static_cast<std::uint16_t>(frame_[35]) |
        (static_cast<std::uint16_t>(frame_[36]) << 8U);
    if (crc16_ccitt_false(frame_.data(), 35U) != expected) { return false; }
    sample_.sequence = frame_[2];
    for (std::size_t i = 0U; i < sample_.strength.size(); ++i) {
        const auto offset = 3U + i * 2U;
        sample_.strength[i] = static_cast<std::uint16_t>(frame_[offset]) |
            (static_cast<std::uint16_t>(frame_[offset + 1U]) << 8U);
    }
    sample_.received_ms = now_ms;
    sample_.valid = true;
    return true;
}

bool Jy61Decoder::feed(std::uint8_t byte, std::uint64_t now_ms) noexcept {
    if (size_ == 0U && byte != 0x55U) { return false; }
    frame_[size_++] = byte;
    if (size_ == 2U && frame_[1] != 0x52U && frame_[1] != 0x53U) {
        size_ = byte == 0x55U ? 1U : 0U;
        return false;
    }
    if (size_ != frame_.size()) { return false; }
    size_ = 0U;
    std::uint8_t sum = 0U;
    for (std::size_t i = 0U; i < 10U; ++i) { sum = static_cast<std::uint8_t>(sum + frame_[i]); }
    if (sum != frame_[10]) { return false; }
    if (frame_[1] == 0x52U) {
        sample_.yaw_rate_dps = static_cast<float>(signed_word(frame_[6], frame_[7])) / 32768.0F * 2000.0F;
        sample_.rate_valid = true;
        sample_.rate_received_ms = now_ms;
    } else {
        sample_.yaw_deg = static_cast<float>(signed_word(frame_[6], frame_[7])) / 32768.0F * 180.0F;
        sample_.angle_valid = true;
        sample_.angle_received_ms = now_ms;
    }
    return true;
}

}  // namespace sailing
