#include "sailing/infrared_link.hpp"

namespace sailing {

std::uint16_t crc16_ccitt_false(const std::uint8_t* data, std::size_t size) noexcept {
    if (data == nullptr && size != 0U) { return 0U; }
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t index = 0U; index < size; ++index) {
        crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

EncodedInfraredFrame encode_infrared_frame(
    const std::array<std::uint16_t, kInfraredChannelCount>& strength,
    std::uint8_t sequence) noexcept {
    EncodedInfraredFrame frame{};
    frame.bytes[0] = 0xA5U;
    frame.bytes[1] = 0x5AU;
    frame.bytes[2] = sequence;
    for (std::size_t index = 0U; index < strength.size(); ++index) {
        const auto offset = 3U + index * 2U;
        frame.bytes[offset] = static_cast<std::uint8_t>(strength[index] & 0xFFU);
        frame.bytes[offset + 1U] = static_cast<std::uint8_t>(strength[index] >> 8U);
    }
    const auto crc = crc16_ccitt_false(frame.bytes.data(), 35U);
    frame.bytes[35] = static_cast<std::uint8_t>(crc & 0xFFU);
    frame.bytes[36] = static_cast<std::uint8_t>(crc >> 8U);
    return frame;
}

bool InfraredStreamDecoder::feed(std::uint8_t byte, std::uint64_t now_ms) noexcept {
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
    if (has_sequence_ && frame_[2] == sample_.sequence) { return false; }
    sample_.sequence = frame_[2];
    for (std::size_t index = 0U; index < sample_.strength.size(); ++index) {
        const auto offset = 3U + index * 2U;
        sample_.strength[index] = static_cast<std::uint16_t>(frame_[offset]) |
            (static_cast<std::uint16_t>(frame_[offset + 1U]) << 8U);
    }
    sample_.received_ms = now_ms;
    sample_.valid = true;
    has_sequence_ = true;
    return true;
}

}  // namespace sailing
