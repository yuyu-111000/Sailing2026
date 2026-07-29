#include "sailing/jy61.hpp"

#include <cstdint>

namespace sailing::l431 {
namespace {

[[nodiscard]] std::int16_t signed_word(std::uint8_t low, std::uint8_t high) noexcept {
    return static_cast<std::int16_t>(
        static_cast<std::uint16_t>(low) | (static_cast<std::uint16_t>(high) << 8U));
}

}  // namespace

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
    for (std::size_t index = 0U; index < 10U; ++index) {
        sum = static_cast<std::uint8_t>(sum + frame_[index]);
    }
    if (sum != frame_[10]) { return false; }
    if (frame_[1] == 0x52U) {
        sample_.yaw_rate_dps = static_cast<float>(signed_word(frame_[6], frame_[7])) /
            32768.0F * 2000.0F;
        sample_.rate_valid = true;
        sample_.rate_received_ms = now_ms;
    } else {
        sample_.yaw_deg = static_cast<float>(signed_word(frame_[6], frame_[7])) /
            32768.0F * 180.0F;
        sample_.angle_valid = true;
        sample_.angle_received_ms = now_ms;
    }
    return true;
}

}  // namespace sailing::l431
