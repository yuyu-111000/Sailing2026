#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sailing {

inline constexpr std::size_t kInfraredChannelCount = 16U;
inline constexpr std::size_t kInfraredFrameSize = 37U;

struct InfraredSample {
    std::array<std::uint16_t, kInfraredChannelCount> strength{};
    std::uint8_t sequence{0U};
    std::uint64_t received_ms{0U};
    bool valid{false};
};

struct EncodedInfraredFrame {
    std::array<std::uint8_t, kInfraredFrameSize> bytes{};
};

[[nodiscard]] std::uint16_t crc16_ccitt_false(
    const std::uint8_t* data, std::size_t size) noexcept;

[[nodiscard]] EncodedInfraredFrame encode_infrared_frame(
    const std::array<std::uint16_t, kInfraredChannelCount>& strength,
    std::uint8_t sequence) noexcept;

class InfraredStreamDecoder {
public:
    bool feed(std::uint8_t byte, std::uint64_t now_ms) noexcept;
    [[nodiscard]] const InfraredSample& sample() const noexcept { return sample_; }

private:
    std::array<std::uint8_t, kInfraredFrameSize> frame_{};
    std::size_t size_{0U};
    InfraredSample sample_{};
    bool has_sequence_{false};
};

}  // namespace sailing
