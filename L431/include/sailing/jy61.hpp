#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sailing::l431 {

struct ImuSample {
    float yaw_deg{0.0F};
    float yaw_rate_dps{0.0F};
    std::uint64_t angle_received_ms{0U};
    std::uint64_t rate_received_ms{0U};
    bool angle_valid{false};
    bool rate_valid{false};
};

class Jy61Decoder {
public:
    bool feed(std::uint8_t byte, std::uint64_t now_ms) noexcept;
    [[nodiscard]] const ImuSample& sample() const noexcept { return sample_; }

private:
    std::array<std::uint8_t, 11U> frame_{};
    std::size_t size_{0U};
    ImuSample sample_{};
};

}  // namespace sailing::l431
