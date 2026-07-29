#pragma once

#include "sailing/f407_hal.hpp"

#include <cstdint>

namespace sailing::f407 {

class InfraredPublisher {
public:
    InfraredPublisher(InfraredSampler& sampler, ByteOutput& output) noexcept;
    [[nodiscard]] bool publish() noexcept;
    [[nodiscard]] std::uint8_t next_sequence() const noexcept { return sequence_; }

private:
    InfraredSampler& sampler_;
    ByteOutput& output_;
    std::uint8_t sequence_{0U};
};

}  // namespace sailing::f407
