#pragma once

#include "sailing/infrared_link.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace sailing::f407 {

class InfraredSampler {
public:
    virtual ~InfraredSampler() = default;
    virtual bool snapshot_and_reset(
        std::array<std::uint16_t, kInfraredChannelCount>& strength) noexcept = 0;
};

class ByteOutput {
public:
    virtual ~ByteOutput() = default;
    virtual bool write(const std::uint8_t* data, std::size_t size) noexcept = 0;
};

}  // namespace sailing::f407
