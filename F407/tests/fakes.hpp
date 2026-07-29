#pragma once

#include "sailing/f407_hal.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace sailing::f407 {

class FakeInfraredSampler final : public InfraredSampler {
public:
    bool snapshot_and_reset(
        std::array<std::uint16_t, kInfraredChannelCount>& destination) noexcept override {
        ++snapshot_calls;
        if (fail) { return false; }
        destination = strength;
        strength.fill(0U);
        return true;
    }
    std::array<std::uint16_t, kInfraredChannelCount> strength{};
    std::size_t snapshot_calls{0U};
    bool fail{false};
};

class FakeByteOutput final : public ByteOutput {
public:
    bool write(const std::uint8_t* data, std::size_t size) noexcept override {
        ++write_calls;
        if (fail || data == nullptr || size != bytes.size()) { return false; }
        std::copy_n(data, size, bytes.begin());
        size_written = size;
        return true;
    }
    std::array<std::uint8_t, kInfraredFrameSize> bytes{};
    std::size_t size_written{0U};
    std::size_t write_calls{0U};
    bool fail{false};
};

}  // namespace sailing::f407
