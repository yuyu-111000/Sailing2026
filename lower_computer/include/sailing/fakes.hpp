#pragma once

#include "sailing/hal.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace sailing::hal {

class FakeClock final : public MonotonicClock {
public:
    [[nodiscard]] std::uint64_t now_ms() const noexcept override { return value_; }
    void set_ms(std::uint64_t value) noexcept { value_ = value; }
    void advance_ms(std::uint64_t value) noexcept { value_ += value; }
private:
    std::uint64_t value_{0U};
};

class FakeByteInput final : public ByteInput {
public:
    static constexpr std::size_t kCapacity = 512U;
    bool push(const std::uint8_t* data, std::size_t size) noexcept {
        if ((data == nullptr && size != 0U) || size_ + size > buffer_.size()) { return false; }
        std::copy_n(data, size, buffer_.begin() + static_cast<std::ptrdiff_t>(size_));
        size_ += size;
        return true;
    }
    std::size_t read(std::uint8_t* destination, std::size_t capacity) noexcept override {
        if (destination == nullptr && capacity != 0U) { return 0U; }
        const auto count = std::min(size_, capacity);
        std::copy_n(buffer_.begin(), count, destination);
        std::move(buffer_.begin() + static_cast<std::ptrdiff_t>(count),
                  buffer_.begin() + static_cast<std::ptrdiff_t>(size_), buffer_.begin());
        size_ -= count;
        return count;
    }
private:
    std::array<std::uint8_t, kCapacity> buffer_{};
    std::size_t size_{0U};
};

class FakeDrive final : public DifferentialDrive {
public:
    bool apply(float left_value, float right_value) noexcept override {
        ++apply_calls;
        if (fail) { return false; }
        left = left_value; right = right_value; neutral_state = false; return true;
    }
    bool neutral() noexcept override {
        ++neutral_calls;
        if (fail) { return false; }
        left = 0.0F; right = 0.0F; neutral_state = true; return true;
    }
    float left{0.0F};
    float right{0.0F};
    std::size_t apply_calls{0U};
    std::size_t neutral_calls{0U};
    bool neutral_state{true};
    bool fail{false};
};

class FakeArmInput final : public ArmInput {
public:
    [[nodiscard]] bool armed() const noexcept override { return armed_value; }
    [[nodiscard]] bool emergency_stop() const noexcept override { return estop_value; }
    bool armed_value{false};
    bool estop_value{false};
};

}  // namespace sailing::hal
