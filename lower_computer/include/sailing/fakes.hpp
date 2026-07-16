#pragma once

#include "sailing/hal.hpp"

#include <array>
#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace sailing::hal {

class FakeClock final : public MonotonicClock {
public:
    [[nodiscard]] std::uint64_t now_ms() const noexcept override { return now_ms_; }
    void set_ms(std::uint64_t value) noexcept { now_ms_ = value; }
    void advance_ms(std::uint64_t delta) noexcept { now_ms_ += delta; }

private:
    std::uint64_t now_ms_{0U};
};

class FakeMotionOutput final : public MotionOutput {
public:
    bool apply(float propulsion, float steering) noexcept override {
        ++apply_calls;
        if (fail_apply) {
            return false;
        }
        propulsion_value = propulsion;
        steering_value = steering;
        neutral_state = false;
        return true;
    }

    bool neutral() noexcept override {
        ++neutral_calls;
        if (fail_neutral) {
            return false;
        }
        propulsion_value = 0.0F;
        steering_value = 0.0F;
        neutral_state = true;
        return true;
    }

    float propulsion_value{0.0F};
    float steering_value{0.0F};
    std::size_t apply_calls{0U};
    std::size_t neutral_calls{0U};
    bool neutral_state{true};
    bool fail_apply{false};
    bool fail_neutral{false};
};

class FakeLauncherOutput final : public LauncherOutput {
public:
    bool set_armed(bool value) noexcept override {
        ++arm_calls;
        if (fail_arm) {
            return false;
        }
        armed = value;
        return true;
    }

    bool fire_once(std::uint32_t shot_id) noexcept override {
        ++fire_calls;
        if (fail_fire || !armed || shot_count >= shot_ids.size()) {
            return false;
        }
        shot_ids[shot_count] = shot_id;
        ++shot_count;
        return true;
    }

    std::array<std::uint32_t, 10U> shot_ids{};
    std::size_t shot_count{0U};
    std::size_t arm_calls{0U};
    std::size_t fire_calls{0U};
    bool armed{false};
    bool fail_arm{false};
    bool fail_fire{false};
};

class FakeEventSink final : public EventSink {
public:
    static constexpr std::size_t kCapacity = 128U;

    void record(const Event& event) noexcept override {
        if (size_ < events_.size()) {
            events_[size_] = event;
            ++size_;
        } else {
            ++dropped;
        }
    }

    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] const Event& at(std::size_t index) const noexcept { return events_[index]; }

    [[nodiscard]] bool contains(EventCode code) const noexcept {
        return std::any_of(
            events_.begin(),
            events_.begin() + static_cast<std::ptrdiff_t>(size_),
            [code](const Event& event) { return event.code == code; });
    }

    std::size_t dropped{0U};

private:
    std::array<Event, kCapacity> events_{};
    std::size_t size_{0U};
};

class FakeByteTransport final : public ByteTransport {
public:
    static constexpr std::size_t kCapacity = 1024U;

    std::size_t read(std::uint8_t* destination, std::size_t capacity) noexcept override {
        if (destination == nullptr && capacity != 0U) {
            return 0U;
        }
        const auto count = std::min(capacity, input_size_);
        std::copy_n(input_.begin(), count, destination);
        std::move(input_.begin() + static_cast<std::ptrdiff_t>(count), input_.begin() +
            static_cast<std::ptrdiff_t>(input_size_), input_.begin());
        input_size_ -= count;
        return count;
    }

    bool write(const std::uint8_t* data, std::size_t size) noexcept override {
        if ((data == nullptr && size != 0U) || size > output_.size()) {
            return false;
        }
        std::copy_n(data, size, output_.begin());
        output_size_ = size;
        return true;
    }

    bool push_input(const std::uint8_t* data, std::size_t size) noexcept {
        if ((data == nullptr && size != 0U) || input_size_ + size > input_.size()) {
            return false;
        }
        std::copy_n(data, size, input_.begin() + static_cast<std::ptrdiff_t>(input_size_));
        input_size_ += size;
        return true;
    }

    [[nodiscard]] std::size_t output_size() const noexcept { return output_size_; }

private:
    std::array<std::uint8_t, kCapacity> input_{};
    std::array<std::uint8_t, kCapacity> output_{};
    std::size_t input_size_{0U};
    std::size_t output_size_{0U};
};

}  // namespace sailing::hal
