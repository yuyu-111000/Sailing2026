#pragma once

#include <cstddef>
#include <cstdint>

namespace sailing::hal {

class MonotonicClock {
public:
    virtual ~MonotonicClock() = default;
    [[nodiscard]] virtual std::uint64_t now_ms() const noexcept = 0;
};

class ByteInput {
public:
    virtual ~ByteInput() = default;
    virtual std::size_t read(std::uint8_t* destination, std::size_t capacity) noexcept = 0;
};

class DifferentialDrive {
public:
    virtual ~DifferentialDrive() = default;
    virtual bool apply(float left, float right) noexcept = 0;
    virtual bool neutral() noexcept = 0;
};

class ArmInput {
public:
    virtual ~ArmInput() = default;
    [[nodiscard]] virtual bool armed() const noexcept = 0;
    [[nodiscard]] virtual bool emergency_stop() const noexcept = 0;
};

}  // namespace sailing::hal
