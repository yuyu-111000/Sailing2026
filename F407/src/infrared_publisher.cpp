#include "sailing/infrared_publisher.hpp"

#include "sailing/infrared_link.hpp"

#include <array>

namespace sailing::f407 {

InfraredPublisher::InfraredPublisher(InfraredSampler& sampler, ByteOutput& output) noexcept
    : sampler_(sampler), output_(output) {}

bool InfraredPublisher::publish() noexcept {
    std::array<std::uint16_t, kInfraredChannelCount> strength{};
    if (!sampler_.snapshot_and_reset(strength)) { return false; }
    const auto frame = encode_infrared_frame(strength, sequence_);
    if (!output_.write(frame.bytes.data(), frame.bytes.size())) { return false; }
    ++sequence_;
    return true;
}

}  // namespace sailing::f407
