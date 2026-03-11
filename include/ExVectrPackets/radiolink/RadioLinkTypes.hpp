#ifndef EXVECTR_RADIOLINK_HPP
#define EXVECTR_RADIOLINK_HPP

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <stdint.h>

#include "ExVectrCore/CanSerialize.hpp"

namespace VCTR::packets::radiolink {

/**
 * Packet types for the radio link.
 * The naming convention is A for Analog channels from -1000 - 1000,
 * D for Digital channels values 0-3.
 */
enum class RadioLinkTypes : size_t {
  A4D0, // 4 analog channels, 0 digital channels
  A4D2, // 4 analog channels, 2 digital channels
  A4D4, // 4 analog channels, 4 digital channels
  A6D6, // 6 analog channels, 6 digital channels
  MAX
};

} // namespace VCTR::packets::radiolink
#endif // EXVECTR_RADIOLINK_HPP