#ifndef EXVECTR_TELECOM_HPP
#define EXVECTR_TELECOM_HPP

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <stdint.h>

#include "ExVectrCore/CanSerialize.hpp"

namespace VCTR::packets {

enum class PacketType : size_t {
  Telecommand, // Telecoms
  Telemetry,   // Telecoms
  MAX          //
};

} // namespace VCTR::packets

namespace VCTR::packets {

template <typename T>
concept IsPacket = VCTR::Core::CanSerialize<T> && requires(const T a) {
  { a.getPacketType() } -> std::same_as<size_t>;
  { a.getPacketDataType() } -> std::same_as<size_t>;
};

} // namespace VCTR::packets

#endif // EXVECTR_TELECOM_HPP