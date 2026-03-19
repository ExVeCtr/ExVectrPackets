#ifndef EXVECTR_TELEMETRY_HPP
#define EXVECTR_TELEMETRY_HPP

#include <cstddef>
#include <stdint.h>

#include "ExVectrPackets/telecoms/TelecomTypes.hpp"

namespace VCTR::packets::telecoms {

using namespace VCTR::packets::telecoms;

class Telemetry_Heartbeat {
public:
  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  static Telemetry_Heartbeat deserialize(const uint8_t *buffer);
};

class Telemetry_GNSS {
public:
  uint32_t latitude = 0;  // degrees * 1e7
  uint32_t longitude = 0; // degrees * 1e7
  uint32_t altitude = 0;  // mm

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  static Telemetry_GNSS deserialize(const uint8_t *buffer);
};

} // namespace VCTR::packets::telecoms

#endif // EXVECTR_TELEMETRY_HPP