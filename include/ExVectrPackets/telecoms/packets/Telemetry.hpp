#ifndef EXVECTR_TELEMETRY_HPP
#define EXVECTR_TELEMETRY_HPP

#include <cstddef>
#include <stdint.h>

#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/telecoms/TelecomTypes.hpp"

namespace VCTR::packets::telecoms {

using namespace VCTR::packets::telecoms;

class Telemetry_Heartbeat {
public:
  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
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
  bool deserialize(const uint8_t *buffer);
};

class Telemetry_Battery {
public:
  float voltage = 0.0f;  // Volts, max expected range ~0-65.535V
  float current = 0.0f;  // Amps, max expected range ~0-655.35A
  float capacity = 0.0f; // mAh, max expected range ~0-655350 mAh

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

} // namespace VCTR::packets::telecoms

#endif // EXVECTR_TELEMETRY_HPP