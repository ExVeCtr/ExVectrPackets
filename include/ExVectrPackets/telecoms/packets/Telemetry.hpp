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
  uint16_t speed = 0; // CRSF GPS speed encoding
  uint8_t satellites = 0;

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

class Telemetry_Battery {
public:
  float voltage = 0.0f; // Volts, max expected range ~0-65.535V
  float current = 0.0f; // Amps, max expected range ~0-655.35A

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

} // namespace VCTR::packets::telecoms

#endif // EXVECTR_TELEMETRY_HPP