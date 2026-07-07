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

/**
 * GPS "status" fields -- everything except position, which doesn't fit in
 * the same OTA frame (see Telemetry_GNSSPosition).
 */
class Telemetry_GNSS {
public:
  uint16_t speed = 0; // km/h
  uint8_t satellites = 0;
  int16_t altitudeM = 0; // metres

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * GPS position. Sent as its own OTA frame separate from Telemetry_GNSS's
 * other fields since latitude+longitude alone already use the full OTA
 * frame budget. Wire-packed as 24-bit fixed-point (~2.8m resolution,
 * plenty for telemetry display) rather than the 32-bit CRSF wire encoding,
 * which wouldn't fit -- see Telemetry.cpp for the exact scale.
 */
class Telemetry_GNSSPosition {
public:
  double latitude = 0.0;  // degrees
  double longitude = 0.0; // degrees

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