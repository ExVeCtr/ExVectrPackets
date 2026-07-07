#include <cstring>

#include "ExVectrPackets/telecoms/packets/Telemetry.hpp"

namespace VCTR::packets::telecoms {

namespace {

// Latitude/longitude packed as 24-bit fixed point instead of CRSF's 32-bit
// wire encoding, since 2x int32 (8 bytes) plus the 2-byte packet header
// wouldn't fit in one OTA frame. 40000 units/degree keeps the extremes of
// longitude (+-180 deg -> +-7,200,000) comfortably inside the 24-bit signed
// range (+-8,388,607) and gives ~2.8m resolution -- plenty for telemetry
// display over a link this constrained.
constexpr double GNSS_POSITION_SCALE = 40000.0;
constexpr int32_t GNSS_POSITION_MAX = 8388607;
constexpr int32_t GNSS_POSITION_MIN = -8388608;

int32_t packGnssCoordinate(double degrees) {
  double scaled = degrees * GNSS_POSITION_SCALE;
  if (scaled > GNSS_POSITION_MAX)
    scaled = GNSS_POSITION_MAX;
  if (scaled < GNSS_POSITION_MIN)
    scaled = GNSS_POSITION_MIN;
  return (int32_t)scaled;
}

double unpackGnssCoordinate(int32_t raw) {
  return (double)raw / GNSS_POSITION_SCALE;
}

void writeI24(uint8_t *buffer, int32_t value) {
  buffer[0] = (uint8_t)((value >> 16) & 0xFF);
  buffer[1] = (uint8_t)((value >> 8) & 0xFF);
  buffer[2] = (uint8_t)(value & 0xFF);
}

int32_t readI24(const uint8_t *buffer) {
  uint32_t raw = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) |
                 (uint32_t)buffer[2];
  if (raw & 0x800000u) {
    raw |= 0xFF000000u; // Sign-extend 24 -> 32 bits.
  }
  return (int32_t)raw;
}

} // namespace

// ---- Telemetry_Heartbeat ----
size_t Telemetry_Heartbeat::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::Telemetry);
}

size_t Telemetry_Heartbeat::getPacketDataType() const {
  return static_cast<size_t>(TelemetryType::Heartbeat);
}

size_t Telemetry_Heartbeat::numBytes() const { return 2; }

void Telemetry_Heartbeat::serialize(uint8_t *buffer) const {
  buffer[0] = static_cast<uint8_t>(getPacketType());
  buffer[1] = static_cast<uint8_t>(getPacketDataType());
}

bool Telemetry_Heartbeat::deserialize(const uint8_t *buffer) {
  return buffer[0] == static_cast<uint8_t>(getPacketType()) &&
         buffer[1] == static_cast<uint8_t>(getPacketDataType());
}

// ---- Telemetry_GNSS ----

size_t Telemetry_GNSS::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::Telemetry);
}

size_t Telemetry_GNSS::getPacketDataType() const {
  return static_cast<size_t>(TelemetryType::GNSS);
}

size_t Telemetry_GNSS::numBytes() const {
  return sizeof(speed) + sizeof(satellites) + sizeof(altitudeM) + 2;
}

void Telemetry_GNSS::serialize(uint8_t *buffer) const {
  size_t offset = 0;
  buffer[offset++] = static_cast<uint8_t>(getPacketType());
  buffer[offset++] = static_cast<uint8_t>(getPacketDataType());
  memcpy(buffer + offset, &speed, sizeof(speed));
  offset += sizeof(speed);
  memcpy(buffer + offset, &satellites, sizeof(satellites));
  offset += sizeof(satellites);
  memcpy(buffer + offset, &altitudeM, sizeof(altitudeM));
}

bool Telemetry_GNSS::deserialize(const uint8_t *buffer) {
  size_t offset = 0;
  if (buffer[offset++] != static_cast<uint8_t>(getPacketType()))
    return false;
  if (buffer[offset++] != static_cast<uint8_t>(getPacketDataType()))
    return false;
  memcpy(&speed, buffer + offset, sizeof(speed));
  offset += sizeof(speed);
  memcpy(&satellites, buffer + offset, sizeof(satellites));
  offset += sizeof(satellites);
  memcpy(&altitudeM, buffer + offset, sizeof(altitudeM));
  return true;
}

// ---- Telemetry_GNSSPosition ----

size_t Telemetry_GNSSPosition::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::Telemetry);
}

size_t Telemetry_GNSSPosition::getPacketDataType() const {
  return static_cast<size_t>(TelemetryType::GNSSPosition);
}

size_t Telemetry_GNSSPosition::numBytes() const { return 2 + 3 + 3; }

void Telemetry_GNSSPosition::serialize(uint8_t *buffer) const {
  buffer[0] = static_cast<uint8_t>(getPacketType());
  buffer[1] = static_cast<uint8_t>(getPacketDataType());
  writeI24(buffer + 2, packGnssCoordinate(latitude));
  writeI24(buffer + 5, packGnssCoordinate(longitude));
}

bool Telemetry_GNSSPosition::deserialize(const uint8_t *buffer) {
  if (buffer[0] != static_cast<uint8_t>(getPacketType()))
    return false;
  if (buffer[1] != static_cast<uint8_t>(getPacketDataType()))
    return false;
  latitude = unpackGnssCoordinate(readI24(buffer + 2));
  longitude = unpackGnssCoordinate(readI24(buffer + 5));
  return true;
}

// ---- Telemetry_Battery ----
size_t Telemetry_Battery::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::Telemetry);
}
size_t Telemetry_Battery::getPacketDataType() const {
  return static_cast<size_t>(TelemetryType::Battery);
}
size_t Telemetry_Battery::numBytes() const { return sizeof(uint16_t) * 2 + 2; }
void Telemetry_Battery::serialize(uint8_t *buffer) const {
  size_t offset = 0;
  uint16_t temp;
  buffer[offset++] = static_cast<uint8_t>(getPacketType());
  buffer[offset++] = static_cast<uint8_t>(getPacketDataType());
  temp = (uint16_t)(voltage * 1000.0f); // mV
  memcpy(buffer + offset, &temp, sizeof(temp));
  offset += sizeof(temp);
  temp = (uint16_t)(current * 100.0f); // cA
  memcpy(buffer + offset, &temp, sizeof(temp));
}
bool Telemetry_Battery::deserialize(const uint8_t *buffer) {
  size_t offset = 0;
  if (buffer[offset++] != static_cast<uint8_t>(getPacketType()))
    return false;
  if (buffer[offset++] != static_cast<uint8_t>(getPacketDataType()))
    return false;
  uint16_t temp;
  memcpy(&temp, buffer + offset, sizeof(temp));
  voltage = static_cast<float>(temp) / 1000.0f;
  offset += sizeof(temp);
  memcpy(&temp, buffer + offset, sizeof(temp));
  current = static_cast<float>(temp) / 100.0f;
  return true;
}

} // namespace VCTR::packets::telecoms