#include <cstring>

#include "ExVectrPackets/telecoms/packets/Telemetry.hpp"

namespace VCTR::packets::telecoms {

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
  return sizeof(latitude) + sizeof(longitude) + sizeof(altitude) + 2;
}

void Telemetry_GNSS::serialize(uint8_t *buffer) const {
  size_t offset = 0;
  buffer[offset++] = static_cast<uint8_t>(getPacketType());
  buffer[offset++] = static_cast<uint8_t>(getPacketDataType());
  memcpy(buffer + offset, &latitude, sizeof(latitude));
  offset += sizeof(latitude);
  memcpy(buffer + offset, &longitude, sizeof(longitude));
  offset += sizeof(longitude);
  memcpy(buffer + offset, &altitude, sizeof(altitude));
  offset += sizeof(altitude);
}

bool Telemetry_GNSS::deserialize(const uint8_t *buffer) {
  size_t offset = 0;
  if (buffer[offset++] != static_cast<uint8_t>(getPacketType())) {
    return false;
  }
  if (buffer[offset++] != static_cast<uint8_t>(getPacketDataType())) {
    return false;
  }
  memcpy(&latitude, buffer + offset, sizeof(latitude));
  offset += sizeof(latitude);
  memcpy(&longitude, buffer + offset, sizeof(longitude));
  offset += sizeof(longitude);
  memcpy(&altitude, buffer + offset, sizeof(altitude));
  offset += sizeof(altitude);
  return true;
}

} // namespace VCTR::packets::telecoms