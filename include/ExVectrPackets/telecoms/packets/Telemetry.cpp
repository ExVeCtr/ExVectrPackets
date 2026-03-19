#include <cstring>

#include "ExVectrPackets/telecoms/packets/Telemetry.hpp"

namespace VCTR::packets::telecoms {

// ---- Telemetry_Heartbeat ----
size_t Telemetry_Heartbeat::getPacketType() const {
  return static_cast<size_t>(TelemetryType::Heartbeat);
}

size_t Telemetry_Heartbeat::getPacketDataType() const {
  return static_cast<size_t>(TelemetryType::Heartbeat);
}

size_t Telemetry_Heartbeat::numBytes() const { return 0; }

void Telemetry_Heartbeat::serialize(uint8_t * /*buffer*/) const {}

Telemetry_Heartbeat
Telemetry_Heartbeat::deserialize(const uint8_t * /*buffer*/) {
  return Telemetry_Heartbeat{};
}

// ---- Telemetry_GNSS ----

size_t Telemetry_GNSS::getPacketType() const {
  return static_cast<size_t>(TelemetryType::GNSS);
}

size_t Telemetry_GNSS::getPacketDataType() const {
  return static_cast<size_t>(TelemetryType::GNSS);
}

size_t Telemetry_GNSS::numBytes() const {
  return sizeof(latitude) + sizeof(longitude) + sizeof(altitude);
}

void Telemetry_GNSS::serialize(uint8_t *buffer) const {
  size_t offset = 0;
  memcpy(buffer + offset, &latitude, sizeof(latitude));
  offset += sizeof(latitude);
  memcpy(buffer + offset, &longitude, sizeof(longitude));
  offset += sizeof(longitude);
  memcpy(buffer + offset, &altitude, sizeof(altitude));
  offset += sizeof(altitude);
}

Telemetry_GNSS Telemetry_GNSS::deserialize(const uint8_t *buffer) {
  Telemetry_GNSS gnss;
  size_t offset = 0;
  memcpy(&gnss.latitude, buffer + offset, sizeof(gnss.latitude));
  offset += sizeof(gnss.latitude);
  memcpy(&gnss.longitude, buffer + offset, sizeof(gnss.longitude));
  offset += sizeof(gnss.longitude);
  memcpy(&gnss.altitude, buffer + offset, sizeof(gnss.altitude));
  offset += sizeof(gnss.altitude);
  return gnss;
}

} // namespace VCTR::packets::telecoms