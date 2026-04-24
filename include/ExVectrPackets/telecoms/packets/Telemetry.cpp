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
  return sizeof(speed) + sizeof(satellites) + 2;
}

void Telemetry_GNSS::serialize(uint8_t *buffer) const {
  size_t offset = 0;
  buffer[offset++] = static_cast<uint8_t>(getPacketType());
  buffer[offset++] = static_cast<uint8_t>(getPacketDataType());
  memcpy(buffer + offset, &speed, sizeof(speed));
  offset += sizeof(speed);
  memcpy(buffer + offset, &satellites, sizeof(satellites));
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