#include "ExVectrPackets/telecoms/packets/Telecommands.hpp"

namespace VCTR::packets::telecoms {

// ---- Telecommand_Reboot ----

size_t Telecommand_Reboot::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::Telecommand);
}

size_t Telecommand_Reboot::getPacketDataType() const {
  return static_cast<size_t>(TelecommandType::Reboot);
}

size_t Telecommand_Reboot::numBytes() const { return 2; }

void Telecommand_Reboot::serialize(uint8_t *buffer) const {
  buffer[0] = static_cast<uint8_t>(getPacketType());
  buffer[1] = static_cast<uint8_t>(getPacketDataType());
}

bool Telecommand_Reboot::deserialize(const uint8_t *buffer) {
  return buffer[0] == static_cast<uint8_t>(getPacketType()) &&
         buffer[1] == static_cast<uint8_t>(getPacketDataType());
}

// ---- Telecommand_UpdateMode ----

size_t Telecommand_UpdateMode::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::Telecommand);
}

size_t Telecommand_UpdateMode::getPacketDataType() const {
  return static_cast<size_t>(TelecommandType::UpdateMode);
}

size_t Telecommand_UpdateMode::numBytes() const { return 2; }

void Telecommand_UpdateMode::serialize(uint8_t *buffer) const {
  buffer[0] = static_cast<uint8_t>(getPacketType());
  buffer[1] = static_cast<uint8_t>(getPacketDataType());
}

bool Telecommand_UpdateMode::deserialize(const uint8_t *buffer) {
  return buffer[0] == static_cast<uint8_t>(getPacketType()) &&
         buffer[1] == static_cast<uint8_t>(getPacketDataType());
}

} // namespace VCTR::packets::telecoms