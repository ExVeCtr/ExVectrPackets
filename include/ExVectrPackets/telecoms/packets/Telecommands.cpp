#include "ExVectrPackets/telecoms/packets/Telecommands.hpp"

namespace VCTR::packets::telecoms {

// ---- Telecommand_Reboot ----

size_t Telecommand_Reboot::getPacketType() const {
  return static_cast<size_t>(TelecommandType::Reboot);
}

size_t Telecommand_Reboot::getPacketDataType() const {
  return static_cast<size_t>(TelecommandType::Reboot);
}

size_t Telecommand_Reboot::numBytes() const { return 0; }

void Telecommand_Reboot::serialize(uint8_t * /*buffer*/) const {}

Telecommand_Reboot Telecommand_Reboot::deserialize(const uint8_t * /*buffer*/) {
  return Telecommand_Reboot{};
}

// ---- Telecommand_UpdateMode ----

size_t Telecommand_UpdateMode::getPacketType() const {
  return static_cast<size_t>(TelecommandType::UpdateMode);
}

size_t Telecommand_UpdateMode::getPacketDataType() const {
  return static_cast<size_t>(TelecommandType::UpdateMode);
}

size_t Telecommand_UpdateMode::numBytes() const { return 0; }

void Telecommand_UpdateMode::serialize(uint8_t * /*buffer*/) const {}

Telecommand_UpdateMode
Telecommand_UpdateMode::deserialize(const uint8_t * /*buffer*/) {
  return Telecommand_UpdateMode{};
}

} // namespace VCTR::packets::telecoms