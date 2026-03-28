#ifndef EXVECTR_TELECOMMANDS_HPP
#define EXVECTR_TELECOMMANDS_HPP

#include <cstddef>
#include <stdint.h>

#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/telecoms/TelecomTypes.hpp"

namespace VCTR::packets::telecoms {

using namespace VCTR::packets::telecoms;

class Telecommand_Reboot {
public:
  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

class Telecommand_UpdateMode {
public:
  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

} // namespace VCTR::packets::telecoms

#endif // EXVECTR_TELECOMMANDS_HPP