#ifndef EXVECTR_RADIOLINK_PACKETS_HPP
#define EXVECTR_RADIOLINK_PACKETS_HPP

#include <cstddef>
#include <stdint.h>

#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/radiolink/RadioLinkTypes.hpp"

namespace VCTR::packets::radiolink {

template <size_t Ax, size_t Dx, RadioLinkTypes LinkType> class RadioLinkPacket {
public:
  // Usually 4 ch with roll, pitch, yaw, thr
  int16_t analogChannels[Ax];  // Values from -1000 to 1000
  uint8_t digitalChannels[Dx]; // Values from 0-3

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

using RadioLinkPacket_A4D0 = RadioLinkPacket<4, 0, RadioLinkTypes::A4D0>;
using RadioLinkPacket_A4D2 = RadioLinkPacket<4, 2, RadioLinkTypes::A4D2>;
using RadioLinkPacket_A4D4 = RadioLinkPacket<4, 4, RadioLinkTypes::A4D4>;
using RadioLinkPacket_A6D6 = RadioLinkPacket<6, 6, RadioLinkTypes::A6D6>;

} // namespace VCTR::packets::radiolink
#endif // EXVECTR_RADIOLINK_PACKETS_HPP