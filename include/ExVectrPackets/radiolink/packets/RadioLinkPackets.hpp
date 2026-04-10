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

namespace VCTR::packets::radiolink {

/**
 * @brief The RadioLinkPacketFilter class is responsible for filtering out false
 * values.
 */
template <size_t Ax, size_t Dx, RadioLinkTypes LinkType>
class RadioLinkPacketFilter {
private:
  int16_t analogRejectionThreshold = 200; // Values from -1000 to 1000
  size_t analogmaxRejectionCount = 1;     // Number of consecutive rejections

  size_t analogValueRejectedClount[Ax]; // How often a channel has been rejected

  int64_t lastUpdatedTimestamp = 0;

  RadioLinkPacket<Ax, Dx, LinkType> filteredLinkPacket;

public:
  RadioLinkPacketFilter() {
    for (size_t i = 0; i < Ax; i++) {
      analogValueRejectedClount[i] = 0;
      filteredLinkPacket.analogChannels[i] = 0;
    }
    for (size_t i = 0; i < Dx; i++) {
      filteredLinkPacket.digitalChannels[i] = 0;
    }
  }

  void setAnalogRejectionThreshold(int16_t threshold) {
    analogRejectionThreshold = threshold;
  }

  void setAnalogMaxRejectionCount(size_t count) {
    analogmaxRejectionCount = count;
  }

  void setFilteredPacket(const RadioLinkPacket<Ax, Dx, LinkType> &packet) {
    filteredLinkPacket = packet;
  }

  const RadioLinkPacket<Ax, Dx, LinkType> &getFilteredPacket() const {
    return filteredLinkPacket;
  }

  void updateFromPacket(const RadioLinkPacket<Ax, Dx, LinkType> &packet,
                        int64_t timeReceived) {

    for (size_t i = 0; i < Ax; i++) {
      auto value = packet.analogChannels[i];
      auto lastValue = filteredLinkPacket.analogChannels[i];
      auto delta = value - lastValue;

      if (std::abs(delta) > analogRejectionThreshold) {
        analogValueRejectedClount[i]++;
        if (analogValueRejectedClount[i] > analogmaxRejectionCount) {
          filteredLinkPacket.analogChannels[i] = value;
          analogValueRejectedClount[i] = 0;
        }
      } else {
        filteredLinkPacket.analogChannels[i] = value;
        analogValueRejectedClount[i] = 0;
      }
    }
    for (size_t i = 0; i < Dx; i++) {
      filteredLinkPacket.digitalChannels[i] = packet.digitalChannels[i];
    }
  }
};

} // namespace VCTR::packets::radiolink

#endif // EXVECTR_RADIOLINK_PACKETS_HPP