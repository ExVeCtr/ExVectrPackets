#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"

namespace VCTR::packets::radiolink /* Helper functions */ {

size_t writeBits(uint8_t *bufferOut, const uint8_t *bufferIn,
                 size_t startBitIndex, size_t numBits) {
  for (size_t i = 0; i < numBits; ++i) {
    size_t byteIndex = (startBitIndex + i) / 8;
    size_t bitInByte = (startBitIndex + i) % 8;
    bufferOut[byteIndex] &= ~(1 << bitInByte);
    bufferOut[byteIndex] |= ((bufferIn[i / 8] >> (i % 8)) & 1) << bitInByte;
  }
  return startBitIndex + numBits;
}

size_t readBits(const uint8_t *bufferIn, uint8_t *bufferOut,
                size_t startBitIndex, size_t numBits) {
  for (size_t i = 0; i < numBits; ++i) {
    size_t byteIndex = (startBitIndex + i) / 8;
    size_t bitInByte = (startBitIndex + i) % 8;
    bufferOut[i / 8] &= ~(1 << (i % 8));
    bufferOut[i / 8] |= ((bufferIn[byteIndex] >> bitInByte) & 1) << (i % 8);
  }
  return startBitIndex + numBits;
}

} // namespace VCTR::packets::radiolink

namespace VCTR::packets::radiolink {

template <size_t Ax, size_t Dx, RadioLinkTypes LinkType>
size_t RadioLinkPacket<Ax, Dx, LinkType>::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::RadioLink);
}

template <size_t Ax, size_t Dx, RadioLinkTypes LinkType>
size_t RadioLinkPacket<Ax, Dx, LinkType>::getPacketDataType() const {
  return static_cast<size_t>(LinkType);
}

template <size_t Ax, size_t Dx, RadioLinkTypes LinkType>
size_t RadioLinkPacket<Ax, Dx, LinkType>::numBytes() const {
  auto numBits =
      Ax * 11 +
      Dx * 2; // 11 bits per analog channel, 2 bits per digital channel
  return (numBits + 7) / 8; // Convert bits to bytes, rounding up
}

template <size_t Ax, size_t Dx, RadioLinkTypes LinkType>
void RadioLinkPacket<Ax, Dx, LinkType>::serialize(uint8_t *buffer) const {
  size_t bitIndex = 0;

  for (size_t i = 0; i < Ax; ++i) {
    uint16_t value = static_cast<uint16_t>(analogChannels[i] + 1000);
    bitIndex =
        writeBits(buffer, reinterpret_cast<uint8_t *>(&value), bitIndex, 11);
  }

  for (size_t i = 0; i < Dx; ++i) {
    uint8_t value = digitalChannels[i] & 0x03;
    bitIndex = writeBits(buffer, &value, bitIndex, 2);
  }
}

template <size_t Ax, size_t Dx, RadioLinkTypes LinkType>
RadioLinkPacket<Ax, Dx, LinkType>
RadioLinkPacket<Ax, Dx, LinkType>::deserialize(const uint8_t *buffer) {
  RadioLinkPacket<Ax, Dx, LinkType> packet;
  size_t bitIndex = 0;

  // Analog channels are packed first — must match serialize() order.
  for (size_t i = 0; i < Ax; ++i) {
    uint16_t value = 0;
    bitIndex =
        readBits(buffer, reinterpret_cast<uint8_t *>(&value), bitIndex, 11);
    packet.analogChannels[i] = static_cast<int16_t>(value) - 1000;
  }

  for (size_t i = 0; i < Dx; ++i) {
    uint8_t value = 0;
    bitIndex = readBits(buffer, &value, bitIndex, 2);
    packet.digitalChannels[i] = value & 0x03;
  }

  return packet;
}

// Explicit instantiations so out-of-line template definitions are compiled.
template class RadioLinkPacket<4, 0, RadioLinkTypes::A4D0>;
template class RadioLinkPacket<4, 2, RadioLinkTypes::A4D2>;
template class RadioLinkPacket<4, 4, RadioLinkTypes::A4D4>;
template class RadioLinkPacket<6, 6, RadioLinkTypes::A6D6>;

} // namespace VCTR::packets::radiolink