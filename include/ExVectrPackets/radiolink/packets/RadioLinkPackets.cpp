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
  auto numBits = Ax * 10 + Dx * 2;
  return (numBits + 7) / 8 + 2; // payload + packet type + packet data type
}

template <size_t Ax, size_t Dx, RadioLinkTypes LinkType>
void RadioLinkPacket<Ax, Dx, LinkType>::serialize(uint8_t *buffer) const {
  const size_t payloadNumBytes = numBytes() - 2;
  for (size_t i = 0; i < payloadNumBytes; ++i) {
    buffer[i] = 0;
  }

  size_t bitIndex = 0;

  for (size_t i = 0; i < Ax; ++i) {
    uint16_t value = static_cast<uint16_t>(float(analogChannels[i] + 1000) /
                                           2000.0f * 1023.0f);
    bitIndex =
        writeBits(buffer, reinterpret_cast<uint8_t *>(&value), bitIndex, 10);
  }

  for (size_t i = 0; i < Dx; ++i) {
    uint8_t value = digitalChannels[i] & 0x03;
    bitIndex = writeBits(buffer, &value, bitIndex, 2);
  }

  buffer[payloadNumBytes] = static_cast<uint8_t>(getPacketType());
  buffer[payloadNumBytes + 1] = static_cast<uint8_t>(getPacketDataType());
}

template <size_t Ax, size_t Dx, RadioLinkTypes LinkType>
bool RadioLinkPacket<Ax, Dx, LinkType>::deserialize(const uint8_t *buffer) {
  const size_t payloadNumBytes = numBytes() - 2;
  if (buffer[payloadNumBytes] != static_cast<uint8_t>(getPacketType())) {
    return false;
  }
  if (buffer[payloadNumBytes + 1] !=
      static_cast<uint8_t>(getPacketDataType())) {
    return false;
  }

  size_t bitIndex = 0;

  // Analog channels are packed first — must match serialize() order.
  for (size_t i = 0; i < Ax; ++i) {
    uint16_t value = 0;
    bitIndex =
        readBits(buffer, reinterpret_cast<uint8_t *>(&value), bitIndex, 10);
    analogChannels[i] = static_cast<int16_t>(value) - 1000;
  }

  for (size_t i = 0; i < Dx; ++i) {
    uint8_t value = 0;
    bitIndex = readBits(buffer, &value, bitIndex, 2);
    digitalChannels[i] = value & 0x03;
  }

  return true;
}

// Explicit instantiations so out-of-line template definitions are compiled.
template class RadioLinkPacket<4, 0, RadioLinkTypes::A4D0>;
template class RadioLinkPacket<4, 2, RadioLinkTypes::A4D2>;
template class RadioLinkPacket<4, 4, RadioLinkTypes::A4D4>;
template class RadioLinkPacket<6, 6, RadioLinkTypes::A6D6>;

} // namespace VCTR::packets::radiolink