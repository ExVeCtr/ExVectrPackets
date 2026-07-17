#ifndef EXVECTR_RADIOLINK_MUX_PACKETS_HPP
#define EXVECTR_RADIOLINK_MUX_PACKETS_HPP

#include <algorithm>
#include <cstddef>
#include <stdint.h>

#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/radiolink/packets/RadioLinkPackets.hpp"

/**
 * Multiplexed RC channel transport.
 *
 * A classic RadioLinkPacket sends every channel in every packet, so the
 * channel count is hard-limited by the on-air packet size. This system
 * splits the channels into two groups:
 *
 *  - PRIORITY channels: present in every packet, full update rate.
 *    (Typically roll/pitch/yaw/throttle plus e.g. an arm switch.)
 *  - MULTIPLEXED channels: one per packet. Each packet carries a channel
 *    index plus that channel's value, and the sender cycles through them,
 *    so they update at 1/(Ma+Md) of the packet rate.
 *
 * Channel numbering is unified and stable: analog channels 0..Pa-1 are the
 * priority ones, Pa..Pa+Ma-1 the multiplexed ones; digital channels 0..Pd-1
 * priority, Pd..Pd+Md-1 multiplexed.
 *
 * The three pieces:
 *  - RadioLinkMuxPacket:      the fixed-size wire packet.
 *  - RadioLinkMuxCommander:   TX side. Holds the current value of every
 *                             channel and emits the next packet, rotating
 *                             the multiplexed slot.
 *  - RadioLinkMuxAccumulator: RX side. Consumes packets and offers every
 *                             channel together with its last-update
 *                             timestamp; can also export a full classic
 *                             RadioLinkPacket containing all channels.
 *
 * All parties must be instantiated with identical template parameters --
 * the configuration is part of the wire format.
 */
namespace VCTR::packets::radiolink {

namespace detail {
/// Number of bits needed to represent indices 0..count-1. 0 for count <= 1.
constexpr size_t bitsForIndexCount(size_t count) {
  size_t bits = 0;
  size_t maxIndex = count > 0 ? count - 1 : 0;
  while (maxIndex > 0) {
    ++bits;
    maxIndex >>= 1;
  }
  return bits;
}

/// Same -1000..1000 <-> 10-bit mapping as RadioLinkPacket.
constexpr uint16_t encodeAnalog10(int16_t value) {
  const int32_t clamped = value < -1000 ? -1000 : (value > 1000 ? 1000 : value);
  return static_cast<uint16_t>(((clamped + 1000) * 1023 + 1000) / 2000);
}
constexpr int16_t decodeAnalog10(uint16_t raw) {
  return static_cast<int16_t>(
      ((static_cast<int32_t>(raw & 0x03FF) * 2000 + 511) / 1023) - 1000);
}
} // namespace detail

/**
 * @brief Fixed-size RC channel packet with priority + multiplexed channels.
 *
 * Wire layout (LSB-first bit stream):
 *   Pa x 10 bit   priority analog channels (-1000..1000)
 *   Pd x  2 bit   priority digital channels (0..3)
 *   N  bits       multiplexed channel index (N = bits for Ma+Md indices)
 *   10 bit        multiplexed channel value (analog mapping; digital
 *                 channels use the raw 0..3 value so the size stays fixed)
 *   1 byte        (6-bit PacketType::RadioLinkMux << 2) | 2-bit DATATYPE
 *
 * @tparam Pa Number of priority analog channels (sent every packet).
 * @tparam Pd Number of priority digital channels (sent every packet).
 * @tparam Ma Number of multiplexed analog channels.
 * @tparam Md Number of multiplexed digital channels.
 * @tparam DATATYPE 2-bit sub-type, lets independent configurations coexist.
 */
template <size_t Pa, size_t Pd, size_t Ma, size_t Md, size_t DATATYPE = 0>
class RadioLinkMuxPacket {
  static_assert(Ma + Md >= 1,
                "No multiplexed channels: use a plain RadioLinkPacket.");
  static_assert(DATATYPE < 4, "DATATYPE must fit the 2-bit sub-type field.");

public:
  static constexpr size_t kNumMuxChannels = Ma + Md;
  static constexpr size_t kMuxIndexBits =
      detail::bitsForIndexCount(kNumMuxChannels);

  int16_t analogChannels[Pa];  ///< Priority, values -1000..1000.
  uint8_t digitalChannels[Pd]; ///< Priority, values 0..3.

  /// Which multiplexed channel this packet carries: 0..Ma-1 selects
  /// multiplexed analog channel Pa+index, Ma..Ma+Md-1 selects multiplexed
  /// digital channel Pd+(index-Ma).
  uint16_t muxChannelIndex = 0;
  /// The selected channel's value: -1000..1000 if analog, 0..3 if digital.
  int16_t muxChannelValue = 0;

  size_t getPacketType() const {
    return static_cast<size_t>(VCTR::packets::PacketType::RadioLinkMux);
  }
  size_t getPacketDataType() const { return DATATYPE; }

  size_t numBytes() const {
    constexpr size_t payloadBits = Pa * 10 + Pd * 2 + kMuxIndexBits + 10;
    return (payloadBits + 7) / 8 + 1; // + type byte
  }

  void serialize(uint8_t *buffer) const {
    const size_t payloadNumBytes = numBytes() - 1;
    for (size_t i = 0; i < payloadNumBytes; ++i) {
      buffer[i] = 0;
    }

    size_t bitIndex = 0;

    for (size_t i = 0; i < Pa; ++i) {
      const uint16_t value = detail::encodeAnalog10(analogChannels[i]);
      const uint8_t valueBytes[2] = {static_cast<uint8_t>(value & 0xFF),
                                     static_cast<uint8_t>((value >> 8) & 0xFF)};
      bitIndex = writeBits(buffer, valueBytes, bitIndex, 10);
    }

    for (size_t i = 0; i < Pd; ++i) {
      const uint8_t value = digitalChannels[i] & 0x03;
      bitIndex = writeBits(buffer, &value, bitIndex, 2);
    }

    if constexpr (kMuxIndexBits > 0) {
      const uint8_t indexBytes[2] = {
          static_cast<uint8_t>(muxChannelIndex & 0xFF),
          static_cast<uint8_t>((muxChannelIndex >> 8) & 0xFF)};
      bitIndex = writeBits(buffer, indexBytes, bitIndex, kMuxIndexBits);
    }

    const uint16_t muxRaw =
        muxChannelIndex < Ma
            ? detail::encodeAnalog10(muxChannelValue)
            : static_cast<uint16_t>(muxChannelValue & 0x03);
    const uint8_t muxBytes[2] = {static_cast<uint8_t>(muxRaw & 0xFF),
                                 static_cast<uint8_t>((muxRaw >> 8) & 0xFF)};
    bitIndex = writeBits(buffer, muxBytes, bitIndex, 10);

    // Same composite type byte as RadioLinkPacket: data type in the lower
    // 2 bits, packet type in the upper 6 bits.
    buffer[payloadNumBytes] = static_cast<uint8_t>(
        (getPacketDataType() & 0x03) | ((getPacketType() & 0x3F) << 2));
  }

  bool deserialize(const uint8_t *buffer) {
    const size_t payloadNumBytes = numBytes() - 1;
    const uint8_t typeByte = buffer[payloadNumBytes];
    if (((typeByte >> 2) & 0x3F) != static_cast<uint8_t>(getPacketType()) ||
        (typeByte & 0x03) != static_cast<uint8_t>(getPacketDataType())) {
      return false;
    }

    size_t bitIndex = 0;

    for (size_t i = 0; i < Pa; ++i) {
      uint8_t valueBytes[2] = {0, 0};
      bitIndex = readBits(buffer, valueBytes, bitIndex, 10);
      analogChannels[i] = detail::decodeAnalog10(
          static_cast<uint16_t>(valueBytes[0]) |
          (static_cast<uint16_t>(valueBytes[1]) << 8));
    }

    for (size_t i = 0; i < Pd; ++i) {
      uint8_t value = 0;
      bitIndex = readBits(buffer, &value, bitIndex, 2);
      digitalChannels[i] = value & 0x03;
    }

    muxChannelIndex = 0;
    if constexpr (kMuxIndexBits > 0) {
      uint8_t indexBytes[2] = {0, 0};
      bitIndex = readBits(buffer, indexBytes, bitIndex, kMuxIndexBits);
      muxChannelIndex = static_cast<uint16_t>(indexBytes[0]) |
                        (static_cast<uint16_t>(indexBytes[1]) << 8);
    }
    if (muxChannelIndex >= kNumMuxChannels) {
      return false;
    }

    uint8_t muxBytes[2] = {0, 0};
    bitIndex = readBits(buffer, muxBytes, bitIndex, 10);
    const uint16_t muxRaw = static_cast<uint16_t>(muxBytes[0]) |
                            (static_cast<uint16_t>(muxBytes[1]) << 8);
    muxChannelValue = muxChannelIndex < Ma
                          ? detail::decodeAnalog10(muxRaw)
                          : static_cast<int16_t>(muxRaw & 0x03);

    return true;
  }
};

/**
 * @brief TX-side channel source: holds the live value of every channel and
 * emits RadioLinkMuxPackets, rotating the multiplexed slot each packet.
 *
 * Feed it channel values via setAnalogChannel()/setDigitalChannel() (unified
 * indices, see file header), then call buildNextPacket() once per TX slot.
 */
template <size_t Pa, size_t Pd, size_t Ma, size_t Md, size_t DATATYPE = 0>
class RadioLinkMuxCommander {
public:
  using Packet = RadioLinkMuxPacket<Pa, Pd, Ma, Md, DATATYPE>;

  static constexpr size_t kNumAnalogChannels = Pa + Ma;
  static constexpr size_t kNumDigitalChannels = Pd + Md;
  static constexpr size_t kNumMuxChannels = Ma + Md;

  void setAnalogChannel(size_t channel, int16_t value) {
    if (channel < kNumAnalogChannels) {
      analogChannels[channel] = value;
    }
  }

  void setDigitalChannel(size_t channel, uint8_t value) {
    if (channel < kNumDigitalChannels) {
      digitalChannels[channel] = value & 0x03;
    }
  }

  int16_t getAnalogChannel(size_t channel) const {
    return channel < kNumAnalogChannels ? analogChannels[channel] : 0;
  }

  uint8_t getDigitalChannel(size_t channel) const {
    return channel < kNumDigitalChannels ? digitalChannels[channel] : 0;
  }

  /**
   * @brief Builds the packet to transmit next: the current priority channel
   * values plus the next multiplexed channel in the round-robin rotation.
   */
  Packet buildNextPacket() {
    Packet packet;

    for (size_t i = 0; i < Pa; ++i) {
      packet.analogChannels[i] = analogChannels[i];
    }
    for (size_t i = 0; i < Pd; ++i) {
      packet.digitalChannels[i] = digitalChannels[i];
    }

    packet.muxChannelIndex = static_cast<uint16_t>(muxCursor);
    packet.muxChannelValue =
        muxCursor < Ma
            ? analogChannels[Pa + muxCursor]
            : static_cast<int16_t>(digitalChannels[Pd + (muxCursor - Ma)]);

    muxCursor = (muxCursor + 1) % kNumMuxChannels;
    return packet;
  }

private:
  int16_t analogChannels[kNumAnalogChannels] = {};
  uint8_t digitalChannels[kNumDigitalChannels] = {};
  size_t muxCursor = 0;
};

/**
 * @brief RX-side channel sink: consumes RadioLinkMuxPackets and accumulates
 * the newest value of every channel, each with a last-updated timestamp.
 *
 * Priority channels are stamped on every packet; a multiplexed channel is
 * stamped whenever a packet carrying it arrives, so its timestamp tells the
 * consumer how stale the value is. Timestamps are whatever clock the caller
 * passes to updateFromPacket() (0 = never received).
 */
template <size_t Pa, size_t Pd, size_t Ma, size_t Md, size_t DATATYPE = 0>
class RadioLinkMuxAccumulator {
public:
  using Packet = RadioLinkMuxPacket<Pa, Pd, Ma, Md, DATATYPE>;

  static constexpr size_t kNumAnalogChannels = Pa + Ma;
  static constexpr size_t kNumDigitalChannels = Pd + Md;

  /**
   * @brief Merges a received packet into the accumulated channel state.
   * @param timeReceived Timestamp stored for every channel the packet
   * carried (the priority ones plus its multiplexed one).
   */
  void updateFromPacket(const Packet &packet, int64_t timeReceived) {
    for (size_t i = 0; i < Pa; ++i) {
      analogChannels[i] = packet.analogChannels[i];
      analogTimestamps[i] = timeReceived;
    }
    for (size_t i = 0; i < Pd; ++i) {
      digitalChannels[i] = packet.digitalChannels[i];
      digitalTimestamps[i] = timeReceived;
    }

    if (packet.muxChannelIndex < Ma) {
      const size_t channel = Pa + packet.muxChannelIndex;
      analogChannels[channel] = packet.muxChannelValue;
      analogTimestamps[channel] = timeReceived;
    } else if (packet.muxChannelIndex < Packet::kNumMuxChannels) {
      const size_t channel = Pd + (packet.muxChannelIndex - Ma);
      digitalChannels[channel] =
          static_cast<uint8_t>(packet.muxChannelValue & 0x03);
      digitalTimestamps[channel] = timeReceived;
    }
  }

  int16_t getAnalogChannel(size_t channel) const {
    return channel < kNumAnalogChannels ? analogChannels[channel] : 0;
  }
  /// When the analog channel last received a value. 0 if never.
  int64_t getAnalogChannelTimestamp(size_t channel) const {
    return channel < kNumAnalogChannels ? analogTimestamps[channel] : 0;
  }

  uint8_t getDigitalChannel(size_t channel) const {
    return channel < kNumDigitalChannels ? digitalChannels[channel] : 0;
  }
  /// When the digital channel last received a value. 0 if never.
  int64_t getDigitalChannelTimestamp(size_t channel) const {
    return channel < kNumDigitalChannels ? digitalTimestamps[channel] : 0;
  }

  /**
   * @brief Fills a classic RadioLinkPacket with the accumulated channel
   * state, e.g. to hand the full channel set to an existing consumer.
   * Channels beyond what this accumulator holds are set to 0.
   */
  template <size_t Ax, size_t Dx, RadioLinkTypes LinkType>
  void fillPacket(RadioLinkPacket<Ax, Dx, LinkType> &packet) const {
    for (size_t i = 0; i < Ax; ++i) {
      packet.analogChannels[i] = getAnalogChannel(i);
    }
    for (size_t i = 0; i < Dx; ++i) {
      packet.digitalChannels[i] = getDigitalChannel(i);
    }
  }

private:
  int16_t analogChannels[kNumAnalogChannels] = {};
  uint8_t digitalChannels[kNumDigitalChannels] = {};
  int64_t analogTimestamps[kNumAnalogChannels] = {};
  int64_t digitalTimestamps[kNumDigitalChannels] = {};
};

} // namespace VCTR::packets::radiolink

#endif // EXVECTR_RADIOLINK_MUX_PACKETS_HPP
