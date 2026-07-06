#ifndef EXVECTR_CRSF_PACKETS_HPP
#define EXVECTR_CRSF_PACKETS_HPP

#include <cstddef>
#include <stdint.h>

#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/crsf/CRSFTypes.hpp"

namespace VCTR::packets::crsf {

/**
 * CRSF frame type 0x16, RC Channels Packed Payload.
 * Channel values are raw 11-bit ticks (0-2047), no domain scaling applied.
 */
class CRSFPacket_RcChannelsPacked {
public:
  uint16_t channels[16] = {0};

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x14, Link Statistics.
 */
class CRSFPacket_LinkStatistics {
public:
  uint8_t upRssiAnt1 = 0;   // Uplink RSSI Antenna 1 (dBm * -1)
  uint8_t upRssiAnt2 = 0;   // Uplink RSSI Antenna 2 (dBm * -1)
  uint8_t upLinkQuality = 0; // Uplink link quality (%)
  int8_t upSnr = 0;         // Uplink SNR (dB)
  uint8_t activeAntenna = 0;
  uint8_t rfProfile = 0;
  uint8_t upRfPower = 0;
  uint8_t downRssi = 0;       // Downlink RSSI (dBm * -1)
  uint8_t downLinkQuality = 0; // Downlink link quality (%)
  int8_t downSnr = 0;         // Downlink SNR (dB)

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x08, Battery Sensor.
 */
class CRSFPacket_BatterySensor {
public:
  float voltage = 0.0f;           // Volts
  float current = 0.0f;           // Amps
  uint32_t capacityUsedMah = 0;   // mAh, 24-bit range on the wire
  uint8_t remainingPercent = 0;   // %

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x02, GPS.
 */
class CRSFPacket_GPS {
public:
  double latitude = 0.0;      // degrees
  double longitude = 0.0;     // degrees
  float groundspeedKmh = 0.0f;
  float headingDeg = 0.0f;
  float altitudeM = 0.0f;
  uint8_t satellites = 0;

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x32, Direct Command, CommandID 0x10 (Crossfire) /
 * SubCommand 0x01 (Set receiver in bind mode). Extended-header frame.
 */
class CRSFPacket_CommandRxBind {
public:
  uint8_t destAddress = 0xEC; // R/C Receiver / Crossfire Rx
  uint8_t origAddress = 0xC8; // Flight controller

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

} // namespace VCTR::packets::crsf

#endif // EXVECTR_CRSF_PACKETS_HPP
