#include "ExVectrPackets/crsf/packets/CRSFPackets.hpp"

namespace VCTR::packets::crsf /* Helper functions */ {

constexpr uint8_t CRSF_SYNC_BYTE = 0xC8;

// CRC-8, poly 0xD5, init 0. Covers Type + Payload of a standard CRSF frame.
uint8_t crc8_d5(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// CRC-8, poly 0xBA, init 0. Only used for the inner Command_CRC8 of 0x32
// Direct Command frames.
uint8_t crc8_ba(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xBA) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// CRSF is big-endian on the wire.
void writeU16BE(uint8_t *buffer, uint16_t value) {
  buffer[0] = (uint8_t)(value >> 8);
  buffer[1] = (uint8_t)(value & 0xFF);
}

uint16_t readU16BE(const uint8_t *buffer) {
  return (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
}

void writeI16BE(uint8_t *buffer, int16_t value) {
  writeU16BE(buffer, (uint16_t)value);
}

int16_t readI16BE(const uint8_t *buffer) {
  return (int16_t)readU16BE(buffer);
}

void writeU24BE(uint8_t *buffer, uint32_t value) {
  buffer[0] = (uint8_t)((value >> 16) & 0xFF);
  buffer[1] = (uint8_t)((value >> 8) & 0xFF);
  buffer[2] = (uint8_t)(value & 0xFF);
}

uint32_t readU24BE(const uint8_t *buffer) {
  return ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) |
         (uint32_t)buffer[2];
}

void writeI32BE(uint8_t *buffer, int32_t value) {
  const uint32_t u = (uint32_t)value;
  buffer[0] = (uint8_t)((u >> 24) & 0xFF);
  buffer[1] = (uint8_t)((u >> 16) & 0xFF);
  buffer[2] = (uint8_t)((u >> 8) & 0xFF);
  buffer[3] = (uint8_t)(u & 0xFF);
}

int32_t readI32BE(const uint8_t *buffer) {
  const uint32_t u = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) |
                      ((uint32_t)buffer[2] << 8) | (uint32_t)buffer[3];
  return (int32_t)u;
}

// 16 x 11-bit channels packed LSB-first into 22 bytes (crsf_channels_s
// layout). Mirrors CRSFOutput::packChannels, plus the missing unpack side.
void packChannels11(const uint16_t *channels, uint8_t *payload) {
  for (size_t i = 0; i < 22; ++i) {
    payload[i] = 0;
  }

  uint32_t bitOffset = 0;
  for (uint8_t ch = 0; ch < 16; ++ch) {
    const uint32_t val = channels[ch] & 0x7FF;
    const uint32_t byteIdx = bitOffset / 8;
    const uint8_t bitIdx = bitOffset % 8;

    payload[byteIdx] |= (uint8_t)(val << bitIdx);
    payload[byteIdx + 1] |= (uint8_t)(val >> (8 - bitIdx));
    if (bitIdx > 5) {
      payload[byteIdx + 2] |= (uint8_t)(val >> (16 - bitIdx));
    }

    bitOffset += 11;
  }
}

void unpackChannels11(const uint8_t *payload, uint16_t *channels) {
  uint32_t bitOffset = 0;
  for (uint8_t ch = 0; ch < 16; ++ch) {
    const uint32_t byteIdx = bitOffset / 8;
    const uint8_t bitIdx = bitOffset % 8;

    uint32_t val = (uint32_t)payload[byteIdx] >> bitIdx;
    val |= (uint32_t)payload[byteIdx + 1] << (8 - bitIdx);
    if (bitIdx > 5) {
      val |= (uint32_t)payload[byteIdx + 2] << (16 - bitIdx);
    }

    channels[ch] = (uint16_t)(val & 0x7FF);
    bitOffset += 11;
  }
}

constexpr uint8_t CRSF_COMMAND_ID_CROSSFIRE = 0x10;
constexpr uint8_t CRSF_SUBCOMMAND_SET_BIND_MODE = 0x01;

} // namespace VCTR::packets::crsf

namespace VCTR::packets::crsf /* CRSFPacket_RcChannelsPacked */ {

size_t CRSFPacket_RcChannelsPacked::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::CRSF);
}

size_t CRSFPacket_RcChannelsPacked::getPacketDataType() const {
  return static_cast<size_t>(CRSFFrameType::RcChannelsPacked);
}

size_t CRSFPacket_RcChannelsPacked::numBytes() const {
  return 2 /*sync+len*/ + 1 /*type*/ + 22 /*payload*/ + 1 /*crc*/;
}

void CRSFPacket_RcChannelsPacked::serialize(uint8_t *buffer) const {
  buffer[0] = CRSF_SYNC_BYTE;
  buffer[1] = 24; // type(1) + payload(22) + crc(1)
  buffer[2] = static_cast<uint8_t>(CRSFFrameType::RcChannelsPacked);
  packChannels11(channels, buffer + 3);
  buffer[25] = crc8_d5(buffer + 2, 23);
}

bool CRSFPacket_RcChannelsPacked::deserialize(const uint8_t *buffer) {
  if (buffer[1] != 24) {
    return false;
  }
  if (buffer[2] != static_cast<uint8_t>(CRSFFrameType::RcChannelsPacked)) {
    return false;
  }
  if (crc8_d5(buffer + 2, 23) != buffer[25]) {
    return false;
  }
  unpackChannels11(buffer + 3, channels);
  return true;
}

} // namespace VCTR::packets::crsf

namespace VCTR::packets::crsf /* CRSFPacket_LinkStatistics */ {

size_t CRSFPacket_LinkStatistics::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::CRSF);
}

size_t CRSFPacket_LinkStatistics::getPacketDataType() const {
  return static_cast<size_t>(CRSFFrameType::LinkStatistics);
}

size_t CRSFPacket_LinkStatistics::numBytes() const {
  return 2 + 1 + 10 + 1;
}

void CRSFPacket_LinkStatistics::serialize(uint8_t *buffer) const {
  buffer[0] = CRSF_SYNC_BYTE;
  buffer[1] = 12; // type(1) + payload(10) + crc(1)
  buffer[2] = static_cast<uint8_t>(CRSFFrameType::LinkStatistics);
  buffer[3] = upRssiAnt1;
  buffer[4] = upRssiAnt2;
  buffer[5] = upLinkQuality;
  buffer[6] = (uint8_t)upSnr;
  buffer[7] = activeAntenna;
  buffer[8] = rfProfile;
  buffer[9] = upRfPower;
  buffer[10] = downRssi;
  buffer[11] = downLinkQuality;
  buffer[12] = (uint8_t)downSnr;
  buffer[13] = crc8_d5(buffer + 2, 11);
}

bool CRSFPacket_LinkStatistics::deserialize(const uint8_t *buffer) {
  if (buffer[1] != 12) {
    return false;
  }
  if (buffer[2] != static_cast<uint8_t>(CRSFFrameType::LinkStatistics)) {
    return false;
  }
  if (crc8_d5(buffer + 2, 11) != buffer[13]) {
    return false;
  }
  upRssiAnt1 = buffer[3];
  upRssiAnt2 = buffer[4];
  upLinkQuality = buffer[5];
  upSnr = (int8_t)buffer[6];
  activeAntenna = buffer[7];
  rfProfile = buffer[8];
  upRfPower = buffer[9];
  downRssi = buffer[10];
  downLinkQuality = buffer[11];
  downSnr = (int8_t)buffer[12];
  return true;
}

} // namespace VCTR::packets::crsf

namespace VCTR::packets::crsf /* CRSFPacket_BatterySensor */ {

size_t CRSFPacket_BatterySensor::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::CRSF);
}

size_t CRSFPacket_BatterySensor::getPacketDataType() const {
  return static_cast<size_t>(CRSFFrameType::BatterySensor);
}

size_t CRSFPacket_BatterySensor::numBytes() const {
  return 2 + 1 + 8 + 1;
}

void CRSFPacket_BatterySensor::serialize(uint8_t *buffer) const {
  buffer[0] = CRSF_SYNC_BYTE;
  buffer[1] = 10; // type(1) + payload(8) + crc(1)
  buffer[2] = static_cast<uint8_t>(CRSFFrameType::BatterySensor);
  writeI16BE(buffer + 3, (int16_t)(voltage * 10.0f));  // LSB = 100 mV
  writeI16BE(buffer + 5, (int16_t)(current * 10.0f));  // LSB = 100 mA
  writeU24BE(buffer + 7, capacityUsedMah & 0xFFFFFFu);
  buffer[10] = remainingPercent;
  buffer[11] = crc8_d5(buffer + 2, 9);
}

bool CRSFPacket_BatterySensor::deserialize(const uint8_t *buffer) {
  if (buffer[1] != 10) {
    return false;
  }
  if (buffer[2] != static_cast<uint8_t>(CRSFFrameType::BatterySensor)) {
    return false;
  }
  if (crc8_d5(buffer + 2, 9) != buffer[11]) {
    return false;
  }
  voltage = (float)readI16BE(buffer + 3) / 10.0f;
  current = (float)readI16BE(buffer + 5) / 10.0f;
  capacityUsedMah = readU24BE(buffer + 7);
  remainingPercent = buffer[10];
  return true;
}

} // namespace VCTR::packets::crsf

namespace VCTR::packets::crsf /* CRSFPacket_GPS */ {

size_t CRSFPacket_GPS::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::CRSF);
}

size_t CRSFPacket_GPS::getPacketDataType() const {
  return static_cast<size_t>(CRSFFrameType::GPS);
}

size_t CRSFPacket_GPS::numBytes() const {
  return 2 + 1 + 15 + 1;
}

void CRSFPacket_GPS::serialize(uint8_t *buffer) const {
  buffer[0] = CRSF_SYNC_BYTE;
  buffer[1] = 17; // type(1) + payload(15) + crc(1)
  buffer[2] = static_cast<uint8_t>(CRSFFrameType::GPS);
  writeI32BE(buffer + 3, (int32_t)(latitude * 1e7));
  writeI32BE(buffer + 7, (int32_t)(longitude * 1e7));
  // Groundspeed LSB = 0.1 km/h. (The published spec text says "/100"; real
  // implementations, e.g. ExpressLRS's crsf_sensor_gps_t, use "/10".)
  writeU16BE(buffer + 11, (uint16_t)(groundspeedKmh * 10.0f));
  writeU16BE(buffer + 13, (uint16_t)(headingDeg * 100.0f));
  writeU16BE(buffer + 15, (uint16_t)(altitudeM + 1000.0f));
  buffer[17] = satellites;
  buffer[18] = crc8_d5(buffer + 2, 16);
}

bool CRSFPacket_GPS::deserialize(const uint8_t *buffer) {
  if (buffer[1] != 17) {
    return false;
  }
  if (buffer[2] != static_cast<uint8_t>(CRSFFrameType::GPS)) {
    return false;
  }
  if (crc8_d5(buffer + 2, 16) != buffer[18]) {
    return false;
  }
  latitude = (double)readI32BE(buffer + 3) / 1e7;
  longitude = (double)readI32BE(buffer + 7) / 1e7;
  groundspeedKmh = (float)readU16BE(buffer + 11) / 10.0f;
  headingDeg = (float)readU16BE(buffer + 13) / 100.0f;
  altitudeM = (float)readU16BE(buffer + 15) - 1000.0f;
  satellites = buffer[17];
  return true;
}

} // namespace VCTR::packets::crsf

namespace VCTR::packets::crsf /* CRSFPacket_CommandRxBind */ {

size_t CRSFPacket_CommandRxBind::getPacketType() const {
  return static_cast<size_t>(VCTR::packets::PacketType::CRSF);
}

size_t CRSFPacket_CommandRxBind::getPacketDataType() const {
  return static_cast<size_t>(CRSFFrameType::Command);
}

size_t CRSFPacket_CommandRxBind::numBytes() const {
  return 2 + 1 + 2 + 2 + 1 + 1; // sync+len, type, dest+orig, cmd+subcmd, cmdCrc, crc
}

void CRSFPacket_CommandRxBind::serialize(uint8_t *buffer) const {
  buffer[0] = destAddress; // sync byte == destination address for extended frames
  buffer[1] = 7; // type+dest+orig+cmd+subcmd+cmdCrc+crc
  buffer[2] = static_cast<uint8_t>(CRSFFrameType::Command);
  buffer[3] = destAddress;
  buffer[4] = origAddress;
  buffer[5] = CRSF_COMMAND_ID_CROSSFIRE;
  buffer[6] = CRSF_SUBCOMMAND_SET_BIND_MODE;
  buffer[7] = crc8_ba(buffer + 2, 5);
  buffer[8] = crc8_d5(buffer + 2, 6);
}

bool CRSFPacket_CommandRxBind::deserialize(const uint8_t *buffer) {
  if (buffer[1] != 7) {
    return false;
  }
  if (buffer[2] != static_cast<uint8_t>(CRSFFrameType::Command)) {
    return false;
  }
  if (buffer[5] != CRSF_COMMAND_ID_CROSSFIRE ||
      buffer[6] != CRSF_SUBCOMMAND_SET_BIND_MODE) {
    return false;
  }
  if (crc8_ba(buffer + 2, 5) != buffer[7]) {
    return false;
  }
  if (crc8_d5(buffer + 2, 6) != buffer[8]) {
    return false;
  }
  destAddress = buffer[3];
  origAddress = buffer[4];
  return true;
}

} // namespace VCTR::packets::crsf
