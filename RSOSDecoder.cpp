// -*- mode: C; indent-tabs-mode: nil -*-

/* Created 2025 May 17 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Handles decoding Radio Shack/Oregon Scientific sensor signal
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#include "RSOSDecoder.h"
#include <Arduino.h>

int bit_length(uint64_t val)
{
  return val == 0 ? 0 : 64 - __builtin_clzll(val);
}

RSOSDecoder::RSOSDecoder(uint32_t duration, uint64_t pattern, unsigned int length)
{
  fullBit = duration;
  halfBit = fullBit / 2;
  state = WAIT_CLOCK;
  payloadLength = length;
  syncPattern = pattern;
  syncLength = bit_length(syncPattern);
  reset();
  return;
}

bool RSOSDecoder::near(uint32_t duration, uint32_t center, float percent)
{
  uint32_t margin = center * percent;


  if (center - margin < duration && duration < center + margin)
    return true;
  return false;
}

void RSOSDecoder::reset()
{
  synced = hasPayload = false;
  syncBuffer = 0;
  payloadBufferPos = 0;
  nybblePos = nybble = 0;
  return;
}

int RSOSDecoder::pulse(uint32_t duration, bool level)
{
  unsigned int pulse;


  pulse = PULSE_INVALID;
  if (near(duration, halfBit, 0.5))
    pulse = PULSE_HALF;
  else if (near(duration, fullBit, 0.25))
    pulse = PULSE_FULL;

  if (pulse == PULSE_INVALID) {
    state = WAIT_CLOCK;
    return -1;
  }

  if (state == WAIT_CLOCK) {
    state = WAIT_HALF;
    return 1 - level;
  }

  if (pulse == PULSE_HALF) {
    state = WAIT_CLOCK;
    return -1;
  }

  return 1 - level;
}

int RSOSDecoder::received(uint32_t duration, bool level)
{
  int bit;
  uint64_t mask;


  bit = pulse(duration, level);
  if (bit < 0)
    return bit;

  if (!synced) {
    syncBuffer <<= 1;
    syncBuffer |= bit;
    mask = 1;
    mask <<= syncLength;
    mask -= 1;
    if ((syncBuffer & mask) == syncPattern) {
      payloadBufferPos = 0;
      nybblePos = nybble = 0;
      synced = true;
    }
  }
  else if (!hasPayload) {
    nybble |= bit << nybblePos;
    nybblePos++;
    if (nybblePos == 4) {
      payloadBuffer[payloadBufferPos] = nybble;
      payloadBufferPos++;
      nybblePos = nybble = 0;
    }
    if (payloadBufferPos == payloadLength)
      hasPayload = true;
  }

  return bit;
}

unsigned int RSOSDecoder::calculateChecksum()
{
  unsigned int checksum, idx;


  for (checksum = idx = 0; idx < payloadLength - 2; idx++)
    checksum += payloadBuffer[idx];
  return checksum & 0xff;
}

void RSOSDecoder::printPayloadHex()
{
  unsigned int idx;


  for (idx = payloadBufferPos; idx; idx--)
    Serial.print(payloadBuffer[idx - 1], HEX);
  return;
}

int RSOSDecoder::decodeBCD(unsigned int pos, unsigned int places, bool negative)
{
  int bcd;
  unsigned int idx, digit;


  for (bcd = 0, idx = places; idx; idx--) {
    digit = payloadBuffer[pos + idx - 1];
    // FIXME - check if digit is 0-9?
    bcd *= 10;
    bcd += digit;
  }
  if (negative)
    bcd = -bcd;

  return bcd;
}

unsigned int RSOSDecoder::decodeUnsigned(unsigned int pos, unsigned int width)
{
  unsigned int val, idx;


  for (val = 0, idx = width; idx; idx--) {
    val <<= 4;
    val |= payloadBuffer[pos + idx - 1];
  }

  return val;
}

RSOSv1::RSOSv1(): RSOSDecoder(1000000 / 342, 0xFFF, 8)
{
  return;
}

unsigned int RSOSv1::calculateChecksum()
{
  unsigned int checksum, idx;


  for (idx = 0; idx < payloadLength - 2; idx += 2)
    checksum += decodeUnsigned(idx, 2);
  return checksum & 0xff;
}

sensor_data RSOSv1::decodePayload()
{
  sensor_data decoded;


  decoded.channel = payloadBuffer[1] >> 2;
  decoded.flags = ((payloadBuffer[1] & 0x03) << 4) | payloadBuffer[5];
  decoded.battery_low = !!(decoded.flags & 0x08);
  decoded.celsius = decodeBCD(2, 3, decoded.flags & 0x2);
  decoded.humidity = 0;
  decoded.rolling_code = payloadBuffer[0];
  decoded.checksum = decodeUnsigned(payloadLength - 2, 2);
  decoded.calculated_checksum = calculateChecksum();
  decoded.device_id = 0;
  decoded.version = 1;

  return decoded;
}

RSOSv2::RSOSv2(): RSOSDecoder(1000000 / 1024, 0x2AAAAAAAB33, 14)
{
  return;
}

void RSOSv2::reset()
{
  RSOSDecoder::reset();
  return;
}

#define COMPACT_EVEN_BITS(x) ( \
                               (((x) >> 0) & 0x01) |    \
                               (((x) >> 1) & 0x02) |    \
                               (((x) >> 2) & 0x04) |    \
                               (((x) >> 3) & 0x08) )

int RSOSv2::received(uint32_t duration, bool level)
{
  int bit;
  unsigned int idx;
  uint32_t val;


  bit = RSOSDecoder::received(duration, level);
  /* The v2 sensors send every bit twice, first normal and the second
       copy is inverted. Read in twice as many nibbles and then
       de-interlace the bits we want. */
  if (synced) {
    hasPayload = payloadBufferPos == payloadLength * 2;
    if (hasPayload) {
      for (idx = 0; idx < payloadLength; idx++)
        payloadBuffer[idx] = COMPACT_EVEN_BITS(decodeUnsigned(idx * 2, 2));
      payloadBufferPos = payloadLength;
    }
  }

  return bit;
}

sensor_data RSOSv2::decodePayload()
{
  sensor_data decoded;
  unsigned int val;


  decoded.channel = bit_length(payloadBuffer[4]);
  if (decoded.channel)
    decoded.channel--;
  decoded.flags = payloadBuffer[7];
  decoded.battery_low = !!(decoded.flags & 0x04);
  decoded.celsius = decodeBCD(8, 3, payloadBuffer[11]);
  decoded.humidity = 0;
  decoded.rolling_code = decodeUnsigned(5, 2);
  decoded.checksum = decodeUnsigned(payloadLength - 2, 2);
  decoded.calculated_checksum = calculateChecksum();
  decoded.device_id = decodeUnsigned(0, 4) & 0xFFCF;
  decoded.version = 2;

  return decoded;
}

RSOSv3::RSOSv3(): RSOSDecoder(1000000 / 1024, 0xFFFFFF5, 17)
{
  return;
}

sensor_data RSOSv3::decodePayload()
{
  sensor_data decoded;


  decoded.channel = payloadBuffer[4];
  decoded.flags = (payloadBuffer[14] << 4) | payloadBuffer[7];
  decoded.battery_low = !(decoded.flags & 0x04);
  decoded.celsius = decodeBCD(8, 3, payloadBuffer[11]);
  decoded.humidity = decodeBCD(12, 2, 0) * 10;
  decoded.rolling_code = decodeUnsigned(5, 2);
  decoded.checksum = decodeUnsigned(payloadLength - 2, 2);
  decoded.calculated_checksum = calculateChecksum();
  decoded.device_id = decodeUnsigned(0, 4) & 0xFFCF;
  decoded.version = 3;

  return decoded;
}
