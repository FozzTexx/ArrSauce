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
    mask = (1 << syncLength) - 1;
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

sensor_data RSOSDecoder::decodePayload()
{
  sensor_data decoded;


  decoded.channel = payloadBuffer[4];
  decoded.flags = (payloadBuffer[14] << 4) | payloadBuffer[7];
  decoded.battery_low = !(decoded.flags & 0x04);
  decoded.celsius = decodeBCD(8, 3, payloadBuffer[11]);
  decoded.humidity = decodeBCD(12, 2, 0);
  decoded.rolling_code = decodeUnsigned(5, 2);
  decoded.checksum = decodeUnsigned(payloadLength - 2, 2);
  decoded.calculated_checksum = calculateChecksum();
  decoded.device_id = decodeUnsigned(0, 4) & 0xFFCF;
  decoded.version = 3;

  return decoded;
}
