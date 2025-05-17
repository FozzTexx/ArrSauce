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
  data = 0;
  bitPos = 0;
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
    data <<= 1;
    data |= bit;
    mask = (1 << syncLength) - 1;

#if 0
    Serial.print(duration);
    Serial.print(" ");
    Serial.println(fullBit);
#endif

    if ((data & mask) == syncPattern) {
#if 1
      Serial.print("Sync length: 0x");
      Serial.print(mask, HEX);
      Serial.print(" ");
      Serial.print(syncPattern, HEX);
      Serial.print(" ");
      Serial.print(data & mask, HEX);
      Serial.print(" ");
      Serial.println(syncLength);
#endif

#if 0
      Serial.print("Found sync: 0x");
      Serial.print((unsigned long long) bits, HEX);
#endif

      data = 0;
      bitPos = 0;
      synced = true;
    }
  }
  else if (!hasPayload) {
#if 0
    Serial.print("Data length: 0x");
    Serial.print((unsigned long long) bits, HEX);
    Serial.print(" ");
    Serial.print(payloadLength);
    Serial.print(" ");
    Serial.println(bitPos);
#endif
    data |= ((uint64_t) bit) << bitPos;
    bitPos++;
    if (bitPos == payloadLength) {
      hasPayload = true;
    }
  }

  return bit;
}

unsigned int RSOSDecoder::calculateChecksum()
{
  unsigned int checksum, idx;


  for (checksum = 0, idx = payloadLength - 8; idx; idx -= 4)
    checksum += (data >> idx) & 0x0f;
  return checksum & 0xff;
}

int RSOSDecoder::decodeBCD(unsigned int val, unsigned int places, bool negative)
{
  int bcd;
  unsigned int idx, digit;


  for (bcd = 0, idx = places; idx; idx--) {
    digit = (val >> (idx - 1) * 4) & 0x0f;
    // FIXME - check if digit is 0-9?
    bcd *= 10;
    bcd += digit;
  }
  if (negative)
    bcd = -bcd;

  return bcd;
}
