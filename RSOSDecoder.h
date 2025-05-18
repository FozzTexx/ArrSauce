// -*- mode: C; indent-tabs-mode: nil -*-

/* Created 2025 May 17 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Handles decoding Radio Shack/Oregon Scientific sensor signal
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#ifndef DECODER_H
#define DECODER_H

#include "mqtt.h"
#include <stdint.h>

enum {
  PULSE_INVALID = 1,
  PULSE_HALF,
  PULSE_FULL,
};

enum {
  WAIT_CLOCK = 1,
  WAIT_HALF,
};

class RSOSDecoder
{
 private:
  uint32_t fullBit, halfBit;
  unsigned int state;
  bool synced;
  uint64_t syncPattern, syncBuffer;
  unsigned int syncLength, syncPos;
  unsigned int nybble, nybblePos;
  unsigned int payloadLength;
  uint8_t payloadBuffer[32];
  unsigned int payloadBufferPos;

  bool near(uint32_t duration, uint32_t center, float percent);

 protected:
  int pulse(uint32_t duration, bool level);

 public:
  bool hasPayload;

  RSOSDecoder(uint32_t duration, uint64_t pattern, unsigned int length);
  void reset();
  int received(uint32_t duration, bool level);
  unsigned int calculateChecksum();
  void printPayloadHex();
  int decodeBCD(unsigned int pos, unsigned int places, bool negative);
  unsigned int decodeUnsigned(unsigned int pos, unsigned int width);
  sensor_data decodePayload();
};

#endif /* DECODER_H */
