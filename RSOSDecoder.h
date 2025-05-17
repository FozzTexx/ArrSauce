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
  uint64_t syncPattern;
  unsigned int syncLength;
  unsigned int bitPos;
  unsigned int payloadLength;

  bool near(uint32_t duration, uint32_t center, float percent);

 protected:
  int pulse(uint32_t duration, bool level);

 public:
  uint64_t data;
  bool hasPayload;

  RSOSDecoder(uint32_t duration, uint64_t pattern, unsigned int length);
  void reset();
  int received(uint32_t duration, bool level);
  unsigned int calculateChecksum();

  int decodeBCD(unsigned int val, unsigned int places, bool negative);
};

#endif /* DECODER_H */
