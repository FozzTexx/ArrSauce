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
  uint64_t syncPattern, syncBuffer;
  unsigned int syncLength, syncPos;

  bool near(uint32_t duration, uint32_t center, float percent);

 protected:
  bool synced;
  unsigned int nybble, nybblePos;
  unsigned int payloadLength;
  uint8_t payloadBuffer[32];
  unsigned int payloadBufferPos;

  int pulse(uint32_t duration, bool level);

 public:
  bool hasPayload;

  RSOSDecoder(uint32_t duration, uint64_t pattern, unsigned int length);
  void printPayloadHex();
  int decodeBCD(unsigned int pos, unsigned int places, bool negative);
  unsigned int decodeUnsigned(unsigned int pos, unsigned int width);

  virtual void reset();
  virtual int received(uint32_t duration, bool level);
  virtual unsigned int calculateChecksum();
  virtual sensor_data decodePayload() = 0;
};

class RSOSv1 : public RSOSDecoder
{
 public:
  RSOSv1();
  unsigned int calculateChecksum() override;
  sensor_data decodePayload() override;
};

class RSOSv2 : public RSOSDecoder
{
 private:
  unsigned int payloadDup;

 public:
  RSOSv2();
  void reset() override;
  int received(uint32_t duration, bool level) override;
  sensor_data decodePayload();
};

class RSOSv3 : public RSOSDecoder
{
 public:
  RSOSv3();
  sensor_data decodePayload() override;
};

#endif /* DECODER_H */
