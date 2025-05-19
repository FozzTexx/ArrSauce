// -*- mode: C; indent-tabs-mode: nil -*-

/* Created 2021 Jun 26 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Receives and decodes the temperature sensor data sent from a Radio
 * Shack/Oregon Scientific temperature probe over 433MHz.
 *
 * Reference: https://www.osengr.org/WxShield/Downloads/Weather-Sensor-RF-Protocols.pdf
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

/* ---------------------------------------------------------------------
 * User Configuration
 *
 * To get started:
 * 1. Copy the file 'config.h.template' to 'config.h'
 * 2. Open 'config.h' and fill in your WiFi credentials and other settings
 *
 * Note: 'config.h' is listed in '.gitignore', so it won't be tracked by Git.
 * ---------------------------------------------------------------------
 */
#include "config.h"


#include "RSOSDecoder.h"
#include "mqtt.h"
#include <DHTStable.h>

#define RINGLEN   256

DHTStable DHT;

uint32_t ringbuf[RINGLEN];
unsigned int ringpos = 0, ringlast = 0;

RSOSDecoder *decoders[] = {new RSOSv1(), new RSOSv2(), new RSOSv3()};
#define array_len(x)     (sizeof(x) / sizeof(x[0]))

void sig_rx()
{
  static uint32_t last = 0;
  static boolean merge_next = false;
  uint32_t now, dur;
  unsigned int prevpos;


  now = micros();
  dur = now - last;
  if (merge_next || dur < 200) {
    prevpos = (ringpos + RINGLEN - 1) % RINGLEN;
    ringbuf[prevpos] += dur << 1;
    merge_next = !merge_next;
  }
  else {
    dur <<= 1;
    dur |= digitalRead(PIN_433);
    ringbuf[ringpos] = dur;
    ringpos = (ringpos + 1) % RINGLEN;
    merge_next = false;
  }

  last = now;

  return;
}

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_433, INPUT);

  while (!Serial)
    ;

  Serial.println("Started.");

#ifdef WIFI_SSID
  setup_mqtt();

  // WiFi enabled causes so much interference that sensor signal can't be received
  disable_wifi();
#endif /* WIFI_SSID */

  // Must attach interrupt after WiFi is connected
  attachInterrupt(digitalPinToInterrupt(PIN_433), sig_rx, CHANGE);

  return;
}

void ring_dump(int idx, int len)
{
  for (; len > 0; len--, idx++) {
    Serial.print(idx);
    Serial.print(":");
    Serial.print(ringbuf[idx % RINGLEN]);
    Serial.print(" ");
  }

  Serial.println();
  return;
}

#ifdef WIFI_SSID
void publish_data(sensor_data *data)
{
  detachInterrupt(digitalPinToInterrupt(PIN_433));
  enable_wifi();
  publishTemperature(data);
  disable_wifi();
  attachInterrupt(digitalPinToInterrupt(PIN_433), sig_rx, CHANGE);
  return;
}
#endif /* WIFI_SSID */

#ifdef PIN_DHT22
void get_dht()
{
  float hum, cels;
  int chk;
  sensor_data data;


  chk = DHT.read22(PIN_DHT22);
  if (chk == DHTLIB_OK) {
    hum = DHT.getHumidity();
    cels = DHT.getTemperature();
    Serial.print(F("Humidity: "));
    Serial.print(hum);
    Serial.print(F("%  Temperature: "));
    Serial.print(cels);
    Serial.println();

    memset(&data, 0, sizeof(data));
    data.channel = 0xF;
    data.celsius = cels * 10;
    data.humidity = hum * 10;
    publish_data(&data);
  }
  else {
    Serial.println("Failed to read DHT22");
  }

  return;
}
#endif /* PIN_DHT22 */

void loop()
{
  static int count = 0;
  static uint32_t last = 0;
  uint32_t now, delta;
  uint64_t val;
  unsigned int idx;
  RSOSDecoder *decoder;


  //mqtt_loop();

  now = millis();
  delta = now - last;
#ifdef PIN_DHT22
  if (delta > 30000) {
    get_dht();
    last = now;
  }
#endif /* PIN_DHT22 */

  while (ringlast != ringpos) {
    for (idx = 0; idx < array_len(decoders); idx++) {
      decoder = decoders[idx];
      decoder->received(ringbuf[ringlast] >> 1, ringbuf[ringlast] & 1);
      if (decoder->hasPayload) {
        Serial.print("Payload: 0x");
        decoder->printPayloadHex();
        Serial.println();

        Serial.print("Checksum: 0x");
        Serial.print(decoder->calculateChecksum(), HEX);
        Serial.println();

#ifdef WIFI_SSID
        {
          sensor_data tdata = decoder->decodePayload();
          printSensorData(&tdata);
          publish_data(&tdata);
        }
#endif /* WIFI_SSID */

        decoder->reset();
      }
    }
    ringlast = (ringlast + 1) % RINGLEN;
  }

  return;
}
