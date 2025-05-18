// -*- mode: C; indent-tabs-mode: nil -*-

/* Created 2025 May 17 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Handles decoding Radio Shack/Oregon Scientific sensor signal
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#ifndef MQTT_H
#define MQTT_H

typedef struct {
  unsigned int channel, flags, battery_low;
  int celsius, humidity;
  unsigned int rolling_code, checksum, device_id;
  unsigned int calculated_checksum;
} sensor_data;

extern void setup_wifi();
extern void setup_mqtt();
extern void publishTemperature(sensor_data *tdata);
extern void enable_wifi();
extern void disable_wifi();

#endif /* MQTT_H */
