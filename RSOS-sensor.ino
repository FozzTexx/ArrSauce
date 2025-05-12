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

#include <DHTStable.h>

#define RCV_PIN   16
#define RINGLEN   256
#define DELTA_MAX 300
#define GAP       8000
#define MAXBITS   128
#define DHT_PIN   21

DHTStable DHT;

volatile int sync_idx = -1;
volatile byte sync_type;
volatile byte received = 0;
unsigned long ringbuf[RINGLEN];
unsigned int ringpos = 0;
unsigned int decode_buf[MAXBITS / 8 / sizeof(unsigned int)];

const unsigned int bitlen_v1[] = {1585, 3050, 1350, 2810};
const unsigned int bitlen_v2[] = {396, 884, 581, 1069};

const unsigned int sync_v1[] = {1200, 1700, 1200, 1700, 4200, 5700};
const unsigned int sync_v2[] = {bitlen_v2[1], bitlen_v2[3], bitlen_v2[1], bitlen_v2[3],
                                bitlen_v2[1], bitlen_v2[3], bitlen_v2[1], bitlen_v2[3],
                                bitlen_v2[1], bitlen_v2[3], bitlen_v2[1], bitlen_v2[3],
                                bitlen_v2[1], bitlen_v2[3], bitlen_v2[1], bitlen_v2[2],
                                bitlen_v2[0]};
#define array_size(x)     (sizeof(x) / sizeof(x[0]))

typedef struct {
  byte channel, flags, battery_low;
  int celsius, humidity;
  unsigned int rolling_code, checksum, device_id;
  unsigned int calculated_checksum;
} sensor_data;

boolean found_sync(const unsigned int *pattern, unsigned int slen, unsigned int delta_max)
{
  unsigned int chkpos;
  unsigned int idx;
  long dur;


  chkpos = (ringpos + RINGLEN - slen) % RINGLEN;
  for (idx = 0; idx < slen; idx++) {
    dur = ringbuf[(chkpos + idx) % RINGLEN];
    dur -= pattern[idx];
    dur = abs(dur);
    if (dur > delta_max)
      break;
  }

  return idx == slen;
}

void sig_rx()
{
  static unsigned long last = 0;
  static boolean merge_next = false;
  unsigned long now;
  long dur;
  unsigned int prevpos;


  if (!received) {
    now = micros();
    dur = now - last;
    if (merge_next || dur < 200) {
      prevpos = (ringpos + RINGLEN - 1) % RINGLEN;
      ringbuf[prevpos] += dur;
      merge_next = !merge_next;
    }
    else {
      ringbuf[ringpos] = dur;
      ringpos = (ringpos + 1) % RINGLEN;
      merge_next = false;
    }

    if (sync_idx >= 0 && ringpos == sync_idx)
      sync_idx = -1;

    if (found_sync(sync_v1, array_size(sync_v1), DELTA_MAX / 2)) {
      sync_idx = ringpos;
      sync_type = 1;
    }
    else if (found_sync(sync_v2, array_size(sync_v2), DELTA_MAX / 2)) {
      sync_idx = ringpos;
      sync_type = 2;
    }

    if (dur > GAP && sync_idx >= 0)
      received = 1;
  }

  last = now;

  return;
}

void setup()
{
  Serial.begin(115200);
  pinMode(RCV_PIN, INPUT);

  while (!Serial)
    ;

  Serial.println("Started.");

  setup_mqtt();

  // WiFi enabled causes so much interference that sensor signal can't be received
  disable_wifi();

  // Must attach interrupt after WiFi is connected
  attachInterrupt(digitalPinToInterrupt(RCV_PIN), sig_rx, CHANGE);

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

unsigned int decode(unsigned int start, boolean inverse_double, const unsigned int *rflen)
{
  unsigned int idx, count, bit_group, bit_idx;
  long dur, delta_s, delta_l;
  boolean skip, bv, rf;
  byte group;


  for (idx = 0; idx < array_size(decode_buf); idx++)
    decode_buf[idx] = 0;

  for (idx = skip = count = 0; idx < RINGLEN; idx++) {
    dur = ringbuf[(start + idx) % RINGLEN];
    if (dur > GAP)
      break;
    bv = 1 - idx & 1;
    group = bv;
    group ^= 1 - inverse_double;
    group *= 2;
    delta_s = rflen[group];
    delta_s -= dur;
    delta_s = abs(delta_s);
    delta_l = rflen[group + 1];
    delta_l -= dur;
    delta_l = abs(delta_l);
    if (delta_s > DELTA_MAX && delta_l > DELTA_MAX) {
      break;
    }

    if (delta_l < DELTA_MAX || !skip) {
      if (!inverse_double || (count & 1) == 0) {
        bit_idx = count;
        if (inverse_double)
          bit_idx /= 2;
        bit_group = bit_idx / (sizeof(decode_buf[0]) * 8);
        bit_idx %= sizeof(decode_buf[0]) * 8;
        decode_buf[bit_group] |= bv << bit_idx;
      }
      skip = 1;
      count++;
    }
    else {
      skip = 0;
    }
  }

  ring_dump(start, idx);

  return count;
}

void shift_bits(int distance)
{
  unsigned int idx, mask, bits, pad;


  if (!distance)
    return;
  if (distance < 0) {
    // right shift

    distance = abs(distance);
    mask = (1 << distance) - 1;
    pad = sizeof(decode_buf[idx]) * 8 - distance;

    for (idx = 0; idx < array_size(decode_buf); idx++) {
      decode_buf[idx] >>= distance;
      if (idx < array_size(decode_buf) - 1) {
        bits = decode_buf[idx + 1] & mask;
        bits <<= pad;
        decode_buf[idx] |= bits;
      }
    }
  }
  else {
    // left shift

    mask = (1 << distance) - 1;
    pad = sizeof(decode_buf[idx]) * 8 - distance;
    mask <<= pad;

    for (idx = array_size(decode_buf); idx > 0; idx--) {
      decode_buf[idx - 1] <<= distance;
      if (idx > 1) {
        bits = decode_buf[idx - 2] & mask;
        bits >>= pad;
        decode_buf[idx - 1] |= bits;
      }
    }
  }

  return;
}

void print_bits()
{
  unsigned int idx;


  Serial.print("0x");
  for (idx = 0; idx < array_size(decode_buf); idx++)
    Serial.print(decode_buf[array_size(decode_buf) - idx - 1], HEX);

  return;
}

void publish_data(sensor_data *data)
{
  detachInterrupt(digitalPinToInterrupt(RCV_PIN));
  enable_wifi();
  publishTemperature(data);
  disable_wifi();
  attachInterrupt(digitalPinToInterrupt(RCV_PIN), sig_rx, CHANGE);
  return;
}

void get_dht()
{
  float hum, cels;
  int chk;
  sensor_data data;


  chk = DHT.read22(DHT_PIN);
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

void loop()
{
  static int count = 0;
  static unsigned long last = 0;
  unsigned long now, delta;
  uint64_t val;


  //mqtt_loop();

  now = millis();
  delta = now - last;
#if 0
  if (delta > 30000) {
    get_dht();
    last = now;
  }
#endif

  if (received) {
    Serial.print("Count: ");
    Serial.print(count);
    Serial.print(" ");
    Serial.print(now);
    Serial.print(" ");
    Serial.print(sync_type);
    Serial.println();

    if (sync_type == 1) {
      decode(sync_idx + 1, false, bitlen_v1);
      if (ringbuf[sync_idx] > 5300)
        shift_bits(1);
    }
    else if (sync_type == 2) {
      int idx;


      idx = sync_idx - array_size(sync_v2);
      decode(sync_idx - 2, true, bitlen_v2);
      shift_bits(-4);
    }

    val = decode_buf[3];
    val <<= sizeof(decode_buf[2]) * 8;
    val |= decode_buf[2];
    val <<= sizeof(decode_buf[1]) * 8;
    val |= decode_buf[1];
    val <<= sizeof(decode_buf[0]) * 8;
    val |= decode_buf[0];

    print_bits();
    Serial.println();

    {
      byte celsius_l, celsius_m, celsius_h;
      unsigned int checksum_calc = 0xAA55;
      int neg;
      int fahrenheit;
      int idx;
      sensor_data tdata;


      neg = 1;
      if (sync_type == 2) {
        tdata.device_id = val & 0xffff;
        for (idx = (val >> 16) & 0x0F, tdata.channel = 0; idx; tdata.channel++, idx >>= 1)
          ;
        if (tdata.channel)
          tdata.channel--;

        tdata.rolling_code = (val >> 20) & 0xff;
        tdata.flags = (val >> 28) & 0x0F;

        if (tdata.device_id == 0x04ce) {
          celsius_l = (val >> 32) & 0x0F;
          celsius_m = (val >> 36) & 0x0F;
          celsius_h = (val >> 40) & 0x0F;
          if ((val >> 44) & 0x0F)
            neg = -1;

          tdata.battery_low = (tdata.flags & 0x4) >> 2;

          tdata.checksum = (val >> 48) & 0xff;
          for (idx = checksum_calc = 0; idx < 48; idx += 4)
            checksum_calc += (val >> idx) & 0x0F;
          checksum_calc &= 0xff;
        }
      }
      else {
        tdata.device_id = 0;
        tdata.rolling_code = val & 0x0F;

        tdata.channel = (val >> 6) & 0x3;

        tdata.flags = (val >> 4) & 0x3;
        tdata.flags <<= 4;
        tdata.flags |= (val >> 20) & 0x0F;

        celsius_l = (val >> 8) & 0x0F;
        celsius_m = (val >> 12) & 0x0F;
        celsius_h = (val >> 16) & 0x0F;
        if (tdata.flags & 0x2)
          neg = -1;

        tdata.battery_low = (tdata.flags & 0x8) >> 3;

        tdata.checksum = (val >> 24) & 0xff;
        checksum_calc = (val & 0xFF) + ((val >> 8) & 0xFF) + ((val >> 16) & 0xFF);
        checksum_calc += 0xE2;
        checksum_calc &= 0xFF;
      }

      tdata.calculated_checksum = checksum_calc;
      tdata.celsius = celsius_h * 100 + celsius_m * 10 + celsius_l;
      tdata.celsius *= neg;
      fahrenheit = (tdata.celsius * 9 + 1602) / 5;

      Serial.print("Temp: ");
      Serial.print(tdata.celsius / 10);
      Serial.print(".");
      Serial.print(abs(tdata.celsius % 10));
      Serial.print("C ");
      Serial.print(fahrenheit / 10);
      Serial.print(".");
      Serial.print(abs(fahrenheit % 10));
      Serial.print("F  Channel: ");
      Serial.print(tdata.channel + 1);
      Serial.print("  Device: ");
      Serial.print(tdata.device_id, HEX);
      Serial.print("  Battery: ");
      Serial.print(tdata.battery_low ? "low" : "good");
      Serial.print("  Flags: 0b");
      Serial.print(tdata.flags, BIN);
      Serial.print("  RC: 0x");
      Serial.print(tdata.rolling_code, HEX);
      Serial.print("  Checksum: 0x");
      Serial.print(tdata.checksum, HEX);
      Serial.print(" 0x");
      Serial.print(checksum_calc, HEX);
      Serial.println();
      Serial.println();

      if (1 || tdata.checksum == checksum_calc) {
        publish_data(&tdata);
      }
    }

    sync_idx = -1;
    received = 0;
    count++;
  }

  return;
}
