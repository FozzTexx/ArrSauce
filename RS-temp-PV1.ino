// -*- C -*-

// Reference: https://www.osengr.org/WxShield/Downloads/Weather-Sensor-RF-Protocols.pdf

#define RCV_PIN   7
#define RINGLEN   256
#define DELTA_MAX 300
#define GAP       8000
#define MAXBITS   128

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
  Serial.begin(9600);
  pinMode(RCV_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RCV_PIN), sig_rx, CHANGE);

  while (!Serial)
    ;
  Serial.println("Started.");
  
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

unsigned int decode(unsigned int start, boolean half, const unsigned int *rflen)
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
    group ^= 1 - half;
    group *= 2;
    delta_s = rflen[group];
    delta_s -= dur;
    delta_s = abs(delta_s);
    delta_l = rflen[group + 1];
    delta_l -= dur;
    delta_l = abs(delta_l);
    if (delta_s > DELTA_MAX && delta_l > DELTA_MAX) {
#if 0
      Serial.print("BAD DUR ");
      Serial.print(dur);
      Serial.print("  idx: ");
      Serial.print(idx);
      Serial.print("  half: ");
      Serial.print(half);
      Serial.print("  group: ");
      Serial.print(group);
      Serial.print("  bv: ");
      Serial.print(bv);
      Serial.print("  delta: ");
      Serial.print(delta_s);
      Serial.print(" ");
      Serial.print(delta_l);
      Serial.println();
#endif
      break;
    }

#if 0
    Serial.print(delta_l < DELTA_MAX ? 'L' : 'S');
#endif
    if (delta_l < DELTA_MAX || !skip) {
#if 0
      byte chk;
      Serial.print(dur);
      Serial.print(delta_l < DELTA_MAX ? 'L' : 'S');
      Serial.print(bv);
      Serial.print(" ");
      chk = !half || (count & 1) == 0;
      Serial.print(chk);
      Serial.print(" ");
#endif
      if (!half || (count & 1) == 0) {
        bit_idx = count;
        if (half)
          bit_idx /= 2;
#if 0
        Serial.print(" C:");
        Serial.print(bit_idx);
#endif
        bit_group = bit_idx / (sizeof(decode_buf[0]) * 8);
        bit_idx %= sizeof(decode_buf[0]) * 8;
#if 0
        Serial.print("_G:");
        Serial.print(bit_group);
        Serial.print("_I:");
        Serial.print(bit_idx);
        Serial.print(" ");
#endif
        decode_buf[bit_group] |= bv << bit_idx;
#if 0
        Serial.print(decode_buf[bit_group], HEX);
        Serial.print(" ");
        #if 0
        Serial.print((unsigned long) val, HEX);
        Serial.print("+");
        #endif
        Serial.print(bv);
#endif
      }
#if 0
      else
        Serial.print(" ");
#endif
      skip = 1;
      count++;
    }
    else {
      skip = 0;
#if 0
      Serial.print(" ");
#endif
    }
  }

#if 0
  Serial.println();
  Serial.print("Bit count: ");
  Serial.print(count);
  Serial.print(" ");
  print_bits();
  Serial.println();
#endif

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

void loop() 
{
  static int count = 0;
  uint64_t val;

  
  if (received) {
    Serial.print("Count: ");
    Serial.print(count);
    Serial.print(" ");
    Serial.print(millis());
    Serial.println();

#if 0
    Serial.print("Sync start: ");
    Serial.print(ringbuf[sync_idx]);
    Serial.println();
#endif
    
    if (sync_type == 1) {
      decode(sync_idx + 1, false, bitlen_v1);
      if (ringbuf[sync_idx] > 5300)
        shift_bits(1);
    }
    else if (sync_type == 2) {
      int idx;
      idx = sync_idx - array_size(sync_v2);
#if 0
      ring_dump((idx + RINGLEN) % RINGLEN, ringpos - idx);
#endif
      decode(sync_idx - 2, true, bitlen_v2);
      shift_bits(-4);
    }

#if 1
    print_bits();
    Serial.println();
#endif
    
    val = decode_buf[3];
    val <<= sizeof(decode_buf[2]) * 8;
    val |= decode_buf[2];
    val <<= sizeof(decode_buf[1]) * 8;
    val |= decode_buf[1];
    val <<= sizeof(decode_buf[0]) * 8;
    val |= decode_buf[0];

#if 0
    Serial.print("0x");
    Serial.print((unsigned long) (val >> 32), HEX);
    Serial.print((unsigned long) val, HEX);
    Serial.print(" ");
    Serial.print(sync_type);
    Serial.println();
#endif

    {
      byte channel;
      byte celsius_l, celsius_m, celsius_h;
      byte flags;
      byte battery;
      unsigned int rolling_code, checksum, checksum_calc;
      int celsius, neg;
      int fahrenheit;
      unsigned int device_id;
      int idx;


      neg = 1;
      if (sync_type == 2) {
        device_id = val &0xffff;
        for (idx = (val >> 16) & 0xf, channel = 0; idx; channel++, idx >>= 1)
          ;
        if (channel)
          channel--;
       
        rolling_code = (val >> 20) & 0xff;
        flags = (val >> 28) & 0xf;

        if (device_id == 0x04ce) {
          celsius_l = (val >> 32) & 0xf;
          celsius_m = (val >> 36) & 0xf;
          celsius_h = (val >> 40) & 0xf;
          if ((val >> 44) & 0xf)
            neg = -1;

          battery = (flags & 0x4) >> 2;

          checksum = (val >> 48) & 0xff;
          for (idx = checksum_calc = 0; idx < 48; idx += 4)
            checksum_calc += (val >> idx) & 0xf;
          checksum_calc &= 0xff;
        }
      }
      else {
        device_id = 0;
        rolling_code = val & 0xf;
        
        channel = (val >> 6) & 0x3;
        
        flags = (val >> 4) & 0x3;
        flags <<= 4;
        flags |= (val >> 20) & 0xf;
        
        celsius_l = (val >> 8) & 0xf;
        celsius_m = (val >> 12) & 0xf;
        celsius_h = (val >> 16) & 0xf;
        if (flags & 0x2)
          neg = -1;
        
        battery = (flags & 0x8) >> 3;

        checksum = (val >> 24) & 0xff;
        checksum_calc = (val & 0xFF) + ((val >> 8) & 0xff) + ((val >> 16) & 0xff);
        checksum_calc &= 0xff;
      }

      celsius = celsius_h * 100 + celsius_m * 10 + celsius_l;
      celsius *= neg;
      fahrenheit = (celsius * 9 + 1602) / 5;
      
      Serial.print("Temp: ");
      Serial.print(celsius / 10);
      Serial.print(".");
      Serial.print(abs(celsius % 10));
      Serial.print("C ");
      Serial.print(fahrenheit / 10);
      Serial.print(".");
      Serial.print(abs(fahrenheit % 10));
      Serial.print("F  Channel: ");
      Serial.print(channel + 1);
      Serial.print("  Device: ");
      Serial.print(device_id, HEX);
      Serial.print("  Battery: ");
      Serial.print(battery ? "low" : "good");
      Serial.print("  Flags: 0b");
      Serial.print(flags, BIN);
      Serial.print("  RC: 0x");
      Serial.print(rolling_code, HEX);
      Serial.print("  Checksum: 0x");
      Serial.print(checksum, HEX);
      Serial.print(" 0x");
      Serial.print(checksum_calc, HEX);
      Serial.println();
      Serial.println();
    }
    
    sync_idx = -1;
    received = 0;
    count++;
  }

  return;
}
