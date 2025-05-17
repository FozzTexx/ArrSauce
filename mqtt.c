// -*- mode: C; indent-tabs-mode: nil -*-

/* Created 2021 Jul 24 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Handles connecting to WiFi and the MQTT broker.
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>

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

WiFiClient espClient;
PubSubClient client(espClient);

const char *qos1_topic, *qos1_message;
boolean qos1_received = false;

void setup_wifi()
{
  int retries = 20;


  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.config(IPAddress(10, 4, 0, 241), IPAddress(10, 4, 0, 1),
              IPAddress(255, 255, 255, 0), IPAddress(10, 4, 0, 1));
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (retries && WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retries--;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
    WiFi.disconnect();

  return;
}

void callback(const char *topic, const byte *message, unsigned int length)
{
  String messageTemp;
  int i;


  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  for (i = 0; i < length; i++) {
    Serial.print((char) message[i]);
    messageTemp += (char) message[i];
  }
  Serial.println();

  if (!qos1_received
      && length == strlen(qos1_message)
      && !strcmp(topic, qos1_topic)
      && !strncmp((const char *) message, qos1_message, length))
    qos1_received = true;

  return;
}

void reconnect()
{
  while (WiFi.status() == WL_CONNECTED && !client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ArrSauce")) {
      Serial.println("connected");
      // For doing QOS 1
      client.subscribe(TOPIC "/data");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 0.5 seconds");
      delay(500);
    }
  }

  return;
}

void printLocalTime()
{
  struct tm timeinfo;


  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();

  return;
}

void setup_mqtt()
{
  Serial.println("Setting up mqtt");
  setup_wifi();
  if (WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, NTP_SERVER);
    printLocalTime();

    client.setKeepAlive(60);
    client.setSocketTimeout(60);
    client.setServer(MQTT_BROKER, 1883);
    client.setCallback(callback);
  }
  return;
}

boolean publish_qos1(const char *topic, const char *message)
{
  int retries, rcv_retries;
  boolean rc;


  qos1_topic = topic;
  qos1_message = message;
  qos1_received = false;

  for (retries = 10; retries; retries--) {
    rc = client.publish(qos1_topic, qos1_message);
    for (rcv_retries = 10; rcv_retries; rcv_retries--) {
      client.loop();
      if (qos1_received)
        return true;
      Serial.print("QOS RX retry ");
      Serial.println(rcv_retries);
      delay(100);
    }
    Serial.print("QOS retry ");
    Serial.println(retries);
  }

  return false;
}

void publishTemperature(sensor_data *tdata)
{
  time_t now;
  struct tm *gmt;
  char buf[256], tbuf[50];
  unsigned long uid;
  boolean rc;


  now = time(NULL);
  gmt = gmtime(&now);
  strftime(tbuf, sizeof(tbuf) - 1, "%Y-%m-%dT%H:%M:%SZ", gmt);

  uid = tdata->channel + 1;
  uid <<= 8;
  uid |= tdata->rolling_code;
  uid <<= 16;
  uid |= tdata->device_id;

  snprintf(buf, sizeof(buf),
           "{\"id\":\"%lX\""
           ",\"channel\":%i"
           ",\"flags\":%i"
           ",\"battery_low\":%s"
           ",\"celsius\":%i.%i"
           ",\"humidity\":%i.%i"
           ",\"rolling_code\":%i"
           ",\"checksum\":%i"
           ",\"calculated_checksum\":%i"
           ",\"device_id\":%i"
           ",\"timestamp\":\"%s\"}",
           uid, tdata->channel + 1,
           tdata->flags, tdata->battery_low ? "true" : "false",
           tdata->celsius / 10, abs(tdata->celsius % 10),
           tdata->humidity / 10, abs(tdata->humidity % 10),
           tdata->rolling_code, tdata->checksum, tdata->calculated_checksum,
           tdata->device_id, tbuf);
  // Make sure there's always a terminating NULL
  buf[sizeof(buf)-1] = 0;

  rc = publish_qos1(TOPIC "/data", buf);
  Serial.print("Publish: ");
  Serial.println(rc);

  return;
}

void enable_wifi()
{
  if (WiFi.status() != WL_CONNECTED)
    setup_wifi();

  if (!client.connected())
    reconnect();

  return;
}

void disable_wifi()
{
  if (WiFi.status() == WL_CONNECTED)
    WiFi.disconnect();

  return;
}
