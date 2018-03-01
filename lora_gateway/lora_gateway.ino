/***************************************************
  Adafruit MQTT Library WINC1500 Example

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <SPI.h>
#include <WiFi101.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Blinky
#define LED 13

/* WiFI Setup */
#define WINC_CS   8
#define WINC_IRQ  7
#define WINC_RST  4
#define WINC_EN   2     // or, tie EN to VCC

/* Security!! */
char ssid[] = "Your SSID";    //  your network SSID (name)
char pass[] = "Your Wifi password";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

/* Lora setup */
#define RFM95_CS 11
#define RFM95_RST 12
#define RFM95_INT 10

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the emtech SX1276/77/78/79 radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

/* Lora packet mgmt */
#define LORA_GATEWAY_ADDRESS 0
RHReliableDatagram rf95_manager(rf95, LORA_GATEWAY_ADDRESS);

/*
 * FIXME: This really needs to be a class, with capability to add a sensor 
 * dynamically, with security
 */
struct loraNodeStruct {
  uint8_t sensorAddress;
  Adafruit_MQTT_Publish *pNodePublishFeed;
} LORA_NODE_STRUCT;
struct loraNodeStruct loraNodeArray[256];

#define TEMP_SENSOR 1;
#define MOTION_SENSOR 2;

/* MQTT Configuration */

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "rmemory"
#define AIO_KEY         "2ea8c31fffe94cca944d2e6c00e8148a"

//Set up the wifi MQTT client
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/* MQTT Feeds */
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish tempNode = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/southbigfish.temp");
Adafruit_MQTT_Publish motionNode = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/southbigfish.motion");

// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/southbigfish.onoffbutton");

/* Setup Code */

void setup() {
  pinMode(LED, OUTPUT);

  /* Initialize loraNodeArray */
  /*
   * FIXME: Need a way to make this not static
   */
  memset(loraNodeArray, 0, sizeof(loraNodeArray));

  loraNodeArray[0].pNodePublishFeed = &tempNode;
  loraNodeArray[0].sensorAddress = TEMP_SENSOR;
  loraNodeArray[1].pNodePublishFeed = &motionNode;
  loraNodeArray[1].sensorAddress = MOTION_SENSOR;
  
  /* 
   * Set up Lora radio on gateway 
   */
  /* Reset radio */
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95_manager.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  /* 
   * Explicitly set number of retries to wait for ack from
   * sendToWait
   */
//  rf95_manager.setRetries(3);

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set LoRa Freq to: "); Serial.println(RF95_FREQ);
  
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  
  WiFi.setPins(WINC_CS, WINC_IRQ, WINC_RST, WINC_EN);

  while (!Serial);
  Serial.begin(9600);

  // Initialise the Client
  Serial.print(F("\nInit the WiFi module..."));
  // check for the presence of the breakout
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WINC1500 WiFi chip not present");
    // don't continue:
    while (true);
  }
  Serial.println("WiFi OK!");

  // Get return messages from the broker
  mqtt.subscribe(&onoffbutton);
}

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton.lastread);

      if (0 == strcmp((char *)onoffbutton.lastread, "OFF")) {
        digitalWrite(LED, LOW);
      }
      if (0 == strcmp((char *)onoffbutton.lastread, "ON")) {
        digitalWrite(LED, HIGH);
      }
    }
  }

  // Check sensors
  if (rf95_manager.available()) {
    uint8_t sensorAddress = rf95_manager.headerFrom();

    /* sensorAddress can be between 1 and 255, translating to index values between 0 and 254 inclusive */
    if (sensorAddress > 0 && sensorAddress < 256 /*&& loraNodeArray[sensorAddress - 1] != 0L*/) {
      uint8_t len = sizeof(buf);
      uint8_t gatewayAddress = LORA_GATEWAY_ADDRESS;
      if (rf95_manager.recvfromAckTimeout(buf, &len, 100, &sensorAddress, &gatewayAddress, NULL, NULL)) {
        Serial.print("Received data from sensor: ");
        Serial.println((char *)buf);
        Serial.print(F("Publishing to MQTT: "));
        if (! loraNodeArray[sensorAddress - 1].pNodePublishFeed->publish((char*)buf)) {
          Serial.println(F("...Failed\n"));
        } 
        else {
          Serial.println(F("...OK!\n"));
        }
      }
    }
  } 
}
 
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  uint8_t ret;
  uint8_t status = WL_IDLE_STATUS;

  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    uint8_t timeout = 10;
    while (timeout && (WiFi.status() != WL_CONNECTED)) {
      timeout--;
      delay(1000);
    }
  }
  
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 1 seconds...");
       mqtt.disconnect();
       delay(1000);  // wait 1 seconds
  }
  Serial.println("MQTT Connected!");
}

