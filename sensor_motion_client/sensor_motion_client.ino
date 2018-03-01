// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

#include <Wire.h>
#include "Adafruit_VCNL4010.h"

/* VCNL motion sensor */
Adafruit_VCNL4010 vcnl;

/* 
 *  Configuration for Lora
 */
 
// Lora pins for feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

/* Lora frequency must match gateway's freq! */
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// FIXME: These need to be a common file (or base class)
#define LORA_GATEWAY_ADDRESS 0
#define MOTION_SENSOR_ADDRESS 2

/* 
 * Class to manage message delivery and receipt or Lora messages, 
 * using the driver declared above 
 */
RHReliableDatagram rf95_manager(rf95, MOTION_SENSOR_ADDRESS);

/* 
 *  Initial setup (constructor)
 */
void setup()
{
  while (!Serial);
  Serial.begin(9600);
  delay(100);

  /*
   * This part needs refactoring to handle different kinds of sensors
   */
  Serial.println("Motion sensor node initializing ...");

  // Initialize the motion sensor
  if (! vcnl.begin()){
    Serial.println("VCNL4010 sensor not found");
    while (1);
  }
  Serial.println("Found VCNL4010");
  
  /*
   * Lora radio initialization
   * 
   * FIXME: All of this belongs in a base class
   */
  // LoRa radio reset
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95_manager.init()) {
    Serial.println("Lora radio init failed");
    while (1);
  }
  Serial.println("Lora radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  
  rf95_manager.setRetries(5);
}

// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
boolean proximityDetected = false;

/*
 * main loop
 */
void loop() {
  // Read raw value from sensor
  float proximity = vcnl.readProximity();

  /* 
   *  FIXME: Magical threshold value needs to be replaced
   */
  if (proximity > 3500) {
    proximityDetected = true;
  
    Serial.print("Proximity raw value:");
    Serial.println(proximity);

    /* Convert to ascii  */
    itoa((int)1, (char *)&buf, 10); // buf will be null terminated
    
    Serial.print("Proxmimity data ON: ");
    Serial.println((char *)buf);
  
    // Send the data to the gateway, wait for ack (with retries)
    if (!rf95_manager.sendtoWait(buf, sizeof(buf), LORA_GATEWAY_ADDRESS)) {
      Serial.print("Failed to send ON packet to gateway: ");
      Serial.println((char *)buf);
    }
  }
  else if (proximityDetected == true) {
    itoa((int)0, (char *)&buf, 10);
    Serial.print("Proxmimity data OFF: ");
    Serial.println((char *)buf);
  
    // Send the data to manager_server
    if (!rf95_manager.sendtoWait(buf, sizeof(buf), LORA_GATEWAY_ADDRESS)) {
      Serial.print("Failed to send OFF packet to gateway: ");
      Serial.println((char *)buf);
    }
    proximityDetected = false;
  }

  // FIXME: Needs to be configurable
  delay(500);
}

