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
#include "Adafruit_MCP9808.h"

/* MCP9808 temperature sensor object */
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

/* 
 *  Configuration for Lora 
 */
// Lora pins for feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

/* Lora frequency must match gateway's freq! */
#define RF95_FREQ 915.0

/* Singleton instance of the radio driver */
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// FIXME: These need to be a common file (or base class)
#define LORA_GATEWAY_ADDRESS 0
#define TEMP_SENSOR_ADDRESS 1

/* 
 * Class to manage message delivery and receipt or Lora messages, 
 * using the driver declared above 
 */
RHReliableDatagram rf95_manager(rf95, TEMP_SENSOR_ADDRESS);

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
  Serial.println("Temperature sensor node initializing ...");

  // Make sure the temp sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x19) for example
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808!");
    while (1);
  }

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

/*
 * main loop
 */
void loop() {
  float temperature = 0;
  
  readTemperature(&temperature);

  /* Format temperature into ascii */
  int frac;
  frac = (unsigned int)(temperature * 1000) % 1000; //get three numbers to the right of the deciaml point

  // First, add the integer part of the value
  memset(&buf, sizeof(buf), 0);
  itoa((int)temperature, (char *)&buf, 10); // buf will be null terminated by itoa
  
  // Now add fractional part
  strcat((char *)&buf, ".");
  itoa(frac, (char *)&buf[strlen((char *)&buf)], 10); //put the frac after the deciaml

  Serial.print("Temperature data: "); Serial.println((char *)buf);
  
  // Send the data to gateway, wait for ack (with retries)
  if (!rf95_manager.sendtoWait(buf, sizeof(buf), LORA_GATEWAY_ADDRESS)){
    Serial.print("Failed to send packet to gateway: ");
    Serial.println((char *)buf);
  }

  /*
   * FIXME:
   * (1) likely want this configurable
   * (2) Likely want to reduce this if the temperature changes drastically 
   *     between subsequent measurements
   */
  delay(2000);
}

/*
 * Read temperature
 * 
 * FIXME:
 * Likely should be a private or protected in a temp specific child sensor class
 */
uint8_t readTemperature(float * pTemperature) {

  //Serial.println("wake up MCP9808.... "); // wake up MSP9808 - power consumption ~200 mikro Ampere
  //tempsensor.wake();   // wake up, ready to read!

  // Read and print out the temperature, then convert to *F
  float c = tempsensor.readTempC();
  float f = c * 9.0 / 5.0 + 32;
  //  Serial.print("Temp: "); Serial.print(c); Serial.print("*C\t");
  //  Serial.print(f); Serial.println("*F");

  //Serial.println("Shutdown MCP9808.... ");
  //tempsensor.shutdown(); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere

  *pTemperature = f;
  return 0;
}


