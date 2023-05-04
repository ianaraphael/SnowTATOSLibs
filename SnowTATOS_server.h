/*

  A library to collect common libraries and functions necessary for SnowTATOS
  server.

  Ian Raphael
  2023.05.03
  ian.a.raphael.th@dartmouth.edu

*/

#ifndef SnowTATOS_server_h
#define SnowTATOS_server_h

#define STATION_ID SERVER_ADDRESS // our station id is the server address (0)
const int flashChipSelect = 4;

#include "dataFile.h"
#include "tempRadio.h"
#include "TimeSnowTATOS.h"
#include <SerialFlash.h>
#include "SnowTATOS_i2c.h"

/************ Board setup ************/
void boardSetup() {

  unsigned char pinNumber;

  // Pull up all unused pins to prevent floating vals
  for (pinNumber = 0; pinNumber < 23; pinNumber++) {
    pinMode(pinNumber, INPUT_PULLUP);
  }
  for (pinNumber = 32; pinNumber < 42; pinNumber++) {
    pinMode(pinNumber, INPUT_PULLUP);
  }
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // set our readout resolution
  analogReadResolution(12);

  // put the flash chip to sleep since we are using SD
  SerialFlash.begin(flashChipSelect);
  SerialFlash.sleep();

  // Initialize I2C comms as slave
  Wire.begin(SLAVE_ADDR);

  // Function to run when data requested from master
  Wire.onRequest(requestEvent);

  // pull the chip select pin down
  pinMode(SENSORCONTROLLER_CS,INPUT_PULLDOWN);


  SerialUSB.println("Sensor controller initiated, waiting for SIMB to initialize");
  //TODO: don't wait for SIMB to initialize
  // wait around until SIMB gives us the go ahead by shifting the chip select high
  while(digitalRead(SENSORCONTROLLER_CS) == LOW) {
  }

  SerialUSB.println("SIMB activated, attaching interrupt");

  // pull up the chip select pin
  pinMode(SENSORCONTROLLER_CS,INPUT_PULLUP);

  // delay for a moment
  delay(10);

  // then attach an interrupt to trigger on a low pin
  attachInterrupt(digitalPinToInterrupt(SENSORCONTROLLER_CS), simbInterruptHandler, LOW);

}

#endif
