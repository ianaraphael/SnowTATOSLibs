/*

  A library to collect common libraries and functions necessary for SnowTATOS
  client.

  Ian Raphael
  2023.05.03
  ian.a.raphael.th@dartmouth.edu

*/

#ifndef SnowTATOS_client_h
#define SnowTATOS_client_h

#include "dataFile.h"
#include "TempSensors.h"
#include "tempRadio.h"
#include "SnowPinger.h"
#include "TimeSnowTATOS.h"
#include <SerialFlash.h>

const int flashChipSelect = 4;

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
}

#endif
