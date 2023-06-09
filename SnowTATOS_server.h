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
// size of the data that simb expects over i2c for transmission. fx of individual
// sample size, number of stations, sampling frequency, transmit frequency
#define SIMB_DATASIZE 60

// start time as a unix timestamp (seconds)
#define START_TIME_UNIX

// declare a buffer to hold simb data
uint8_t simbData[SIMB_DATASIZE];

#define AWAKE_PERIOD 10 // number of minutes to stay awake during active period

const int flashChipSelect = 4;

#include "dataFile.h"
#include "tempRadio.h"
#include "TimeSnowTATOS.h"
#include <SerialFlash.h>
#include "SnowTATOS_i2c.h"
#include <TimeLib.h>

// declare a global to keep track of initial time
time_t startTime;

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
