/*

  A library to deal with i2c comms between SnowTATOS sensor controller and simb

  Ian Raphael
  2023.05.04
  ian.a.raphael.th@dartmouth.edu

*/

#ifndef SnowTATOS_i2c_h
#define SnowTATOS_i2c_h

#include <Wire.h>

#define SLAVE_ADDR 9 // Define sensor controller (SC) I2C Address
#define MAX_PACKET_SIZE 32 // define
#define SENSORCONTROLLER_CS 6 // chip select
volatile bool withSimb false; // indicates that we're in transaction with SIMB

volatile uint8_t i2cSendBuf[MAX_PACKET_SIZE+1]; // add one byte for a null terminator so that we can print on this side. not for transmission.

// service routine for getting a Wire data request from the SIMB. Puts the requested
// data on the line
void requestEvent() {

  // write the data to the line
  Wire.write((uint8_t *)i2cSendBuf,(uint16_t)currI2cPacketSize);

  // if we're done with the SIMB transaction after this
  if (i2cCollectionState == 0) {

    // set withSimb false to indicate that we're no longer in a transaction
    withSimb = false;

    // enter sleep state
    i2cCollectionState = -1;

  } else if (i2cCollectionState == 2) {

    // go to data packaging state
    i2cCollectionState = 3;
  }
}


// service routine for handling chip select interrupt from SIMB
void simbInterruptHandler(void) {

  // set withSimb true to indicate that we're in a transaction with the simb
  withSimb = true;

  // if chip select is actually low
  if (digitalRead(SENSORCONTROLLER_CS) == LOW) {

    // detach the interrupt so we don't get stuck in a loop
    detachInterrupt(digitalPinToInterrupt(SENSORCONTROLLER_CS));

    // and set collection state variable to 1 (package metadata)
    i2cCollectionState = 1;
  }
}





#endif
