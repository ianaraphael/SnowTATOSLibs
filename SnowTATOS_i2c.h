/*

A library to deal with SC side i2c comms between SnowTATOS sensor controller and simb

Ian Raphael
2023.05.04
ian.a.raphael.th@dartmouth.edu

*/

#ifndef SnowTATOS_i2c_h
#define SnowTATOS_i2c_h

#include <Wire.h>

#define SENSORCONTROLLER_ADDRESS 9 // Define sensor controller (SC) I2C Address
#define MAX_PACKET_SIZE 32 // maximum i2c packet size
#define SERVER_CS 11 // simb i2c chip select on server side
// #define SERVER_CS A2 // simb i2c chip select on server side


// collection state variable to keep track of what information we've sent to the
// simb.
// i2cCollectionState = -1
//    free state. SC is not communicating with simb and is free to do whatever it wants.
// i2cCollectionState = 0
//    pre free state. finished data transfer and SC is ready to go back to free state after this
// i2cCollectionState = 1
//    standby state. wait for a data reqest interrupt from the SIMB, then put the data buffer on the wire
// i2cCollectionState = 2
//    data packing state. simb is asking for the next packet of data. we'll package the
//    next packet then go back to standby state.
volatile int i2cCollectionState = -1;
volatile bool simbRequestFlag = false; // indicates that the simb has requested data from us
volatile uint8_t i2cSendBuf[MAX_PACKET_SIZE+1]; // add one byte for a null terminator so that we can print on this side. not for transmission.
volatile uint16_t currI2cPacketSize;

// service routine for getting a Wire data request from the SIMB. Puts the requested
// data on the line
void requestEvent() {
  // write the data to the line
  Wire.write((uint8_t *)i2cSendBuf,(uint16_t)currI2cPacketSize);

  // set withSimb false to indicate that we're no longer in a transaction
  simbRequestFlag = false;

  // enter sleep state
  i2cCollectionState = -1;
}

// // service routine for getting a Wire data request from the SIMB. Puts the requested
// // data on the line
// void requestEvent() {
//
//   // write the data to the line
//   Wire.write((uint8_t *)i2cSendBuf,(uint16_t)currI2cPacketSize);
//
//   // if we're done with the SIMB transaction after this
//   if (i2cCollectionState == 0) {
//
//     // set withSimb false to indicate that we're no longer in a transaction
//     simbRequestFlag = false;
//
//     // enter sleep state
//     i2cCollectionState = -1;
//
//   } else if (i2cCollectionState == 1) {
//
//     // go to data packaging state
//     i2cCollectionState = 2;
//   }
// }


// service routine for handling chip select interrupt from SIMB
void simbInterruptHandler(void) {

  // // detach the interrupt so we don't get stuck in a loop
  // detachInterrupt(digitalPinToInterrupt(SERVER_CS));
  //
  // // set simbRequestFlag true
  // simbRequestFlag = true;

  // if chip select is actually low
  if ((digitalRead(SERVER_CS) == LOW) && (i2cCollectionState == -1)) {

    // detach the interrupt so we don't get stuck in a loop
    detachInterrupt(digitalPinToInterrupt(SERVER_CS));

    // set simbRequestFlag true
    simbRequestFlag = true;
  }
}


// init fx for setting up i2c on sensor controller side
void init_I2C_scSide() {

  // // a little shenanigans to make sure this pin does not read low on startup
  // pinMode(SERVER_CS,OUTPUT);
  // digitalWrite(SERVER_CS,HIGH);

  // pull up the chip select pin
  pinMode(SERVER_CS,INPUT_PULLUP);

  // delay for a moment
  delay(10);

  // Initialize I2C comms as slave
  Wire.begin(SENSORCONTROLLER_ADDRESS);

  // Function to run when data requested from master
  Wire.onRequest(requestEvent);

  // then attach an interrupt to trigger on a falling pin
  attachInterrupt(digitalPinToInterrupt(SERVER_CS), simbInterruptHandler, FALLING);
  // attachInterrupt(digitalPinToInterrupt(SERVER_CS), simbInterruptHandler, LOW);
}

// function to check if the simb has asked for data
bool simbRequestedData() {
  return simbRequestFlag;
}


// function to send data over to simb by i2c. MAX DATAFRAME 32 BYTES!
void sendDataToSimb(uint8_t *simbData) {

  uint8_t* data = simbData;

  Serial.print("SIMB requested data. We have ");
  Serial.print(SIMB_DATASIZE,DEC);
  Serial.println(" bytes to send: ");
  for (int i=0;i<SIMB_DATASIZE;i++){
    Serial.print(simbData[i],HEX);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.println("Packaging data and standing by.");

  // get the packet size
  currI2cPacketSize = SIMB_DATASIZE;

  // copy all of the data in
  for (int i=0; i<currI2cPacketSize; i++) {
    i2cSendBuf[i] = data[i];
  }

  // add a null terminator to the buffer
  i2cSendBuf[currI2cPacketSize] = '\0';

  // preview print
  Serial.print("SIMB has requested data. Packaging: ");
  // Serial.println((char*) i2cSendBuf);
  for (int i=0;i<SIMB_DATASIZE;i++){
    Serial.print(i2cSendBuf[i],HEX);
    Serial.print(" ");
  }
  Serial.println("");

  // go into standby mode, wait for request event
  i2cCollectionState = 1;

  // hang out while the simb has not picked up the data
  while (i2cCollectionState != -1) {
  }

  Serial.println("SIMB collected data");

  // now reset the buffer with generic mask
  maskSimbData(simbData);

  // now that we're done sending data, reattach our chip select interrupt
  attachInterrupt(digitalPinToInterrupt(SERVER_CS), simbInterruptHandler, FALLING);
  // attachInterrupt(digitalPinToInterrupt(SERVER_CS), simbInterruptHandler, LOW);

  // if we're supposed to be asleep right now
  if (timeToSleep) {
    // go back to sleep
    LowPower.standby();
  }
}

// // function to send data over to simb.
// void sendDataToSimb(uint8_t *simbData) {
//
//   uint8_t* data = simbData;
//
//   // reset our num bytes left to send
//   int n_bytesLeftToSendSIMB = SIMB_DATASIZE;
//
//   Serial.print("SIMB requested data size. We have ");
//   Serial.print(n_bytesLeftToSendSIMB,DEC);
//   Serial.println(" bytes to send: ");
//   for (int i=0;i<SIMB_DATASIZE;i++){
//     Serial.print(simbData[i],HEX);
//     Serial.print(" ");
//   }
//   Serial.println("");
//   Serial.println("Packaging data and standing by.");
//
//   // switch to i2cCollectionState 2 (data packing)
//   i2cCollectionState = 2;
//
//   // while we haven't sent all the data
//   while (i2cCollectionState != -1) {
//
//     // switch on the i2cCollectionState variable
//     switch(i2cCollectionState) {
//
//       // standby
//       // waiting for a data request from simb
//       case 1:
//       break; // do nothing. handled in ISR.
//
//       // data packing
//       // simb has requested data. package the data and move back to standby
//       case 2:
//
//       // if we have more data than the max packet size
//       if (n_bytesLeftToSendSIMB > MAX_PACKET_SIZE) {
//
//         // send the max amount of data
//         currI2cPacketSize = MAX_PACKET_SIZE;
//
//       } else { // otherwise
//
//         // send the data we have left
//         currI2cPacketSize = n_bytesLeftToSendSIMB;
//
//         // set collection state to 0 (pre-sleep) since we'll be done sending data after this
//         i2cCollectionState = 0;
//       }
//
//       // figure out where our current data starts and stops
//       int startIndex = SIMB_DATASIZE - n_bytesLeftToSendSIMB;
//       int endIndex = startIndex + currI2cPacketSize - 1;
//
//       // copy all of the data in
//       for (int i=startIndex; i<=endIndex; i++){
//         i2cSendBuf[i-startIndex] = data[i];
//       }
//
//       // and update n_bytesLeftToSendSIMB
//       n_bytesLeftToSendSIMB = SIMB_DATASIZE - endIndex - 1;
//
//       // add a null terminator to the buffer
//       i2cSendBuf[currI2cPacketSize] = '\0';
//
//       // preview print
//       Serial.print("SIMB has requested data. Packaging: ");
//       // Serial.println((char*) i2cSendBuf);
//       for (int i=0;i<sizeof(i2cSendBuf);i++){
//         Serial.print(i2cSendBuf[i],HEX);
//         Serial.print(" ");
//       }
//       Serial.println("");
//
//       // print how much we have left to send
//       Serial.print(n_bytesLeftToSendSIMB,DEC);
//       Serial.println(" bytes left to send.");
//
//       // if we're not supposed to go to sleep
//       if (i2cCollectionState == 2) {
//         // go back into standby, wait for request event
//         i2cCollectionState = 1;
//       }
//       break;
//     }
//   }
//
//   // now that we're done sending data, reattach our chip select interrupt
//   attachInterrupt(digitalPinToInterrupt(SERVER_CS), simbInterruptHandler, FALLING);
//
//   // if we're supposed to be asleep right now
//   if (timeToSleep) {
//     // go back to sleep
//     LowPower.standby();
//   }
// }

#endif
