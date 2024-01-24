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
#define SERVER_CS 14 // simb i2c chip select on server side
#define I2C_TIMEOUT 30000 // 30 seconds


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

  // reset request flag to false
  simbRequestFlag = false;

  // set our state variable to indicate that we're no longer in simb transaction
  i2cCollectionState = -1;
}


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

    // flash led
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,HIGH);
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

  // start the timer
  uint32_t startTime = millis();

  bool simbReadError = false;
  // hang out while the simb has not picked up the data
  while (i2cCollectionState != -1) {
    // if we've rolled over
    if (millis() - startTime >= I2C_TIMEOUT) {
      // call request event manually
      requestEvent();
      simbReadError = true;
    }
  }

  if (simbReadError) {
    Serial.println("SIMB I2C timed out");
  } else {
    Serial.println("SIMB collected data");
  }

  // now reset the buffer with generic mask
  maskSimbData(simbData);

  // now that we're done sending data, reattach our chip select interrupt
  // attachInterrupt(digitalPinToInterrupt(SERVER_CS), simbInterruptHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(SERVER_CS), simbInterruptHandler, LOW);
}

#endif
