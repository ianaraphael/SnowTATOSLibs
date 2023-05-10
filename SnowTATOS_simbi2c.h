/*

A library to deal with simb side i2c comms between SnowTATOS sensor controller and simb

Ian Raphael
2023.05.05
ian.a.raphael.th@dartmouth.edu

*/

#ifndef SnowTATOS_simbi2c_h
#define SnowTATOS_simbi2c_h

#include <Wire.h>

#define SENSORCONTROLLER_ADDRESS 9 // sensor controller (SC) I2C Address
#define MAX_PACKET_SIZE 32 // maximum dataframe size
#define SENSORCONTROLLER_CS 6 // sensor controller chip select on simb side
#define CS_DELAY 10 // number of milliseconds to wait with chip select pin low
#define DATA_SIZE 68 // agreed upon data size


// init i2c communications with sensor controller as a controller
void init_I2C_simbSide(int dataSize) {

  // init i2c as master
  Wire.begin();

  // set pin mode for sensor controller chip select and write it high (active low)
  pinMode(SENSORCONTROLLER_CS,INPUT_PULLUP);
  pinMode(SENSORCONTROLLER_CS,OUTPUT);
  digitalWrite(SENSORCONTROLLER_CS,HIGH);
}

void alertSensorController(){

  SerialUSB.println("Requesting data from sensor controller");

  // write sc chip select low (active) to wake up SC and say we're going to ask for data
  digitalWrite(SENSORCONTROLLER_CS,LOW);
  // delay for a moment
  delay(CS_DELAY);
  // then write high again
  digitalWrite(SENSORCONTROLLER_CS,HIGH);
}

void getDataFromSensorController(char *dataBuf) {

  // flush the wire before we start reading anything
  while(Wire.available()){
    Wire.read();
  }

  // update how much data we're requesting
  uint16_t n_dataToGet = DATA_SIZE;

  // counter variable to track where we are in the total data retrieval session
  int i_session = 0;

  // while we haven't gotten all of the data
  while (n_dataToGet > 0) {

    // declare variable to keep track of the packet size we're asking for
    int currPacketSize;

    // if what's left is more than the maximum packet size
    if (n_dataToGet > MAX_PACKET_SIZE) {
      // request the maximum packet size
      currPacketSize = MAX_PACKET_SIZE;
    } else { // otherwise
      // get what's left
      currPacketSize = n_dataToGet;
    }

    SerialUSB.print("We expect this many bytes from SC: ");
    SerialUSB.println(currPacketSize,DEC);

    // ask the SC to put the data on the wire
    SerialUSB.print("SC put this many bytes on the wire: ");
    int nBytesCurrPacket = Wire.requestFrom(SENSORCONTROLLER_ADDRESS,currPacketSize);
    SerialUSB.println(nBytesCurrPacket,DEC);

    // counter variable to track where we are in the current data packet
    int i_currPacket = 0;

    // then, while we haven't read all of this packet
    while (i_currPacket<currPacketSize) {

      // read the next byte off the wire into the array
      dataBuf[i_session] = Wire.read();

      // and increment the counters
      i_session++;
      i_currPacket++;
    }

    // now decrement n_dataToGet by the amount that we read
    n_dataToGet = n_dataToGet - currPacketSize;
  }
}



#endif
