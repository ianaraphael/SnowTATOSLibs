/*

  A library for handling radio transmissins in temperature datalogger
  deployments.

  Ian Raphael
  2023.03.27
  ian.a.raphael.th@dartmouth.edu

*/

#ifndef tempRadio_h
#define tempRadio_h

#include "RHReliableDatagram.h"
#include "RH_RF95.h"
#include "dataFile.h"

#define SERVER_ADDRESS 0 // server address always 0

#define MAX_TRANSMISSION_ATTEMPTS 3 // maximumum number of times to attempt transmission per sampling cycle
#define RADIO_TIMEOUT 5000 // five seconds
#define TIMEOUT 20000 // max wait time for radio transmissions in ms
#define RADIO_FREQ 915.0 // radio frequency
#define RADIO_POWER 23 // max power

// Radio pins
#define RADIO_CS 5 // radio chip select pin
#define RADIO_INT 2 // radio interrupt pin

RH_RF95 driver(RADIO_CS, RADIO_INT); // Singleton instance of the radio driver
RHReliableDatagram manager(driver, STATION_ID); // Class to manage message delivery and receipt, using the driver declared above

/************ init_Radio() ************/
/*
Function to initialize the radio
*/
void init_Radio() {

  // wait while the radio initializes
  manager.init();
  delay(1000);

  manager.setTimeout(RADIO_TIMEOUT); // set timeout

  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  driver.setTxPower(RADIO_POWER, false);
  driver.setFrequency(RADIO_FREQ);

  // print surccess
  Serial.println("Client radio initialized successfully");
}



/************ radio transmission ************/
/*
function to transmit a generic data file to the server
*/
void sendData_fromClient(String filename) {

  // send a handshake message (1 byte for message type, then size of filename)
  int filenameLength = (filename).length() + 1; // get the length of the filename
  char charFilename[filenameLength]; // allocate array to hold filename as chars
  (filename).toCharArray(charFilename,filenameLength); // convert to a char array
  uint8_t handShake[1 + filenameLength]; // allocate memory for handshake message
  handShake[0] = (uint8_t) 0; // write our message type code in (0 for handshake, 1 for data)
  memcpy(&handShake[1], charFilename, filenameLength); // copy the filename in

  // Dont put this on the stack:
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  // char msg[sizeof(buf)];

  // set number of transmission attempts to 0
  int nTransmissionAttempts = 0;

  // // goto label for radio transmission block
  // handshake:

  do {

    Serial.println("Attempting to send a message to the server");

    // manager.sendtoWait((uint8_t*) dataString.c_str(), dataString.length()+1, SERVER_ADDRESS))

    // disable intterupts during comms
    // noInterrupts();

    // send the initial handshake message to the server
    if (manager.sendtoWait((uint8_t*) handShake, sizeof(handShake), SERVER_ADDRESS)) {

      // Now wait for the reply from the server telling us how much of the data file it has
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (manager.recvfromAckTimeout(buf, &len, TIMEOUT, &from)) {

        // print the server reply
        // strcpy(msg, (char*)buf);
        // strcat(msg, " ");
        // convert to an integer
        // unsigned long serverFileLength = *msg;
        Serial.print("Received a handshake from server with length: ");
        Serial.println(len,DEC);
        Serial.println("");

        unsigned long serverFileLength = *(unsigned long*)buf;

        Serial.print("Server file length: ");
        Serial.println(serverFileLength, DEC);

        // open the file for reading
        File dataFile = openFile_read(filename);

        // if the file is available
        if (dataFile) {

          // get the file length
          unsigned long stationFileLength = getFileSize(filename);

          Serial.print("Station file length: ");
          Serial.println(stationFileLength, DEC);

          // if the file is longer than what the server has
          if (serverFileLength < stationFileLength) {

            Serial.println("Sending: ");

            // seek to that point in the file
            seekToPoint(dataFile,serverFileLength);

            // create a buffer with max message length
            uint8_t sendBuf[RH_RF95_MAX_MESSAGE_LEN];

            // leave some room to prevent buffer overflow
            uint8_t sendLength = RH_RF95_MAX_MESSAGE_LEN - 4;

            // while we haven't sent all information
            while (stationFileLength > getFilePosition(dataFile)) {

              memset(sendBuf, 0, sizeof(sendBuf));

              // put our message type code in (1 for data)
              sendBuf[0] = (uint8_t) 1;

              // read a 256 byte chunk
              int numBytesRead = readFileBytes(dataFile, &sendBuf[1], sendLength-1);

              // print the data that we're going to send
              // Serial.println("Sending the following data to the server: ");
              Serial.print((char*) sendBuf);
              // Serial.println("");

              // send the data to the server
              manager.sendtoWait((uint8_t*) sendBuf, sendLength, SERVER_ADDRESS);
            }
          }

          // close the file
          closeFile(dataFile);
        }

        // send a closure message
        uint8_t closureMessage = 0;
        manager.sendtoWait((uint8_t*) &closureMessage, sizeof(&closureMessage), SERVER_ADDRESS);

        // break out of the loop
        break;
      }
      else {

        // in this case, the server received a message but we haven't gotten a
        // handshake back. the server is busy with another client. try again later
        Serial.println("Server's busy, gonna wait a few seconds.");

        // increment the number of attempts we've made
        nTransmissionAttempts++;

        // build in a random delay between 10 and 20 seconds
        // TODO: in the future, make this a multiple of station ID to prevent collision?
        delay(1000*random(10,21));

        // continue next cycle of the loop
        continue;
      }
    }
    else {
      // in this case, the server did not recieve the message. we'll try again at
      // the next sampling instance.
      Serial.println("Server failed to acknowledge receipt");
      break;
    }

    // try again if we failed and haven't reached max attempts
  } while (nTransmissionAttempts < MAX_TRANSMISSION_ATTEMPTS);
}

#endif
