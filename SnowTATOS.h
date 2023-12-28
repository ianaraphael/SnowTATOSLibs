/*

A library for functions common to snowtatos server and client

Updated for Rocket Scream Mini Ultra LoRa board

Ian Raphael
2023.11.28
ian.a.raphael.th@dartmouth.edu

*/
#ifndef SnowTATOS_h
#define SnowTATOS_h


#define STATION_ID 0 // server is always 0
#define TEST true // false for deployment
#define IRIDIUM_ENABLE false // deactivate for testing
#define simbDeployment true // true it deploying system with SIMB
static uint8_t SAMPLING_INTERVAL_MIN = 120; // sampling interval in minutes
static uint8_t SERVER_WAKE_DURATION = 4; // number of minutes for the server to stay awake

#define NUM_TEMP_SENSORS 0 // number of sensors
#define NUM_STATIONS 10 // number of nodes

#define CLIENT_DATA_SIZE ((NUM_TEMP_SENSORS*2) + 1) // data size for each client transmission: 3temps * 2bytes + 1pinger*1byte

#define SIMB_DATASIZE NUM_STATIONS*CLIENT_DATA_SIZE // datasize for the simb buffer


#ifndef FORSIMB
/****************** nothing below this line shared with SIMB ******************/



/***************************************************************************/
/********************************** board **********************************/
/***************************************************************************/

/************ Board setup ************/
void boardSetup() {

  unsigned char pinNumber;

  // TODO: update these pin numbers
  // // Pull up all unused pins to prevent floating vals
  // for (pinNumber = 0; pinNumber < 23; pinNumber++) {
  //   pinMode(pinNumber, INPUT_PULLUP);
  // }
  // for (pinNumber = 32; pinNumber < 42; pinNumber++) {
  //   pinMode(pinNumber, INPUT_PULLUP);
  // }
  // pinMode(25, INPUT_PULLUP);
  // pinMode(26, INPUT_PULLUP);
  // pinMode(13, OUTPUT);
  // digitalWrite(13, LOW);

  // pinMode(4, OUTPUT);
  // digitalWrite(4, HIGH);

  // set our readout resolution
  analogReadResolution(12);
}


// creates a default "error mask" that paints every value with the standard error value
void maskSimbData(uint8_t *simbDataBuffer) {

  // create the standard "error" buffer vals
  uint8_t tempHighByte = highByte(uint16_t(32000));
  uint8_t tempLowByte = lowByte(uint16_t(32000));
  uint8_t pingerValue = uint8_t(255);

  // for every station
  for (int i=1;i<=NUM_STATIONS;i++){

    // get the start byte
    int startByte = (i-1)*CLIENT_DATA_SIZE;

    // for every temp sensor
    for (int i2=0;i2<NUM_TEMP_SENSORS;i2++) {
      simbDataBuffer[startByte+(i2*2)] = tempHighByte;
      simbDataBuffer[startByte+(i2*2)+1] = tempLowByte;
    }
    simbDataBuffer[startByte+NUM_TEMP_SENSORS*2] = pingerValue;
  }
}



// TODO: update to be flexible with and without temperature sensors
// pack the provided client data into the radio transmission buffer
void packClientData(float *tempData, uint8_t pingerData, uint8_t *dataBuffer) {

  // for every temp
  for (int i = 0; i<NUM_TEMP_SENSORS;i++){

    // find the right indices
    int hiIndex = i*2;
    int loIndex = hiIndex+1;

    // get the temp as an int *1000
    // multiply by 1000 (eg, -20.125*1000 = -20125.0)
    int16_t currTemp = tempData[i] * 1000;

    // get the hi and lo byte
    uint8_t hiByte = highByte(currTemp);
    uint8_t loByte = lowByte(currTemp);

    // pack them
    dataBuffer[hiIndex] = hiByte;
    dataBuffer[loIndex] = loByte;
  }

  // put pinger data in last
  dataBuffer[2*NUM_TEMP_SENSORS] = pingerData;
}




// unpack temp data for a particular client from the simb buffer into an
// array of floats
void unpackTempData(uint8_t *dataBuffer, float *tempArray, int stnID) {

  // get the start byte for this station
  int startByte = (stnID-1)*CLIENT_DATA_SIZE;

  // for every temp
  for (int i = 0; i<NUM_TEMP_SENSORS;i++) {

    // get the high byte
    uint8_t hiByte = dataBuffer[startByte+(i*2)];

    // get the low byte
    uint8_t loByte = dataBuffer[startByte+(i*2)+1];

    // put them together
    int currTemp_int = (hiByte << 8) | (0x00ff & loByte);

    tempArray[i] = (float) currTemp_int/1000.00;
  }
}

// unpack pinger data for a particular client from the simb buffer
uint8_t unpackPingerData(uint8_t *dataBuffer, int stnID) {

  // get the pinger index (last value for a given station)
  int startByte = (stnID-1)*CLIENT_DATA_SIZE;
  int pingerIndex = startByte+(CLIENT_DATA_SIZE-1);

  // return the pinger value
  return dataBuffer[pingerIndex];
}



/***************************************************************************/
/********************************** radio **********************************/
/***************************************************************************/

#include <RadioLib.h>
#include <Wire.h>

// server radio address is always 0
#define SERVER_ADDRESS 0
#define RADIO_ID STATION_ID

// radio presets
#define RADIO_TIMEOUT 10000 // max time to spend in radio comms in ms
#define TRANSMISSION_TIMEOUT 1000 // max time to spend on individual transmit ms
#define ACK_TIMEOUT 3000 // max time to wait for ack after transmit ms
#define MAX_TRANSMISSION_ATTEMPTS 5 // maximum number of times for a client to retry sending data
#define RADIO_FREQ 868.0 // radio frequency
#define RADIO_BANDWIDTH 125.0
#define RADIO_POWER 10
// #define RADIO_POWER 23 // max power

// Radio pins
#define RADIO_CS 12 // radio chip select pin
#define RADIO_DIO 10 // radio dio pin

// create a new radio module
SX1276 radio = new Module(RADIO_CS, RADIO_DIO, RADIOLIB_NC, RADIOLIB_NC);

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// radio.standby() to go to standby from sleep, or radio.startReceive()

/************ init_Radio() ************/
/*
Function to initialize the radio
*/
int init_Radio() {

  Serial2.print(F("[SX1276] Initializing ... "));
  // init the radio
  // default settings viewable at: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem
  int state = radio.begin(RADIO_FREQ,RADIO_BANDWIDTH, 12, 7, 0x12, RADIO_POWER, 8, 0);
  if (state == RADIOLIB_ERR_NONE) {
    Serial2.println(F("success!"));
  } else {
    Serial2.print(F("failed, code "));
    Serial2.println(state);
  }

  // set the functions that will be called when packet transmission/receival is finished
  radio.setPacketSentAction(setTransmissionFlag);
  radio.setPacketReceivedAction(setReceiveFlag);

  return state;
}

/************ client radio transmission ************/
/*
function to transmit a byte stream to server over lora

blocks until ack or timeout
*/
bool sendData_fromClient(uint8_t *data) {

  // allocate a buffer to hold the data + 1 byte for station id
  byte sendBuf[sizeof(data)+1];

  // put our station ID in
  sendBuf[0] = STATION_ID;

  // copy the data in
  memcpy(sendBuf[1], data,sizeof(data));

  // get the start time
  uint32_t transmitStartTime = millis();
  // start counting our attempts
  uint8_t transmissionAttempts = 0;

  // start a transmission attempt
  do {

    // start the transmission
    transmissionState = radio.startTransmit(sendBuf, CLIENT_DATA_SIZE);

    // while we haven't timed out
    while (millis()-transmitStartTime < TRANSMISSION_TIMEOUT) {

      // check if the transmission is finished
      if (packetTransmitted) {

        // if it finished without error
        if (transmissionState == RADIOLIB_ERR_NONE) {

          // packet was successfully sent
          Serial.println(F("transmission finished!"));

        } else {

          // otherwise it failed
          Serial.print(F("failed, code "));
          Serial.println(transmissionState);
        }

        // break out
        break;
      }
    }

    // clean up after transmission is finished
    // this will ensure transmitter is disabled, RF switch is powered down etc.
    radio.finishTransmit();

    // if we failed to transmit
    if (!packetTransmitted) {
      // return error
      return false;
    }

    // ...otherwise, success, we'll continue on to listen for an ack

    // first reset the transmission flag
    packetTransmitted = false;

    // then start listening
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
      // return error if we failed to start listening
      return false;
    }

    // get our start time
    uint32_t ackWaitStartTime = millis();

    // while we haven't timed out waiting for the ack
    while (millis() - ackWaitStartTime > ACK_TIMEOUT) {

      // check if the flag is set
      if (packetReceived) {

        // reset flag
        packetReceived = false;

        // figure out how many bytes we received
        uint16_t numBytesReceived = radio.getPacketLength();

        // allocate an array to hold the data
        byte ackArray[numBytes];

        // read the data into the array
        uint16_t state = radio.readData(ackArray, numBytesReceived);

        // if there was an error
        if (state != RADIOLIB_ERR_NONE) {
          // return error
          return false;
        }

        // otherwise, if the message is from the server
        if (ackArray[0] == (byte) SERVER_ADDRESS) {

          // TODO: set syncedWithServer here since we got a message from the server we know it's awake

          // if the acknowledge bit is 0
          if (ackArray[1] == 0) {
            // return error
            return false;
          } else if (ackArray[1] == (byte) STATION_ID) { // otherwise if it's our station ID
            // return success
            return true;
          }
        }
        // if we enter and fall through this control, we received a message
        // from/for someone other than the server. continue listening.
      }
    }

    // TODO: delay for a random moment to prevent intractable collisions

    // increment our transmission attempts
    transmissionAttempts++;

    // do another loop while we have not exceeded our transmission attempts
  } while (transmissionAttempts < MAX_TRANSMISSION_ATTEMPTS);

  // return false if we exceeded transmission attempts
  return false;
}


/************ server radio receipt ************/
/*
function to receive a byte stream from a client by radio
*/
int receiveData_fromClient(uint8_t* dataBuffer) {

  // copy the size of the buffer
  uint8_t len = CLIENT_DATA_SIZE;

  // allocate a short to store station id
  uint8_t from;

  // check if the flag is set
  if (packetReceived) {

    // figure out how many bytes we received
    uint16_t numBytesReceived = radio.getPacketLength();

    // allocate a hold buffer to hang on to the data for a second
    byte holdBuf[numBytesReceived];

    // read the data into the array
    uint16_t state = radio.readData(holdBuf, numBytesReceived);

    // reset flag
    packetReceived = false;

    // if there was an error
    if (state != RADIOLIB_ERR_NONE) {
      // return error/none received
      return = -1;
    }

    // get the sender ID
    from = holdBuf[0];

    // copy the data into the passed buffer excepting the leading byte (station ID)
    memcpy(dataBuffer,holdBuf[1],sizeof(numBytesReceived)-1);

    // TODO: send ack back to client
    // build an ack message
    byte ack[2];
    ack[0] = SERVER_ADDRESS;
    ack[1] = from;

    // transmit the acknowledgement
    transmissionState = radio.startTransmit(ack, sizeof(ack));

    //TODO: fix timer while we haven't timed out
    while (millis()-transmitStartTime < TRANSMISSION_TIMEOUT) {

      // check if the transmission is finished
      if (packetTransmitted) {

        // if it finished without error
        if (transmissionState == RADIOLIB_ERR_NONE) {

          // packet was successfully sent
          Serial.println(F("transmission finished!"));

        } else {

          // otherwise it failed
          Serial.print(F("failed, code "));
          Serial.println(transmissionState);
        }

        // break out
        break;
      }
    }

    // clean up after transmission is finished
    // this will ensure transmitter is disabled, RF switch is powered down etc.
    radio.finishTransmit();


    // return the sender ID
    return from;
  }

  // otherwise return -1 (no message received)
  return -1;
}


// /************ syncWithServer ************/
// /*
// function to sync up with the server
// */
// bool attemptSyncWithServer() {
//
//   // set a timeout for 15 seconds
//   setSyncTimeout((uint16_t)CLIENT_SYNC_TIMEOUT);
//
//   while (!syncTimedOut) {
//     // if the manager isn't busy right now
//     if (manager.available()) {
//
//       // allocate a throwaway buffer to put the message in (we don't need it)
//       buf[10];
//
//       // allocate a short to store station id
//       uint8_t from;
//
//       // if we've gotten a message
//       if (manager.recvfromAck(buf, sizeof(buf), &from)) {
//
//         // if it's from the server
//         if (from == 0) {
//           // we've synced with the server
//           return true;
//         }
//       }
//     }
//   }
//
//   // we timed out before syncing
//   return false;
// }
//


#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif

// flag to indicate that a packet was sent
volatile bool packetTransmitted = false;
// this function is called when a complete packet is transmitted by the module
// IMPORTANT: this function MUST be 'void' type and MUST NOT have any arguments!
void setTransmissionFlag(void) {
  // we sent a packet, set the flag
  packetTransmitted = true;
}

// flag to indicate that a packet was received
volatile bool packetReceived = false;
// this function is called when a complete packet is received by the module
// IMPORTANT: this function MUST be 'void' type and MUST NOT have any arguments!
void setReceiveFlag(void) {
  // we got a packet, set the flag
  packetReceived = true;
}



/***************************************************************************/
/*********************************** rtc ***********************************/
/***************************************************************************/

#include <RocketScream_LowPowerAVRZero.h>
#include <RocketScream_RTCAVRZero.h>

#define CLIENT_SYNC_TIMEOUT 15 // number of seconds for client to spend on a sync attempt
bool syncedWithServer = false;

/************ init_RTC() ************/
/*
Function to init the real time counter
*/
bool init_RTC() {
  /* true: external 32.768 kHz crystal */
  /* false: internal 32.768 kHz ULP oscillator */
  RTCAVRZero.begin(false);
}


/************************ alarms ************************/
bool timeToSleep = false;
bool justWokeUp = true;

/************ wakeup_routine ************/
/*
*/
void wakeup_routine() {
  timeToSleep = false;
  justWokeUp = true;
}

/************ bedtime_routine ************/
/*
*/
void bedtime_routine() {
  timeToSleep = true;
}

bool syncTimedOut = false;
syncTimeout_routine() {
  // if we time out, this function is called and the flag is set
  syncTimedOut = true;
}

/************ setSleepAlarm ************/
/*
Function to set RTC alarm to wake up for the next comms session
*/
void setSleepAlarm(uint16_t sleepDuration_minutes) {

  // if we are the server
  if(STATION_ID == 0) {

    // set the alarm for (SAMPLING_INTERVAL_MIN-SERVER_WAKE_DURATION)
    uint16_t secondsUntilNextAlarm = (sleepDuration_minutes - SERVER_WAKE_DURATION)*60;
    RTCAVRZero.enableAlarm(secondsUntilNextAlarm, false);

  } else {
    // otherwise we're a node, set it for the standard interval
    uint16_t secondsUntilNextAlarm = sleepDuration_minutes * 60;
    RTCAVRZero.enableAlarm(secondsUntilNextAlarm, false);
  }
  RTCAVRZero.attachInterrupt(wakeup_routine);
}


/************ setWakeAlarm ************/
/*
Function to set RTC alarm for server to go to sleep at the end of the comms session
*/
void setWakeAlarm(uint16_t wakeDurationMinutes) {
  // set the time for the server wake duration
  uint16_t secondsUntilNextAlarm = wakeDurationMinutes * 60;
  RTCAVRZero.enableAlarm(secondsUntilNextAlarm, false);
  RTCAVRZero.attachInterrupt(bedtime_routine);

  // flip our flag so we don't reset our alarm
  justWokeUp = false;
}

/************ setSyncTimout ************/
/*
Function to set RTC alarm for client attempt
*/
void setSyncTimeout(uint16_t syncTimoutSeconds) {
  syncTimedOut = false;
  RTCAVRZero.enableAlarm(syncTimoutSeconds, false);
  RTCAVRZero.attachInterrupt(syncTimeout_routine());
}



// if we're not the server
#if STATION_ID != 0
/***************************************************************************/
/********************************* station *********************************/
/***************************************************************************/



/****************************************************************************/
/****************************** temp sensors ********************************/
/****************************************************************************/

// Temp sensors
#define ONE_WIRE_BUS 24 // temp probe data line
#define TEMP_POWER 25 // temp probe power line

#include <OneWire.h>
#include <DallasTemperature.h>

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors;
DeviceAddress tempAddresses[8];

// initialize the temperature sensors
bool initTemps() {

  // set the power pin to output
  pinMode(TEMP_POWER, OUTPUT);
  // turn the power on
  digitalWrite(TEMP_POWER, HIGH);

  tempSensors = DallasTemperature(&oneWire);

  // init the sensors
  tempSensors.begin();

  // set max resolution
  tempSensors.setResolution(12);

  // get their addresses
  // for every sensor
  for (int i = 0; i < NUM_TEMP_SENSORS; i++) {

    // if error getting the address
    if (tempSensors.getAddress(tempAddresses[i], i)) {

      // print error
      Serial.print("Couldn't find sensor at index ");
      Serial.println(Serial.print(i), DEC);
    }
  }

  // shut down the sensors until we need them
  digitalWrite(TEMP_POWER, LOW);

  return true;
}

// function to read temperatures from temp sensors. provides int values of temp
// multiplied by 1000. for temp in celcius, divide by 1000.
void readTemps(float* tempData) {

  pinMode(TEMP_POWER, OUTPUT);
  digitalWrite(TEMP_POWER, HIGH);

  delay(1000);

  // init the sensors
  tempSensors.begin();

  // delay for a second
  delay(1000);

  // Call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus
  tempSensors.requestTemperatures();

  // delay for a second
  delay(1000);

  // for every sensor on the line
  for (int i = 0; i < NUM_TEMP_SENSORS; i++) {

    // get the temp
    float currTemp = tempSensors.getTempC(tempAddresses[i]);

    // if it's greater than 32 or less than -32
    if (currTemp > 32.0 || currTemp < -32.0){
      currTemp = 32.0; // set it to 32
    }

    // put it in the buffer
    tempData[i] = currTemp;
  }

  // write the power pin low
  digitalWrite(TEMP_POWER, LOW);
}



/**************************************************************************/
/********************************* pinger *********************************/
/**************************************************************************/

// pinger
#define PINGER_BUS Serial1 // serial port to read pinger
#define PINGER_POWER 22 // pinger power line
#define PINGER_TIMEOUT 100 // pinger timeout
#define N_PINGERSAMPLES 5 // number of samples to average for each pinger reading
#define MAX_PINGER_SAMPLE_ATTEMPTS 30 // maximum number of samples to attempt in order to achieve N_PINGERSAMPLES successfully


uint8_t readPinger() {

  // write the power pin high
  pinMode(PINGER_POWER, OUTPUT);
  digitalWrite(PINGER_POWER, HIGH);

  // establish serial comms with the pinger
  PINGER_BUS.begin(9600);
  delay(1000);

  // declare a short to hold the pinger data
  uint8_t pingerData;

  // query the pinger
  if (PINGER_BUS.available()) {

    char pingerReadout[6]; // a 6 byte array to hold 5 bytes read off the pinger + null terminator

    // set a timeout for reading the serial line
    PINGER_BUS.setTimeout(PINGER_TIMEOUT);

    // declare a float to hold the pinger value samples before averaging
    float runningSum = 0;

    // keep track of how many samples we've successfully collected
    int nGoodSamples = 0;

    int nLoops = 0;

    // loop while we have fewer than N_PINGERSAMPLES
    do {

      // increment our loop counter
      nLoops++;

      // get the pinger readout
      // The format of the pingers output is ascii: Rxxxx\r where x is a digit
      // Thus we can read the streaming input until we catch '\r'
      int nReturnedBytes = PINGER_BUS.readBytesUntil('\r',pingerReadout,sizeof(pingerReadout)-1);

      // add the null terminator
      pingerReadout[5] = '\0';

      // if the readout doesn't conform to the R + 4 digits format
      if ((pingerReadout[0] != 'R') || (nReturnedBytes != 5)) {

        // set the output to empty val (null terminator)
        pingerReadout[0] = '\0';

      } else {
        // then for the following four chars
        for (int i=1; i<5; i++) {

          // if any of them aren't digits
          if (!isDigit(pingerReadout[i])) {

            // set the readout to empty val (null terminator)
            pingerReadout[0] = '\0';

            // and escape
            break;
          }
        }
      }

      // if it's a nan reading
      if (pingerReadout[0] == '\0') {

        // continue to the next reading
        continue;

        // else it's numerical
      } else {

        // allocate a 4 byte array to return the cleaned pinger range + null terminator
        byte pingerRange_char[5];
        // copy the numerical data over, skipping the leading R
        for (int i=1;i<6;i++) {
          pingerRange_char[i-1] = pingerReadout[i];
        }

        // convert the pinger reading to float cm
        uint16_t pingerRange_long_mm = atoi(pingerRange_char);
        float pingerRange_float_cm = (float)pingerReturn_long_mm/10;

        // and add to the running sum
        runningSum += pingerRange_float_cm;

        // increment our counter
        nGoodSamples++;
      }

      // while we have fewer than N_PINGERSAMPLES and we haven't timed out
    } while (nGoodSamples < N_PINGERSAMPLES & nLoops < MAX_PINGER_SAMPLE_ATTEMPTS);


    // if we got the req'd number of samples before timing out
    if (nGoodSamples >= N_PINGERSAMPLES) {

      // average the running sum
      float pingerAverage_float_cm = runningSum/N_PINGERSAMPLES;

      // TODO:\ check threshold value, make sure we are not including/passing error vals. max range
      // pinger to surface
      // send back two byte (mm res) value

      // if it's greater than 255
      if (pingerAverage_float_cm > 255.0) {
        // set to 255
        pingerAverage_float_cm = 255.0;
      }

      // round off and convert to an unsigned short
      pingerData = (uint8_t) round(pingerAverage_float_cm);

    } else {
      // set to error
      pingerData = (uint8_t) 255;
    }

    // if we weren't able to talk to the pinger
  } else {

    // write 255 (error)
    pingerData = (uint8_t) 255;
  }

  // write the power pin low
  digitalWrite(PINGER_POWER, LOW);

  return pingerData;
}


/************************ end station stuff ************************/
#endif



/**************************************************************************/
/****************************** server stuff ******************************/
/**************************************************************************/
// if we're the server
#if STATION_ID == 0


/***************************************************************************/
/********************************* iridium *********************************/
/***************************************************************************/
// // if we're not deploying with the simb and we need iridium
// # if !simbDeployment
//
// #include <IridiumSBD.h>
//
// #define IRIDIUM_CS 11 // chip select pin for the iridium unit
// // #define TRANSMISSION_INTERVAL 4   // 1 = hourly, 4 = every 4 hours
// #define IRIDIUM_ATTEMPTS 3
// #define IRIDIUM_RETRY_DELAY 20000 // 20 seconds
//
// int iridiumError;
//
// IridiumSBD iridium(Serial1, IRIDIUM_CS);
//
// typedef union {
//
//   struct {
//     byte data[SIMB_DATASIZE]; // actual data
//     int32_t timestamp; // timestamp (unix time, from RTC)
//
//   } __attribute__((packed));
//
//   uint8_t bytes[0];
//
// } SBDMessage;
//
//
// SBDMessage message;
//
// void clearMessage() {
//
//   memset(message.bytes, 0, sizeof(message));
//
// }
//
//
// void configureIridium(){
//   Serial.println("Configuring the iridium");
//   iridium.attachConsole(Serial);
//   iridium.attachDiags(Serial);
//   Serial.println("after iridium begin");
//   iridium.setPowerProfile(0);
//   Serial.println("after set power profile");
//   iridium.useMSSTMWorkaround(false);
// }
//
//
//
// void iridiumOn() {
//   pinMode(IRIDIUM_CS,OUTPUT);
//   digitalWrite(IRIDIUM_CS, HIGH);
//
//   Serial.println("Turned on the iridium");
//
//   Serial1.begin(19200);
//
//   configureIridium();
//
//   Serial.println("configured the iridium");
//
//   iridium.begin();
// }
//
//
//
// void iridiumOff() {
//   digitalWrite(IRIDIUM_CS, LOW);
// }
//
//
//
// void sendIridium() {
//   iridiumError  = -1;
//
//   int retries = 0;
//
//   // while we haven't successfully transmitted or exhausted our attempts
//   while ((retries < IRIDIUM_ATTEMPTS) and (iridiumError != 0)) {
//
//     // try to send the data
//     iridiumError = iridium.sendSBDBinary(message.bytes, sizeof(message));
//
//     // if we sent succesfully, break out of the loop
//     if (iridiumError = 0) {
//       break;
//     }
//
//     // otherwise increment our retries
//     retries += 1;
//
//     // then delay for a bit
//     delay(IRIDIUM_RETRY_DELAY);
//   }
// }
//
// /************************ end iridium stuff ************************/
// # endif

/************************ end server stuff ************************/
#endif

/************************ end not shared with simb stuff ************************/
#endif

/************************ end header file ************************/
#endif
