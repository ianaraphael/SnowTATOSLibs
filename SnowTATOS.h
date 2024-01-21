/*

A library for functions common to snowtatos server and client

Updated for Rocket Scream Mini Ultra LoRa board

Ian Raphael
2023.11.28
ian.a.raphael.th@dartmouth.edu

*/
#ifndef SnowTATOS_h
#define SnowTATOS_h


/***************************************************************************/
/***************************** !USER SETTINGS! *****************************/
/***************************************************************************/
#define STATION_ID 1 // 1,2,...n. server is always 0.
#define SIMB_ID 1 // whichever SIMB buoy number this network is attached to
// #define TEST true // false for deployment
#define IRIDIUM_ENABLE false // deactivate for testing
#define simbDeployment true // true it deploying system with SIMB
static uint8_t SAMPLING_INTERVAL_MIN = 1; // sampling interval in minutes
static uint8_t SERVER_WAKE_DURATION = 4; // number of minutes for the server to stay awake
#define NUM_TEMP_SENSORS 0 // number of sensors
#define NUM_STATIONS 10 // number of nodes


#define PINGER_DATA_SIZE 2
#define TEMP_DATA_SIZE 2
#define VOLTAGE_DATA_SIZE 1
#define CLIENT_DATA_SIZE ((NUM_TEMP_SENSORS*TEMP_DATA_SIZE) + PINGER_DATA_SIZE + VOLTAGE_DATA_SIZE) // data size for each client transmission: 3temps * 2bytes + 1pinger bytes*1bytes + 1voltage * 1byte
#define SIMB_DATASIZE NUM_STATIONS*CLIENT_DATA_SIZE // datasize for the simb buffer

#define PINGER_MAX_VALUE 2000 // max value over which we do not consider data. 2000 mm (2 m) for now
#define PINGER_MIN_VALUE 500 // min value under which we don't consider data. 500 mm (50 cm) for now

#define PINGER_DEFAULT_VALUE 5000 // default "no data from node/nan" value
#define PINGER_READ_ERROR_VALUE 5001 // node returned "read error" value
#define PINGER_UNDERMIN_ERROR_VALUE 5002 // node returned under minimum threshold error value
#define PINGER_OVERMAX_ERROR_VALUE 5003 // node returned over maximum threshold error value

#define VOLTAGE_ERROR_VALUE 255 // max 8bit val (= 25.5 volts)

#define TEMP_ERROR_VALUE 32000 // +32 degrees

/****************** nothing below this line shared with SIMB ******************/
#ifndef FORSIMB


/***************************************************************************/
/********************************** board **********************************/
/***************************************************************************/

#include <RocketScream_LowPowerAVRZero.h>

/* Example on a 32-pin ATMega4808, LED on pin D7, using internal ULP OSC32K */
const uint8_t unusedPins_client[] = {0, 1, 2, 3, 4, 5, 6, 11, 13,
  14, 15, 16, 17, 18, 19, 20, 21, 23};

const uint8_t unusedPins_server[] = {0, 1, 4, 5, 6, 8, 9, 13,
  14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};

/************ Board setup ************/
void boardSetup(const uint8_t unusedPins[]) {

  analogReadResolution(12);

  // generate a random seed value off of a floating analog pin
  randomSeed(analogRead(17));

  uint8_t index;

  for (index = 0; index < sizeof(unusedPins); index++) {
    pinMode(unusedPins[index], OUTPUT);
    digitalWrite(unusedPins[index],LOW);
    LowPower.disablePinISC(unusedPins[index]);
  }

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
}


// function to get battery voltage
float readBatteryVoltage(){

  // get the voltage
  float voltage = (((float) analogRead(A5) / 1024) * 6.6);

  // and return
  return voltage;
}


// creates a default "error mask" that paints every value with the standard error value
void maskSimbData(uint8_t *simbDataBuffer) {

  // create the standard "error" buffer vals
  uint8_t tempHighByte = highByte(uint16_t(TEMP_ERROR_VALUE));
  uint8_t tempLowByte = lowByte(uint16_t(TEMP_ERROR_VALUE));
  uint8_t pingerHighByte = highByte(uint16_t(PINGER_DEFAULT_VALUE));
  uint8_t pingerLowByte = lowByte(uint16_t(PINGER_DEFAULT_VALUE));
  uint8_t voltageValue = uint8_t(VOLTAGE_ERROR_VALUE);


  // for every station
  for (int i=1;i<=NUM_STATIONS;i++){

    // get the start byte
    int tempStartByte = (i-1)*CLIENT_DATA_SIZE;

    // for every temp sensor
    for (int i2=0;i2<NUM_TEMP_SENSORS;i2++) {
      // pack the high and low byte
      simbDataBuffer[tempStartByte+(i2*2)] = tempHighByte;
      simbDataBuffer[tempStartByte+(i2*2)+1] = tempLowByte;
    }

    // get the pinger start byte
    int pingerStartByte = tempStartByte+NUM_TEMP_SENSORS*TEMP_DATA_SIZE;
    // pack the pinger high and low byte
    simbDataBuffer[pingerStartByte] = pingerHighByte;
    simbDataBuffer[pingerStartByte+1] = pingerLowByte;

    // pack the voltage byte
    simbDataBuffer[pingerStartByte+PINGER_DATA_SIZE] = voltageValue;
  }
}


// pack the provided client data into the radio transmission buffer
void packClientData(float *tempData, uint16_t pingerData, float voltage, uint8_t *dataBuffer) {

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

  // put pinger data in after temp data
  int pingerHiIndex = TEMP_DATA_SIZE*NUM_TEMP_SENSORS;
  int pingerLoIndex = pingerHiIndex+1;
  dataBuffer[pingerHiIndex] = highByte(pingerData);
  dataBuffer[pingerLoIndex] = lowByte(pingerData);

  // multiply voltage by ten and round to a short (so 3.35 v becomes 34, e.g., and later converts back to 3.4)
  uint8_t voltageByte_times10 = round(voltage*10.0);

  // put it in the buffer
  dataBuffer[pingerHiIndex + PINGER_DATA_SIZE] = voltageByte_times10;
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
uint16_t unpackPingerData(uint8_t *dataBuffer, int stnID) {

  // get the start index for this station's data
  int startByte = (stnID-1)*CLIENT_DATA_SIZE;

  // then get the pinger indices
  int pingerHiByte = startByte+(NUM_TEMP_SENSORS*TEMP_DATA_SIZE);
  int pingerLoByte = pingerHiByte + 1;

  // get the high byte
  uint8_t hiByte = dataBuffer[pingerHiByte];

  // get the low byte
  uint8_t loByte = dataBuffer[pingerLoByte];

  // put them together
  uint16_t pingerRange_mm = (hiByte << 8) | (0x00ff & loByte);

  // return the pinger value
  return pingerRange_mm;
}

// unpack voltage data data for a particular client from the simb buffer
float unpackVoltageData(uint8_t *dataBuffer, int stnID) {

  // get the pinger index (last value for a given station)
  int startByte = (stnID-1)*CLIENT_DATA_SIZE;
  int voltageIndex = startByte+(CLIENT_DATA_SIZE-VOLTAGE_DATA_SIZE);

  // unpack the byte
  uint8_t voltageByte_times10 = dataBuffer[voltageIndex];

  // turn it into a float
  float voltage = (float) voltageByte_times10/10.0;

  // return the pinger value
  return voltage;
}



/***************************************************************************/
/********************************** radio **********************************/
/***************************************************************************/

#include <RadioLib.h>
#include <Wire.h>

// server radio address is always 0
#define SERVER_ADDRESS 0
#define RADIO_ID STATION_ID
// #define CLIENT_SYNC_TIMEOUT 15
#define CLIENT_SYNC_TIMEOUT_SECS 20 // number of seconds for client to spend on a sync attempt
volatile bool syncedWithServer = false;
int nFailedTransmits = 0;
#define MAX_FAILED_TRANSMITS_TO_SYNC 2
#define SYNC_TERM SERVER_ADDRESS
#define ACK_TERM STATION_ID

// radio presets
// #define RADIO_TIMEOUT 10000 // max time to spend in radio comms in ms
#define TRANSMISSION_TIMEOUT 1000 // max time to spend on individual transmit ms
#define ACK_TIMEOUT 3000 // max time to wait for ack after transmit ms
#define MAX_TRANSMISSION_ATTEMPTS 10 // maximum number of times for a client to retry sending data
#define RADIO_FREQ 915.0 // radio frequency
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

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif

// flag to indicate that a packet was sent/received
volatile bool packet_sentOrReceived = false;
// this function is called when a complete packet is transmitted by the module
// IMPORTANT: this function MUST be 'void' type and MUST NOT have any arguments!
void setRadioFlag(void) {
  // we sent a packet, set the flag
  packet_sentOrReceived = true;
}


// radio.standby() to go to standby from sleep, or radio.startReceive()

/************ init_Radio() ************/
/*
Function to initialize the radio
*/
bool init_Radio() {

  // Serial2.print(F("[SX1276] Initializing ... "));
  // init the radio
  // default settings viewable at: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem
  int state = radio.begin(RADIO_FREQ,RADIO_BANDWIDTH, 9, 7, 0x12, RADIO_POWER, 8, 0);

  bool returnVal;

  if (state == RADIOLIB_ERR_NONE) {
    returnVal = true;
  } else {
    returnVal = false;
  }

  // set the functions that will be called when packet transmission/receival is finished
  radio.setDio0Action(setRadioFlag,RISING);

  return returnVal;
}


/*
send a packet with a timeout using radiolib
*/
bool sendPacketTimeout(byte *sendBuf,uint8_t len) {

  // start the transmission
  transmissionState = radio.startTransmit(sendBuf, len);

  // get the start time
  uint32_t transmitStartTime = millis();

  // while we haven't timed out
  while (millis()-transmitStartTime < TRANSMISSION_TIMEOUT) {

    // check if the transmission is finished
    if (packet_sentOrReceived) {

      // if it finished without error
      if (transmissionState == RADIOLIB_ERR_NONE) {
        //
        // // packet was successfully sent
        // Serial.println(F("transmission finished!"));

      } else {
        //
        // // otherwise it failed
        // Serial.print(F("failed, code "));
        // Serial.println(transmissionState);
      }

      // break out
      break;
    }
  }

  // if we failed to transmit
  if (!packet_sentOrReceived) {
    // return error
    return false;
  }

  // first reset the transmission flag
  packet_sentOrReceived = false;

  // then return true
  return true;
}


/************ client radio transmission ************/
/*
function to transmit a byte stream to server over lora

blocks until ack or timeout
returns:
0 (successful, ack'd transmit)
1 (transmit timout)
2 (listening error)
3 (ack packet read error)
4 (ack timeout error)
*/
int sendData_fromClient(uint8_t data[],uint8_t len) {

  // allocate a buffer to hold the data + 2 bytes for network/station id
  byte sendBuf[len+2];

  // put our station ID in
  sendBuf[0] = (uint8_t) SIMB_ID;
  sendBuf[1] = (uint8_t) STATION_ID;

  // copy the data in
  memcpy(&sendBuf[2], data,len);

  // start counting our attempts
  uint8_t transmissionAttempts = 0;

  // start a transmission attempt
  do {

    // try sending the packet
    if (!sendPacketTimeout(sendBuf,sizeof(sendBuf))) {
      // if we failed to transmit, return error 1 (transmit timout)
      return 1;
    }

    // otherwise, start listening
    int state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
      // return error 2 if we failed to start listening
      return 2;
    }

    // get our start time
    uint32_t ackWaitStartTime = millis();

    // while we haven't timed out waiting for the ack
    while (millis() - ackWaitStartTime < ACK_TIMEOUT) {

      // check if the flag is set
      if (packet_sentOrReceived) {

        // reset flag
        packet_sentOrReceived = false;

        // figure out how many bytes we received
        uint16_t numBytesReceived = radio.getPacketLength();

        // allocate an array to hold the data
        byte ackArray[numBytesReceived];

        // read the data into the array
        uint16_t state = radio.readData(ackArray, numBytesReceived);

        // if there was an error
        if (state != RADIOLIB_ERR_NONE) {
          // return error 3 (ack packet read error)
          return 3;
        }

        // otherwise, if the message is from the server
        if (ackArray[0] == (byte) SIMB_ID && ackArray[1] == (byte) SERVER_ADDRESS) {

          // if the ack term is valid
          if (ackArray[2] == (uint8_t) ACK_TERM) {

            // reset failed transmits count
            nFailedTransmits = 0;

            // return success (0)
            return 0;

          } else if (ackArray[2]==SYNC_TERM) {
            // otherwise it's a sync message. set synced with server true
            syncedWithServer = true;
          }
        }
        // if we entered and fell through the above control, we received a message
        // from someone other than the server or for someone else. continue listening.
      }
    }

    // get a random delay between 0.02 and 1 seconds to prevent intractable collisions
    uint16_t clientTransmitDelay = random(20,1001);
    uint32_t transmitDelayStartTime = millis();
    while(millis() - transmitDelayStartTime < clientTransmitDelay);

    // increment our transmission attempts
    transmissionAttempts++;

    // do another loop while we have not exceeded our transmission attempts
  } while (transmissionAttempts < MAX_TRANSMISSION_ATTEMPTS);

  // increment the number of failed transmits if we make it here
  nFailedTransmits++;

  // and return error 4 if we exceeded transmission attempts
  return 4;
}



/************ server radio receipt ************/
/*
function to receive a byte array from a client by radio
*/
int receiveData_fromClient(uint8_t* dataBuffer) {

  // copy the size of the buffer
  uint8_t len = sizeof(dataBuffer);

  // allocate an int to store the sender id
  int from;

  // check if the flag is set
  if (!packet_sentOrReceived) {
    // if not, assign -1 (error/no packet received)
    from = -1;

    // Serial.println("Here1");

    // otherwise, we received a packet
  } else {

    // figure out how many bytes we received
    uint16_t numBytesReceived = radio.getPacketLength();

    // allocate a hold buffer to hang on to the data for a second
    byte holdBuf[numBytesReceived];

    // read the data into the array
    uint16_t state = radio.readData(holdBuf, numBytesReceived);

    // reset the flag now that we have the data
    packet_sentOrReceived = false;

    // if there was an error
    if (state != RADIOLIB_ERR_NONE) {
      // return error/no data received
      return -1;
    }

    // if data is from a different network
    if (holdBuf[0] != SIMB_ID) {
      // return error/no data received
      return -1;
    }

    // otherwise get the sender ID
    from = holdBuf[1];

    // copy the data into the passed buffer excepting the leading byte (station ID)
    memcpy(dataBuffer,&holdBuf[2],numBytesReceived-2);

    // build an ack message
    byte ack[3];
    ack[0] = SIMB_ID; // own ID
    ack[1] = STATION_ID;
    ack[2] = from; // sender ID

    // transmit the acknowledgement
    sendPacketTimeout(ack,sizeof(ack));

    // start listening again
    radio.startReceive();
  }

  // return the sender ID
  return from;
}


/************ syncWithServer ************/
/*
function to sync up with the server
*/
bool attemptSyncWithServer() {

  // save the current sync state
  bool savedSyncState = syncedWithServer;

  // reset the sync flag
  syncedWithServer = false;

  // get the start time
  uint32_t startTime = millis();

  // while we haven't timed out, wait for a sync message from the server
  while (millis() - startTime < (CLIENT_SYNC_TIMEOUT_SECS*1000)) {

    // if we got a packet
    if (packet_sentOrReceived) {

      // reset flag
      packet_sentOrReceived = false;

      // figure out how many bytes we received
      uint16_t numBytesReceived = radio.getPacketLength();

      // allocate an array to hold the data
      byte ackArray[numBytesReceived];

      // read the data into the array
      uint16_t state = radio.readData(ackArray, numBytesReceived);

      // if the message is from our server
      if (ackArray[0] == (byte) SIMB_ID && ackArray[1] == (byte) SERVER_ADDRESS) {
        // if it's a sync message
        if (ackArray[2]==SYNC_TERM) {
          // set our flag true
          syncedWithServer = true;
          return true;
        }
      }
    }
  }

  // otherwise, reset the state to whatever it was before and return failed sync
  syncedWithServer = savedSyncState;
  return false;
}




/***************************************************************************/
/*********************************** rtc ***********************************/
/***************************************************************************/

#include <RocketScream_RTCAVRZero.h>

/************ init_RTC() ************/
/*
Function to init the real time counter
*/
bool init_RTC() {
  /* true: external 32.768 kHz crystal */
  /* false: internal 32.768 kHz ULP oscillator */
  RTCAVRZero.begin(true);
}


/************************ alarms ************************/
bool timeToSleep = false;
bool justWokeUp = true;
bool sleepAlarmActivated = false; // sleep alarm activated

/************ wakeup_routine ************/
/*
*/
void wakeup_routine() {
  timeToSleep = false;
  sleepAlarmActivated = false;
  justWokeUp = true;
}

/************ bedtime_routine ************/
/*
sets the timetosleep flag when the wakealarm rings
*/
void bedtime_routine() {
  timeToSleep = true;
}

// bool syncTimedOut = false;
// syncTimeout_routine() {
//   // if we time out, this function is called and the flag is set
//   syncTimedOut = true;
// }

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
  sleepAlarmActivated = true;
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

// /************ setSyncTimout ************/
// /*
// Function to set RTC alarm for client attempt
// */
// void setSyncTimeout(uint16_t syncTimoutSeconds) {
//   syncTimedOut = false;
//   RTCAVRZero.enableAlarm(syncTimoutSeconds, false);
//   RTCAVRZero.attachInterrupt(syncTimeout_routine());
// }



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
#define PINGER_POWERUP_DELAY 500 // ms to allow pinger startup (maxbotix spec'd ~156 ms)
#define PINGER_TIMEOUT 1000 // pinger TTL serial timeout
#define PINGER_SAMPLING_DELAY 200 // ms between TTL serial read attempts (mb spec'd sampling period of ~150 ms)
#define N_PINGERSAMPLES 10 // number of samples to average for each pinger reading
#define MAX_PINGER_SAMPLE_ATTEMPTS 100 // maximum number of samples to attempt in order to achieve N_PINGERSAMPLES successfully

// returns pinger range in mm
uint16_t readPinger() {

  // write the power pin high
  pinMode(PINGER_POWER, OUTPUT);
  digitalWrite(PINGER_POWER, HIGH);

  // let the pinger get through its powerup routine
  delay(PINGER_POWERUP_DELAY);

  // make sure we're on the right serial pins
  PINGER_BUS.pins(8,9);

  // begin our serial
  PINGER_BUS.begin(9600);

  // wait five seconds for serial comms
  delay(10*PINGER_POWERUP_DELAY);

  // declare a short to hold the pinger data
  uint16_t pingerData;

  // query the pinger
  if (PINGER_BUS.available()>0) {

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

        // set return value to read error
        pingerData = (uint16_t) PINGER_READ_ERROR_VALUE;

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

        // convert from ascii to an unsigned 16bit int
        uint16_t pingerRange_mm = atoi(pingerRange_char);

        // if it's greater than our max value
        if (pingerRange_mm > PINGER_MAX_VALUE){

          // set it to the over max error value
          pingerData = (uint16_t) PINGER_OVERMAX_ERROR_VALUE;

          // and continue before adding to the running sum
          continue;

          // or less than our threshold value
        } else if (pingerRange_mm <  PINGER_MIN_VALUE) {

          // set it to the under min error value
          pingerData = PINGER_UNDERMIN_ERROR_VALUE;

          // and continue before adding to the running sum
          continue;
        }

        // otherwise add to the running sum
        runningSum += pingerRange_mm;

        // and increment our counter
        nGoodSamples++;
      }

      // delay for a moment
      delay(PINGER_SAMPLING_DELAY);

      // while we have fewer than N_PINGERSAMPLES and we haven't maxed out attempts
    } while (nGoodSamples < N_PINGERSAMPLES && nLoops < MAX_PINGER_SAMPLE_ATTEMPTS);


    // if we got the req'd number of samples before timing out
    if (nGoodSamples >= N_PINGERSAMPLES) {

      // average the running sum
      float pingerAverage_float_mm = (float)runningSum/(float)nGoodSamples;

      // if it's greater than our max value
      if (pingerAverage_float_mm > PINGER_MAX_VALUE) {

        // set it to the over max error value
        pingerAverage_float_mm = PINGER_OVERMAX_ERROR_VALUE;

        // or less than threshold
      } else if (pingerAverage_float_mm < PINGER_MIN_VALUE) {

        // set it to the under min error value
        pingerAverage_float_mm = PINGER_UNDERMIN_ERROR_VALUE;
      }

      // round off and convert to an unsigned 16bit
      pingerData = (uint16_t) round(pingerAverage_float_mm);
    }
    // else {
    //   // set to error
    //   pingerData = (uint16_t) PINGER_ERROR_VALUE;
    // }

    // if we weren't able to talk to the pinger
  } else {

    // write the read error value
    pingerData = (uint16_t) PINGER_READ_ERROR_VALUE;
  }

  // write the power pin low
  digitalWrite(PINGER_POWER, LOW);

  // end the serial
  PINGER_BUS.end();

  // pull the serial pins low
  pinMode(8,OUTPUT);
  digitalWrite(8,LOW);
  pinMode(9,INPUT);
  digitalWrite(9,LOW);

  // return the data
  return pingerData;
}


/************************ end station stuff ************************/
#endif



/**************************************************************************/
/****************************** server stuff ******************************/
/**************************************************************************/
// if we're the server
// #if STATION_ID == 0


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
// #endif

/************************ end not shared with simb stuff ************************/
#endif

/************************ end header file ************************/
#endif
