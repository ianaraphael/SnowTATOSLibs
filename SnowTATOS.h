/*

A library for functions common to snowtatos server and client

Updated for Rocket Scream Mini Ultra LoRa board

Ian Raphael
2023.11.28
ian.a.raphael.th@dartmouth.edu

*/
#ifndef SnowTATOS_h
#define SnowTATOS_h


#define STATION_ID 1 // server is always 0
#define TEST true // false for deployment
#define IRIDIUM_ENABLE false // deactivate for testing
#define simbDeployment true // true it deploying system with SIMB
static uint8_t SAMPLING_INTERVAL_MIN = 120; // sampling interval in minutes
static uint8_t SERVER_WAKE_DURATION = 4; // number of minutes for the server to stay awake

#define NUM_TEMP_SENSORS 0 // number of sensors
#define NUM_STATIONS 9 // number of nodes

#define CLIENT_DATA_SIZE NUM_TEMP_SENSORS*2 + 1 // data size for each client transmission: 3temps * 2bytes + 1pinger*1byte

#define SIMB_DATASIZE NUM_STATIONS*CLIENT_DATA_SIZE // datasize for the simb buffer

/************************ board ************************/

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


// TODO: update mask to be flexible with/without temperature sensors
// creates a default "error mask" that paints every value with the standard error value
void maskSimbData(uint8_t *simbDataBuffer) {

  // create the standard "error" buffer
  uint8_t tempHighByte = highByte(uint16_t(32000));
  uint8_t tempLowByte = lowByte(uint16_t(32000));
  uint8_t pingerValue = uint8_t(255);

  // for every station
  for (int i=1;i<=NUM_STATIONS;i++){

    // paint error values
    int startByte = (i-1)*CLIENT_DATA_SIZE;
    simbDataBuffer[startByte] = tempHighByte;
    simbDataBuffer[startByte+1] = tempLowByte;
    simbDataBuffer[startByte+2] = tempHighByte;
    simbDataBuffer[startByte+3] = tempLowByte;
    simbDataBuffer[startByte+4] = tempHighByte;
    simbDataBuffer[startByte+5] = tempLowByte;
    simbDataBuffer[startByte+6] = pingerValue;
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
    int currTemp = tempData[i] * 1000;

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


/************************ radio ************************/


#include "RHReliableDatagram.h"
#include "RH_RF95.h"

// radio presets
#define RADIO_TIMEOUT 10000 // max wait time for radio transmissions in ms
#define RADIO_FREQ 915.0 // radio frequency
#define RADIO_POWER 23 // max power

// Radio pins
#define RADIO_CS 5 // radio chip select pin
#define RADIO_INT 2 // radio interrupt pin

// server radio address is always 0
#define SERVER_ADDRESS 0
#define RADIO_ID STATION_ID

#define MAX_TRANSMISSION_ATTEMPTS 5 // maximum number of times for a client to retry sending data

RH_RF95 driver(RADIO_CS, RADIO_INT); // Singleton instance of the radio driver
RHReliableDatagram manager(driver, RADIO_ID); // Class to manage message delivery and receipt, using the driver declared above

/************ init_Radio() ************/
/*
Function to initialize the radio
*/
bool init_Radio() {

  // wait while the radio initializes
  manager.init();
  delay(1000);

  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  driver.setTxPower(RADIO_POWER, false);
  driver.setFrequency(RADIO_FREQ);

  manager.setTimeout(RADIO_TIMEOUT); // set timeout
  manager.setRetries(MAX_TRANSMISSION_ATTEMPTS);

  return true;
}

/************ client radio transmission ************/
/*
function to transmit a byte stream to server over rf95 radio
*/
bool sendData_fromClient(uint8_t *data) {

  if(manager.sendtoWait(data,CLIENT_DATA_SIZE, SERVER_ADDRESS)){
    return true;
  } else {
    return false;
  }
}


/************ syncWithServer ************/
/*
function to sync up with the server
*/
bool attemptSyncWithServer() {

  // if the manager isn't busy right now
  if (manager.available()) {

    // allocate a throwaway buffer to put the message in (we don't need it)
    buf[2];

    // allocate a short to store station id
    uint8_t from;

    // if we've gotten a message
    if (manager.recvfromAck(buf, sizeof(buf), &from)) {

      // if it's from the server
      if (from == 0) {
        // we've synced with the server
        return true;
      }
    }
  }
  // we have not synced with the server
  return false;
}

/************ server radio receipt ************/
/*
function to receive a byte stream from a client by radio
*/
int receiveData_fromClient(uint8_t* dataBuffer) {

  // // memset the buffer to 0s to make sure we're
  // memset(dataBuffer,0,sizeof(dataBuffer));

  // if the manager isn't busy right now
  if (manager.available()) {

    // copy the size of the buffer
    uint8_t len = CLIENT_DATA_SIZE;

    // allocate a short to store station id
    uint8_t from;

    // if we've gotten a message, receive and store it, its length, and station id
    if (manager.recvfromAck(dataBuffer, &len, &from)) {

      // and return the station id
      return from;
    }
  }

  // otherwise return -1 (no message received)
  return -1;
}




/************************ rtc ************************/


#include <RocketScream_LowPowerAVRZero.h>
#include <RocketScream_RTCAVRZero.h>

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
bool timeToSleep_server = false;

/************ alarm_one_routine ************/
/*
*/
void alarm_one_routine() {
  // record the time at which we woke up
  timeToSleep_server = false;
}

/************ alarm_two_routine ************/
/*
*/
void alarm_two_routine() {
  timeToSleep_server = true;
}

/************ setSleepAlarm ************/
/*
Function to set RTC alarm to wake up for the next comms session
*/
void setSleepAlarm() {

  // if we are the server
  if(station_ID == 0) {
    // set the alarm for (SAMPLING_INTERVAL_MIN-SERVER_WAKE_DURATION)
    uint16_t secondsUntilNextAlarm = (SAMPLING_INTERVAL_MIN - SERVER_WAKE_DURATION)*60;
    RTCAVRZero.enableAlarm(secondsUntilNextAlarm, false);
  } else {
    // otherwise we're a node, set it for the standard interval
    uint16_t secondsUntilNextAlarm = SAMPLING_INTERVAL_MIN * 60;
    RTCAVRZero.enableAlarm(secondsUntilNextAlarm, false);
  }
  RTCAVRZero.attachInterrupt(alarm_one_routine());
  // TODO: maybe go into standby mode here
  // if we're the server, enable both the pin interrupt and the alarm
}


/************ setWakeAlarm ************/
/*
Function to set RTC alarm for server to go to sleep at the end of the comms session
*/
void setWakeAlarm() {
  // set the time for the server wake duration
  uint16_t secondsUntilNextAlarm = SERVER_WAKE_DURATION * 60;
  RTCAVRZero.enableAlarm(secondsUntilNextAlarm, false);
  RTCAVRZero.attachInterrupt(alarm_two_routine());
}



// if we're not the server
#if STATION_ID != 0
/************************ station stuff ************************/



/************************ temp sensors ************************/

// Temp sensors
#define ONE_WIRE_BUS 7 // temp probe data line
#define TEMP_POWER 8 // temp probe power line

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


/************************ pinger ************************/

// pinger
#define PINGER_BUS Serial1 // serial port to read pinger
#define PINGER_POWER 11 // pinger power line
#define PINGER_TIMEOUT 100 // pinger timeout
#define N_PINGERSAMPLES 5 // number of samples to average for each pinger reading
#define MAX_PINGER_SAMPLE_ATTEMPTS 30 // maximum number of samples to attempt in order to achieve N_PINGERSAMPLES successfully


uint8_t readPinger() {

  // write the power pin high
  pinMode(PINGER_POWER, OUTPUT);
  digitalWrite(PINGER_POWER, HIGH);

  // establish serial comms with the pinger
  // pingerBus->begin(9600);
  PINGER_BUS.begin(9600);
  delay(1000);

  // declare a short to hold the pinger data
  uint8_t pingerData;

  // query the pinger
  if (PINGER_BUS.available()) {

    String pingerReadout; // a string to hold what's read off the pinger
    String pingerReturn; // a string to return the cleaned pinger info

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
      pingerReadout = PINGER_BUS.readStringUntil('\r');

      // copy pinger readout into the return string starting after the first char
      pingerReturn = pingerReadout.substring(1);

      // if the readout doesn't conform to the R + 4 digits format
      if ((pingerReadout[0] != 'R') || (pingerReadout.length()!= 5)) {

        // set the output to string "NaN"
        pingerReturn = "NaN";

      } else {
        // then for the following four chars
        for (int i=1; i<5; i++) {

          // if any of them aren't digits
          if (!isdigit(pingerReadout[i])) {

            // set the output to string "NaN"
            pingerReturn = "NaN";

            // and escape
            break;
          }
        }
      }

      // if it's a nan reading
      if (pingerReturn == "NaN") {

        // continue to the next reading
        continue;

        // else it's numerical
      } else {

        // convert the pinger reading to float cm
        uint16_t pingerReturn_long_mm = pingerReturn.toInt();
        float pingerReturn_float_cm = (float)pingerReturn_long_mm/10;

        // and add to the running sum
        runningSum += pingerReturn_float_cm;

        // increment our counter
        nGoodSamples++;
      }

      // while we have fewer than N_PINGERSAMPLES and we haven't timed out
    } while (nGoodSamples < N_PINGERSAMPLES & nLoops < MAX_PINGER_SAMPLE_ATTEMPTS);


    // if we got the req'd number of samples before timing out
    if (nGoodSamples >= N_PINGERSAMPLES) {

      // average the running sum
      float pingerAverage_float_cm = runningSum/N_PINGERSAMPLES;

      // if it's greater than 255
      if (pingerAverage_float_cm > 255.0) {
        // set to 255
        pingerAverage_float_cm = 255.0;
      }

      // round off and convert to int
      pingerData = (uint8_t) round(pingerAverage_float_cm);

    } else {
      // set to error
      pingerData = (uint8_t) 255;
    }

    // if we weren't able to talk to the pinger
  } else {

    // write 255 to the data string (error)
    pingerData = (uint8_t) 255;
  }

  // write the power pin low
  digitalWrite(PINGER_POWER, LOW);

  return pingerData;
}


/************************ end station stuff ************************/
#endif



/************************ server stuff ************************/
// if we're the server
#if STATION_ID == 0

/************************ iridium ************************/

// if we're not deploying with the simb and we need iridium
# if !simbDeployment

#include <IridiumSBD.h>

#define IRIDIUM_CS 11 // chip select pin for the iridium unit
// #define TRANSMISSION_INTERVAL 4   // 1 = hourly, 4 = every 4 hours
#define IRIDIUM_ATTEMPTS 3
#define IRIDIUM_RETRY_DELAY 20000 // 20 seconds

int iridiumError;

IridiumSBD iridium(Serial1, IRIDIUM_CS);

typedef union {

  struct {
    byte data[SIMB_DATASIZE]; // actual data
    int32_t timestamp; // timestamp (unix time, from RTC)

  } __attribute__((packed));

  uint8_t bytes[0];

} SBDMessage;


SBDMessage message;

void clearMessage() {

  memset(message.bytes, 0, sizeof(message));

}


void configureIridium(){
  Serial.println("Configuring the iridium");
  iridium.attachConsole(Serial);
  iridium.attachDiags(Serial);
  Serial.println("after iridium begin");
  iridium.setPowerProfile(0);
  Serial.println("after set power profile");
  iridium.useMSSTMWorkaround(false);
}



void iridiumOn() {
  pinMode(IRIDIUM_CS,OUTPUT);
  digitalWrite(IRIDIUM_CS, HIGH);

  Serial.println("Turned on the iridium");

  Serial1.begin(19200);

  configureIridium();

  Serial.println("configured the iridium");

  iridium.begin();
}



void iridiumOff() {
  digitalWrite(IRIDIUM_CS, LOW);
}



void sendIridium() {
  iridiumError  = -1;

  int retries = 0;

  // while we haven't successfully transmitted or exhausted our attempts
  while ((retries < IRIDIUM_ATTEMPTS) and (iridiumError != 0)) {

    // try to send the data
    iridiumError = iridium.sendSBDBinary(message.bytes, sizeof(message));

    // if we sent succesfully, break out of the loop
    if (iridiumError = 0) {
      break;
    }

    // otherwise increment our retries
    retries += 1;

    // then delay for a bit
    delay(IRIDIUM_RETRY_DELAY);
  }
}
# endif

/************************ end server stuff ************************/
#endif

/************************ end header file ************************/
#endif
