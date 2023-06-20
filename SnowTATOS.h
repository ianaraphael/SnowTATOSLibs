/*

A library for functions common to snowtatos ArcWatch_v1 server and client

Ian Raphael
2023.06.16
ian.a.raphael.th@dartmouth.edu

*/
#ifndef SnowTATOS_h
#define SnowTATOS_h


#define STATION_ID 0 // server is always 0

uint8_t SAMPLING_INTERVAL_MIN = 1; // one minute sampling
uint8_t SERVER_WAKE_DURATION = 10; // number of minutes for the server to stay awake

#define NUM_TEMP_SENSORS 3 // number of sensors
#define NUM_STATIONS 10 // number of nodes

#define CLIENT_DATA_SIZE 7 // data size for each client transmission: 6temps * 2bytes + 1pinger*1byte

#define SIMB_DATASIZE NUM_STATIONS*CLIENT_DATA_SIZE // datasize for the simb buffer

/************************ board ************************/


const int flashChipSelect = 4;
#include <SerialFlash.h>

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
#define RADIO_TIMEOUT 5000 // max wait time for radio transmissions in ms
#define RADIO_FREQ 915.0 // radio frequency
#define RADIO_POWER 23 // max power

// Radio pins
#define RADIO_CS 5 // radio chip select pin
#define RADIO_INT 2 // radio interrupt pin

// server radio address is always 0
#define SERVER_ADDRESS 0
#define RADIO_ID STATION_ID

#define MAX_TRANSMISSION_ATTEMPTS 3 // maximum number of times for a client to retry sending data



RH_RF95 driver(RADIO_CS, RADIO_INT); // Singleton instance of the radio driver
RHReliableDatagram manager(driver, RADIO_ID); // Class to manage message delivery and receipt, using the driver declared above

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
}

/************ client radio transmission ************/
/*
function to transmit a byte stream to server over rf95 radio
*/
bool sendData_fromClient(uint8_t *data) {

  // set number of transmission attempts to 0
  int nTransmissionAttempts = 0;

  do {
    SerialUSB.println("Attempting to send a message to the server");

    // send the data to the server
    if (manager.sendtoWait(data, CLIENT_DATA_SIZE, SERVER_ADDRESS)) {
      SerialUSB.println("Received receipt acknowledgement from server");

      // successfully transmitted
      return true;
    } else {
      // the server did not recieve the message. we'll try again after a random delay
      SerialUSB.println("Server failed to acknowledge receipt");

      // increment the number of attempts we've made
      nTransmissionAttempts++;

      // build in a random delay between 1 and 20 seconds based on a multiple of stationID
      delay(STATION_ID*100*random(10,21));

      // continue next cycle of the loop
      continue;
    }
    // try again if we failed and haven't reached max attempts
  } while (nTransmissionAttempts < MAX_TRANSMISSION_ATTEMPTS);

  // failed to transmit
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

  // otherwise return -1 (no message recieved)
  return -1;
}




/************************ rtc ************************/


#include <TimeLib.h>
#include <RTCZero.h>
RTCZero rtc; // real time clock object

uint8_t ALARM_MINUTES = 0; // minute to sample on

bool synchronizedWithNetwork = false;

/************ getTime() ************/
/*
Function to parse time from system __TIME__ string. From Paul Stoffregen
DS1307RTC library example SetTime.ino
*/
bool getTime(const char *str, uint8_t* timeArray) {
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;

  // pack everything into a return array
  timeArray[1] = (uint8_t) Hour;
  timeArray[2] = (uint8_t) Min;
  timeArray[3] = (uint8_t) Sec;

  return true;
}

/************ getDate() ************/
/*
Function to parse date from system __TIME__ string. Modified from Paul
Stoffregen's DS1307RTC library example SetTime.ino
*/
bool getDate(const char *str, uint8_t *dateArray) {
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  const char *monthName[12] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  };

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;

  // convert the int year into a string
  String twoDigitYear = String(Year);
  // extract the last two digits of the year
  twoDigitYear = twoDigitYear.substring(2);

  // pack everything into a return array as uint8_ts
  dateArray[1] = (uint8_t) twoDigitYear.toInt();
  dateArray[2] = monthIndex + 1;
  dateArray[3] = (uint8_t) Day;

  return true;
}

/************ init_RTC() ************/
/*
Function to init the RTC and sync to compile-time timestamp
*/
bool init_RTC() {

  // get the date and time the compiler was run
  uint8_t dateArray[3];
  uint8_t timeArray[3];
  getDate(__DATE__, dateArray);
  getTime(__TIME__, timeArray);

  rtc.begin();
  rtc.setTime(timeArray[1], timeArray[2], timeArray[3]);
  rtc.setDate(dateArray[3], dateArray[2], dateArray[1]);
}




/************************ temp sensors ************************/



// Temp sensors
#define ONE_WIRE_BUS 7 // temp probe data line
#define TEMP_POWER 8 // temp probe power line

#include "OneWire.h"
#include "DallasTemperature.h"

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors = DallasTemperature(&oneWire);
uint8_t tempAddresses[NUM_TEMP_SENSORS][8];

// initialize the temperature sensors
void initTemps() {

  // set the power pin to output
  pinMode(TEMP_POWER, OUTPUT);
  // turn the power on
  digitalWrite(TEMP_POWER, HIGH);

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

      // set to error
      pingerData = (uint8_t) 255;

    } else {

      // average the running sum
      float pingerAverage_float_cm = runningSum/N_PINGERSAMPLES;

      // round off and convert to int
      pingerData = (uint8_t) round(pingerAverage_float_cm);
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




/************************ alarms ************************/


/************ alarm_one_routine ************/
/*
dummy routine.
*/
void alarm_one_routine() {
}



/************ setAlarm_client ************/
/*
Function to set RTC alarm

takes:
bool initFlag: indicates whether this is the initial instance of the alarm.
If it is, then set the sample times as zero to sample first at the top of the interval
*/
// bool initFlag
void setAlarm_client() {

  // always sample on the 0th second
  rtc.setAlarmSeconds(0);

  // if we're sampling at less than an hourly rate
  if (SAMPLING_INTERVAL_MIN < 60) {

    // if we've rolled over
    if (ALARM_MINUTES >= 60) {
      // reset
      ALARM_MINUTES = ALARM_MINUTES - 60;
    }

    // set for the next elapsed interval
    ALARM_MINUTES = rtc.getMinutes() + SAMPLING_INTERVAL_MIN;
    rtc.setAlarmMinutes(ALARM_MINUTES);

    // and enable the alarm to match
    rtc.enableAlarm(rtc.MATCH_MMSS);

    // otherwise
  } else if(SAMPLING_INTERVAL_MIN >= 60) {

    // figure out how long our interval is in hours
    uint8_t SAMPLING_INTERVAL_HOUR = SAMPLING_INTERVAL_MIN/60;

    // now figure out which sampling interval we're in
    uint8_t currSamplingHour = (floor(rtc.getHours()/SAMPLING_INTERVAL_HOUR)*SAMPLING_INTERVAL_HOUR);

    // and add an interval to get to the next one
    uint8_t ALARM_HOURS = SAMPLING_INTERVAL_HOUR;

    // if it's supposed to be midnight
    if (ALARM_HOURS == 24) {
      // make it 0
      ALARM_HOURS = 0;
    }

    // sample on the 0th minute
    rtc.setAlarmMinutes(0);
    // and set hours
    rtc.setAlarmHours(ALARM_HOURS);

    // and enable the alarm to match
    rtc.enableAlarm(rtc.MATCH_HHMMSS);
  }

  rtc.attachInterrupt(alarm_one_routine);
}



/************ setAlarm_server ************/
// Function to set RTC wakeup alarm for server side
void setAlarm_server() {

  // always wake up on the 0th second
  rtc.setAlarmSeconds(0);

  // if we're sampling at less than an hourly rate
  if (SAMPLING_INTERVAL_MIN < 60) {

    // if we've rolled over
    if (ALARM_MINUTES >= 60) {
      // reset
      ALARM_MINUTES = ALARM_MINUTES - 60;
    }

    // get the next elapsed interval
    ALARM_MINUTES = rtc.getMinutes() + SAMPLING_INTERVAL_MIN;

    // then set the alarm with awake period centered around ALARM_MINUTES
    rtc.setAlarmMinutes(ALARM_MINUTES-floor(SERVER_WAKE_DURATION/2));

    // finally, enable the alarm to match
    rtc.enableAlarm(rtc.MATCH_MMSS);

    // otherwise
  } else if(SAMPLING_INTERVAL_MIN >= 60) {

    // figure out how long our interval is in hours
    uint8_t SAMPLING_INTERVAL_HOUR = SAMPLING_INTERVAL_MIN/60;

    // now figure out which sampling interval we're in
    uint8_t currSamplingHour = (floor(rtc.getHours()/SAMPLING_INTERVAL_HOUR)*SAMPLING_INTERVAL_HOUR);

    // and add an interval to get to the next one
    uint8_t ALARM_HOURS = SAMPLING_INTERVAL_HOUR;

    // now subtract 1 because we're going to wake up SERVER_WAKE_DURATION/2 minutes before the hour
    ALARM_HOURS = ALARM_HOURS - 1;

    // if it's supposed to be midnight
    if (ALARM_HOURS == 24) {
      // make it 0
      ALARM_HOURS = 0;
    }

    // now get the alarm minutes as 60 - SERVER_WAKE_DURATIONOD/2
    ALARM_MINUTES = 60 - ceil(SERVER_WAKE_DURATION/2);

    // set minutes
    rtc.setAlarmMinutes(ALARM_MINUTES);
    // set hours
    rtc.setAlarmHours(ALARM_HOURS);

    // and enable the alarm to match
    rtc.enableAlarm(rtc.MATCH_HHMMSS);
  }

  rtc.attachInterrupt(alarm_one_routine);
}

#endif
