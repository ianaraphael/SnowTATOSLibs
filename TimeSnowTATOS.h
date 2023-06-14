#ifndef TimeSnowTATOS_h
#define TimeSnowTATOS_h

#include <TimeLib.h>
#include <RTCZero.h>

RTCZero rtc; // real time clock object
// uint8_t SAMPLING_INTERVAL_MIN = 240; // four hour sampling
uint8_t SAMPLING_INTERVAL_MIN = 1; // one minute sampling
uint8_t ALARM_MINUTES = 0; // minute to sample on
uint8_t AWAKE_PERIOD = 10; // number of minutes to stay awake

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
Function to sync the RTC to compile-time timestamp
*/
bool init_RTC() {

  // get the date and time the compiler was run
  uint8_t dateArray[3];
  uint8_t timeArray[3];

  if (getDate(__DATE__, dateArray) && getTime(__TIME__, timeArray)) {

    rtc.begin();
    rtc.setTime(timeArray[1], timeArray[2], timeArray[3]);
    rtc.setDate(dateArray[3], dateArray[2], dateArray[1]);

  } else { // if failed, hang
    SerialUSB.println("failed to init RTC");
    while (1);
  }
}

/************ getDateString() ************/
/*
Function to get the current datestring from RTC
*/
String getDateString(void) {

  // get date and timestamps
  String dateString = "";
  dateString = String(20) + String(rtc.getYear()) + ".";

  int month = rtc.getMonth();
  int day = rtc.getDay();

  // append months and days to datestamp, 0 padding if necessary
  if (month < 10) {
    dateString += String(0) + String(month) + ".";
  } else {
    dateString += String(month) + ".";
  }

  if (day < 10) {
    dateString += String(0) + String(day);
  } else {
    dateString += String(day);
  }

  // return the datestring
  return dateString;
}

/************ getTimeString() ************/
/*
Function to get the current timestring from RTC
*/
String getTimeString(void) {

  // allocate an empty string
  String timeString = "";
  int hours = rtc.getHours();
  int mins = rtc.getMinutes();
  int secs = rtc.getSeconds();

  // append hours, mins, secs to the timestamp, 0 padding if necessary
  if (hours < 10) {
    timeString = String(0) + String(hours) + ":";
  } else {
    timeString = String(hours) + ":";
  }

  if (mins < 10) {
    timeString += String(0) + String(mins) + ":";
  } else {
    timeString += String(mins) + ":";
  }

  if (secs < 10) {
    timeString += String(0) + String(secs);
  } else {
    timeString += String(secs);
  }

  // return the timestring
  return timeString;
}

#endif
