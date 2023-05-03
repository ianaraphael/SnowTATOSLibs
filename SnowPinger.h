/*

  A library for handling maxbotix snow pingers in temperature datalogger
  deployments. Allows creation and deletion of snow pinger object with a unique
  datafile filename, reading snow pinger, etc.

  Ian Raphael
  2023.03.23
  ian.a.raphael.th@dartmouth.edu

*/

#ifndef SnowPinger_h
#define SnowPinger_h

#include "SPI.h"
#include "dataFile.h"

#define PINGER_TIMEOUT 100 // timeout to prevent hang in pinger reading (ms)


/*************** SnowPinger class ***************/
// define a class for the snow pinger
class SnowPinger {

  // make public access for everything
public:

  /* Attributes */
  int powerPin; // power pin for the pinger
  HardwareSerial *pingerBus; // Serial pin for the pinger

  int stationID; // measurement station ID

  String filename = ""; // a filename for the data file
  String* headerInformation; // the header information for the data file
  int numHeaderLines = 3;

  /*************** SnowPinger object destructor ***************/
  // destroy a SnowPinger object. Frees the dynamically allocated memory (called
  // automatically once object goes out of scope)
  ~SnowPinger() {

    // deallocate header info memory
    delete [] headerInformation;
    // zero out the address
    headerInformation = 0;
  }


  /*************** SnowPinger object constructor ***************/
  // Constructor. This takes in the input information and creates the header
  // information for the data file.
  // IMPT: This uses dynamically allocated memory via `new`! You _must_ free
  // the header information via the SnowPinger destructor method when you are done.
  SnowPinger(HardwareSerial *pinger_Bus, int power_pin, int station_ID) {

    // save the pinger bus
    this->pingerBus = pinger_Bus;

    // save the power pin
    powerPin = power_pin;

    // save the station id
    stationID = station_ID;

    // power up the pinger
    pinMode(powerPin, OUTPUT);
    digitalWrite(powerPin, HIGH);

    // try talking to the pinger
    pingerBus->begin(9600);
    delay(800);

    if (!pingerBus->available()) {
      // print error
      Serial.println("Error opening snow pinger serial comms.");
    }

    // now create the data filename
    filename += "stn";
    filename += stationID;
    filename += "_p.txt"; // p for pinger

    // define the header information
    headerInformation = new String[numHeaderLines];

    headerInformation[0] = "Snow pinger";

    headerInformation[1] = "Station ID: ";
    headerInformation[1] += stationID;

    headerInformation[2] = "Date, Time";
    headerInformation[2] += ", Snow pinger range [cm]";

    // create a new file
    newFile(filename,headerInformation,numHeaderLines);

    // shut down the pinger until we need it
    digitalWrite(powerPin, LOW);
  }


  /*************** SnowPinger readSnowPinger ***************/
  // Reads a snow pinger, returning a string with a timestamp and the
  // range value [cm] for the snow pinger
  // inputs: timestamp
  String readSnowPinger_dateStamp(String date, String time, int nPingerSamples, int maxSampleAttempts) {

    // write the power pin high
    pinMode(powerPin, OUTPUT);
    digitalWrite(powerPin, HIGH);

    // establish serial comms with the pinger
    pingerBus->begin(9600);
    delay(1000);

    // declare a string to hold the read data
    String readString = "";

    // throw the timestamp on there
    readString = date + ", " + time;

    // add our comma
    readString += ", ";

    // query the pinger
    if (pingerBus->available()) {

      String pingerReadout; // a string to hold what's read off the pinger
      String pingerReturn; // a string to return the cleaned pinger info

      // TODO: Include a timer here to protect against errors. We don't want to get into
      // a situation where the pinger is just spitting back garbage (so Serial1.available() still
      // returns true) but this just hangs the board since it never spits out '\r'
      // and possibly runs us out of memory.
      // set a timeout for reading the serial line
      pingerBus->setTimeout(PINGER_TIMEOUT);

      // declare a float to hold the pinger value samples before averaging
      float runningSum = 0;

      // keep track of how many samples we've successfully collected
      int nGoodSamples = 0;

      int nLoops = 0;

      // loop while we have fewer than nPingerSamples
      do {

        // increment our loop counter
        nLoops++;

        // get the pinger readout
        // The format of the pingers output is ascii: Rxxxx\r where x is a digit
        // Thus we can read the streaming input until we catch '\r'
        pingerReadout = pingerBus->readStringUntil('\r');

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
          // // add it directly to the readString
          // readString += pingerReturn;

          // continue to the next reading
          continue;

          // else it's numerical
        } else {

          // convert the pinger reading to float cm
          unsigned long pingerReturn_long_mm = pingerReturn.toInt();
          float pingerReturn_float_cm = (float)pingerReturn_long_mm/10;

          // and add to the running sum
          runningSum += pingerReturn_float_cm;

          // increment our counter
          nGoodSamples++;
        }

        // while we have fewer than nPingerSamples and we haven't timed out
      } while (nGoodSamples < nPingerSamples & nLoops < maxSampleAttempts);

      // if we failed to get the req'd number of samples before timing out
      if (nGoodSamples < nPingerSamples) {

        // set readstring to NaN
        readString += "NaN";

        Serial.println("Failed to get req'd samples.");

      } else {

        // average the running sum
        float pingerAverage_float_cm = runningSum/nPingerSamples;

        // round off and convert to int
        int pingerAverage_int_cm = (int) round(pingerAverage_float_cm);

        // then add to the string
        readString += pingerAverage_int_cm;
      }

      // if we weren't able to talk to the pinger
    } else {

      // write a nan to the data string
      readString += "NaN";
    }

    // write the power pin low
    digitalWrite(powerPin, LOW);

    // return the data string
    return readString;
  }
};

#endif
