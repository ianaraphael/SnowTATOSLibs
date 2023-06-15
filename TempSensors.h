/*

  A library for handling ds18b20 temp sensors in temperature datalogger
  deployments. Allows creation and deletion of temp sensors object with a unique
  datafile filename, reading multiple temp sensors, etc.

  Ian Raphael
  2023.03.23
  ian.a.raphael.th@dartmouth.edu

*/

#ifndef TempSensors_h
#define TempSensors_h

#include "OneWire.h"
#include "DallasTemperature.h"
#include "dataFile.h"

/*************** TempSensors class ***************/
// define a class for the temp sensors
class TempSensors {

  // make public access for everything
public:

  /* Attributes */
  int powerPin; // power pin for the sensors
  int dataPin; // data pin for the sensors

  OneWire oneWire; // onewire obect for the dallas temp object to hold
  DallasTemperature sensors; // the temp sensor object
  uint8_t (*addresses)[8]; // pointer to array of device addresses

  int numSensors; // number of temperature sensors in the array
  int stationID; // measurement station ID

  String filename = ""; // a filename for the data file
  String* headerInformation; // the header information for the data file
  int numHeaderLines = 4;


  /*************** TempSensors object destructor ***************/
  // destroy a TempSensors object. Frees the dynamically allocated memory (called
  // automatically once object goes out of scope)
  ~TempSensors() {

    // // deallocate the address array
    delete [] addresses;
    // zero out the address
    addresses = 0;

    delete [] headerInformation;
    // zero out the address
    headerInformation = 0;
  }


  /*************** TempSensors object constructor ***************/
  // Constructor. This takes in the input information, addresses all of the sensors,
  // and creates the header information for the data file.
  // IMPT: This uses dynamically allocated memory via `new`! You _must_ free the address
  // array and the header information via the tempsensors destructor method when you are done.
  // TODO: fully sleep the sensors at the end of this function
  TempSensors(int data_pin, int power_pin, int num_tempSensors, int station_ID) {

    // Setup a oneWire instance to communicate with any OneWire devices
    OneWire currOneWire = OneWire(data_pin);

    this->oneWire = currOneWire;

    // Pass our oneWire reference to create a Dallas Temp object
    DallasTemperature currDallasTemp = DallasTemperature(&oneWire);

    this->sensors = currDallasTemp;

    // copy over the other stuff that we need
    powerPin = power_pin;
    numSensors = num_tempSensors;
    stationID = station_ID;

    // init the address array
    addresses = new DeviceAddress[num_tempSensors];

    // // and point the object's addresses attribute here
    // addresses = curr_addresses;

    pinMode(powerPin, OUTPUT);
    digitalWrite(powerPin , HIGH);

    // init temp sensors themselves
    this->sensors.begin();

    // and set their resolution
    this->sensors.setResolution(12);

    // for every sensor
    for (int i = 0; i < numSensors; i++) {

      // if error getting the address
      if (!(this->sensors.getAddress(addresses[i], i))) {

        // print error
        Serial.print("Couldn't find sensor at index ");
        Serial.println(Serial.print(i), DEC);
      }
    }

    // now create the data filename
    filename += "stn";
    filename += stationID;
    filename += "_t.txt";
    // filename += "_tempArray.txt";

    // define the header information
    headerInformation = new String[numHeaderLines];

    headerInformation[0] = "DS18B20 array";

    headerInformation[1] = "Station ID: ";
    headerInformation[1] += stationID;

    headerInformation[2] = "Sensor addresses: {";
    appendAddrToStr(addresses[0], &headerInformation[2]);
    for (int i = 1; i < numSensors; i++) {
      headerInformation[2] += ", ";
      appendAddrToStr(addresses[i], &headerInformation[2]);
    }
    headerInformation[2] += "}";

    headerInformation[3] = "Date,Time";
    // for every sensor in the array
    for (int i = 0; i < numSensors; i++) {

      // add a header column for each sensor
      headerInformation[3] += ",Sensor";
      headerInformation[3] += i + 1;
      headerInformation[3] += " [°C]";
    }

    // create a datafile
    newFile(filename,headerInformation,numHeaderLines);

    // shut down the sensors until we need them
    digitalWrite(powerPin, LOW);
  }


  /*************** TempSensors readTempSensors ***************/
  // Reads an array of temp sensors, returning a string with a timestamp and the
  // reading for each sensor.
  // inputs: timestamp
  // TODO: wake/sleep the sensors in this function
  String readTempSensors_dateStamp(String date, String time) {

    // write the power pin high
    digitalWrite(powerPin, HIGH);

    delay(1000);
    
    // init the sensors
    sensors.begin();

    // delay for a second
    delay(1000);

    // Call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus
    sensors.requestTemperatures();

    // delay for a second
    delay(1000);

    // declare a string to hold the read data
    String readString = "";

    // throw the timestamp on there
    readString = date + "," + time;

    // for every sensor on the line
    for (int i = 0; i < numSensors; i++) {

      // add its data to the string
      readString += ",";
      readString += this->sensors.getTempC(addresses[i]);
    }

    // write the power pin low
    digitalWrite(powerPin, LOW);

    // return the readstring. TODO: add timestamp to the string
    return readString;
  }

  void appendAddrToStr(uint8_t *addrPtr, String *strPtr) {
    for (int i = 0; i < 8; i++) {
      // *strPtr += "0x";
      *strPtr += String(addrPtr[i], HEX);
      if (i < 7) {
        *strPtr += " ";
      }
    }
  }
};

#endif
