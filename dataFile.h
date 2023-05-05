/*

  A library for handling file functions in temperature datalogger
  deployments. creates, reads, writes generic datafiles given a filename and
  header information. based on SD library

  Ian Raphael
  2023.03.27
  ian.a.raphael.th@dartmouth.edu

*/

#ifndef dataFile_h
#define dataFile_h

#include "SD.h"


// SD card pins
#define SD_CS 10 // SD card chip select
#define SD_POWER 6 // SD card power
#define SD_CD NAN // SD card chip DETECT. Indicates presence/absence of SD card. High when card is inserted.
#define RADIO_CS 5 // radio chip select pin
#define LED 13


/************ openFile_read ************/
/*
function for opening a file for reading
*/
File openFile_read(String filename) {

  File dataFile = SD.open(filename, FILE_READ);

  return dataFile;
}

/************ openFile_write ************/
/*
function for opening a file for reading
*/
File openFile_write(String filename) {

  File dataFile = SD.open(filename, FILE_WRITE);

  return dataFile;
}

/************ closeFile ************/
/*
function for opening a file for writing
*/
void closeFile(File dataFile) {
  dataFile.close();
}

// create a new file if it doesn't exist
void newFile(String filename, String* headerInformation, int numHeaderLines) {

  // if the file doesn't already exist
  if (!SD.exists(filename)) {

    Serial.print("Creating datafile: ");
    Serial.println(filename);

    File dataFile = openFile_write(filename);

    // write the header information
    // if the file is available
    if (dataFile) {

      // print the header information to the file
      for (int i = 0; i < numHeaderLines; i++) {

        dataFile.println(headerInformation[i]);

        // also print to serial to confirm
        Serial.println(headerInformation[i]);
      }

      // close the file
      dataFile.close();
    }
  }
}


/************ writeToFile ************/
/*
function for writing generic data to generic file
*/
void writeToFile(String filename, String dataString) {

  // open the file for writing
  File dataFile = openFile_write(filename);

  // if the file is available
  if (dataFile) {

    // write the data
    dataFile.println(dataString);

    // close the file
    dataFile.close();
  }
}


/************ getFileSize ************/
/*
wrapper function for getting filesize
*/
unsigned long getFileSize(String filename){

  File dataFile = openFile_write(filename);

  // get the size
  unsigned long fileSize = dataFile.size();

  closeFile(dataFile);

  // return the size
  return fileSize;
}

/************ seekToPoint ************/
/*
wrapper function for seeking within a file
*/
void seekToPoint(File dataFile, unsigned long seekTo) {
  // seek to the point
  dataFile.seek(seekTo);
}

/************ getFilePosition ************/
/*
wrapper function for getting cursor position in file
*/
unsigned long getFilePosition(File dataFile) {

  // get the position
  unsigned long filePosition =  dataFile.position();

  // return it
  return filePosition;
}


/************ readFileBytes ************/
/*
wrapper function for reading a chunk of bytes
*/
int readFileBytes(File dataFile, uint8_t* buffer, int numBytesToRead) {

  // read the data into the passed buffer, save the number of bytes we read
  int numBytesRead = dataFile.readBytes(buffer,numBytesToRead);

  // return the number of bytes we read
  return numBytesRead;
}

/************ dataFile_getData ************/
/*
function for getting the first line of data from every file stored by the sensor.
controller. does NOT check buffer size, file contents, etc. just reads in data
from each file until it hits a new line.
*/
char* dataFile_getData(char* dataPointer) {
  // for every file in the directory
    // read the first line into the buffer

  char testData[] = "Hello this is a really long message that won't fit into one packet.";

  // return the buffer
  dataPointer = testData;
}



/************ init_SD ************/
void init_SD() {

  delay(100);
  // set SS pins high for turning off radio
  pinMode(RADIO_CS, OUTPUT);
  delay(500);
  digitalWrite(RADIO_CS, HIGH);
  delay(500);
  pinMode(SD_CS, OUTPUT);
  delay(1000);
  digitalWrite(SD_CS, LOW);
  delay(2000);
  SD.begin(SD_CS);
  delay(2000);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD initialization failed!");
  }
}

#endif
