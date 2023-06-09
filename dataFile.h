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

    SerialUSB.print("Creating datafile: ");
    SerialUSB.println(filename);

    File dataFile = openFile_write(filename);

    // write the header information
    // if the file is available
    if (dataFile) {

      // print the header information to the file
      for (int i = 0; i < numHeaderLines; i++) {

        dataFile.println(headerInformation[i]);

        // also print to serial to confirm
        SerialUSB.println(headerInformation[i]);
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
void dataFile_getAllData(char* dataBuf,String filename) {

  // get the file size
  unsigned long fileSize = getFileSize(filename);

  // open the file for reading
  File dataFile = openFile_read(filename);

  // read all of the data into the buffer
  readFileBytes(dataFile, dataBuf, fileSize);
  //
  // // for now, allocate test data
  // char testData[] = "Hello this is a really long message that won't fit into one packet.";
  //
  // // copy it into the external array
  // for (int i=0;i<sizeof(testData);i++) {
  //   dataBuf[i] = testData[i];
  // }
}

void getNewestData(char* simbDataBuf, String filename) {

  // determine whether it's a pinger or a temp file
  int index = filename.indexof('_');
  char sensorType = filename[index+1];

  // get the stn number (index is 3: STN#_P.txt)
  int stnID = toInt(filename[3]);

  // get the file size
  unsigned long fileSize = getFileSize(filename);

  // open the file
  File dataFile = openFile_read(filename);

  // seek to the end
  seekToPoint(dataFile, fileSize);

  // allocate a counter starting two chars before the end (to avoid the ending
  // newline chars)
  int i = fileSize-2;

  // now start reading backwards
  for (i;i>=0;i--){

    // seek to the new char
    seekToPoint(dataFile,i);

    // if it's a new line character
    if (dataFile.peek() == '\n') {
      // break out of the loop
      break;
    }
  }

  // seek forward one char
  seekToPoint(dataFile,i+1);

  // allocate a buffer to hold the data
  // (right now making it 100 but it should be the size of whatever data we
  // are supposed to be reading)
  char *temporaryBuffer[100];

  // now for every byte after the newline
  for (i;i<fileSize;i++) {

    // read the byte
    uint8_t holdChar = dataFile.read();

    // if it's a newline or a carriage return
    if (holdChar == '\r' || (holdChar == '\n') {

      // that's the end of our line. write a terminating character
      temporaryBuffer[i] = '\0';

      // then break out of the loop
      break;
    }

    // otherwise write the char to the buffer
    temporaryBuffer[i] = holdChar;
  }

  // close the file
  closeFile(dataFile);

  // tokenize the data
  char *delimiter = ",";

  // first get the datestamp
  char *dateStamp = strtok(temporaryBuffer,delimiter);

  // then the timestamp
  char *timeStamp = strtok(NULL,delimiter);;

  // now, if it's temp data
  if (sensorType == 'T') {

    // get the first bit of data
    char *currTempData = strtok(NULL,delimiter);

    do {

      // write the data to the correct location in the simb buffer

      // get the next bit of data
      currTempData = strtok(NULL,delimiter);

      // until we reach the end
    } while (currTempData != NULL);

    // if it's pinger data
  } else if (sensorType == 'P') {

    // get the pinger data
    char *pingerData = strtok(NULL,delimiter);

    // and write it to the correct location in the simb buffer

  }

  // if there isn't a timestamp in the buffer already
  if () {
    // parse the timestamp and write it to the correct location

    // first set local time
    // hour, min, sec, day, month, year
    setTime(currHour, 00, 00, currMonth, currDay, currYear);

    // get the elapsed seconds since start time
    time_t elapsedSeconds = now() - START_TIME_UNIX;

    // convert to decimal days
    double elapsedDays = (double)elapsedSeconds/(double)86400;
  }
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
    SerialUSB.println("SD initialization failed!");
  }
}

#endif
