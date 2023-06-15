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
#include "TimeSnowTATOS.h"



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
    dataFile.print(dataString);

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

// /************ dataFile_getData ************/
// /*
// function for getting the first line of data from every file stored by the sensor.
// controller. does NOT check buffer size, file contents, etc. just reads in data
// from each file until it hits a new line.
// */
// void dataFile_getAllData(char* dataBuf,String filename) {
//
//   // get the file size
//   unsigned long fileSize = getFileSize(filename);
//
//   // open the file for reading
//   File dataFile = openFile_read(filename);
//
//   // read all of the data into the buffer
//   readFileBytes(dataFile, dataBuf, fileSize);
//   //
//   // // for now, allocate test data
//   // char testData[] = "Hello this is a really long message that won't fit into one packet.";
//   //
//   // // copy it into the external array
//   // for (int i=0;i<sizeof(testData);i++) {
//   //   dataBuf[i] = testData[i];
//   // }
// }

void getNewestData(byte* simbDataBuf, String filename) {

  SerialUSB.print("About to pack data from filename: ");
  SerialUSB.println(filename);


  // determine whether it's a pinger or a temp file
  int index = filename.indexOf('_');
  char sensorType = filename.charAt(index+1);

  // get the stn number (comes right before the '_')
  int stnID = (filename.substring(index-1,index)).toInt();

  SerialUSB.print("packing data for station ");
  SerialUSB.print(stnID,DEC);
  SerialUSB.print(" of type: 0x");
  SerialUSB.println((uint8_t) sensorType,HEX);

  // get the file size
  unsigned long fileSize = getFileSize(filename);

  // open the file
  File dataFile = openFile_read(filename);

  // seek to the end
  seekToPoint(dataFile, fileSize);

  // allocate a counter starting four chars before the end (to avoid the ending
  // newline chars)
  int i = fileSize-4;

  // now start reading backwards
  for (i;i>=0;i--) {

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
  char temporaryBuffer[100];

  int k = 0;
  // now for every byte after the newline
  for (i;i<fileSize;i++) {

    // read the byte
    uint8_t holdChar = dataFile.read();

    // if it's a newline or a carriage return
    if ((holdChar == '\r') || (holdChar == '\n')) {

      // that's the end of our line. write a terminating character
      temporaryBuffer[i] = '\0';

      // then break out of the loop
      break;
    }

    // otherwise write the char to the buffer
    temporaryBuffer[k] = holdChar;
    k++;
  }

  // close the file
  closeFile(dataFile);

  SerialUSB.println("Data pulled from file: ");
  SerialUSB.println(temporaryBuffer);

  // tokenize the data
  char *delimiter = ",";

  // first get the datestamp
  char *dateStamp = strtok(temporaryBuffer,delimiter);

  // then the timestamp
  char *timeStamp = strtok(NULL,delimiter);

  // allocate an int to keep track of where the data starts for this station
  int startByte = (((stnID-1) * 11)+4)/2;

  // now, if it's temp data
  if (sensorType == 'T' || sensorType == 't') {

    // get the three temperatures and convert to unsigned int ((values+56.0) * 67)
    // this pads the minimum temp reading of -55, and gives a max reading of ~+5 degrees
    // without rollover for our twelve bits
    char *temp1_char = strtok(NULL,delimiter);
    uint16_t temp1 = (atof(temp1_char)+56.0) * 67.0;
    char *temp2_char = strtok(NULL,delimiter);
    uint16_t temp2 = (atof(temp2_char)+56.0) * 67.0;
    char *temp3_char = strtok(NULL,delimiter);
    uint16_t temp3 = (atof(temp3_char)+56.0) * 67.0;

    // // print them
    // SerialUSB.println("Packing temps for station ");
    // SerialUSB.println(stnID,DEC);
    // SerialUSB.println("char: ");
    // SerialUSB.println(temp1_char);
    // SerialUSB.println(temp2_char);
    // SerialUSB.println(temp3_char);
    //
    // SerialUSB.println("DEC after conversion: ");
    // SerialUSB.println(temp1,DEC);
    // SerialUSB.println(temp2,DEC);
    // SerialUSB.println(temp3,DEC);

    temp1 = 3082;
    temp2 = 2747;
    temp3 = 2412;

    // write the data to the correct location in the simb buffer

    // if it's an odd station
    if (stnID%2 == 1) {

      // byte1 is the low nibble of the high byte and the high nibble of the low byte of temp 1
      // byte 1 = lowNybble(highByte(temp1)) + highNibble(lowByte(temp1))
      byte byte1 = (highByte(temp1) << 4) | (lowByte(temp1) >> 4);

      // high nibble of the second byte is the low nibble of the low byte of temp 1.
      // we'll just shift it into a high nibble.
      byte byte2 = (lowByte(temp1) << 4);
      // now the low nibble of byte2 is the low nibble of the high byte of temp 2
      byte2 = byte2 | (highByte(temp2) & B00001111);

      // byte 3 is the low byte of temp 2
      byte byte3 = lowByte(temp2);

      // byte4 is the low nibble of the high byte and the high nibble of the low byte of temp 3
      byte byte4 = (highByte(temp3) << 4) | (lowByte(temp3) >> 4);

      // byte5 is the low nibble of the lowbyte of temp3, shifted up 4 bits.
      // we'll put the high byte of the pinger data in later
      byte byte5 = (lowByte(temp3) << 4);

      // write all of the data to the buffer
      simbDataBuf[startByte] = byte1;
      simbDataBuf[startByte+1] = byte2;
      simbDataBuf[startByte+2] = byte3;
      simbDataBuf[startByte+3] = byte4;
      simbDataBuf[startByte+4] = byte5;

      // otherwise, if it's an even station
    } else if (stnID%2 == 0) {

      // byte1 is the high nibble of startByte and the low nibble of the high byte of temp1
      byte byte1 = (simbDataBuf[startByte] & B11110000) | (highByte(temp1) & B00001111);

      // byte2 is the low byte of temp1
      byte byte2 = lowByte(temp1);

      // byte3 is the low nibble of the high byte and the high nibble of the low byte of temp2
      byte byte3 = (highByte(temp2) << 4) | (lowByte(temp2) >> 4);

      // byte4 is the low nibble of the low byte of temp 2 and the low nibble of the high byte of temp 3
      byte byte4 = (lowByte(temp2) << 4) | (highByte(temp3) >> 4);

      // byte5 the lowbyte of temp3
      byte byte5 = lowByte(temp3);

      // write all of the data to the buffer
      simbDataBuf[startByte] = byte1;
      simbDataBuf[startByte+1] = byte2;
      simbDataBuf[startByte+2] = byte3;
      simbDataBuf[startByte+3] = byte4;
      simbDataBuf[startByte+4] = byte5;

    }

    // if it's pinger data
  } else if (sensorType == 'P' || sensorType == 'p') {

    // get the pinger data
    char *pingerData_char = strtok(NULL,delimiter);
    int pingerData;

    // if it's NaN
    if (pingerData_char == "NaN") {
      // set it at 255 (error val)
      uint8_t pingerData = 255;
    } else {
      // convert to an int
      pingerData = atoi(pingerData_char);
    }

    // if it's greater than 255
    if (pingerData > 255) {
      // make it 255 and recast it
      pingerData = 255;
    }

    // recast to a byte
    pingerData = uint8_t(pingerData);

    SerialUSB.print("packing pinger value: ");
    SerialUSB.println(pingerData,DEC);

    // if it's an odd station
    if (stnID%2 == 1) {

      // pinger data is split across a byte boundary

      // high nibble of the pingerData goes in the low nibble of startByte + 4
      byte byte1 = (simbDataBuf[startByte+4] & B11110000) | (pingerData >> 4);

      // low nibble of pingerdata goes in the high nibble of start byte + 5
      byte byte2 = (pingerData << 4) | (simbDataBuf[startByte+5] & B00001111);

      // put the data in the buffer
      simbDataBuf[startByte+4] = byte1;
      simbDataBuf[startByte+5] = byte2;

      // otherwise, if it's an even station
    } else if (stnID%2 == 0) {

      // pinger data starts on a byte boundary.
      // put it in the right spot
      simbDataBuf[startByte+5] = pingerData;
    }
  }

  // get the timestamp out of the buffer
  uint16_t bufferTimestamp = (simbDataBuf[0] << 8) | (simbDataBuf[1] & 0x00ff);

  // if the timestamp is 0
  if (bufferTimestamp == 0) {

    // parse the datestamp
    uint8_t currYear = (uint8_t) atoi(strtok(dateStamp,"."));
    uint8_t currMonth = atoi(strtok(NULL,"."));
    uint8_t currDay = atoi(strtok(NULL,"."));

    // parse the timestamp
    uint8_t currHour = atoi(strtok(timeStamp,":"));
    uint8_t currMinute = atoi(strtok(NULL,":"));
    uint8_t currSecond = atoi(strtok(NULL,":"));

    // now set the local time
    // hour, min, sec, day, month, year
    setTime(currHour, currMinute, 0, currMonth, currDay, currYear);

    // get the elapsed seconds since start time
    time_t elapsedSeconds = now() - START_TIME_UNIX;

    // convert to decimal days, multiplied by 100 and floored automatically
    // by integer math. we will recover this to days and hours later as:
    // recoveredDecimalDays = (elapsedDecimalDays_multHundred)/100
    // days = floor(recoveredDecimalDays)
    // hours = round(recoveredDecimalDays - days)*24
    uint16_t elapsedDecimalDays_multHundred = elapsedSeconds/864;

    // now write to the buffer
    simbDataBuf[0] = highByte(elapsedDecimalDays_multHundred);
    simbDataBuf[1] = lowByte(elapsedDecimalDays_multHundred);
  }
}

// unpacks a pinger value from the simb buffer for the given station ID
uint8_t unpackPingerData(byte* simbBuffer,uint8_t stnID){

  // get the start byte
  int startByte = (((stnID-1) * 11)+4)/2;

  uint8_t pingerData;

  // if it's an odd station
  if (stnID%2 == 1){
    // pinger data is split across a byte boundary
    // high nibble of the data is in the low nibble of startByte+4
    // low nibble of the data is in the high nibble of startByte+5
    pingerData = (simbBuffer[startByte+4] << 4) | (simbBuffer[startByte+5] >> 4);

    // otherwise it's an even station
  } else {
    // pinger data starts on a byte boundary
    // extract it
    pingerData = simbBuffer[startByte+5];

  }

  // return the value
  return pingerData;
}

// unpacks the timestamp from the simb buffer
float unpackTimestamp(byte* simbBuffer) {

  // get the timestamp out of the buffer
  uint16_t bufferTimeStamp = (simbBuffer[0] << 8) | (simbBuffer[1] & 0x00ff);

  // convert to decimal days after init time
  // recoveredDecimalDays = (elapsedDecimalDays_multHundred)/100
  // days = floor(recoveredDecimalDays)
  // hours = round(recoveredDecimalDays - days)*24
  float decimalDays = (bufferTimeStamp)/100.0;

  // return as decimal days
  return decimalDays;
}

// unpacks the three temp values from the simb buffer for the given station ID
void unpackTempData(byte* simbBuffer,float* tempArray,uint8_t stnID) {

  // get the start byte
  int startByte = (((stnID-1) * 11)+4)/2;

  // declare temp vars
  uint16_t temp1;
  uint16_t temp2;
  uint16_t temp3;

  // if it's an odd station
  if (stnID%2 == 1) {

    // low nibble of the high byte of data is the high nibble of the startByte in the array
    // low byte is the low nibble of the first byte and the high nibble of the second byte
    byte hiByte = (simbBuffer[startByte] >> 4);
    byte loByte = (simbBuffer[startByte] << 4) | (simbBuffer[startByte+1] >> 4);
    temp1 = (hiByte << 8) | (loByte & 0x00ff);

    hiByte = simbBuffer[startByte+1] & B00001111;
    loByte = simbBuffer[startByte+2];
    temp2 = (hiByte << 8) | (loByte & 0x00ff);

    hiByte = simbBuffer[startByte+3] >> 4;
    loByte = (simbBuffer[startByte+3] << 4) | (simbBuffer[startByte+4] >> 4);
    temp3 = (hiByte << 8) | (loByte & 0x00ff);

    // otherwise, if it's an even station
  } else if (stnID%2 == 0) {

    byte hiByte = (simbBuffer[startByte] & B00001111);
    byte loByte = simbBuffer[startByte+1];
    temp1 = (hiByte << 8) | (loByte & 0x00ff);

    hiByte = (simbBuffer[startByte+2] >> 4);
    loByte = (simbBuffer[startByte+2] << 4) | (simbBuffer[startByte+3] >> 4);
    temp2 = (hiByte << 8) | (loByte & 0x00ff);

    hiByte = (simbBuffer[startByte+3] & B00001111);
    loByte = simbBuffer[startByte+4];
    temp3 = (hiByte << 8) | (loByte & 0x00ff);
  }

  // now convert the temperatures back to celcius
  tempArray[0] = (temp1/67.0)-56.0;
  tempArray[1] = (temp2/67.0)-56.0;
  tempArray[2] = (temp3/67.0)-56.0;
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
