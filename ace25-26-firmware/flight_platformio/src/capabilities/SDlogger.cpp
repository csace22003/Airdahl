#include <SDlogger.h>
#define CLOCKSPEED 5000000
#define CHIP_SELECT 7
SDLogger::SDLogger()

{
    // Constructs SD object
    fileOpen = false;
    filename = "";
}

bool SDLogger::initSD()
{
    // Initializes SD card
    if (!SD.begin(CLOCKSPEED,CHIP_SELECT))
    {
        // Initialization failed
        return false;
    }
    // Initialization successful
    return true;
}

bool SDLogger::writeCsvData(int numFields, float *dataArray, long dataSize)
{
    // Writes data in CSV format to file, where dataArray consists of dataSize
    // one row of CSV: 1,2,3,4,5,6,7,8,9,10,11
    if (!fileOpen)
    {
        // File not open
        return false;
    }
    for (long i = 0; i < dataSize/numFields; i++) {
        for (int j = 0; j < numFields; j++) {
            dataFile.print(dataArray[i * numFields + j]);
            if (j < numFields  - 1) {
                dataFile.print(",");
            }
        }        
        dataFile.println();
    }
}

bool SDLogger::openFile()
{
    // opens file
    if (fileOpen)
    {
        // File already open
        return true;
    }
    boolean foundFile=false;
    int flightNum = 0;
    while(!foundFile){
        String testFilename = "flight" + String(flightNum) + ".csv";
        if(SD.exists(testFilename)){
            flightNum++;
        }
        else{
            foundFile=true;
            filename = testFilename;
        }
    }
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile)
    {
        fileOpen = true;
        return true;
    }
    else
    {
        // Failed to open file
        return false;
    }
}

bool SDLogger::closeFile()
{
    // closes file
    if (fileOpen)
    {
        dataFile.close();
        fileOpen = false;
        return true;
    }
    else
    {
        // File not open
        return false;
    }
}