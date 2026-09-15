#pragma once
#include <SD.h>
#include <SPI.h>

class SDLogger{
    private:
        String filename;
        bool fileOpen;
        File dataFile;
    public:
        SDLogger();
        bool initSD();
        bool writeCsvData(int numFields, float* dataArray, long dataSize);
        bool openFile();
        bool closeFile();
};