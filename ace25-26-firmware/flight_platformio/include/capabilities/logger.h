#pragma once
#include <SDlogger.h>

class DataLogger{
    private:
        int numDataFields;
    public:
        SDLogger SDcard;
        DataLogger(int numFields);
        bool init(); //Initializes SD Card and creates CSV file
        bool log(float* data, bool launched); // Writes a row of data, returns true if successfull
        void printData(); // DEPRECATED: Print out full data buffers
        bool close(); // Closes CSV file
};