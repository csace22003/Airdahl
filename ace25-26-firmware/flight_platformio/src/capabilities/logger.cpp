#include <logger.h>
#include <IMPORTANT_CONFIG.h>

DataLogger::DataLogger(int numFields) : 
            numDataFields(numFields)
{};

// Initializes SD card and creates CSV file
bool DataLogger::init(){
    if(!SDcard.initSD()){
        return false;
    }
    if (!SDcard.openFile()){
        return false;
    }
    return true;
};

bool DataLogger::log(float* data, bool launched){
    (void)launched;
    return SDcard.writeCsvData(numDataFields, data, sizeof(float) * numDataFields);
}

void DataLogger::printData(){
   //DEPRECATED
}

bool DataLogger::close(){
    SDcard.closeFile();
}