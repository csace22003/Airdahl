#include <Measurement.h>

#include <AdafruitBNO085.h>
#include <AdafruitBMP581.h>
#include <AdafruitADXL345.h>

bool Sample::readMeasurement(Sample::Measurement* currentMeasurement, 
                        AdafruitBNO085* imu, AdafruitBMP581* alt, AdafruitADXL345* acc) {
    bool measurementDataValid = true;

    // Measure Rotation Quat
    if (!imu->measureIMU(currentMeasurement)) {
        measurementDataValid = false;
    }

    // Measure Altitude
    if (!alt->measureAltitude(currentMeasurement)) {
        measurementDataValid = false;
    }

    // Measure Acceleration // NOTE: currently have no redundant accelerometer
    // if (!acc.measureAcceleration(currentMeasurement)) {
    //     measurementDataValid = false;
    // }
    // Serial.print("Current altitude: ");
    // Serial.print(currentMeasurement->altitude);
    // Serial.print("     Current z acceleration: ");
    // Serial.println(currentMeasurement->zAccel);

    return measurementDataValid;
}