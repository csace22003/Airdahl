#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

#include <AdafruitBMP581.h>
#include <Measurement.h>

AdafruitBMP581::AdafruitBMP581() {}

bool AdafruitBMP581::init() {
    Serial.print("| Init BMP581...");
    Wire.begin();

    BASE_PRESSURE_READING = 1013.25;    // MODIFY WITH CALIBRATED BASE READING ON INIT

    // Initialize I2C bus
    if (!instance.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
  // For SPI mode (uncomment the line below and comment out the I2C line above):
  // if (!bmp.begin(BMP5XX_CS_PIN, &SPI)) {
    Serial.println("NOT OK! ALTIMETER NOT FOUND!");
    }
    // Configure sample
    instance.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    instance.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
    instance.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    instance.setOutputDataRate(BMP5XX_ODR_100_2_HZ);    // 50 Hz, 60 Hz, 70 Hz, 80 Hz -> Max is 240 Hz
    instance.setPowerMode(BMP5XX_POWERMODE_NORMAL);

    // Possible update, do interrupts instead of polling?
    // instance.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true);


    Serial.println("OK!");
    didInit = true;

    Serial.println("> Calibrating pressure readings with 100 sample average...");
    int j = 0;
    double baseAltitudes = 0;
    double bAlt = 0;
    while (j < 100) {
        bAlt = getRelativeAltitude();
        if (bAlt < 0) {
            continue;
        }
        if (j > 9) {
            baseAltitudes += bAlt;
        }
        ++j;
        delay(10);  // 100 Hz delay
    }
    BASE_ALTITUDE_OFFSET = baseAltitudes/90;
    Serial.print("> BASE_ALTITUDE_OFFSET: ");
    Serial.println(BASE_ALTITUDE_OFFSET);

    return didInit;
}


// @brief: Obtains altitude relative to an offset (zeroed pressure at init altitude)
double AdafruitBMP581::getRelativeAltitude() {
    if (!didInit || !instance.performReading()) {
        return -1;
    }
    
    double relativeAlt = instance.readAltitude(BASE_PRESSURE_READING) - BASE_ALTITUDE_OFFSET;
    if (relativeAlt < 0) {
        return 0;
    }
    return relativeAlt;
}


// @brief: Prints "raw" (i know, misleading) relative altitudes
void AdafruitBMP581::printRawAltitude(int iters, int sampleFreqMicros) {
    int i = 0;
    while (i < iters) {
        double alt = getRelativeAltitude();

        if (alt < 0) {
            Serial.println("MISS");
            continue;
        }
        Serial.println(getRelativeAltitude(), 6);
        ++i;
        delay(sampleFreqMicros);
    }
}


// @brief: fills measurement struct with altitude data
bool AdafruitBMP581::measureAltitude(Sample::Measurement* measure) {
    double alt = getRelativeAltitude();
    if (alt < 0) {
        return false;
    }

    measure->altitude = (float)alt;

    return true;
}