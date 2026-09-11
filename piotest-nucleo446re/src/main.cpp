#include <Arduino.h>
#include <Wire.h>

#define WIRE Wire

int result, nDevices;
byte error, addr;

// Nucleo-F446RE LD2
#define LED_PIN PA5

// put function declarations here:
int myFunction(int, int);

// Configure Serial monitor for uart1
HardwareSerial Serial3(PA10, PA9);  // RX, TX

void setup() {
  // Configure LD2
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WIRE.begin();

  Serial3.begin(9600);
  while (!Serial3);

  delay(1000);

  result = myFunction(2, 3);
  Serial3.println("Setup Complete!");
}

void loop() {
  // Flash LD2 continuously
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  nDevices = 0;

  for (addr = 1; addr < 127; addr++) {
    WIRE.beginTransmission(addr);
    error = WIRE.endTransmission();

    if (error == 0) {
      Serial3.print("I2C device found at address 0x");
      if (addr < 16)
        Serial3.print("0");
      Serial3.print(addr, HEX);
      Serial3.println(" !");

      nDevices++;
    } else if (error == 4) {
      Serial3.print("Unknown error at address 0x");
      if (addr < 16)
        Serial3.print("0");
      Serial3.print(addr, HEX);
      Serial3.println(" !");
    }
  }

  if (nDevices == 0)
    Serial3.println("No devices found!");

  Serial3.printf("Hello World!\n");
  Serial3.printf("Result: %d\n", result);

  delay(500);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}