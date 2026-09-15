#include <ServoMovement.h>
#include <IMPORTANT_CONFIG.h>

ServoMovement::ServoMovement(): targetAngle(90), lastAngle(90), distanceThreshold(1), 
previousServoUpdate(0), servoUpdateIntervalMicros(10 * 1000), numSteps(2)
{
  // Attach servo pin to D5
  srv.attach(6);

  // Default stow airbrakes on init
  #if SERVO_INVERTED
  //  targetAngle = SRV_MAX_EXTENSION_ANGLE + SRV_ANGLE_DEG_OFFSET;
    targetAngle = SRV_MAX_EXTENSION_ANGLE;
  #else
    targetAngle = SRV_ANGLE_DEG_OFFSET;
  #endif
  srv.write(targetAngle);
  //lastAngle = targetAngle;
}

ServoMovement::~ServoMovement()
{
}


// Function to initialize servo movement parameters
void ServoMovement::setServoPosition(int newAngle) {
  #if SERVO_INVERTED
    targetAngle = SRV_MAX_EXTENSION_ANGLE - newAngle;
  #else
    targetAngle = newTargetAngle;
  #endif
}

void ServoMovement::stowAirbrakes(){
  #if SERVO_INVERTED
    targetAngle = SRV_MAX_EXTENSION_ANGLE;
    updateServoPosition();
  #else
    targetAngle = 0;
  #endif
}

void ServoMovement::updateServoPosition() {
  if(targetAngle>SRV_MAX_EXTENSION_ANGLE){
    srv.write(SRV_MAX_EXTENSION_ANGLE);
  }
  else{
    srv.write(targetAngle);
  }
}