/* Source file for PID controller class */

// include class header
#include <PID_Controller.h>
#include <vlt.h>
#include <Arduino.h>

#define I_ERROR_RESET_THRESHOLD 0.1

// @brief: Allows for mode switching between a P loop, PI loop, and PID loop.

// @brief: Main PID control computation that takes current
//         z-axis velocity and altitude
double PID_Controller::control(double predictedAltitude){
    // Calculate the current velocity (you may need to calibrate this)
    input = predictedAltitude;

    // Calculate the desired velocity from the lookup table
    setpoint = tgtAlt;

    // Calculate proportional error
    double currError = input - setpoint;

    //Serial.print(currError,4);
    //Serial.println(" - current Velocity Error");

    // Calculate integral error
    // integral_error += currError - getErrorDecay(currAltitude);
    integral_error += currError;

    // If current error is now close or integral error negative, reset integral error history
    if (currError < I_ERROR_RESET_THRESHOLD || integral_error < 0) {
      integral_error = 0;
    }

    // Calculate derivative error (currError - prevError)
    double derivative = currError - prevError;
    prevError = currError;  // update prevError

    // Calculate the control signal
    switch (mode) {
      case MODE_P:
        output = Kp * currError;
      break;
      case MODE_PI:
        output = Kp * currError + Ki * integral_error;
      break;
      case MODE_PID:
        default:
        output = Kp * currError + Ki * integral_error + Kd * derivative;
      break;
    }
    // Normalize output as a proportion of max error
    output = output/maxError;
    // if (currAltitude >= maxErrorThresholds[2]) {
    //   output = output/3.0;
    // }
    // else if (currAltitude >= maxErrorThresholds[1]) {
    //   output = output/10.0;
    // }
    // else {
    //   output = output/30.0;
    // }

    Serial.print("PID output after max Error is ");
    Serial.println(output,4);

    // Clamp output between 0 and 1
    if (output > 1) {
      output = 1;
    }
    else if (output < 0) {
      output = 0;
    }

    return output;
}

void PID_Controller::pid_config(double ki, double kp, double kd){
    Ki=ki;
    Kp=kp;
    Kd=kd;
}