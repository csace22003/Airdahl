#pragma once

// Magic number definitions

enum PID_Mode {
  MODE_P, //Proportional Control Only
  MODE_PI, //Proportional and Integral Control
  MODE_PID //Full PID Control
};

class PID_Controller{
    private:
        //PID Mode Switching
        PID_Mode mode = MODE_PID; //Default Mode full PID Control

        double tgtAlt;
        // PID Control variables
        double setpoint;
        double input;
        double output;
        double integral_error;
        double prevError;
        // Configurable constants
        double Ki;
        double Kp;
        double Kd;
        double maxError;

    public:
    //PID Mode Switching
    void setMode(PID_Mode m) {
        mode = m; 
    }
    // Default Constructor
    PID_Controller(double targetAltitude, double maxAltChange) : 
        tgtAlt(targetAltitude),
        integral_error(0),
        prevError(0),
        Ki(0.2),
        Kp(1),
        Kd(0.05),
        maxError(maxAltChange)
    {};

    // Methods
    double control(double predictedAltitude);
    void pid_config(double ki, double kp, double kd);

};
