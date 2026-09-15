
// Note: Important to keep order of includes correct to maintain consistent linking
// 1. External 3rd party library includes
// 2. Global definitions (may be used across our project)
// 3. Local includes
// 4. Local defines (used only in main.cpp)
// 5. Static variables

/* LIBRARY INCLUDES */
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <vector>

/* GLOBAL DEFINES */
// Loop rates (Hz)
#define ONE_SEC_MICROS 1000000
#define SAMPLE_LOOP_FREQ 100 // Sample loop freq in Hz
#define KALMAN_LOOP_FREQ_PER_SAMPLES 1  // Compute per n=1 samples
#define CONTROL_LOOP_FREQ 10  // Control loop freq in Hz

#define NUMBER_LOGFIELDS 13

/* LOCAL INCLUDES */
#include <Measurement.h>
// #include <accelReplace.h>  // Not using saturated IMU sim
#include <simulation.h>
#include <kalman.h>
#include <logger.h>
#include <FlightMonitor.h>
#include <FSM.h>
#include <PID_Controller.h>
#include <AdafruitBMP581.h>
#include <AdafruitBNO085.h>
#include <AdafruitADXL345.h>
#include <ServoMovement.h>
#include <flightSim.h>

/**********************************************
 *** CHECK CONFIG CONSTANTS PRIOR TO LAUNCH ***
 **********************************************/
#include <IMPORTANT_CONFIG.h>
/**********************************************
 **********************************************/

/* LOCAL DEFINES */

/* STATIC VARIABLES */
// Delta timing variables
static unsigned long currentTime;
static unsigned long previousFilterReset;
static unsigned long previousSampleTime;
static unsigned int  previousComputeCounts; // counter for when compute should occur after n=1 samples
static unsigned long previousControlTime;

static float logdata[NUMBER_LOGFIELDS];

static const long sampleLoopMicros  = ONE_SEC_MICROS/SAMPLE_LOOP_FREQ;
// @note: DEPRECATED - compute loop is determined by counter instead of sample loop micros (previousComputeCounts)
// static const long computeLoopMicros = KALMAN_LOOP_FREQ_PER_SAMPLES * sampleLoopMicros;
static const long controlLoopMicros = ONE_SEC_MICROS/CONTROL_LOOP_FREQ;

// Flight monitor and sensor objects/variables
static AdafruitBMP581 alt_sensor;
static AdafruitBNO085 imu_sensor;
static AdafruitADXL345 acc_sensor;
static ServoMovement srvMovement;
static DataLogger logger(NUMBER_LOGFIELDS);
static FlightMonitor fm_ace;
static FlightState currentState;
static flightSim simulator(9.81,0.309,0.022008,18.965,0.05);
static bool measurementDataValid;
static Sample::Measurement currentMeasurement;  // Struct for holding current measurement

#if ENABLE_STATUS_LED
// Status LED pin
static int STATUS_LED_PIN = 9;
#endif

// Kalman filter external variables
extern BLA::Matrix<3> measuredAccel;
extern BLA::Matrix<3> inertialAccel;
extern BLA::Matrix<4> quaternions;

// PID controller object and global control
static PID_Controller pid(ACE_TARGET_APOGEE,MAX_ALT_CHANGE);
static double currentPIDControl = 0;
float predictedApogee = 0;

// Logging objects: ** TODO **

// IMPORTANT CONFIG: Development debug variables
#if IS_DEVELOPMENT_MODE
// Debug variables
static int stateVecPrintCounter = 0;
static int counterSample = 0;
static float simSample[2] {0.0, 0.0};
#endif

// IMPORTANT CONFIG: Forced airbrakes actuation flight demonstration
#if FORCED_EXTENSION_CONTROL_CYCLES
static int numForcedExtensionControlCycles = FORCED_EXTENSION_CONTROL_CYCLES;
#endif

void setup() {

  int numExtensionCycles = 1;
  int aceInitFails = 0;
  Serial.begin(38400);

#if IS_DEVELOPMENT_MODE
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  delay (2000);

  Serial.println("> Initialized Serial comms!");
  Serial.println("\n\n\n\n\n\n\n");
  Serial.println("==== NOTE ====");
  Serial.println("THE ACE IS IN DEVELOPMENT MODE - PLEASE CONFIGURE FOR FLIGHT IN IMPORTANT_CONFIG.h");
  Serial.println("\n\n\n\n\n\n\n");
#endif

  // Initialize FSM state
  Serial.print("| Init program state...");
  currentState = FlightState::detectLaunch;
  Serial.println("OK!");

  // Initialize sensor hardware
  if (!imu_sensor.init()) {
    numExtensionCycles = 2;
    ++aceInitFails;
  }
  if (!alt_sensor.init()) {
    numExtensionCycles=3;
    ++aceInitFails;
  }
  // if (!acc_sensor.init()) { // TODO: Add redundant accelerometer
  //   ++aceInitFails;
  // }

  // Initialize vectors/matrices
  Serial.print("| Init Kalman state...");
  if (!initializeKalmanFilter()) {
    ++aceInitFails;
    Serial.println("NOT OK!");
  } else {
    Serial.println("OK!");
  }
  
  // Initialize Logger
  Serial.print("| Init in-flight logger...");
  if(!logger.init()){
    numExtensionCycles=4;
    ++aceInitFails;
    Serial.println("NOT OK!");
  }
  else{
    Serial.println("OK!");
  }

#if ENABLE_STATUS_LED
  // Initialize Status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif

  // ACE initialization summary status check
  if (!aceInitFails) {
    Serial.println("> Init ACE OK! Starting program...");
  } else {
    Serial.println("\n\n\n");
    Serial.print("WARNING: ACE did not initialize successfully! There were ");
    Serial.print(aceInitFails);
    Serial.println(" fails!");
    Serial.println("> Refer to above output to view failures...");

#if CHECK_STRICT_INITIALIZATION
#if DO_SERVO_ACTUATE_INIT_CHECK // TODO: fis servo actuation
  Serial.println("> Notice: Performing servo actuation status check...");
  for(int x=0;x<numExtensionCycles;++x){
    delay(1000);
      srvMovement.setServoPosition(90);
      srvMovement.updateServoPosition();
      delay(2000);
      srvMovement.setServoPosition(0);
      srvMovement.updateServoPosition();
    delay(1000);
  }
#endif
    // Spin infinitely on failed init
    for ( ; ; ) {
      delay(5000);
      Serial.print("ACE INIT FAILED WITH ");
      Serial.print(aceInitFails);
      Serial.println(" ERRORS");
    } 
#endif
    Serial.println("\n\n\n!!! WARNING: ACE IS CONTINUING WITH FAILED INITIALIZATIONS !!!!\n");
  }

#if DO_SERVO_ACTUATE_INIT_CHECK // TODO: fis servo actuation
  Serial.println("> Notice: Performing servo actuation status check...");
  delay(1000);
  srvMovement.setServoPosition(90);
  srvMovement.updateServoPosition();
  delay(2000);
  srvMovement.setServoPosition(0);
  srvMovement.updateServoPosition();
  delay(1000);
#endif

  delay(5000);
  
  // Select PID Mode: Uncomment One, Comment Others
  pid.setMode(MODE_P);
  //pid.setMode(MODE_PI);
  //pid.setMode(MODE_PID);

  // Update all delta timing timer variables with offset to REAL core loop start time
  previousComputeCounts = 0;
  previousFilterReset = micros();
  previousSampleTime = micros();
  previousControlTime = micros();

}

int num=0;
void loop() {
  
  /* SAMPLE LOOP (100Hz) */
  currentTime = micros();
  if (currentState!=FlightState::landed && currentTime >= previousSampleTime + sampleLoopMicros) {
    previousSampleTime += sampleLoopMicros;
    ++previousComputeCounts;

#if RUN_SIMULATION
    // Temporary test of simulated OR data
    getSimulatedData(currentTime/1000000.0+15.96, simSample);
#endif

#if RUN_SIMULATION
    // Debug prints
    if(counterSample%100==0){
      Serial.print("Interp pos: ");
      Serial.println(simSample[0]);
      Serial.print("Interp acc: ");
      Serial.println(simSample[1]);
    }
    counterSample++;
#endif

    measurementDataValid = readMeasurement(&currentMeasurement, &imu_sensor, &alt_sensor, &acc_sensor);  // Reads all SAMPLE loop sensors
    
  }

  /* COMPUTE LOOP (per 1 SAMPLEs : 50Hz) */
  if (currentState!=FlightState::landed && previousComputeCounts >= KALMAN_LOOP_FREQ_PER_SAMPLES) {
    previousComputeCounts = 0; // Reset compute counter

    /* Acceleration transformation (to Earth frame) */
    quaternions = {currentMeasurement.qr,currentMeasurement.qi,currentMeasurement.qj,currentMeasurement.qk};
    measuredAccel = {currentMeasurement.xAccel,
                     currentMeasurement.yAccel,
                     currentMeasurement.zAccel};
    getInertialAccel(); // Transforms acceleration
    getOrientation(); // Updates rocket vertical orientation

#if RUN_SIMULATION
    // Development injected simulated data
    measurementVec = {currentMeasurement.altitude + simSample[0],
                      inertialAccel(0),
                      inertialAccel(1),
                      inertialAccel(2) + simSample[1]};

# else
# if CORRECT_ACCEL
    measurementVec = {currentMeasurement.altitude,
                      inertialAccel(0),
                      inertialAccel(1),
                      inertialAccel(2)};
# else
    measurementVec = {currentMeasurement.altitude,
                          measuredAccel(0),
                          measuredAccel(1),
                          measuredAccel(2)};
# endif
#endif

    /* Kalman filter */
    kalmanPredict();
    if(measurementDataValid){
      kalmanUpdate();
    }

#if IS_DEVELOPMENT_MODE
    // Debug prints
    if(stateVecPrintCounter%25==0){
      Serial.print("FLIGHT STATE: ");
      Serial.println((int)currentState);
      Serial.print("Statevec: ");
      {
        using namespace BLA;
        Serial << stateVec(2) << " , " << stateVec(5) << " , " << stateVec(8) << "\n";
      }
    }
    stateVecPrintCounter++;
#endif
    
    /* FSM transition */
    switch(currentState) {
      case FlightState::detectLaunch:
        currentTime = micros();
        if (currentTime >= previousFilterReset + ONE_SEC_MICROS){
          // If one second has elapsed and not launched, reset kalman filter

          // THIS RESET IS VERY IMPORTANT: ensures velocity behaves well
          Pkalman = {10,0,0,0,0,0,0,0,0,
                    0,10,0,0,0,0,0,0,0,
                    0,0,10,0,0,0,0,0,0,
                    0,0,0,10,0,0,0,0,0,
                    0,0,0,0,10,0,0,0,0,
                    0,0,0,0,0,10,0,0,0,
                    0,0,0,0,0,0,10,0,0,
                    0,0,0,0,0,0,0,10,0,
                    0,0,0,0,0,0,0,0,10};
          stateVec = {0,0,0,0,0,0,0,0,0};

          // Clear data logs
          previousFilterReset = currentTime;
        }

        // Next transition: acceleration detected (motor burn) --> burn
        currentState = Flight_FSM::detectLaunchTransition(&fm_ace, currentState);
        break;
      case FlightState::burn:
        // Next transition: deceleration detected (motor burnout) --> control
        currentState = Flight_FSM::burnTransition(&fm_ace, currentState);
        break;
      case FlightState::control:
        // Next transition: reaching apogee --> stow --> separate --> coast
        currentState = Flight_FSM::controlTransition(&fm_ace, currentState, &srvMovement);
        break;
      case FlightState::controlStandby:
        // Next transition: re-stabilized --> control
        currentState = Flight_FSM::controlStandbyTransition(&fm_ace, currentState, &srvMovement);
        break;
      case FlightState::coast:
        // Next transition: z velocity apprx. 0 and altitude is low --> landed
        currentState = Flight_FSM::coastTransition(&fm_ace, currentState);
        break;
      case FlightState::landed:
        // Next transition: none, continuously write data to storage
        break;
      default:
        // Error state
#if IS_DEVELOPMENT_MODE
        Serial.println("FATAL-ERROR: FSM reached unknown state, resetting to standby");
#endif
        currentState = FlightState::controlStandby;
        break;
    }
    // log data
    //if(stateVecPrintCounter%100==0){
    switch(currentState){
      case FlightState::detectLaunch:
        // Write pre-launch data
          for (int x=0;x<9;++x){
            logdata[x] = (float)stateVec(x);
          }
          logdata[9] = ((float)currentTime/ONE_SEC_MICROS);
          logdata[10] = (float)(currentPIDControl);
          logdata[11] = (float)static_cast<int>(currentState);
          logdata[12] = (float)predictedApogee;
          logger.log(logdata,false);
        break;
      default:
       // Write post-launch data
        for (int x=0;x<9;++x){
          logdata[x] = (float)stateVec(x);
        }
        logdata[9] = ((float)currentTime/ONE_SEC_MICROS);
        logdata[10] = (float)(currentPIDControl);
        logdata[11] = (float)static_cast<int>(currentState);
        logdata[12] = (float)predictedApogee;
        logger.log(logdata,true);
    }
  //}
  }

  /* CONTROL LOOP (1Hz) */
  currentTime = micros();
  if (currentTime >= previousControlTime + controlLoopMicros) {
    previousControlTime += controlLoopMicros;

#if ENABLE_STATUS_LED
    // Status LED control
    if (currentState == FlightState::detectLaunch) {
      digitalWrite(STATUS_LED_PIN, HIGH);
    }
    else {
      digitalWrite(STATUS_LED_PIN, LOW);
    }
#endif
    
    if (currentState==FlightState::control) {
      // Perform PID servo actuation
      // Note: stateVec(2) --> curr_Z_Position, stateVec(5) --> curr_Z_Velocity
      // Perform flight simulation
      simulator.initializeSim({stateVec(2),stateVec(5)});
      
      predictedApogee = simulator.runSim();
      Serial.println(predictedApogee);

      currentPIDControl = pid.control(predictedApogee);
      int angleExtension = (int)(SRV_MAX_EXTENSION_ANGLE * currentPIDControl);  // +0.5 to round to nearest whole int

#if FORCED_EXTENSION_CONTROL_CYCLES
    // Force max extension for flight demonstration purposes
    if (numForcedExtensionControlCycles > 0) {
#if IS_DEVELOPMENT_MODE
      Serial.print("CONTROL: FORCED EXTENSION CYCLE ");
      Serial.println(numForcedExtensionControlCycles);
#endif
      --numForcedExtensionControlCycles;
      angleExtension = SRV_MAX_EXTENSION_ANGLE;
    }
#endif


      srvMovement.setServoPosition(angleExtension);
      srvMovement.updateServoPosition();  // the design is a bit strange, but allows for decentralized servo position updates while centralizing actual writes
      //Serial.print("Updated angle to servo: ");
      //Serial.println((int)(angleExtension));
    }

    if (currentState==FlightState::controlStandby) {
      srvMovement.updateServoPosition();
    }
  }

  /* POST-FLIGHT PROCEDURE */
  if (currentState == FlightState::landed) {
    #if IS_DEVELOPMENT_MODE
      logger.printData();
    #endif
    logger.close();
  // Dump data upon landing (TODO)

    // ACE completed
#if ENABLE_STATUS_LED
    digitalWrite(STATUS_LED_PIN, HIGH);
#endif
    for ( ; ; ) {
      #if IS_DEVELOPMENT_MODE
      Serial.println("ACE completed! Sleeping...");
      #endif
      delay(3000);
    }
  }
}
