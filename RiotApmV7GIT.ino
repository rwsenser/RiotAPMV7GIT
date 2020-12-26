// RiotApmV7
// 2020-07-06: base RiotApmV4
// Senser (06/23/2020)
// GIT upload 2020-12-26
// (c) copyright 2020 by Robert Senser

/*
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
// 
// Created from RiotApmV6
//
const int VERSION = 7;
//
// 2020-12-26: make ready for GIT upload
//
// 2020-10-11: RIOT3B2 configured
//
// 2020-10-10: WIGWAM_MULT_FACTOR added to mission.h to hold rudder multiply factor
//
// 2020-10-07: Fix to add option (new default) to swap Rudder and elevator input channels,
//             this removes the need to make a custom cable
//
// 2020-09-19: Fix issues found while flying RiotLite at Pawnee Grasslands
//             1)  Lower A_LOCK timer to 5 secs (10 or 15)
//             2)  Change Servo4 code to be more noise immune and not toggle
//                 automode when noisy readings are received
//
// 2020-09-07: HMC5883L appears to work with GY271test program [ !!]
//
// 2020-09-04: (??) Gyro appears to get lost after rotations... when back to actual heading 0
//             gyro shows 341 (two quick rotations to right), 320 after 2 more....
//
// 2020-08-23: After garage test, noted throttle starts too late upon increasing throttle control
// 2020-08-22: change all servo/angle ranges from 0, 180 to 30, 150
//
// 2020-07-06:  Review wigwam2  
//
// Arduino Tools Settings:
// BOARD: Arduino Mega/Mega2560
// ISP: ArduinoISP
//
//  ################ PLANE settings  ################
// 
#include "plane.h"
//
#ifndef PLANE_NAME
#error No_plane, barf, barf -- no plane defined
#endif
//          
// ############# end PLANE settings  ################
//
// UNO_MEGA: Configure for Arduino UNO and/or MEGA
#undef UNO_MEGA
#define UNO_MEGA

#include "RiotMission.h"
//
// ############# MISSION settings  ################
// this is the actual mission
//  
#include "mission.h"  
//
// ############# TESTING settings  ################
#undef P_DELTA_GROUND_ONLY
#undef P_TEST_SKIP_FAIL_CLIMB
#undef TEST_ALT_MANUAL_MODE
// For Testing:
// P_DELTA_GROUND_ONLY: override plane's servo delta settings for ground testing & shorten logging timer
// #define P_DELTA_GROUND_ONLY
// P_TEST_SKIP_FAIL_CLIMB: prevent failure to climb override
// #define P_TEST_SKIP_FAIL_CLIMB  
// TEST_ALT_MANUAL_MODE: simulate different altitudes from stationary, ground level
// #define TEST_ALT_MANUAL_MODE
//
// ############# end TESTING ################
//
// ############# DELTA settings  ################
// delta management: These are the control throw values for various actions
//                   possibly NOT FOR wigwam code
//
// when flying, delta should be at most "one or two marks" on the R/C transmitter stick :)
//
#ifdef P_DELTA_GROUND_ONLY 
// these are wildly exagerated for testbed, ground testing only
const int ELEVATOR_FULL_DELTA = 20; 
const int ELEVATOR_CLIMB_DELTA = 10; 
const int RUDDER_FULL_DELTA = 10;  
const int THROTTLE_FULL_DELTA = 20; 
#else
// actual flying values:
const int ELEVATOR_FULL_DELTA = 6; // want 6 to be "one mark", was 20; 
// const int ELEVATOR_CLIMB_DELTA = 2; // was 1
// const int RUDDER_FULL_DELTA = 4; // was 3; 2019-04-17 // was 5; SEPT17   // want 5 to be < "one mark", was (5,20)
// DEBUG DEBUG WIGWAM  -- for ground pre test
// const int ELEVATOR_CLIMB_DELTA = 4; 
const int ELEVATOR_CLIMB_DELTA = 2; // was 1, 2020-06-22
const int RUDDER_FULL_DELTA = 4; // 2019-09-25: back to 4, up to 5, was 4; was 3;
const int THROTTLE_FULL_DELTA = 4; // 20; 
#endif

// const int GLIDE_DELTA_MULTIPLIER = 4; // up elevator factor when power out, was 2, 3 (2019-05-27)
const int GLIDE_DELTA_MULTIPLIER = 5; // up elevator factor when power out, was 2, 3, 4 (2019-06-05)
// DEBUG DEBUG WIGWAM  -- for ground pre test
const int HEAVY_CLIMB_DELTA_MULTIPLIER = 3; // heavy climb multiplier, use with
// const int HEAVY_CLIMB_DELTA_MULTIPLIER = 2; // heavy climb multiplier, use with
                                            // (sharp) turns
const int RUDDER_HARD_MULTIPLIER = 2;

// SLOP factors -- these are swags ....
// 2019-09-24: try very loose values so that *overshooting* is less of an issue
const int HEADING_MAX_SLOP = 3; // was 7 2020-03-08; was 3 2019-09-23       // max slop in degrees -- either way
// DEBUGZZ
const int HEADING_PHASE_1_DIFFERENCE = 6; // was 10; 2020-03-08 // was 22;    // was 6; 2019-09-24 // over this far off course in degrees, try harder :)
const int HEADING_PHASE_2_DIFFERENCE = 25; // was 45;   // was 12; 2019-09-24 // over this far off course in degrees, try much harder :)
const int DESIRED_ALTITUDE_SLOP_OVER = 10;   // in feet
const int DESIRED_ALTITUDE_SLOP_UNDER = 10;  // in feet
//
// ############# end DELTA settings  ################
//
// ---------------------------------------------------------------------------------------------
//
// for IC2
#include <Wire.h>

// these next 2 variables used to selectively turn off RUDDER and ELEVATOR auto function
// needed ony for exotic testing...
// ZZOPTIONS
const int O_RUDDER_ACTIVE = 1; 
const int O_ELEVATOR_ACTIVE = 1;  
//
// ZZTIMERS
const int LEDTIMER_VALUE = 500; // .5 seconds
const int FAILURE_TO_CLIMB_MSECS = 15000; // a *watch dog* timmer in ms 
                                          // once popped, slight elevator change to raise nose
// kludge, this constant passed used below in timers AND in Logger Class
const int LOGARRAYSIZEVALUE = 4;
#ifdef P_DELTA_GROUND_ONLY
const int LOGGINGTIMER_VALUE = 8000 / LOGARRAYSIZEVALUE; // 2 seconds
const int LOGGINTTIMERGPS_VALUE = 200; //  (must be < .5 * LOGGINGTIMER_VALUE)
#else
// const int LOGGINGTIMER_VALUE = 500; // for blocked logging ... 5000; // 5 seconds should be 10 later
// const int LOGGINTTIMERGPS_VALUE = 2000; // 2 seconds (must be < .5 * LOGGINGTIMER_VALUE)
// DEBUG DEBUG 2019-06-13: speed up logging to 2 seconds!  GPS turned off way below
// only every 4th log call results in output.  so .5 * 4 -> 2 seconds
// const int LOGGINGTIMER_VALUE = 10000 / LOGARRAY_SIZE; // 5000 // 2019-10-09 500 // 2000; // 5000; // 5 seconds should be 10 later
const int LOGGINGTIMER_VALUE = 2000 / LOGARRAYSIZEVALUE; // svery 2 secs!!! 2020-06-22b
const int LOGGINTTIMERGPS_VALUE = 3000;  // 2000 //  2 seconds (must be < .5 * LOGGINGTIMER_VALUE)
#endif
const int ELEVATORTIMER_VALUE = 100; // .1 was .5 was 2 seconds
const int RUDDERTIMER_VALUE = 50; // .05 was .1 was .5 was 2 seconds
// this should be a multile of RUDDERTIMER_VALUE..
const int STAGEDRUDDERTIMER_VALUE = 350;    // .35 sec
const int STAGEDRUDDER_FACTOR = STAGEDRUDDERTIMER_VALUE / RUDDERTIMER_VALUE;
const int THROTTLETIMER_VALUE = 250; // to help with level turns ??? (maybe) 500; // 500; // .5 was 2 seconds
// DEBUG this is likey way to high 2020-09-10 with 500
const int INSTRUMENTREADTIMER_VALUE = 100; // 500; // reading instruments is slow...instr
//
const int FULLTHROTTLE_MARKER_VALUE = 135; // degrees ?
const int OFFTHROTTLE_MARKER_VALUE = 45;   // degrees ?
//
// these includes contain executable code -- so their position in this code module is critical
#include "RiotSerialLayer.h"
#include "RiotCalibration.h"
#include "RiotXclock.h"
#include "RiotLogger.h"

// various configuration setting variables
// SER has been moved to RoiotSerialLayer.h
const int LED = 1;         // must be on!!!
      int LOGGING = 0;     // requires SER
      int VERBOSE = 0;     // verbose logging, requires SER
      int DASHBOARD = 0;   // requires SER
      int BT = 0;
      int GYRO = 0;
      // int COMPASS = 0;
      int ALTIMETER = 0;
      int THROTTLE = 0;
      int SERVO4 = 0;      // manage UAV actions   
      int MAG_CALB = 0;
// ***********************************

// ===================================================
// pins:
// radio input: See APM specs, PPMin = 48
// serial: 0,1 (others later with APM)
// SDA, SCL: Trinket: A4, A5; UNO_MEGA 20, 21 ?? controlled by wire library
// actual servos: See APM specs, 9, 10, 11
//         Servo1Pin .. Servo3Pin
// ZZPIN 
// Servo Output
const int Servo1Pin = 12;   // rudder
const int Servo2Pin = 11;   // elevator
const int Servo3Pin = 8;    // throttle
// LED output
const int LEDReadyPin = 25; // blue LED 
const int LEDWarnPin = 26;  // yello for APM, was 13;  // internal LED 
const int LEDAutoPin = 27;   // red
// SPI enable pins
const int baroPin = 40;
const int gyroPin = 53;
// ZZZAPM
// APM R/C input pin and details
const int PpmPin = 48;      // processed R/C input!
const int channumber = 8;
// 2020-10-07, see OLDCHANNEL macro use below
const int APM_CHANNEL_1 = (1-1); // likely rudder or elevator
const int APM_CHANNEL_2 = (2-1); // likely elevator or rudder
const int APM_CHANNEL_3 = (3-1); // likely throttle
const int APM_CHANNEL_8 = (8-1); // likely engage (old "Servo4")
int channel[channumber]; //read Channel values 
// ===================================================
#include <Servo.h>
//
// the mission settings can overwrite these with new values
struct Configuration {
  // these two tend to vary
  int cDesiredHeading;
  int cDesiredAltitude;
  // these are computed
  int cMeasuredAltitudeAtEngagement;
  int cActualDesiredAltitude;
  int cMeasuredHeadingAtEngagement;
  int cActualDesiredHeading;  
  // the following tend to be constants
  int cHeadingSlop;
  int cRudderDelta;
  int cElevatorDelta;
  int cElevatorClimbDelta;
  int cThrottleDelta;
}; 
Configuration flightConfig;
//
// barometer base millibars, etc
//
float baroBaseMB = 0;
const float localAirportFT = 5510; // swag   // was 5669; // @ KJBC
//
// ############# general module CODE starts here  ################
//
float getAngle(int duration) {
  // float a = ((duration - 900.0) / 1200.0) * 180.0;  // generic
  float a = ((duration - 1080.0) / 870.0) * 180.0;  // DX6i + Orange Receiver :)
  if (a < 0.0) a = 0.0;
  return a;
}
// pwmOnOff <---------------------------------------------------- on/off
// 
int pwmOnOff(int value, int flag, const char* comment) { 
  // value is a pulse length, not angle
  float angle = map(value, 490, 1700, 0, 180);   
  int ret = flag;
  if (angle > FULLTHROTTLE_MARKER_VALUE && flag == 0) {   // was 1600    
    ret = 1;
  } else if (angle < OFFTHROTTLE_MARKER_VALUE  && flag == 1) { // was 1000
    ret = 0;          
  } 
  return ret;  
}

#include "RiotGyroMPU6000.h"
RiotGyroSPI gyroSPI;   // note use of SPI

float getHeading() {
  digitalWrite(baroPin, HIGH);  // make sure SPI baro is off 
  float f = gyroSPI.getGyroYaw();  // this is crude!
  return(f);
}

#include "ms5611.h"
ms5611 baroSPI(baroPin);

bool calibrateBaro() {
  baroBaseMB = -9999.0;  //non-sense value
  double sum = 0;
  const int times = 25;
  for (int i=0; i < times; i++) {
    float mb = baroSPI.getPressureCompensated();
    if (mb < 200) {  // something is wrong
      return false;
    }
    sum = sum + mb;
  }
  baroBaseMB = sum / ((float) times);
  // Serial1.print("DEBUG baroBaseMB: "); Serial1.println(baroBaseMB);   
  return true;
}

//
// altitude using a few SFE_BMP180 library ideas
// 
float getAltitudeInFeet() { 
  digitalWrite(gyroPin, HIGH); 
  float mb = baroSPI.getPressureCompensated();
  // DEBUG test:
  // mb -= 10.0;
  // this math based on techniques in SFE_BMP180.h for scaling MB to Feet at "airport"
  // verified against SFE_BMP180.cpp code 2020-06-27 
  // then updated to use baroBaseMB for MB at localAirport

  const float meters = localAirportFT /3.28084;
  float P0 = (baroBaseMB/pow(1.0-(meters/44330.0),5.255)); // sea level mb (1013.25 std) based 
                                                           // off of baroBaseMB
                                                           // this could be moved to config code...
  float pa = (44330.0*(1-pow(mb/P0,1.0/5.255))) * 3.28084; // in feet
#if 0  
  // DEBUG DEBUG   
  // Serial1.print("DEBUG MB: "); Serial1.println(mb);
  // Serial1.print("DEBUG base MB: "); Serial1.println(baroBaseMB);  
  // Serial1.print("DEBUG Sea Level MB (1013.25 std): "); Serial1.println(P0);  
  serialPrintString("DEBUG ALT Feet:"); 
  Serial1.println( pa);
  float tempC = baroSPI.getTemperatureCompensated();  
  serialPrintString("DEBUG Temp C:"); 
  Serial1.println(tempC);
  delay(2000);
#endif          
  return pa;
}
#ifdef P_TESTBED
// with headers and no encrypt
Logger fLog(fLog.MD_ALL - fLog.MD_ENCRYPT); // - fLog.MD_COMMANDS ); // - fLog.MD_GPS); // - Flog.MD_HDR);
#else
// no headers in logging
Logger fLog(fLog.MD_ALL - fLog.MD_ENCRYPT - fLog.MD_HDR ); // - fLog.MD_COMMANDS ); // - fLog.MD_GPS); 
#endif
//
#include "RiotXservo.h"
#include "RiotController.h"
#include "RiotThrottleController.h"
#include "RiotElevatorController.h"
#include "RiotRudderController.h"
#include "RiotAgent007.h" // <--- really "FlightAgent Class"
Xclock flightClock;
//
// eXtended Servos -- drivers for real outputs (servos)
Xservo Xservo2(Servo2Pin);  // elevator
Xservo Xservo1(Servo1Pin);  // rudder
Xservo Xservo3(Servo3Pin);  // throttle

// controllers that drive Servos
ThrottleController controlThrottle(&Xservo3);
ElevatorController controlElevator(&Xservo2, &controlThrottle);
RudderController controlRudder(&Xservo1, &controlElevator); // , &flightClock, STAGEDRUBBERTIMER_VALUE);

// Agent [[ Pilot ]]
FlightAgent flightAgent = FlightAgent(&controlElevator, &controlRudder, &controlThrottle);
// other global variables  ZZGLOBAL
// the biggies:
int missionStepCnt = 0;
float measuredHeading = -1;
float measuredAltitude = -1;
// swags for controls at start
float measuredThrottle = 5;
float measuredRudder = 90;
float measuredElevator = 90;
float measuredServo4 = 5;
//
// control active?
int controlAgentActive = 0;  // this is critial for APM ...
int controlHeading = 0;
int controlAltitude = 0;
int controlThrust = 0;
int controlLED = 0;
// others:
int error = 0;
int Servo4PWMvalue = 0;
// added 2020-09-19 fixes
int servo4OnOff = 0;
int old4OnOff = 0;
// end add
boolean agentFullLock = false;

// timers <--------------------------- TIMERS
//   note numer of timers must be <= timerCnt in Xclock class
int LEDtimer;
int loggingTimer;
int logGpsTimer;
int elevatorTimer;
int rudderTimer;
int throttleTimer;
int missionStepTimer;
int missionActionTimer;
int manualActionTimer;
int stagedRudderTimer;
int instrumentReadTimer;

bool changed = false;
//
// ZZSETUP
void setup() { 
// ****************************************************************
// ----------------------------------------------------------------
  // desired heading... (may not be magentic)
  flightConfig.cDesiredHeading = mission.mStep[0].mHeading;
  flightConfig.cHeadingSlop = HEADING_MAX_SLOP; 
  flightConfig.cRudderDelta = RUDDER_FULL_DELTA;   // rudder rigth/left delta; 
  // altitude...  
  flightConfig.cDesiredAltitude = mission.mStep[0].mAltitude;   // can also be a value in feet :)
  flightConfig.cElevatorDelta = ELEVATOR_FULL_DELTA; // / 3; // 2; 
  flightConfig.cElevatorClimbDelta = ELEVATOR_CLIMB_DELTA;
  // power/throttle
  flightConfig.cThrottleDelta = THROTTLE_FULL_DELTA; 
// ----------------------------------------------------------------  
// ****************************************************************
//  
  boolean ConfigError = false;
  LOGGING = 1; // for now
  switch (mission.mMode) {
    case O_PASSTHRU:
    case O_PASSTHRU_CALIBRATE:
      GYRO = 0;
      ALTIMETER = 0;
      MAG_CALB = 0;       
      SERVO4 = 1;    
      if (mission.mMode == O_PASSTHRU_CALIBRATE) {
        GYRO = 1;
        MAG_CALB = 1;
        VERBOSE = 1;
      }
      break;
    // case O_HOLD_ALTITUDE:      
    case O_HOLD:
      MAG_CALB = 0; // check this !!!
      VERBOSE = 0; // quiet for now, was 1; // SEPT17 DEBUG
      DASHBOARD = 0;
      ALTIMETER = O_ELEVATOR_ACTIVE; 
      GYRO = O_RUDDER_ACTIVE; 
      THROTTLE = 1;    // Altitude control may change throttle setting 
      SERVO4 = 1;
      break;           
  }
  if (!SER) LOGGING = 0;
  if (!SER) VERBOSE = 0;
  if (!SER) DASHBOARD = 0;
  const char startMsg[] = "Setup: ";
#ifdef UNO_MEGA  
  // use UNO_MEGA Serial class
  if (SER) SERIAL.begin(9600);
  serialPrint(startMsg); 
  serialPrintln("(UNO/MEGA)");   
#else
  // for Trinket Pro
  // use brute-force bit-banger routines
  // constants
  const int SerialRX_BT = 0; // 
  const int SerialTX_BT = 1; //    
  if (SER) {
    InitSoftUART(SerialRX_BT, SerialTX_BT); 
    UART_print(startMsg);
    UART_println("(Trinket)");     
  }
#endif  
  if(SER) {
    serialPrintString("Vers: ");
    serialPrintIntln(VERSION); 
#ifdef PLANE_NAME
    serialPrintString("PL: ");
    serialPrintStringln(PLANE_NAME);
#endif
#ifdef C_SETUP_2
    serialPrintString("!!C_2; ");
#endif
#ifdef P_DELTA_GROUND_ONLY
    serialPrintString("!!GRD ONLY; ");
#endif
#ifdef TEST_ALT_MANUAL_MODE
    serialPrintString("!!TM - ALT; ");
#endif  
    if (MAG_CALB) {
      serialPrintString("!!MAG-CALB; ");    
    }        
    serialPrintString("MS: ");
    serialPrintStringln(mission.mName);
    serialPrintString("ActionTimer (Secs): ");
    serialPrintIntln((mission.mActionTimer / 1000));  
    serialPrintString("Action: ");
    if (mission.mAction == A_LOCK) {
      serialPrintStringln("LOCK"); 
    } else {
      serialPrintStringln("TEMP");  
    }  
#ifdef P_TEST_SKIP_FAIL_CLIMB 
    serialPrintStringln("!!TM - Fail Climb Check Off");
#else    
    serialPrintString("Fail Climb Timer (Secs): ");  
    serialPrintIntln((FAILURE_TO_CLIMB_MSECS / 1000));    
#endif        
    serialPrintString("Firewall Alt (ft): ");
    serialPrintIntln(mission.mMaxAGLinFeet);
    serialPrintString("Steps: ");
    serialPrintIntln(mission.mSteps);
    serialPrintString("Alt Step1 (ft): ");
    serialPrintIntln(mission.mStep[0].mAltitude);
    serialPrintString("Head Step1 (deg): ");
    serialPrintIntln(mission.mStep[0].mHeading);      
  } // end of SER

  if (LED == 1) {
    pinMode(LEDWarnPin, OUTPUT);
    digitalWrite( LEDWarnPin, HIGH);    
    pinMode(LEDReadyPin, OUTPUT);
    digitalWrite( LEDReadyPin, HIGH);
    pinMode(LEDAutoPin, OUTPUT);
    digitalWrite( LEDAutoPin, HIGH);        
  }
  Wire.begin();
  if (GYRO == 1) {
    digitalWrite(baroPin, HIGH);  // make sure SPI baro is off
    // this calib value is kept in the Gyro class snd applied in the class methods...
    float f = gyroSPI.setupGyro(gyroPin, LEDWarnPin);
    // DEBUG GYRO
    int fi = f * 1000;
    int ri = fi % 1000;  // frac
    fi = fi / 1000;   // whole
    serialPrintString("DEBUG fi: ");
    serialPrintInt(fi);
    serialPrintString(".");
    serialPrintIntln(ri);
    // while (1) { }  
  } 
  if (ALTIMETER == 1) { 
    digitalWrite(gyroPin, HIGH);  // make sure SPI gyro is off
    baroSPI.init();    
    delay(100);  // not sure still needed       
    if (!calibrateBaro()) {
      serialPrintStringln("Baro Calibrate Failed!");
      ConfigError = true;
    } else {   
      serialPrintStringln("Baro Calibrate OK!");
      measuredAltitude = getAltitudeInFeet(); // ((long)ms5611.getPressureCompensated());        
      serialPrintString("Cur Alt: ");
      serialPrintIntln(measuredAltitude);                
    }
  }  

  if (GYRO == 1) {
      // wait for gyro to get some active time
      delay(100);
      measuredHeading = getHeading(); // in degress, already rotated, etc. 
      serialPrintString("Est Head: ");       
      serialPrintIntln(measuredHeading);
  }  

  if (!ConfigError) {
    LEDtimer = flightClock.allocateTimer();
    loggingTimer = flightClock.allocateTimer();
    logGpsTimer = flightClock.allocateTimer();
    elevatorTimer = flightClock.allocateTimer();
    rudderTimer = flightClock.allocateTimer();
    throttleTimer = flightClock.allocateTimer();
    missionStepTimer = flightClock.allocateTimer();
    missionActionTimer = flightClock.allocateTimer(); 
    manualActionTimer = flightClock.allocateTimer();
    stagedRudderTimer = flightClock.allocateTimer();
    instrumentReadTimer = flightClock.allocateTimer();
    if (LEDtimer + loggingTimer + logGpsTimer +
        elevatorTimer + rudderTimer + throttleTimer +
        missionStepTimer + missionActionTimer + manualActionTimer +
        stagedRudderTimer + instrumentReadTimer < 0) {
        ConfigError = 1; 
        serialPrintln("Timer Fail!");
    }
  }  
  if (ConfigError) {             
    // crap, have config error!  hold the show 
    while (1) {
      if (LED) {   
        digitalWrite( LEDWarnPin, digitalRead(LEDWarnPin) ? LOW : HIGH);  // blink LED
      }  
      delay(250);
    }
  }

  // TIMER setup!
  flightClock.startTimer(LEDtimer, LEDTIMER_VALUE);
  flightClock.startTimer(loggingTimer, LOGGINGTIMER_VALUE);
  flightClock.startTimer(elevatorTimer, ELEVATORTIMER_VALUE);
  flightClock.startTimer(rudderTimer, RUDDERTIMER_VALUE); 
  flightClock.startTimer(throttleTimer, THROTTLETIMER_VALUE);   
  // flightClock.startTimer(manualActionTimer, MANUALACTIONTIMER_VALUE);  
  flightClock.startTimer(instrumentReadTimer, INSTRUMENTREADTIMER_VALUE); 
  
  // configure flightAgent
  flightAgent.configure(&flightClock, elevatorTimer, rudderTimer, throttleTimer);
  if (LED) {
    digitalWrite(LEDReadyPin, LOW); // turn on 
    digitalWrite(LEDWarnPin, HIGH); // turn off  
  }
  // startup mission steps
  missionStepCnt = 0;
  if (LOGGING) {
    fLog.logActive();  
  }  

  if (SER) {
    delay(1000); // arbitary delay helps the logger blocking, < 1000 fails ....    
    serialPrintString("M: ");
    switch (mission.mMode) {
      case (O_PASSTHRU):      serialPrintStringln("O_PS"); break;
      case (O_PASSTHRU_CALIBRATE): serialPrintStringln("O_PC"); break;   
      case (O_HOLD):         
        serialPrintString("O_H, h:");
        serialPrintInt(flightConfig.cDesiredHeading);
        serialPrintString(" alt:");
        serialPrintInt(flightConfig.cDesiredAltitude);        
        break;         
    }                 
    serialPrintStringln(" *run*");
  }  
} // end-of-setup()

//
// TASKS:
//
// task0: get current R/C inputs, set mission parameters  
//
void task0() {  
  // changed this for APM new code ...
  // waits until synchronize arrives > 4 miliseconds
  // this time value can be sloppy, so servos engaged is OK
  // DEBUG DEBUG
  // serialPrintString("get duration: ");
  long int duration = pulseIn(PpmPin, HIGH);
  // serialPrintIntln(duration);
  // Serial.println(duration);
  if(duration > 4000) //If pulse > 4 miliseconds, continues
  {  
    // stops gitter by turning off servos when reading pulsed input ("look Ma, no brains!")
    // this approach is brutal but has eliminated the "jitters"
    Xservo1.detachXservo();
    Xservo2.detachXservo();
    Xservo3.detachXservo();    
    for(int i = 1; i <= channumber; i++) { //Read the pulses of the remainig channels
      channel[i-1]= pulseIn(PpmPin, HIGH);
    }
    Xservo1.attachXservo();
    Xservo2.attachXservo();
    Xservo3.attachXservo(); 
    // delay(1);
    delayMicroseconds(50);  // was 25 -- failed   
#if 0
    for(int i = 1; i <= channumber; i++) { //Prints all the values readed
      SERIAL.print(" CH"); //Channel
      SERIAL.print(i); //Channel number
      SERIAL.print(": "); 
      SERIAL.print(channel[i-1]); //Print the value
    }
    SERIAL.println("");
#endif
    // 2020-08-22: change all servo/angle ranges from 0, 180 to <startAngle>, <endAngle>
    const int startAngle = 30;
    const int endAngle = 150;
    // input to map is pwm value, output is an angle value!
    // these are start pwm, end pwm, startAngle, endAngle
    // 2020-10-07, switch two inputs to match receiver!
#ifdef OLDCHANNEL    
    measuredRudder =   map(channel[APM_CHANNEL_1], 490, 1700, startAngle, endAngle);
    measuredElevator = map(channel[APM_CHANNEL_2], 490, 1700, startAngle, endAngle);
#else
    measuredElevator =   map(channel[APM_CHANNEL_1], 490, 1700, startAngle, endAngle);
    measuredRudder = map(channel[APM_CHANNEL_2], 490, 1700, startAngle, endAngle);
#endif    
    // 2020-08-23:  start throttle higher 490 to 515 ...
    measuredThrottle = map(channel[APM_CHANNEL_3], 515, 1700, startAngle, endAngle);
    measuredServo4 =   map(channel[APM_CHANNEL_8], 490, 1700, startAngle, endAngle);  // odd channnel # is correct
#ifdef VERBOSE
    serialPrint("Radio Angles Ele: ");
    serialPrintInt(measuredElevator); 
    serialPrint(" Rud: ");
    serialPrintInt(measuredRudder);  
    serialPrint(" Thr: ");
    serialPrintInt(measuredThrottle); 
    serialPrint(" Swth: ");
    serialPrintIntln(measuredServo4);        
#endif    
  }     
  // end APM new code
  // missionStepCnt is critical ..
  flightConfig.cDesiredHeading = mission.mStep[missionStepCnt].mHeading;  
  flightConfig.cDesiredAltitude = mission.mStep[missionStepCnt].mAltitude; 
  // manage LED Auto on/off
  if (controlAgentActive == 0) {
    // manual mode
    digitalWrite(LEDAutoPin, HIGH);
    fLog.logAction2(fLog.logMode,'M');     
  } else {
    // agent/auto mode 
    digitalWrite(LEDAutoPin, LOW); 
    fLog.logAction2(fLog.logMode,'A');            
  }
} // end of task0
      
//
// task1: read instruments, as requested and drive servos in manual mode
//
void task1() {  
  // don't read instruments/sensors too often:
  //     reading them can be SLOW & this chokes up other actions like reading R/C input!
  // DEBUG BUT need frequest reads!  2020-09-10
  if (flightClock.checkTimerWithRestart(instrumentReadTimer)) {        
    if (GYRO) {
      measuredHeading = getHeading(); // in degress, already rotated, etc.  
    }
    if (ALTIMETER) { 
#ifndef TEST_ALT_MANUAL_MODE    
      // actual flight mode...
      measuredAltitude = getAltitudeInFeet();  // gotta love those feet!
#else    
      // simulatedAltitude
      // still needs code fixes for APM  
      char c[2];
      c[0] = UART_RecieveWait(10);  // 10 is a swag
      if (c[0] != -1) {
        c[1] = 0;
        serialPrintString(c);
        switch (c[0]){  // '+' is up 100, '-' is down 100, 'R' is reset
          case '+': measuredAltitude += 100; break;
          case '-': measuredAltitude -= 100; break;
          case 'R': measuredAltitude = getAltitudeInMeters()*3.28084; break;
        }       
      }
#endif
    }
  }
  // Logging: note mix of digital instrument readings and throttle
  {
    int tempThr = measuredThrottle; // manual flight
    if (controlAgentActive == 1) {
      tempThr = controlThrottle.getServoCurrent();   
    }
    fLog.logReadings(measuredHeading, measuredAltitude, tempThr);  
  }  

#ifdef C_SETUP_1
  while (1) { // broken....
    serialPrint(">>> C_SETUP_1, EL:");
    serialPrintInt(measuredElevator);    
    serialPrintString(", RD:");   
    serialPrintInt(measuredRubbed);
    serialPrint(", TH:");
    serialPrintIntln(measuredThrottle);  
    delay(2000);   
  }
#endif 
 // critial APM code:
 if (controlAgentActive == 0) {
   // Guess had better assume manual mode
   // converting float to int...
   Xservo1.write(measuredRudder);
   Xservo2.write(measuredElevator);
   Xservo3.write(measuredThrottle);   
 }
} // end of task1
          
// 
// engage and disengage Flight Agent <----------------- SERVO4 
//
void task2() {   
  if (SERVO4 == 1) {
    // change for APM -- this needed to be improved, using angles is better...
    // but this servo4 value is using the pulse width :)
    Servo4PWMvalue = channel[APM_CHANNEL_8];
    // end APM change    
    changed = false;
    // in non-agent modes, let SERVO4 setting drive main LED
    // SERVO4:
    if (mission.mMode == O_PASSTHRU || mission.mMode == O_PASSTHRU_CALIBRATE) {
      controlLED =  pwmOnOff(Servo4PWMvalue, controlLED, "Control LED");
      controlAgentActive = 0; 
      // insurance
      controlHeading = 0;
      controlAltitude = 0;
      controlThrust = 0;      
    } else { 
      // AGENT MODE -- Arduino might take control 
      // 2020-09-19: fixes start here
      old4OnOff = servo4OnOff;  // first time thru this might be junk....
      servo4OnOff = pwmOnOff(Servo4PWMvalue, controlAgentActive, "Agent Control"); 
      int oldAgentActive = controlAgentActive;
      switch (servo4OnOff) {
        case (1): // turn on automode
          controlAgentActive = 1;          
          break;
        case (0): // consider turning off automode
                  // agentFullLock might not permit this....
          if (agentFullLock == false) { // simple case
            // fLog.LogMsg("LKon Off"); // serialPrintln("LKon Off");
            // fLog.LogMsg("Auto Off"); // serialPrintln("Auto Off");             
            controlAgentActive = 0;
          } else {
            // fLog.LogMsg("LKon On"); // serialPrintln("LKon On");  
            // assume we will leave automode on
            controlAgentActive = 1;
            // see if assumption is wrong 
            // servo4OnOff is 0, so trying to turn off
            // and agentFullLock is true, so lock is set
            if (old4OnOff == 1 &&   // poor man's edge trigger :)
                measuredThrottle >= FULLTHROTTLE_MARKER_VALUE) { // and throttle above/equal full throttle
              // fLog.LogMsg("Auto Off"); // serialPrintln("Auto Off");                  
              controlAgentActive = 0; 
              agentFullLock = false;          
            }
          }
      } /* end of switch */
      // junk -- remove
#if 0           
      // enforce FullLock: release only if at full throttle when SERVO4 goes off
      if (agentFullLock == true && 
        oldAgentActive == 1 &&
        controlAgentActive == 0) { // special case           
        if (measuredThrottle < FULLTHROTTLE_MARKER_VALUE) {
          controlAgentActive = 1;  // undo request!
        }      
      }
#endif      
      // end 2020-09-19 fixes
      // migrate code changes back to RiotLite_V6 2020-07-10, marked with FIX01 and FIX02
      // something has changed, mark this and log it...  FIX01
      if (oldAgentActive != controlAgentActive) {
        changed = true;
        fLog.logAgent(controlAgentActive);
      }
      if (controlAgentActive == 1 && // agent is now on 
          changed && // has just changed // FIX01 
          mission.mSteps > 1 && // have multistep mission
          mission.mStep[0].mDuration > 0) { // has a time value
        // start timer
        missionStepCnt = 0; // start/restart mission        
        flightClock.startTimer(missionStepTimer, mission.mStep[missionStepCnt].mDuration);
      }      
    }
  }
} // end of task2

//
// task3: flip/unflip each control to Flight Agent      
//
void task3() {       
  // SERVO4:
  if (GYRO) { // rudder -- servo1; heading
    if (changed) {
      if (controlAgentActive == 1) { 
        flightConfig.cMeasuredHeadingAtEngagement = measuredHeading; // needed in special cases 
        controlHeading = 1;                         
      } else {
        controlHeading = 0;
      }
    }
  }
  // SERVO4:
  if (ALTIMETER) { // elevator -- servo2; altitude
    if (changed) {
      if (controlAgentActive == 1) {
        if (LED) {
          digitalWrite( LEDAutoPin, HIGH); // on
        }
        flightConfig.cMeasuredAltitudeAtEngagement = measuredAltitude; // needed in special cases
        controlAltitude = 1;                      
      } else {
        controlAltitude = 0;          
        if (LED) {
          digitalWrite( LEDAutoPin, LOW); // off
        }        
      }
    } 
  }
  
  // SERVO4:
  if (THROTTLE) { // throttle -- servo3; thrust, which impacts altitude and speed
    if (changed) {
      if (controlAgentActive == 1) {                     
        controlThrust = 1;    
      }  else {
        controlThrust = 0;
      }
    }    
  }
 
  // SERVO4: flightAgent actual activate/deactivate
  if (changed) {
    if (controlAgentActive == 1) {
      // migrate code changes back to RiotLite_V6 2020-07-10, marked with FIX01 and FIX02      
      // start FIX02:
      // if going active, update heading and elevation
      // doing this prevents odd actions when the time to next scheduled reads is long
      measuredHeading = getHeading();
      measuredAltitude = getAltitudeInFeet();       
      // end of FIX02 
      // changed for APM           
      flightAgent.setServoBases(measuredElevator,measuredRudder, measuredThrottle);                                                            
      if (flightAgent.getActive() == false) { 
        // in the event of Agent restart        
        controlThrottle.setForce(false);                   
        controlThrottle.clrFailedClimb();                     
        flightAgent.setActive(true);
        if (VERBOSE == 1) {
          serialPrintln("Agent ON!");
        }           
      }
      // start lock timer ..
      flightClock.startTimer(missionActionTimer, mission.mActionTimer);
      agentFullLock = false;   
      controlThrottle.setForce(false);  // well, a little bit of brute force!                      
    } else {
      flightAgent.setActive(false);
      // doing these makes logging more accurate
      controlThrottle.setForce(false); 
      controlThrottle.clrFailedClimb();             
      if (VERBOSE == 1) {
        serialPrintln("Agent OFF!");
      }  
    }
  }
  else { // !changed
    // manage auto mission actions
    if (controlAgentActive == 1 ) {
      if (flightClock.checkTimer(missionActionTimer)) {
        
        if (mission.mAction == A_LOCK) {
          if (agentFullLock == false) {
            fLog.LogMsg("*A_LOCK*");
            agentFullLock = true;
          }
        } else {
          if (mission.mAction == A_TEMP) {
            fLog.LogMsg("*A_TEMP*");   
            // lower throttle to visually warn that timer has popped!         
            controlThrottle.setForce(true);  // well, a little bit of brute force!
          }              
        }
      }
    }
  } // changed?  
}  // end of task3

//
// task4: let the Flight Agent act as needed
//
void task4() {
  int measuredSpeed = -1; // sigh!
  // it all happens here .... fly babe, fly! This calls three *lesser* act() methods   
  flightAgent.actAgent(measuredAltitude, measuredHeading, measuredSpeed,     // units: feet, degrees, 0
                       measuredRudder, measuredElevator, measuredThrottle);        // units: degrees 
  // phase 4: misc. logging, blinks, and such, are coded here
  // LED blinking
  if (flightClock.checkTimerWithRestart(LEDtimer)) {  
    // timestampOutput = micros(); // check is based on this ...    
    if (mission.mMode == O_PASSTHRU || mission.mMode == O_PASSTHRU_CALIBRATE) { 
      // don't do this for other MODEs ...     
      if (LED) {  // LED on where SERVO4 is full
        if (controlLED) {
          digitalWrite( LEDWarnPin,  HIGH);  // LED on          
        } else {
          digitalWrite( LEDWarnPin, digitalRead(LEDWarnPin) ? LOW : HIGH);  // blink LED
        }
      }
    }
  }  
  // timed logging ZZLOG
  // don't flood log with M entries...
  bool showLogData = true;  // mugged for some debugging...  
  // always log heading indicator, needed to detect spins in log
  fLog.logAction2(fLog.headingIndicator,(measuredHeading / 36));  // single digit approx of heading!  
  if (flightClock.checkTimerWithRestart(loggingTimer)) { 
    fLog.logAction2(fLog.pStep,missionStepCnt);
    fLog.logAction2(fLog.locked,agentFullLock);
    if (controlThrottle.getFailedClimb()) {
      fLog.logAction2(fLog.failedToClimb, 1); 
    } else {
      fLog.logAction2(fLog.failedToClimb, 0);        
    }
    if (showLogData) {
      fLog.logSend(flightClock, logGpsTimer);
      // flightClock.startTimer(logGpsTimer, LOGGINTTIMERGPS_VALUE); 2019-10-09 moved ...          
    } else {
      fLog.count++; // very dirty 
    }
    flightClock.startTimer(logGpsTimer, LOGGINTTIMERGPS_VALUE);    
  } /* end of loggingTimer */ 
}  // end of task4

//  
// task5: see if we need to move to next mission step
//
void task5() {
  //
  // this messy code allows a single step mission not to end, voided by A_TEMP however ....
  if (controlAgentActive == 1 
      && missionStepCnt < (mission.mSteps - 1)) { // active and is there is another step                 
    if (mission.mStep[missionStepCnt].mDuration > 0) { // likely a timer is going
      if (flightClock.isActive(missionStepTimer)) { // yup, we have an active timer
        if (flightClock.checkTimer(missionStepTimer)) { // and has it popped
          // yup, yup         
          missionStepCnt++;         
          if (mission.mStep[missionStepCnt].mHeading == MIS_END) {   
            // sh*t, what do I do???   
            // this lowers the throttle but permits heading to be followed 
            controlThrottle.setForce(true);  // well, a little bit of brute force!     
          } else if (mission.mStep[missionStepCnt].mDuration > 0) {
            flightClock.startTimer(missionStepTimer, mission.mStep[missionStepCnt].mDuration);             
          } else {
            // no timer needed, just stay here
          }
        }     
      }
    } 
  }
}  // end of task5
  
// Arduino LOOP
// ZZZLOOP
void loop() { 
  // serialPrint("#");
  task0(); 
  task1();
  task2();
  task3();
  task4();
  task5(); 
  // sam plays this a lot! 
}

// end of program
