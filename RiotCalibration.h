// RiotCalibration.h
// 2020-06-24 Merge Completed
//

// notes:

// ############# CALIBRATION notes for servos  ################
// HORROR: servos must be/can be manually calibrate for each radio & servo (that is plane) combination  <---- HORROR!
// servo calibration
// see ZZCALIBRATE below for *notion* of table to populate with calibration values:
// 1) Use '#define C_SETUP_1' (below) to find the needed "timer" (angle) readings by each servo:
//    * min, middle, max.  Have an accurate middle is critical.
//    * use these to create new ServoCalibration entries for each servo
//    * be sure to understand the different between timer and angle entries :)
// 2) Put first entries in table (for timer values, guess at angles)
// 3) Use '#define C_SETUP_2' to find the actual "angles" for each servo
//    * This setting zeros the 'delta' values
//    * Makes it possible to read the actual throw angles
//    * Makes it possible to see the servo behavior when autopilot engaged 
// 4) Use P_DELTA_GROUND_ONLY to increase deltas for sanity check of throw directions
//
// DO NOT FLY PLANE with C_SETUP_1 or C_SETUP_2 engaged!
//
// ############# end CALIBRATION notes for servos  ################

//
const int SC_LOWER = 0;
const int SC_LOBLIP = 1;
const int SC_MIDDLE90 = 2;
const int SC_HIBLIP = 3;
const int SC_UPPER = 4;

struct ServoCalibration {
  int preReceiverOffet;
  int receiverValues[5];
  int blipAngle;
  int minRangeAngle;
  int maxRangeAngle;
  int minClipAngle;
  int maxClipAngle;
}; 
// dummy values
//                                            ---------- timer-----------  ***  ^range^  ^clip^
//                                            pre low mid-  mid  mid+ hgh, blp, min, max min, max 
const ServoCalibration servoCalbElevator   = {0,  0,   80,   80,  80, 203, 0,    40, 150,  0, 179};  // 118
const ServoCalibration servoCalbRudder    =  servoCalbElevator;
const ServoCalibration servoCalbThrottle  =  servoCalbElevator;
