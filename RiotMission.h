// RiotMission.h
//
// 2020-06-24 Merge Completed
//
//

// NO CHANGE to these constants, they are used in the mission settings...
const int DYN_VAL = -9998;               // do not change this :)
const int MIS_END = -9997;               // do not change this :)
const int A_LOCK = 1;         // LOCK OUT SWTICH             // do not change this :)
const int A_TEMP = 2;         // MOTOR OFF UNTIL SWITCH OFF  // do not change this :)
const int NOT_IN_USE = -9999;            // do not change this :)
// modes of operation:
const int O_PASSTHRU = 0;           // for manual flight
const int O_PASSTHRU_CALIBRATE = 1; // NO FLIGHT!!
const int O_HOLD = 2;               // for flight, when engaged, follow plan
// END NO CHANGE ^^^^^

struct stepMission {
  int mHeading;      // degrees or DYN_VAL
  int mAltitude;     // feet or DYN_VAL
  unsigned long mDuration;    // Duration in minutes, 0 is no limit on step  
};        
struct planeMission {
  unsigned int mActionTimer;  // milliseconds
  int mAction;       // see SERVO4 code
  int mMaxAGLinFeet; // in feet
  int mMode;         // See O_xxx values in code
  char mName[16];    // Mission/Plane name 
  int mAltitudeJump; // feet to rise when engaged
  int mSteps;        // Mission Steps, often 1, max 4
  stepMission mStep[4];
}; 
