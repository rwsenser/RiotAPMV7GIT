// RiotRudderController.h
// 2020-06-24 Merge Completed
//
// ZZRC
class RudderController: public Controller {
  private:
    ElevatorController* elevatorController;
    // these temp variables are the desired heading with slop factors
    int magHeadingPlus;
    int magHeadingMinus;
    // these are the above, normalized to 0 to 259
    int trueHeading;
    int trueHeadingPlus;
    int trueHeadingMinus; 
    int stagedCnt;
    bool stagedToggle; 
// 2019-10-21: for dyno rudder
    int lastDirection;              
  public:
    // const int RC_LEFT_STEP3 = -3;  
    // const int RC_LEFT_STEP2 = -2;
    const int RC_LEFT = -1;
    const int RC_STRAIGHT = 0;
    const int RC_RIGHT = 1;   
    // const int RC_RIGHT_STEP2 = 2;
    // const int RC_RIGHT_STEP3 = 3;    
  public:
  RudderController(Xservo* _s, ElevatorController* _m /* , Xclock* _c, int _t */ ): Controller(_s, 'R') {
    elevatorController = _m;
    // rudderStagedTimer = _t;
    // clockPtr = _c;
    // 2019-10-21: for dyno rudder
    lastDirection = 0;    
  }

  int makeRaw(int mag) {  
    return mag;  
  }

  int norm(int val) {
    return ((val + 720) % 360);
  }
  // ZZXX
  void configure() {
    setServoDelta(flightConfig.cRudderDelta);   // this sets Servo movement range
    stagedCnt = 0;
    stagedToggle = false;
    return;
  }  
  // this turkey tries to give a sense of being navigated by staying on a course
  int navigate(int actHeading) { // raw is the reality   
      int result = RC_STRAIGHT;
      flightConfig.cActualDesiredHeading = flightConfig.cDesiredHeading;    
      if (flightConfig.cDesiredHeading == DYN_VAL ) {
        flightConfig.cActualDesiredHeading = flightConfig.cMeasuredHeadingAtEngagement;
        configure();  // this needs to be cleaned up!   
      } 
      // should below be in configure??  
      magHeadingMinus = norm(flightConfig.cActualDesiredHeading - flightConfig.cHeadingSlop); // desired heading lower limit (minus)
      magHeadingPlus = norm(flightConfig.cActualDesiredHeading + flightConfig.cHeadingSlop); // desired heading upper limit (plus)
      trueHeadingMinus = norm( makeRaw(magHeadingMinus)); // true desired heading, lower limit
      trueHeading = norm(makeRaw(flightConfig.cActualDesiredHeading)); // true desired heading
      trueHeadingPlus = norm(makeRaw(magHeadingPlus));  // true desired heading, upper limit     
      // optional rotate to get math to work when compass is 0 .. 359 :) 
      int rotTrueHeading = trueHeading;
      int rotTrueHeadingPlus = trueHeadingPlus;
      int rotTrueHeadingMinus = trueHeadingMinus;
      int rotActHeading = actHeading;
      // to this point headings should be 0 .. 359
      // if in trueHeading < 90 or > 270, rotate
      if (trueHeading < 90 || trueHeading > 270) { 
        // rotate 180, normalize back to 0 .. 359
        // this gets us a number line with all the variable > 0
        rotTrueHeadingMinus = norm(trueHeadingMinus + 180);
        rotTrueHeading = norm(trueHeading + 180);
        rotTrueHeadingPlus = norm(trueHeadingPlus + 180);
        rotActHeading = norm(actHeading + 180);
      }
      // this code helps with making harder turns (third range) that don't destabilize
      stagedCnt++;
      if (stagedCnt > STAGEDRUDDER_FACTOR) {
        stagedToggle = !stagedToggle;
        stagedCnt = 0;
      }
      // 2019-09-24: we are trying too hard, need to honor slop factors
      // 1) if in slop range, just go rudder staight so plane can level!!!  **really**
      // 2) if out of slop range then try to adjust
      // With wigman, 3 possible actions because wigwam has a constant rate of turn 
      int lDif = rotActHeading - rotTrueHeadingPlus;
      int rDif = rotTrueHeadingMinus - rotActHeading;             
      if ((rotActHeading > rotTrueHeadingPlus) && (lDif > HEADING_MAX_SLOP)) {
        // think LEFT!
#if 0        
        if (lDif > HEADING_PHASE_2_DIFFERENCE) {
          if (stagedToggle) result = RC_LEFT_STEP3; else result = RC_LEFT_STEP2;          
        } else if (lDif > HEADING_PHASE_1_DIFFERENCE) {
          result = RC_LEFT_STEP2;
        } else {
#endif          
          result = RC_LEFT;    
        // }  
       } else if ((rotActHeading < rotTrueHeadingMinus) && (rDif > HEADING_MAX_SLOP)) {
        // think RIGHT! 
#if 0                    
        if (rDif > HEADING_PHASE_2_DIFFERENCE) {   
          if (stagedToggle) result =  RC_RIGHT_STEP3; else result = RC_RIGHT_STEP2;                  
        } else if (rDif > HEADING_PHASE_1_DIFFERENCE) {        
          result = RC_RIGHT_STEP2;
        } else {
#endif          
          result = RC_RIGHT;
        // }
      }
      if (result == RC_STRAIGHT) {
        // reset staged toggle
        stagedCnt = 0;
        stagedToggle = false;        
      }
      //  NOTE: 2020-02-25 wigwam needs the STEP2 and STEP3 off      
#if 0     
      if (result == RC_LEFT_STEP2 || result == RC_LEFT_STEP3) {
              result = RC_LEFT;
      // if (result == RC_LEFT_STEP3) {        
      //   result = RC_LEFT_STEP2;
      } else if (result == RC_RIGHT_STEP2 || result == RC_RIGHT_STEP3) {
            result = RC_RIGHT;
      }
      // else if (result == RC_RIGHT_STEP3) {        
      //  result = RC_RIGHT_STEP2;
      // }
#endif        
      return result;
   }

   // introduce wigwam into rudder controller class
   // brings in rudder_driver() function
   #include "wigwam2.h"
   
  // 2019-10-21, add rateOfChange use via call to manageRudder
  int setServoAngle( int rawCurHeading, int rateOfChangeHeading) {
    int delta = 0;
    int angle;
    int servoAngle = getServoBase();
#ifndef C_SETUP_2
    // navigate returns with direct code to set intent,
    // code below implements the intent    
    int  direct = navigate(rawCurHeading);
#else
    int direct = RC_STRAIGHT;
#endif 
    //
    // 2020-02-25: replaced old mess with simpler "wigwam" approach
    //
    // servoAngle is set above from servoBase()
    // add wigwam interface code
    // won't use preferred getServoDelta() for now, use delta from wigwam
    if (direct == RC_STRAIGHT) {
      elevatorController -> setOverrideMode(elevatorController -> EL_NORMAL);
    } else {  
      // note this is only a suggestion to elevator
      elevatorController -> setOverrideMode(elevatorController -> EL_HARD_CLIMB);  // note this is a suggestion to elevator
    }
    long timer = millis();  // wigwam has its own timers!
    // heading not currently used
    // possible direct values (from navigate):  RC_LEFT, RC_STRAIGHT, RC_RIGHT
    delta = rudder_driver(timer, rawCurHeading, direct);  // this calls "wigwam"....
    // DEBUG WIGWAM -- for ground pre test
    // note, changed table in wigwam, these fixes not used now
    // delta *= 4;
    // delta *= 3;  // 2020-06-21 fix
    angle = writeServoAngleTrans5(servoAngle, delta, &servoCalbRudder);  
    if (direct == RC_STRAIGHT) {
       fLog.logAction(fLog.rudderStraight, 1);
    } else if (direct == RC_RIGHT 
        // || direct == RC_RIGHT_STEP2 
        // || direct == RC_RIGHT_STEP3
        ) { fLog.logAction(fLog.rudderRight, 1); }
    else if (direct == RC_LEFT 
             // || direct == RC_LEFT_STEP2
             // || direct == RC_LEFT_STEP3 
             ) { fLog.logAction(fLog.rudderLeft, 1);}
    if(direct != RC_STRAIGHT &&
       direct != RC_LEFT &&
       direct != RC_RIGHT) {         
      fLog.logAction(fLog.rudderOther, 1); 
    }     
  }
  // rudder
  // changed 2019-10-21 to pass thru rateOfChange
  void act(int rawCurHeading, int rateOfChangeHeading) {
    setServoAngle(rawCurHeading, rateOfChangeHeading);             
    return;
  }           
};
