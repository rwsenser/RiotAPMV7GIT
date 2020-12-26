// riotElevatorController.h
// 2020-06-24 Merge Completed
//
//ZZEC ZZEL
 class ElevatorController: public Controller {
  private:
    int maxAltitude;
    int desiredAltitude;
    int minAltitude;
    int elevatorOverrideMode;
    ThrottleController* throttleController;

  public:
      const int EL_NORMAL = 1;
      const int EL_HARD_CLIMB = 2;// currently used only in toggled hard turn :)

  ElevatorController(Xservo* _s, ThrottleController* _m): Controller(_s, 'E') {
    throttleController = _m;
    elevatorOverrideMode = EL_NORMAL;
  }  // so super ....

  void configure() {
    setServoDelta(flightConfig.cElevatorDelta);  // in degrees
    return;
  }  
  void setParameters(int _v1, int _v2, int _v3) {
    maxAltitude = _v1;
    desiredAltitude = _v2;
    minAltitude = _v3;         
  }
  void getParameters( int& _v1, int& _v2, int& _v3) {
    _v1 = maxAltitude;
    _v2 = desiredAltitude;
    _v3 = minAltitude;
  }
  void setOverrideMode(int _m) {
    elevatorOverrideMode = _m;
  }
  int getOverrideMode() {
    return elevatorOverrideMode;
  }
  
  void setSpecialElevatorParameters( int encodedBaseAltitude ) {
    // set hase altitude and bounds, watch special cases
    // done for EACH interation, sloppy but bug free
    int baseAltitude = encodedBaseAltitude;     
    if (encodedBaseAltitude == DYN_VAL) {
      baseAltitude = (flightConfig.cMeasuredAltitudeAtEngagement + mission.mAltitudeJump); // up <feet> to show engaged
    }
    // else {
    //  baseAltitude = mission.mStep[1-1].mAltitude;  // in feet
    // }   
    flightConfig.cActualDesiredAltitude = baseAltitude; // helpful for logging     
    setParameters(baseAltitude + DESIRED_ALTITUDE_SLOP_OVER, baseAltitude, baseAltitude - DESIRED_ALTITUDE_SLOP_UNDER); 
    return;  
  }

  // elevator
  void act(int curAltitude, int curThrottle) { // , bool *failedClimbPtr) {
    // these must be set for EACH call to act....
    setSpecialElevatorParameters(flightConfig.cDesiredAltitude); // calls setParameters, which sets max and min Altitude
    // poor coding style
    int delta = 0;  
    int angle;
    static int anglePrev = -1;
    int servoAngle = getServoBase();  
#ifdef C_SETUP_2 
#else 
    // active code
    // SPECIAL CASES...
    if (curAltitude > mission.mMaxAGLinFeet) { // enforce altitude firewall
      // special case 1: over Max AGL      
      // serious descent via motor (lower power)
      delta = 0; 
      throttleController -> setThrottleSetting(throttleController -> MC_FORCE);  
      fLog.LogMsg(">MaxAGL!");     
    } else if (throttleController -> getFailedClimb() ){
      // special case 2: this is a catch all for motor issue(s), i.e.: coming down
      // better glide if nose is raised
      delta =  GLIDE_DELTA_MULTIPLIER * flightConfig.cElevatorClimbDelta ; // moderate nose raised 
      fLog.logAction2(fLog.elevatorUp, 7); // 7 is a visual marker in the log
      // fLog.LogMsg("FailClimb!");                
    } else {
      // NORMAL CASES       
      if (curAltitude > maxAltitude) { // lower power setting
        // DESCEND via motor and elevator
        delta = -getServoDelta(); 
        throttleController -> setThrottleSetting(throttleController -> MC_DESCEND);
        fLog.logAction(fLog.motorDecrease, 1);  
        fLog.logAction(fLog.elevatorDown, 1);              
      } else if (curAltitude < minAltitude) {
        // CLIMB via motor and elevator
        if (elevatorOverrideMode == EL_NORMAL) {
          delta =  flightConfig.cElevatorClimbDelta ; // slightly raise nose       
        } else { // likely EL_HARD_CLIMB
          delta =  HEAVY_CLIMB_DELTA_MULTIPLIER *
                 flightConfig.cElevatorClimbDelta ; // raise nose drastically       
        }
        throttleController -> setThrottleSetting(throttleController -> MC_CLIMB);
        fLog.logAction(fLog.motorIncrease, 1); 
        fLog.logAction(fLog.elevatorUp, 1);          
      } else {
        // LEVEL flight
        delta = 0;            
        throttleController -> setThrottleSetting(throttleController -> MC_LEVEL); 
        fLog.logAction(fLog.motorBase, 1);
        fLog.logAction(fLog.elevatorLevel, 1);                     
      }
    }  
#endif  
// end ifdef C_SETUP_2  
    angle = writeServoAngleTrans5(servoAngle, delta, &servoCalbElevator);    
    if (angle != anglePrev) {
      anglePrev = angle;
    }
  }
 }; // end of ElevatorController
