// RiotThrottleController.h
// 2020-06-24 Merge Completed
//
//ZZTC ZZTH
class ThrottleController: public Controller {
  private:
    int altitudeMode;
    boolean forceLand;
    int prevAltitude;
    bool failedClimb;
    int failedClimbCnt;
  public:
    const int MC_CLIMB = 0;
    const int MC_LEVEL = 1;
    const int MC_DESCEND = 2;
    const int MC_FORCE = 3;
  public:
  ThrottleController(Xservo* _s): Controller(_s, 'M') {
    altitudeMode = MC_LEVEL;
    prevAltitude = -1;
    failedClimb = false;
    failedClimbCnt = 0;
  }  // so super ....

    void configure() {
      setServoDelta(flightConfig.cThrottleDelta);  //  in degrees
      setForce(false);  // this permits forced gliding without loosing altitude mode...       
      clrFailedClimb();      
      return;
    }  

    boolean getForce() {
      return forceLand;
    }
    void setForce(boolean _b) {
      forceLand = _b;      
    }
      
    void setThrottleSetting (int _s) {      
      altitudeMode = _s;     
    }
    int getSetting() {
      return altitudeMode;
    }

    void clrFailedClimb() {
      failedClimb = false;
      failedClimbCnt = 0;       
    }

    void setFailedClimb() {
      failedClimb = true;
      failedClimbCnt = 0;   
    } 

     bool getFailedClimb() {
       return failedClimb;
    }       
    
    int getFailedClimbCnt() {
      return failedClimbCnt;
    }
    void setFailedClimbCnt(int value) {
      failedClimbCnt = value;
    }

    // throttle
    void act(int curAltitude, int curSpeed) { 
      // curSpeed is (not valid)/useless :(
      int angle;
      int servoAngle = getServoBase();
      // default is level flight MC_LEVEL
      int delta = 0;
#ifdef C_SETUP_2  
#else
      // this is active code ....
      // forceLand used when end to A_TEMP or end of multi-step FlightPlan
      // MC_FORCE used with altitude firewall 
      // MC_FORCE can be easily unset,
      // forceLand is usually set once, can be reset if mission restarted 
      if (forceLand || getSetting() == MC_FORCE) {
        // delta = 0;
        // 2018-08-18: try 33% power LOSS
        // 2020-06-21: go with 10% power loss        
        // servoAngle = (2 * servoAngle) / 3;  // 0;   // not reversed!
        servoAngle = servoAngle - ( servoAngle / 10);        
        fLog.logAction2(fLog.motorOff, 1);
        // turn off failed to climb 
        clrFailedClimb();            
      } else {
        // turn FORCE log marker off..
        fLog.logAction2(fLog.motorOff, 0); // means motor is ON!        
        // this odd code senses failure to climb (battery/motor failure),
        // we have a prev altitude, being asked to climb, and have not      
        if (prevAltitude > 0 &&
            getSetting() == MC_CLIMB && 
            prevAltitude >= curAltitude) {
#ifndef P_TEST_SKIP_FAIL_CLIMB            
// can turn off to avoid messing up the testing :) 
          failedClimbCnt++;
          if (failedClimbCnt > (FAILURE_TO_CLIMB_MSECS / THROTTLETIMER_VALUE)) { // declare failure at nn ms
            setFailedClimb();
            // Serial1.println("DEBUG set failed climb!"); 
          }          
#endif               
        } else {
          // fix a problem with noisy readings from altimiter :)  this is suspect code
          if (curAltitude > (prevAltitude + 11)) { // 11 feet is a swag...
            failedClimbCnt = 0;
          }
        }   
        prevAltitude = curAltitude;        
        // Arduino switch stmt has issues with MC ... constants!
        // MC_<whatever> set ???
        if (getSetting() == MC_CLIMB) {
          //
          // could add code here to increase delta when FailedClimb is present....  ZZFAILEDCLIMB
          delta = getServoDelta();
        } else if (getSetting() == MC_DESCEND) {
          delta = -getServoDelta();        
        }
        // failedClimb turn off when situation has changed
        if (getSetting() != MC_CLIMB) {       
          clrFailedClimb(); 
        }
      }    
#endif        
      angle = writeServoAngleTrans5(servoAngle, delta, &servoCalbThrottle);
      return;
    }
}; // end of ThrottleController
