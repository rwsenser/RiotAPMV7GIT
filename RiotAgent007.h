// AGENT007 :)
// 2020-06-24 Merge Completed
//
class FlightAgent {
  private:
      
    ElevatorController* elevatorController;
    RudderController* rudderController;
    ThrottleController* throttleController;
    Xclock* flightClock;
    int elevatorTimer;
    int rudderTimer;
    int throttleTimer;
    boolean active;

  public:

  FlightAgent (ElevatorController* _ec, RudderController* _rc, ThrottleController* _mc) {
    elevatorController = _ec;
    rudderController = _rc;
    throttleController = _mc;
    active = false;
  }  

  void configure(Xclock* _fc, int _et, int _rt, int _tt) {  // cannot be part of Constructor ...
    flightClock = _fc;
    elevatorTimer = _et;    
    rudderTimer = _rt;
    throttleTimer = _tt;
    throttleController -> configure();
    rudderController -> configure();
    elevatorController -> configure();
  }

  void setActive(bool _b) {
    active = _b;   
    if (active == true) {
      if (THROTTLE) setThrottleActive(_b);
      // LiteV6: if (COMPASS) setRudderActive(_b);
      if (GYRO) setRudderActive(_b);
      if (ALTIMETER) setElevatorActive(_b);
    } else {
      setThrottleActive(_b);
      setRudderActive(_b);
      setElevatorActive(_b);
    }    
  }
    
  boolean getActive() {
    return active;
  }

  void setServoBases(int elevatorServoSetting,
                     int rudderServoSetting,
                     int throttleServoSetting) {
    throttleController -> setServoBase(throttleServoSetting);
    rudderController -> setServoBase(rudderServoSetting); 
    elevatorController ->  setServoBase(elevatorServoSetting);                               
  }

  void actAgent( int curAltitude, int curHeading, int curSpeed,
            int curAngleRudder, int curAngleElevator, int curAngleThrottle) {
             // ??? boolean *failClimbPtr) { 
    if (active && MAG_CALB != 1) {
      // LiteV6:  if (flightClock -> checkTimerWithRestart(rudderTimer) && COMPASS) {
      if (flightClock -> checkTimerWithRestart(rudderTimer) && GYRO) {         
        // 2019-10-20: get rate-of-change easy way...
        static int lastHeading = -999;
        if (lastHeading < -360) lastHeading = curHeading; // avoid first-time junk
        //  move to a line (not a mod circle)
        int rateOfChangeHeading = curHeading  - lastHeading;
        if (abs(rateOfChangeHeading) > 180) {  // nonsense, fix it
          int c = (curHeading + 180) % 360;
          int l = (lastHeading + 180) % 360;
          rateOfChangeHeading = c - l;        
        }
        lastHeading = curHeading;
        rudderController -> act(curHeading, rateOfChangeHeading);        
      }
      if (flightClock -> checkTimerWithRestart(elevatorTimer) && ALTIMETER) {        
        elevatorController -> act(curAltitude, curAngleThrottle); //, failClimbPtr);     
      }
      if (flightClock -> checkTimerWithRestart(throttleTimer) && THROTTLE) {         
        throttleController -> act(curAltitude, curSpeed); // , failClimbPtr);
      }      
    } 
  }

  // ELEVATOR
  // void setElevatorParameters(int _v1, int _v2, int _v3) {
    // maxAltitude = _v1;
    // desiredAltitude = _v2;
    // minAltitude = _v3;  
    // elevatorController -> setParameters( _v1, _v2, _v3);       
  // }
  void getElevatorParameters( int& _v1, int& _v2, int& _v3) {
    // _v1 = maxAltitude;
    // _v2 = desiredAltitude;
    // _v3 = minAltitude;
    elevatorController -> getParameters( _v1, _v2, _v3);      
  }
  void setElevatorActive(boolean _b) {
    elevatorController -> setActive(_b);   
  }
  boolean getElevatorActive() {
    return elevatorController -> getActive();
  }

  // THROTTLE
  void setThrottleSetting(int _s) { // MC_xxxxx
    throttleController -> setThrottleSetting(_s);   
  }
  boolean getThrottleSetting() {
    return throttleController -> getSetting();
  } 
  void setThrottleActive(boolean _b) {
    throttleController -> setActive(_b);   
  }
  boolean getThrottleActive() {
    return throttleController -> getActive();
  }  

  // RUDDER
  void setRudderActive(boolean _b) {
    rudderController -> setActive(_b);   
  }
  boolean getRudderActive() {
    return rudderController -> getActive();
  }  
  int parseInt(char* num) {
    int cnt = 0;
    int val = 0;
    while (*num != '$') {
      if (cnt > 5) return -1;
      if (*num >= '0' && *num <= '9') {
        val *= 10;
        val += (*num - '0');
        cnt++;
        num++;
      } else return -1;
    }
    return val;
  }
};
