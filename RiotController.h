// RiotController.h
// 2020-06-24 Merge Completed
//
class Controller {
  private:
  boolean active;
  int baseServoSetting;
  int baseServoDelta;
  int currentServoSetting;
  char servoName;
  Xservo* servoPtr;
  
  public:
  Controller(Xservo* _s, char _n) {
    active = false;
    servoPtr = _s;
    baseServoSetting = NOT_IN_USE;  // this is important :)
    baseServoDelta = NOT_IN_USE;
    currentServoSetting = NOT_IN_USE;
    servoName = _n;
  }
  void setActive(boolean _b) {
    active = _b;
    if (1) { // baseServoDela > NOT_IN_USE) { // important because of this, needed when testing :)
      if (active == true) {
        if (VERBOSE) {
          char buf[2];
          buf[0] = servoName;
          buf[1] = 0;
          serialPrint(buf);  
          serialPrint(": Servo Basis angle: ");  
          serialPrintInt(baseServoSetting);
          serialPrint(", Delta angle ");
          serialPrintIntln(baseServoDelta);         
        }
        servoPtr -> attachXservo(); // APM change, no baseServoSetting
      } else {
        servoPtr -> detachXservo();    
      }
    }
  }
  boolean getActive() {
    return active;
  }

  Xservo* getServoXptr() {
    return servoPtr;
  }

  // ZZTRANS
  // enable linear transformation of receiver timmer units to servo.h class actual angle 
  int writeServoAngleTrans5(int _a, int _d, const ServoCalibration *p ) {
    int distanceRange;
    int distanceRec;
    float ratio;
    int computedAngle = 0;    
    int effectiveAngleRange;

#if 0
    SERIAL.print("T5 DEBUG: ");
    SERIAL.print(_a);
    SERIAL.print( " ");
    SERIAL.println(_d);
#endif
    _a += (p -> preReceiverOffet);
    // cleanup input reading
    if (_a < (p -> receiverValues[SC_LOWER]) )  { _a = (p -> receiverValues[SC_LOWER]); }
    if (_a > (p -> receiverValues[SC_UPPER]) ) { _a = (p -> receiverValues[SC_UPPER]); }
    computedAngle = _a;
    // unused with APM...    
#if 0      
    //
    // WARNING: this is dangerous code to change while sleepy!
    //
    // Translate, two cases... blipAngle is zero or not zero
    if ((p -> blipAngle) == 0) { // only 2 ranges!
      if (_a <= (p -> receiverValues[SC_MIDDLE90])) { // range 1 of 2
        // these are values from receiver
        distanceRange = (p -> receiverValues[SC_MIDDLE90]) - (p -> receiverValues[SC_LOWER]);
        distanceRec = (p -> receiverValues[SC_MIDDLE90]) - _a;
        // ratio is the fraction of range <= 90 that is used
        ratio = ((float) distanceRec) / ((float) distanceRange); // ratio, used from recvr
        // effectiveAngleRange is the degrees used from 90 on down, varies by servo and recvr!
        effectiveAngleRange = 90 - (p -> minRangeAngle);
        // computerAngle is the angle <=90 that maps ratio onto the angle range fpr this servo
        computedAngle = 90 - ((int) (ratio * ((float) effectiveAngleRange)) );
      } else { // range 2 of 2
        // see comments above
        distanceRange = (p -> receiverValues[SC_UPPER]) - (p -> receiverValues[SC_MIDDLE90]);
        distanceRec = _a - (p -> receiverValues[SC_MIDDLE90]);
        ratio = ((float) distanceRec) / ((float) distanceRange);
        effectiveAngleRange = (p -> maxRangeAngle) - 90;
        computedAngle = 90 + ((int) (ratio * ((float) effectiveAngleRange)) );      
      }   
    } else { // full 4 ranges: blips provide inner ranges about middle
      int range = 0;
      if (_a >= (p -> receiverValues[SC_HIBLIP])) { range = 3; }
      else if (_a >= (p -> receiverValues[SC_MIDDLE90])) { range = 2; }
      else if (_a >= (p -> receiverValues[SC_LOBLIP])) { range = 1; } 
      switch (range) {
        case (0): // 0 to SC_LOGBLIP
          distanceRange = (p -> receiverValues[SC_LOBLIP]);
          distanceRec = _a;
          ratio = ((float) distanceRec) / ((float) distanceRange);
          computedAngle = ((int) (ratio * (90.0 - ((float) p -> blipAngle) ) ) );   
          break;    
        case (1): // SC_LOGBLIP to 90
          distanceRange =(p -> receiverValues[SC_MIDDLE90]) - (p -> receiverValues[SC_LOBLIP]) ;
          distanceRec = p -> blipAngle;
          ratio = ((float) distanceRec) / ((float) distanceRange);
          computedAngle = ((int) (ratio * ((float) p -> blipAngle)  ) ) + (90 - p -> blipAngle) ;         
          break;
        case (2): // 90 to SC_HIBLOIP
          distanceRange =(p -> receiverValues[SC_HIBLIP]) - (p -> receiverValues[SC_MIDDLE90]) ;
          distanceRec = p -> blipAngle;
          ratio = ((float) distanceRec) / ((float) distanceRange);
          computedAngle = 90 + ((int) (ratio * ((float) p -> blipAngle)  ) );         
          break;
        case (3): // SC_HIBLIP to 179 or 180
          distanceRange =(p -> receiverValues[SC_UPPER]) - (p -> receiverValues[SC_HIBLIP]) ;
          distanceRec = 90 - (p -> blipAngle);
          ratio = ((float) distanceRec) / ((float) distanceRange);
          computedAngle = 90 + ((int) (ratio * (90.0  - ((float) p -> blipAngle) ) ) );         
          break;     
      }
      //
    } 
#endif    
    // add what is likely the delta value (adjust supplied by autopilot)
    computedAngle += _d;
    // cleanup output anngle range (limit final angle to a clip range -- presents bad-date induced crashes)
    if (computedAngle < (p -> minClipAngle) )  { _a = (p -> minClipAngle); }
    if (computedAngle > (p -> maxClipAngle) )  { _a = (p -> maxClipAngle); }
    // beam us out of here Scotty!
    writeServoAngle(computedAngle);
    return computedAngle;
  }
  // enable linear transformation of transmitter units to servo.h class angles 
  void writeServoAngleTrans(int _a, float _slope, int _offset, int _min, int _max) {
    float angle = _a;
    angle *= _slope;
    int iAngle = (int) angle;
    iAngle += _offset;
    if (iAngle < _min) iAngle = _min;
    if (iAngle > _max) iAngle = _max;
    writeServoAngle(iAngle);
  }
  
  void writeServoAngle(int _a) {
    servoPtr -> write(_a);
    currentServoSetting = _a;
#ifdef C_SETUP_2
    if (VERBOSE) {
      char buf[2];
      buf[0] = servoName;
      buf[1] = 0;
      serialPrint(buf);       
      serialPrint(" Servo Basis angle: ");  
      serialPrintInt(baseServoSetting);
      serialPrint(", Servo angle ");
      serialPrintIntln(_a);       
      }
#endif     
  }
  void setServoBase(int _b) {
    baseServoSetting = _b;
  }
  int getServoBase() {
    return baseServoSetting;
  }
  void setServoDelta(int _d) {
    baseServoDelta = _d;
  }
  int getServoDelta() {
    return baseServoDelta;
  }
  int getServoCurrent() {
    return currentServoSetting;
  }  
}; // end of Controller class
