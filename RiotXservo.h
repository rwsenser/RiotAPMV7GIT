// RiotXservo.h
// 2020-06-24 Merge Completed
//
// Xservo is a thin wrapper for the Servo object; it makes the shifting
// between radio-driven and program-driven servo control more obvious.
// 
// The attachXservo() and detachXservo() methods manage the mode of operation
//
class Xservo {
  private:
  int myPin;
  Servo myServo;
  boolean isAttached;
  int value;

  public:
  Xservo(int _pin) {
    setPin(_pin);
    pinMode(myPin, OUTPUT);
    isAttached = false;      
  }

  void setPin(int _pin) {
    myPin = _pin;
  }
  int getPin(void) {
    return myPin;
  }

  void attachXservo(){ // APM change int _value){
    myServo.attach(myPin);
    isAttached = true;  
    // value = _value;
    // write(value);     
  };
  void detachXservo() { 
    myServo.detach();  
    isAttached = false;    
  };

  void write(int _value) {    
    if (isAttached) {
      value = _value;  
      myServo.write(value);  
    } 
  };

  void intrDigitalWrite(int _value) {
    // old PLAN_B area, for PLAN_B disable this write  
    if (!isAttached) {
      digitalWrite(myPin, _value); 
    }  
  }
}; // end of Xservo class
