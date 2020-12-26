// RiotXclock.h
// 2020-06-24 Merge Completed
//
// poor man's flight clock and timers
// * no use of interrupts
// * preset table size
// * very little error handling
//

class Xclock {
  private:
    const static int timerCnt = 11; // was 10;
    int nextTimerAvailable;
    long clockBasis = 0;
    long timerBasis[timerCnt];
    long timerDelta[timerCnt];
    bool timerActive[timerCnt];
  public:
  Xclock () {
    clockBasis = millis();
    nextTimerAvailable = 0;
  }

  long getCurrentTime() {
    return micros() - clockBasis;
  }

  int allocateTimer() {
    if (nextTimerAvailable < timerCnt) {
      int index = nextTimerAvailable;
      timerDelta[index] = 0;
      timerActive[index] = false;
      nextTimerAvailable++;
      return index;
    } else {
      while (1) {
        return NOT_IN_USE; // dirty!
      }  
    }
  }

  void freeTimer(int _timer) {
    return;
  }

  void startTimer(int _timer, unsigned int _delta) {
    timerBasis[_timer] = millis();
    timerDelta[_timer] = _delta;
    timerActive[_timer] = true;    
  }

  boolean checkTimer( int _timer) {
    boolean r = false;
    if  (timerActive[_timer] && 
         timerDelta[_timer] > 0 && (millis() > (timerBasis[_timer] + timerDelta[_timer]))) {
      timerBasis[_timer] = 0;
      timerDelta[_timer] = 0;
      timerActive[_timer] = false;       
      r = true;
    }
    return r;
  }

  boolean checkTimerWithRestart(int _timer) {
    boolean r = false;
    if  (timerActive[_timer] &&
         timerDelta[_timer] > 0 && (millis() > (timerBasis[_timer] + timerDelta[_timer]))) {
      timerBasis[_timer] = millis();
      timerActive[_timer] = true;        
      r = true;
    }
    return r;    
  }

  boolean isActive( int _timer) {
    return timerActive[_timer];
  }
};
