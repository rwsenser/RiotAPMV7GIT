// RiotLogger.h
// 2020-06-24 Merge Completed
//
// ZZLOGGER
class Logger {
  public:
    const static int LOGARRAY_SIZE = LOGARRAYSIZEVALUE;  // set outside of class...  
  private:
    const static int LogDataItemSize = 11;
    struct LogDataItem {
      char data[LogDataItemSize];    // this goes to the radio 
    };  
    const static int LogBufferSize = 60;      // max in TrackerPlane1 is 60!! So really 59!!
    const static int arraySize = 17;
    const static int overrideMsgSize = 11;    
    char commandCnts[arraySize]; // note small(char) datatype, this is input
    const static int itemArraySize = LOGARRAYSIZEVALUE;  
    LogDataItem commandItems[itemArraySize];     
    unsigned char mode;
    bool active;
    bool flipflop; 
    int heading;
    int alt;
    int throttle;
    int agentActive;
    boolean override; 
    char overrideMsg[overrideMsgSize];       
#include "encrypt.h"  
  void logClear() {
    for (int i=0; i < arraySize; i++) { commandCnts[i] = 0; }
  }
  public: 
  // DEBUG DEBUG move count to public and make long
    long int count;  
    char makeDEC(int v) {
        if (v > 9) v = 9;     
        return ('0' + v);       
    }
    char makeHEX(int v) {
      if (v <10) return makeDEC(v);
      v -= 10; // Hex 'A' is (v-10) + 'A'
               // Hex 'B' is 1 + 'A', etc....
      // if (v > 21) v = 21;   /// extended HEX ?? :) 
      if (v > 5) v = 5;             
      return ('A' + v);
    }    
    enum LOGCOMMAND { 
      // keep in sync with arraySize !!
    logMode = 0,
    headingIndicator = 1,  
    // 4 for rudder
    rudderLeft = 2,      
    rudderStraight = 3,
    rudderRight = 4,
    rudderOther = 5,
    // 3 for elevator
    elevatorUp = 6,
    elevatorLevel = 7,    
    elevatorDown = 8,
    // 3 for motor
    motorIncrease = 9,
    motorBase = 10,    
    motorDecrease = 11,
    // 
    // 5 for flags --> 1 in actual log    
    motorOff = 12,
    failedToClimb = 13,
    panic = 14,
    pStep = 15,
    locked = 16};

    // mode bit options
    const static unsigned char MD_ALL = 0xff;
    const static unsigned char MD_GPS = 0x01;
    const static unsigned char MD_COMMANDS = 0x02;     
    const static unsigned char MD_ENCRYPT = 0x04;
    const static unsigned char MD_HDR = 0x08;

  Logger(unsigned char _mode) {
    mode = _mode;
    active = false;
    flipflop = false;    
    count = 0;
    agentActive = 0;
    logClear();
    override = false;        
  }
  void LogMsg(char* msg) {
    override = true;  // override normal log output
    int len = strlen(msg);
    if (len > (overrideMsgSize - 1)) {
      len = (overrideMsgSize - 1);
    }
    memcpy(overrideMsg, msg, len);
    overrideMsg[len] = '\0';
    return;
  }   
  void logAgent(int val) {
    agentActive = val;
  }
  int getMode() {
    return mode;
  }
  // ignore val and just bump counter
  bool logAction (LOGCOMMAND cmd, int val) {
    int i = (int) cmd;
    if (i < arraySize) {
      commandCnts[i]++;
      if (commandCnts[i] > 99) commandCnts[i] = 99;
      return true;
    } else { 
      return false;
    }
  }
  // put val into array
  bool logAction2 (LOGCOMMAND cmd, int val) {
    int i = (int) cmd;
    if (i < arraySize) {
      commandCnts[i] = val;
      if (commandCnts[i] > 99) commandCnts[i] = 99;
      return true;
    } else {
      return false;
    }
  }  
  bool logReadings (float _heading, float _alt, int _throttle) {
    int i = count % itemArraySize;
    // set these from the first call / set of calls
    if (i == 0) {
      heading = (int) _heading;
      alt = (int) _alt;
      throttle = _throttle;
    }
    return true;
  }  
  bool logSendGPS() {
      if (mode & MD_GPS) {
        serialPrintln("@$$");  // this special code signals the
                               // smart phone to output the GPS
                               // data, which the radio processes, etc.
                               // poor man's airborn "LAN"
      }      
  }
        // keep these comments... 
        //  : heading range
        //    4 bytes for rudder
        // L: rudder Left
        // R: rudder Straight
        // H: rudder Right
        //  : rudder Other
        //    3 bytes for elevator
        // u: elevator Up
        // d: elevator Level
        // o: elevator Down
        //    2 bytes for bit flags
        // +: motor Increase
        // -: motor Base
        // b: motor Decrease
        //  : motor Off
        //  : open bit
        // F: Failed-to-climb
        // P: PANIC!
        // L: Locked
        //    
  //
  // totally rewritten 2019-06-16 to contain 4 control reading items per log record
  //  
  //ZZZ
  bool logSend(Xclock& _flightClock, int _logGpsTimer) {
    bool ret = true;
    if (active) {       
      count++;     // this value drives FSM....        
      // first collect auto controller data items
      int i = count % itemArraySize;
      // 2020-06-26: fix bug, back port this to RiotLite  ZZRIOTLITE
      if (!isprint(commandCnts[0])) { commandCnts[0] = ' '; }
      commandItems[i].data[((int) logMode)] = commandCnts[0];    // noHEX, is mode :)        
      commandCnts[0] = ' ';       
      // commandItems[i].data[logMode] = commandCnts[0];    // noHEX, is mode :)  
      // commandCnts[0] = 0; // clear

      // stop at motorIncrease (likely used 9 ...)  
      for (int j=1; j < ((int) motorIncrease); j++) {
        commandItems[i].data[j] = makeHEX(commandCnts[j]);  // might copy 1 item too many..   
        commandCnts[j] = 0; // clear           
      }  
      // pack remaining into in one field
#if 1   
      int v =  (commandCnts[motorIncrease] > 0)?8:0;
          v += (commandCnts[motorBase] > 0)?4:0;
          v += (commandCnts[motorDecrease] > 0)?2:0;                    
          v += (commandCnts[motorOff] > 0)?1:0 ;
      int w = 0; // open bit
          w += (commandCnts[failedToClimb] > 0)?4:0;
          w += (commandCnts[panic] > 0)?2:0;
          w += (commandCnts[locked] > 0)?1:0;          
      commandItems[i].data[LogDataItemSize-2] = makeHEX(v);
      commandItems[i].data[LogDataItemSize-1] = makeHEX(w);      
#else
      commandItems[i].data[LogDataItemSize-2] = 'V'; 
      commandItems[i].data[LogDataItemSize-1] = 'W';             
#endif         
      if (i == (itemArraySize - 1)) { // when log record is full, transmit it
        // second output log record 
        long countS = count / itemArraySize;
        char buf[LogBufferSize];  // hum, not too big!
        bool type2; 
        char *ptr = buf;
        char *ptr2 = 0;
        // keep these comments...
        //     X: '1' or '2'
        //   cnt: line count
        //   NO-   TS: timestamp
        //   HHH: heading in degrees
        // if '1'
        // =alta: alt in feet
        // if '2'
        //   THR: throttle (units not clear)
        //     a: M:manual, A:auto
        //  step:           
        *ptr++ = '{';
        if (flipflop) {
          type2 = false;
          *ptr++ = '1';
        } else {
          type2 = true;
          *ptr++ = '2';           
        }
        flipflop = !flipflop;
#if 0        
        // 5 digit timestamp
        long ts = millis() % 100000;
        itoa( ((ts / 10000) % 10),ptr++,10);
        itoa( ((ts / 1000) % 10),ptr++,10); 
        itoa( ((ts / 100) % 10),ptr++,10); 
        itoa( ((ts / 10) % 10),ptr++,10);      
        itoa( (ts % 10),ptr++,10);
#endif                
        // 3 digit count
        itoa( ((countS / 100) % 10),ptr++,10);
        itoa( ((countS / 10) % 10),ptr++,10);      
        itoa( (countS % 10),ptr++,10);    
        // 3 digit heading
        itoa( ((heading / 100) % 10),ptr++,10);
        itoa( ((heading / 10) % 10),ptr++,10);      
        itoa( (heading % 10),ptr++,10);
        if (type2) {
          // Agent
          // changed
          if (agentActive) {
            // need Step from command cnts...    
            *ptr++ =  makeHEX(commandCnts[pStep]);            
          } else {
            *ptr++ = 'X'; // no step is in use
          }
          // 3 digit throttle
          itoa( ((throttle / 100) % 10),ptr++,10);
          itoa( ((throttle / 10) % 10),ptr++,10);      
          itoa( (throttle % 10),ptr++,10);
        } else { // type1
          // 5 digit alt
          // 4 digit alt really
          // itoa( ((alt / 10000) % 10),ptr++,10);  // we fly down low...
          itoa( ((alt / 1000) % 10),ptr++,10); 
          itoa( ((alt / 100) % 10),ptr++,10); 
          itoa( ((alt / 10) % 10),ptr++,10);      
          itoa( (alt % 10),ptr++,10);
        }        
        ptr2 = ptr;
        // header takes { + 11 chars, so 12       
        if ((mode & MD_COMMANDS) && (countS > 0)) { 
          for (int i=0; i < itemArraySize; i++) {
            for (int j=0; j < LogDataItemSize; j++) {  // kludge
              *ptr++ = commandItems[i].data[j];
            }
          }       
        } 
        if (override) {       
          for (int i=0; i < strlen(overrideMsg); i++) {
             *ptr2++ = overrideMsg[i];
          }                   
        }
        if (ptr2 > ptr) {
          ptr = ptr2;  // in the event there we no log blocks...          
        }
        *ptr++ = '}';    
        // *ptr++ = '\n';  // DEBUG DEBUG        
        *ptr++ = 0;  // Arnold...
        int len = strlen(buf);
        // let's clean up -- just in case
        for (int i=0; i < len; i++) {
          if (!isprint(buf[i])) buf[i] = '.';
        }
        encrypt(buf,len);
        serialPrintln(buf);  // ca= false;uses transmit!
        logClear();
        override = false;
        // QQHERE 
        // patch to put GPS radio data in separate buffer (via a delay...)
        // allows send of GPS data to be delayed (this is kinda tricky)
        // DEBUG DEBUG 2019-10-09, turn back on for testing, 2019-6-13: turn off GPS so can have faster timers for logging  
#if 1  
          // if (_flightClock.checkTimer(_logGpsTimer)) {
          //  logSendGPS();
          // }
        _flightClock.startTimer(_logGpsTimer, LOGGINTTIMERGPS_VALUE);
#endif                
      }         
    } else {
      ret = false;
    }
 
    return ret;
  }
  
  // ZZZ END
  void logActive() {
    // express multiple times due to serial errors
    for (int i=0; i<3; i++) {
      serialPrintln("@on");
    }
    active = true;
  }
  void logInactive() {
    serialPrintln("@off");
    active = false;
  }
  boolean isActive () {
    return active;
  }
};
