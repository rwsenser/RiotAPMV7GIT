// RiotSerialLayer.h 

// wrappers for Serial output

// ZZSERIAL
// keep string wrappers for now...
#define serialPrintString(sss) {  serialPrint(sss); }
#define serialPrintStringln(sss) {  serialPrintln(sss); }
#define SERIAL Serial1 // Serial1

const int SER = 1;
void serialPrint( const char * str) {
  if (SER) {
    #ifdef UNO_MEGA
    SERIAL.print(str);
    #else
      // UART_print(str);
    #endif
  }
}

void serialPrintln(const char * str) {
  if (SER) {
    #ifdef UNO_MEGA
      SERIAL.println(str);
    #else
      // UART_println(str);
  #endif  
  }
}

void serialPrintInt(const int d) {
  if (SER) {
    #ifdef UNO_MEGA
      SERIAL.print(d);
    #else
      // char buff[32];
      // sprintf(buff,"%d", d);
      // UART_print(buff);
    #endif  
  }
}

void serialPrintIntln(const int d) {
  if (SER) {
    #ifdef UNO_MEGA
      SERIAL.println(d);
    #else
      // char buff[32];
      // sprintf(buff,"%d", d);
      // UART_println(buff);
    #endif  
  }
}