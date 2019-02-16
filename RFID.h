#ifndef RFID_h
#define RFID_h

#include "arduino.h"

void rfidSetup(boolean encodeType, char* thisEncode, int rfidClockPin, int rfidDemodPin, boolean wantDebug = false);
boolean captureReady(int sampleSize = 50); // provide default if no arg supplied
uint64_t getTag();
void resetCapture();
void tagStr(uint64_t thisTag, char* thisTagStr);

#endif
