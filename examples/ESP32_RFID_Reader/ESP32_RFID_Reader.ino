// Example sketch to demonstrate RFID reader 

#include "RFID.h"

#define demodRfid 4 // input - demod pin from RDM6300, pullup high
#define rfidClock 26 // output - clock pin to RDM6300
static const boolean encoding = false; // false for EM4100, true for FXD-B
static const boolean wantDebug = true; // if debug output wanted
static const int dumpSampleSize = 50; // number of elements to output on debug dump

void setup() {
  pinMode(demodRfid, INPUT_PULLUP);
  pinMode(rfidClock, OUTPUT);
  Serial.begin(115200);
  char encodeStr[7]; // holds string representation of encoding
  rfidSetup(encoding, encodeStr, rfidClock, demodRfid, wantDebug); 
  Serial.printf("Recognise tag type: %s\n", encodeStr);
}

void loop() {  
  char currentTagStr[16]; // holds string representation of tag
  if (captureReady()) {
    uint64_t currentTag = getTag();
    if (currentTag) {
      tagStr(currentTag, currentTagStr);
      Serial.printf("Read tag - %s\n", currentTagStr);
    } else Serial.println("Tag not read");
    resetCapture();
  }
}


 
