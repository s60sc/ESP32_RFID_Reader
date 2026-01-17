/*
* App to decode EM4100 or FDX-B tags (pet chips)
* and display reading on a web page and an optional LCD 1602 
* 
* s60sc 2023
* 
*/

#include "appGlobals.h"

static bool startedUp = false;

void setup() {
  logSetup();
  // prep SD card storage and load saved user configuration
  if (startStorage()) loadConfig();

#ifdef DEV_ONLY
  devSetup();
#endif

  // connect wifi or start config AP if router details not available
  startNetwork();
  if (startWebServer()) {
    // start app services
    prepI2C();
    rfidSetup(); // setup rfid reader
    checkMemory();
    startedUp = true;
  }
  LOG_INF("=============== Total tasks: %u ===============\n", uxTaskGetNumberOfTasks());
}

void loop() {
  if (startedUp) rfidRead();
  delay(50);
}
