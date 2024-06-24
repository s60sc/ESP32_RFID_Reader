// RFID specific web & prefs functions
//
// s60sc 2023

#include "appGlobals.h"

const size_t prvtkey_len = 0;
const size_t cacert_len = 0;
const char* prvtkey_pem = "";
const char* cacert_pem = "";

void updateLcd(bool showTag) {
  // output result to LCD if present
  lcdClear();
  lcdSetCursorPos(0, 0);
  lcdPrint(encodingStr);
  lcdSetCursorPos(1, 0);
  if (!showTag) {
    currentTag = 0;
    currentTagStr[0] = 0;
  }
  // update config for latest stats to return on next main page call
  updateConfigVect("EncodeType", encodingStr);
  updateConfigVect("currentTag", currentTagStr);
  lcdPrint(currentTagStr);
}

/************************ webServer callbacks *************************/

bool updateAppStatus(const char* variable, const char* value) {
  // update vars from configs and browser input
  bool res = true;
  int intVal = atoi(value);
  float fltVal = atof(value);
  if (!strcmp(variable, "rfidDemod")) rfidDemod = intVal;
  else if (!strcmp(variable, "rfidClock")) rfidClock = intVal;
  else if (!strcmp(variable, "rfidFreq")) rfidFreq = fltVal;
  else if (!strcmp(variable, "encodeFDX")) encodeFDX = (bool)intVal;
  else if (!strcmp(variable, "I2C_SDA")) I2C_SDA = intVal;
  else if (!strcmp(variable, "I2C_SCL")) I2C_SCL = intVal;
  else if (!strcmp(variable, "clearPin")) clearPin = intVal;
  else if (!strcmp(variable, "resetTag")) {
    currentTag = 0;
    updateLcd(false);
  }
  return res;
}

void appSpecificWsBinHandler(uint8_t* wsMsg, size_t wsMsgLen) {
  LOG_ERR("Unexpected websocket binary frame");
}

void appSpecificWsHandler(const char* wsMsg) {
  // message from web socket
  int wsLen = strlen(wsMsg) - 1;
  switch ((char)wsMsg[0]) {
    case 'X':
    break;
    case 'H':
      // keepalive heartbeat, return status
    break;
    case 'S':
      // status request
      buildJsonString(wsLen); // required config number
      logPrint("%s\n", jsonBuff);
    break;
    case 'U':
      // update or control request
      memcpy(jsonBuff, wsMsg + 1, wsLen); // remove 'U'
      parseJson(wsLen);
    break;
    case 'K':
      // kill websocket connection
      killSocket();
    break;
    default:
      LOG_WRN("unknown command %c", (char)wsMsg[0]);
    break;
  }
}

void buildAppJsonString(bool filter) {
  // build app specific part of json string
  char* p = jsonBuff + 1;
  *p = 0;
}

esp_err_t appSpecificWebHandler(httpd_req_t *req, const char* variable, const char* value) {
  return ESP_OK;
}

esp_err_t appSpecificSustainHandler(httpd_req_t* req) {
  return ESP_OK;
}

void externalAlert(const char* subject, const char* message) {
  // alert any configured external servers
}

bool appDataFiles() {
  // callback from setupAssist.cpp, for any app specific files
  return true;
}

void doAppPing() {}

void OTAprereq() {
  stopPing();
}

/************** default app configuration **************/
const char* appConfig = R"~(
restart~~99~T~na
ST_SSID~~0~T~Wifi SSID name
ST_Pass~~0~T~Wifi SSID password
ST_ip~~0~T~Static IP address
ST_gw~~0~T~Router IP address
ST_sn~255.255.255.0~0~T~Router subnet
ST_ns1~~0~T~DNS server
ST_ns2~~0~T~Alt DNS server
AP_Pass~~0~T~AP Password
AP_ip~~0~T~AP IP Address if not 192.168.4.1
AP_sn~~0~T~AP subnet
AP_gw~~0~T~AP gateway
allowAP~1~0~C~Allow simultaneous AP
timezone~GMT0~0~T~Timezone string: tinyurl.com/TZstring
logType~0~99~N~Output log selection
Auth_Name~~0~T~Optional user name for web page login
Auth_Pass~~0~T~Optional user name for web page password
wifiTimeoutSecs~30~0~N~WiFi connect timeout (secs)
refreshVal~1~0~N~Web page refresh rate (secs)
rfidDemod~~1~N~Input pin for RFID data
rfidClock~~1~N~Output pin for RFID clock
clearPin~~1~N~Input pin to clear tag display
encodeFDX~0~2~B:EM4100:FXD-B~Encoding type
I2C_SDA~21~1~N~I2C_SDA pin
I2C_SCL~22~1~N~I2C_SCL pin
rfidFreq~~1~N~Antenna frequency in kHz (blank for default)
EncodeType~~2~D~RFID Encoding
currentTag~~2~D~Current Tag ID
resetTag~Clear~2~A~Clear Tag
usePing~1~0~C~Use ping
)~";
