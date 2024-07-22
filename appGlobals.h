// Global RFID declarations
//
// s60sc 2023

#pragma once
#include "globals.h"

#define ALLOW_SPACES false // set true to allow whitespace in configs.txt key values

// web server ports
#define HTTP_PORT 80 // app access
#define HTTPS_PORT 443 // secure app access

/*********************** Fixed defines leave as is ***********************/ 
/** Do not change anything below here unless you know what you are doing **/

//#define DEV_ONLY // leave commented out
#define STATIC_IP_OCTAL "143" // dev only
#define DEBUG_MEM false // leave as false
#define FLUSH_DELAY 0 // for debugging crashes
#define DBG_ON false // esp debug output
#define DOT_MAX 50
#define HOSTNAME_GRP 0
#define USE_IP6 false
 
#define APP_NAME "ESP32_RFID" // max 15 chars
#define APP_VER "2.1"

#define HTTP_CLIENTS 2 // http, ws
#define MAX_STREAMS 0
#define INDEX_PAGE_PATH DATA_DIR "/RFID" HTML_EXT
#define FILE_NAME_LEN 64
#define IN_FILE_NAME_LEN 128
#define JSON_BUFF_LEN (1024 * 4) // set big enough to hold json string
#define MAX_CONFIGS 100 // > number of entries in configs.txt
#define GITHUB_PATH ""

#define STORAGE LittleFS // One of LittleFS or SD_MMC
#define RAMSIZE (1024 * 8) 
#define CHUNKSIZE (1024 * 4)
#define MIN_RAM 8 // min object size stored in ram instead of PSRAM default is 4096
#define MAX_RAM 4096 // max object size stored in ram instead of PSRAM default is 4096
#define TLS_HEAP (64 * 1024) // min free heap for TLS session
#define WARN_HEAP (32 * 1024) // low free heap warning
#define WARN_ALLOC (16 * 1024) // low free max allocatable free heap block
#define MAX_ALERT 1024

#define INCLUDE_FTP_HFS false // ftp.cpp (file upload)
#define INCLUDE_SMTP false    // smtp.cpp (email)
#define INCLUDE_MQTT false    // mqtt.cpp
#define INCLUDE_TGRAM false   // telegram.cpp
#define INCLUDE_CERTS false   // certificates.cpp (https and server certificate checking)
#define INCLUDE_WEBDAV true   // webDav.cpp (WebDAV protocol)

#define IS_IO_EXTENDER false // must be false except for IO_Extender
#define EXTPIN 100

// to determine if newer data files need to be loaded
#define CFG_VER 1

#ifdef CONFIG_IDF_TARGET_ESP32S3 
#define SERVER_STACK_SIZE (1024 * 8)
#define DS18B20_STACK_SIZE (1024 * 2)
#define STICK_STACK_SIZE (1024 * 4)
#else
#define SERVER_STACK_SIZE (1024 * 4)
#define DS18B20_STACK_SIZE (1024)
#define STICK_STACK_SIZE (1024 * 2)
#endif
#define BATT_STACK_SIZE (1024 * 2)
#define EMAIL_STACK_SIZE (1024 * 6)
#define FS_STACK_SIZE (1024 * 4)
#define LOG_STACK_SIZE (1024 * 3)
#define MQTT_STACK_SIZE (1024 * 4)
#define PING_STACK_SIZE (1024 * 5)
#define SERVO_STACK_SIZE (1024)
#define SUSTAIN_STACK_SIZE (1024 * 4)
#define TGRAM_STACK_SIZE (1024 * 6)
#define TELEM_STACK_SIZE (1024 * 4)
#define UART_STACK_SIZE (1024 * 2)

// task priorities
#define HTTP_PRI 5
#define TGRAM_PRI 1
#define EMAIL_PRI 1
#define FTP_PRI 1
#define LOG_PRI 1
#define UART_PRI 1
#define BATT_PRI 1
#define IDLEMON_PRI 5

// devices requiring separate libraries
#define USE_BMP280 false
#define USE DS3231 false
#define USE_SSD1306 false
#define USE_DS18B20 false

// devices not requiring separate libraries
#define USE_LCD1602 true
#define USE_PCF8591 false
#define USE_MPU6050 false


/******************** Function declarations *******************/

// global app specific functions
bool checkI2Cdevices(bool showWarn = false);
bool startI2C();

void rfidSetup();
void rfidRead();

bool lcdInit();
void lcdPrint(const char* str);
void lcdClear();
void lcdHome(); 
void lcdDisplay(bool setDisplay);
void lcdBlinkCursor(bool showBlink);
void lcdLineCursor(bool showLine);
void lcdScrollText(bool scrollLeft);
void lcdTextDirection(bool scrollLeft);
void lcdAutoScroll(bool autoScroll);
void lcdSetCursorPos(uint8_t row, uint8_t col);
void lcdBacklight(bool lightOn);
void lcdLoadCustom(uint8_t charLoc, uint8_t charmap[]);
void lcdWriteCustom(uint8_t charLoc);
void updateLcd(bool showTag);

/******************** Global app declarations *******************/

extern const char* appConfig;

extern int I2C_SDA;
extern int I2C_SCL;

extern int rfidDemod; // input - demod pin from RDM6300, pullup high
extern int rfidClock; // output - clock pin to RDM6300
extern int clearPin; // input - clear tag display
extern float rfidFreq; // antenna frequency in kHz
extern bool encodeFDX; // false for EM4100, true for FXD-B
extern char encodingStr[]; // encoding type string
extern uint64_t currentTag;
extern char currentTagStr[]; // holds string representation of tag
