/*
* library to decode EM4100 or FDX-B tags
* The following library https://github.com/Sthing/AccessThing/tree/master/Arduino/libraries/EM4100
* was used as a starting point to derive this library
* together with info for decoding FDX-B from http://www.gizmolab.co.za/fdx-b-protocol/
* 
* s60sc 2018
*/

#include "RFID.h"
#include "arduino.h"
#include "esp_system.h"
#include "driver/ledc.h"

// read tag value
static uint64_t currentTag; 

// Buffer sizing
static const uint16_t bitBufferSize = 256;     // Room for 2 x FDX-B codeLength bits
static uint8_t captureData[bitBufferSize*2];   // conversion of pulses to level bits (2 level bits per data bit)
static uint16_t slBoundary, minPulse, maxPulse; // boundary between long and short pulses
static int codeLength, captureIndex, captureSize;

// debugging
static volatile uint16_t rawEM[512];  
static volatile int emcount = 0;
static boolean doDebug = false;

// Variables used during decoding
static bool encodingFDX_B; // false for EM4100, true for FDX-B
static uint8_t columnSums[4];                       // Used for EM4100 parity calculation of the four columns
static uint8_t bitBuffer[bitBufferSize];            // Each byte contains data for 4 full bit periods. LSB is first bit.
static uint32_t lastMicros; // elapsed uS between interrupts
static const char* codeStr[2] = {"EM4100", "FDX-B"};
static int rfidDemod;

// forward refs
boolean parseCaptureData();
bool parseNibble(word offset, uint8_t *nibble);
bool findHeader();
bool parseBitBufferEM(uint8_t currIndex);
bool parseBitBufferFDX(uint8_t currIndex);
void IRAM_ATTR demodISR();
void runClock(int LEDC_FREQ, uint8_t LEDC_HS_CH0_GPIO);

void rfidSetup(boolean encodeType, char* thisEncode, int rfidClockPin, int rfidDemodPin, boolean wantDebug) {
  // call from sketch setup()
  doDebug = wantDebug;
  // set encoding to FDX-B (true) or EM4100 (false)
  encodingFDX_B = encodeType;
  strcpy(thisEncode, codeStr[encodingFDX_B]);
  codeLength = (encodingFDX_B) ? 128 : 64; // EM4100 is 64 bits, FDX-B is 128 bits
  int rfidClockRate = (encodingFDX_B) ? 134200 : 125000; // Antenna clock in Hz
  // short pulse length in uS between minPulse & slBoundary
  // long pulse length in uS between slBoundary & maxPulse
  minPulse = (encodingFDX_B) ? 85 : 170; 
  maxPulse = (encodingFDX_B) ? 320 : 560; 
  slBoundary = (encodingFDX_B) ? 180 : 360; // boundary between short and long pulse time in uS. 
  captureSize = codeLength * 4; // Capture at 4 x codeLength bit periods (to catch at a complete tag regardless of start) and each bit period is 2 pulse
  rfidDemod = rfidDemodPin;
  runClock(rfidClockRate, rfidClockPin); // output square wave clock to RDM6300
  attachInterrupt(digitalPinToInterrupt(rfidDemodPin), demodISR, CHANGE);
}

void IRAM_ATTR demodISR() {
  // obtain the encoded bit stream from the reader demodulated output
  // consists of short and long pulses - store pulse length and level
  // could be done by ESP32 PCNT module & ringbuffer but didnt work
  uint32_t elapsedMicros = micros() - lastMicros;
  lastMicros = micros();

  if (captureIndex < captureSize) {
    if (elapsedMicros > minPulse && elapsedMicros < maxPulse) { 
      // valid pulse
      int level = digitalRead(rfidDemod);
      captureData[captureIndex] = level;
      captureIndex++;
      if (elapsedMicros > slBoundary) { // long pulse, so add extra bit
         captureData[captureIndex] = level;
         captureIndex++;
      }
      if (doDebug) {
        if (emcount < captureSize) {
          rawEM[emcount] = elapsedMicros;
          emcount++;
        } 
      }
    } else captureIndex = 0; // restart on outliers
  }
}

void dumpCaptureData(int sample) {
  // dump initial sample of captured metadata for debugging
  if (captureIndex >= captureSize) {
    Serial.println();
    Serial.println("------------------------------------------");
    Serial.printf("First %d entries\n", sample);
    Serial.printf("slBoundary: %d\n", slBoundary);
    Serial.print("bitDuration:");
    for (int i = 0; i < sample; i++) Serial.printf("%d/", rawEM[i]);
    Serial.println(); 
    Serial.print("captureData:"); 
    for (uint16_t i = 0; i < sample; i++) Serial.print(captureData[i]);
    Serial.println();
  }
} // End dumpCaptureData()


uint64_t getTag() {
  // demodulate and decode RFID tag 
  captureIndex = 0; // empty buffer ready for next read
  emcount = 0;
  return (parseCaptureData()) ? currentTag : 0;
}

boolean captureReady(int sampleSize) {
  // indicate if capture data ready for processing
  if (doDebug && emcount >= captureSize) dumpCaptureData(sampleSize);
  return (captureIndex < captureSize) ? false : true;
}

void resetCapture() {
  captureIndex = 0; // empty buffer ready for next tag
}

/*
 * The captureData array now contains two level bits for each encoded bit period.
 * Convert the intermediate bitstream into decoded data, using the protocol specific combination of: 
 * - phase offset 0 or 1 bits
 * - manchester or biphase encoding
 * - normal or inverted output
 * Parses into bitBuffer and calls parseBitBufferXX via findHeader().
 * return True if parsing and decoding succeeds.
 */
boolean parseCaptureData() {
  uint8_t level; // 0 or 1 - logic level of the sample being inspected
  uint8_t lastLevel; // The last level inspected
  uint16_t tickIndex, outIndex; 
  uint8_t isTick; // True on every second sample
  uint8_t foundBit, offset, invert; 
  boolean biphase;
  if (encodingFDX_B) {
    // FDX-B encoding
    biphase = true;
    offset = 0;
    invert = 0;
  } else {
    // EM4100 coding 
    biphase = false; // uses manchester
    offset = 0;
    invert = 0;
  }
  if (doDebug) Serial.printf("offset=%d, invert=%d, encoding=%d: ", offset, invert, biphase);

  // Initialize output buffer
  for (int i = 0; i < bitBufferSize; i++) bitBuffer[i] = 0;
  tickIndex = outIndex = 0;
  for (uint16_t inIndex = 0; inIndex < captureSize; inIndex++) {
    lastLevel = level; // Not valid for first input, but that is fixed below
    level = captureData[inIndex];

    // Special handling for first input
    if (inIndex == 0) {
      // a transition happened before this sample
      lastLevel = level ^ 1;
      // Skip first sample if offset == 1
      if (offset == 1) continue; 
    }

    // For both encodings need a transition on each clock tick. Otherwise the phase offset is wrong or the sampled data is illegal.
    isTick = tickIndex & 1 ? 0 : 1;
    tickIndex++;
    if (isTick && level == lastLevel) { // Transition missing
      if (doDebug) Serial.println("\n  -Missing transition");
      static int minLength = (encodingFDX_B) ? 65 : codeLength; // for FDX-B, only first 65 bits are relevant
      if (outIndex > minLength) return findHeader(); // try anyway as might have enough
      return false;
    }

    // Extract a bit for every clock tick
    if (biphase) { // Biphase
      // The bit is encoded between clock tick transitions: No transition => 1, a transition => 0.
      if (isTick) continue;
      foundBit = (level == lastLevel) ? 1 : 0;
    } else { // Manchester encoding
      // The bit is the value after the clock tick transition
      if (!isTick) continue;
      foundBit = level;
    }

    // Invert?
    if (invert) foundBit ^= 1;
    if (doDebug) Serial.print(foundBit);

    // Add bit to output buffer
    bitBuffer[outIndex] = foundBit;
    outIndex++;
    if (outIndex == captureSize/2) { // output buffer full
      if (doDebug) Serial.println();
      return findHeader();
    }
  } // End for(inIndex)  
  if (doDebug)Serial.println();
  return false;
} // End parseCaptureData()

/*
 * The bitbuffer contains the decoded binary
 * Search for the start of the relevant protocol header
 * return True if header found.
 */
bool findHeader() {
  // Determine if EM4100 tag or FDX-B tag depending on which header expected
  uint8_t theBit;
  uint8_t countBit = 0;
  uint8_t countEnd = (encodingFDX_B) ? 10 : 9;
  
  /* Look for the header, either:
     - 10 0-bits in a row for FDX-B (fdx-b header: 100000000001)
     - 9 1-bits in a row for em4100
     There is no point looking for a header after codelength
  */
  for (uint16_t i = 0; i <= codeLength; i++) {
    theBit = bitBuffer[i];
    // (re)start count on an opposite bit
    if (theBit) {
      if (encodingFDX_B) countBit = 0;  
      else countBit++;
    } else {
      if (encodingFDX_B) countBit++;  
      else countBit = 0;
    }
    // if appropriate header found, parse data
    if (countBit == countEnd) {
      if (encodingFDX_B) return parseBitBufferFDX(i); 
      else return parseBitBufferEM(i);
    }  
  }
  if (doDebug) Serial.printf("No %s header found\n", codeStr[encodingFDX_B]);
  return false;
}

bool parseBitBufferEM(uint8_t currIndex) {
  // Look for EM4100 data
  uint8_t theBit;
  // Clear buffers
  for (uint8_t i = 0; i < 4; i++) columnSums[i] = 0;
  uint16_t countryId = 0;
  uint64_t cardTag = 0;

  // Bit currIndex in bitBuffer is the 9th and final header bit (index 8).
  // Verify that bit 64 is 0 (stop bit, index 63))
  word offset = currIndex - 8; // How many bits were skipped before the header started (ie index of first header bit)
  if (doDebug) Serial.printf("%s header start at: %d\n", codeStr[encodingFDX_B], offset);

  word index = offset + 63;
  theBit = bitBuffer[index];
  if (theBit) {
    if (doDebug) Serial.println("EM stopbit not 0");
    return false;
  }

  // Next: 2+8 rows of 4 bits and an even parity bit. MSB first.
  // The first 2 rows are the customer id (or version number/manufacturer code), the last 8 rows the data.
  // Collect the data in variables cardFacility (8 bits) and cardUid (32 bits)
  uint8_t nibble;
  for (uint8_t count = 0; count < 2; count ++) {
    if ( !parseNibble(offset + 9 + 5 * count, &nibble)) {
      if (doDebug) Serial.println(" - Wrong parity for facility, nibble " + String(count));
      return false;
    }
    countryId |= nibble << (4 * (1 - count)); // Most significant nibble first
  }

  for (uint8_t count = 0; count < 8; count ++) {
    if ( ! parseNibble(offset + 19 + 5 * count, &nibble)) {
      if (doDebug) Serial.println(" EM Wrong parity for uid, nibble " + String(count));
      return false;
    }
    cardTag |= ((long)nibble) << (4 * (7 - count)); // Most significant nibble fir
  }
  // Finally: 4 column parity bits, even
  index = offset + 9 + 10 * 5;
  for (uint8_t i = 0; i < 4; i++) {
    theBit = bitBuffer[index];
    if ((columnSums[i] & 1) != theBit) {
      if (doDebug) Serial.println(" - Wrong parity for column " + String(i));
      return false;
    }
    index++;
  }

  // valid tag found
  currentTag = countryId;
  currentTag = (currentTag<<32) + cardTag; 
  if (doDebug) {
    Serial.print("Valid: ");
    for (uint16_t i = offset; i < offset+64; i++) Serial.print(bitBuffer[i]);
    Serial.println();
  }
  return true;
} // End parseBitBufferEM()

/*
 * Extracts a nibble from the given offset in bitBuffer and checks for even parity in the 5th bit.
 * return  True if the parity bit matches the data bits.
 */
bool parseNibble(word offset, uint8_t *nibble) {
  uint8_t theBit;
  uint8_t bitSum = 0;

  *nibble = 0;
  for (uint8_t i = 0; i < 4; i++) {
    theBit = bitBuffer[offset];
    columnSums[i] += theBit; // Used later for column parity check
    bitSum += theBit;
    *nibble |= (theBit << (3 - i)); // First bit is MSB
    offset++;
  }

  // Check for even parity
  theBit = bitBuffer[offset];
  if ((bitSum & 1) != theBit) return false;

  return true;
} // End parseNibble()


/*
 * The captured transitions have now been translated.
 * Try find data matching the FDX-B encoding.
 * If decoding succeeds the results are store in cardTag and countryId.
 * return true if decoding succeeds.
 * Once tag data read, rest can be ignored
 */
bool parseBitBufferFDX(uint8_t currIndex) {
  // Look for FDX-B data
  uint8_t theBit;
  uint16_t countryId = 0;
  uint64_t cardTag = 0;
  
  // now check that next bit is a 1
  currIndex++;
  theBit = bitBuffer[currIndex];
  if (!theBit) {
    if (doDebug) Serial.println("FDX-B header not terminated");
    return false;
  }

  // next 54 bits (38 bits + 10 bits + control bits) are card id 
  uint8_t tagId[48];
  uint8_t idCntr = 0;
  word count = 0;
  uint8_t offset = currIndex+1;
  for (uint8_t i = offset; i < offset+54; i++) {
    theBit = bitBuffer[i];
    if (count < 8) {
      // save data bit
      tagId[idCntr] = theBit;
      count++;
      idCntr++;
    } else {    
      // check control bit and skip it
      if (!theBit) {
        if (doDebug) Serial.println("FDX-B control bit wrong");
        return false;
      } else count = 0;
    }
  }
  // convert tag id to decimal, reversing bit order
  for (uint8_t i = 38; i > 0; i--) {
    cardTag = cardTag << 1;
    if (tagId[i-1]) cardTag |= 1; 
  }

  //  convert country code to decimal, reversing bit order
  for (uint8_t i = 48; i > 38; i--) {
    countryId = countryId << 1;
    if (tagId[i-1]) countryId |= 1; 
  }

  // valid tag found
  currentTag = countryId;
  currentTag = (currentTag<<38) + cardTag; 
  if (doDebug) Serial.println(String(codeStr[encodingFDX_B]));
  return true;
} // End parseBitBufferFDX()


void tagStr(uint64_t thisTag, char* thisTagStr) {
  // format 64 bit tag into decimal string
  // 13 digits for EM41000
  // 15 digits for FDX-B
  uint64_t thisCardTag;
  uint16_t thisCountryId;
  uint64_t bitmask = 0x00000000FFFFFFFF; // EM4100
  int cardTagBits = 32; // EM4100
  int cardTagLen = 10; // EM4100 
  if (encodingFDX_B) { // FDX-B
    cardTagBits = 38; 
    cardTagLen = 12;
    bitmask = 0x0000003FFFFFFFFF;
  }
  thisCardTag = (thisTag & bitmask);
  thisCountryId = (uint16_t)(thisTag >> cardTagBits);
  char catTagStr[cardTagLen+1]; // +1 for terminator
  catTagStr[cardTagLen] = 0; // terminator
  for (size_t i = cardTagLen; i--; thisCardTag /= 10) catTagStr[i] = '0' + (thisCardTag % 10);  
  strcpy(thisTagStr, String(thisCountryId).c_str());
  strcpy(thisTagStr+strlen(thisTagStr), catTagStr);
}

void runClock(int LEDC_FREQ, uint8_t LEDC_HS_CH0_GPIO) {
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     * using required frequency in hertz and gpio from high speed channel group
     // high speed channel group gpio 18, 19
     // low speed channel group gpio 4, 5
     * to create a pwn square wave of 50% duty at required frequency
     */
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;  // hi speed timer mode
    ledc_timer.duty_resolution = LEDC_TIMER_2_BIT; // resolution of PWM duty
    ledc_timer.timer_num = LEDC_TIMER_0;            // timer index
    ledc_timer.freq_hz = LEDC_FREQ;                // frequency of PWM signal
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - output duty cycle - the value of which depends on duty_resolution
     * - GPIO number where signal is output
     * - speed mode, either high or low
     * - timer servicing selected channel
     */
    ledc_channel_config_t ledc_channel = {0};
    ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_0;
    ledc_channel.timer_sel  = LEDC_TIMER_0;
    ledc_channel.duty       = 2;  // 50% for LEDC_TIMER_2_BIT

    // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel);
}
