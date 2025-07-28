/*
* Decode EM4100 or FDX-B tags
* The following library https://github.com/Sthing/AccessThing/tree/master/Arduino/libraries/EM4100
* was used as a starting point to derive this library
* together with info for decoding FDX-B from http://www.gizmolab.co.za/fdx-b-protocol/
* 
* s60sc 2018, 2024
*/

#include "appGlobals.h"
#include "driver/ledc.h"

int rfidDemod; // input - demod pin from RDM6300, pullup high
int rfidClock; // output - clock pin to RDM6300
int clearPin; // input - clear tag display
float rfidFreq; // RDM6300 antenna frequency
bool encodeFDX = false; // false for EM4100, true for FXD-B
uint64_t currentTag; // read tag value
char encodingStr[17]; // encoding type string
char currentTagStr[17]; // holds string representation of tag
static bool buttonPressed = false; // to clear tag display

// Buffer sizing
static const uint16_t bitBufferSize = 256;     // Room for 2 x FDX-B codeLength bits, EM4100 is smaller
static uint16_t slBoundary, minPulse, maxPulse; // boundary between long and short pulses
static int codeLength, captureSize;
static const int minLength = 64;  // min length for complete tag, for FDX-B, only first 64 bits are relevant
static int captureIndex = 0;

// Variables used during decoding
static uint8_t columnSums[4];   // Used for EM4100 parity calculation of the four columns
static uint8_t bitBuffer[bitBufferSize]; // Each byte contains data for 4 full bit periods. LSB is first bit.
static const char* codeStr[2] = {"EM4100", "FDX-B"};
static volatile bool captured = false; // whether sufficient pulses have been captured for processing
static int headerPos = 0;

// simulation and debugging
#define simulation false
static uint16_t pulses[bitBufferSize * 2];  // save pulse lengths, 1 bit can be represented by 2 pulses
static int pulseIndex = 0;
static uint32_t simPulse;

// forward refs
static bool parseNibble(word offset, uint8_t *nibble);
static bool parseBitBufferEM(uint8_t currIndex);
static bool parseBitBufferFDX(uint8_t currIndex);

static void runClock() {
  // output PWM clock to RDM6300 at required frequency
  int dutyBits = 1; // 50% duty
  ledcAttach(rfidClock, (int)(rfidFreq * 1000), dutyBits); 
  ledcWrite(rfidClock, 1);
}

void IRAM_ATTR buttonISR() {
  buttonPressed = true;
} 

static void IRAM_ATTR demodISR() {
  // obtain the encoded bit stream from the reader demodulated output
  // consists of short and long pulses, short pulses must occur in pairs (for one bit)
  // FDX-B demodulation: One long pulse = logic 1, two short pulses = logic 0
  // EM4100 demodulation: One long pulse = change logic level, two short pulses = same as previous logic level
  //  - but dont know logic polarity
  
  bool invalidPulse = false;
  static uint8_t level = 1;
  static bool gotFirstShort = false;
  static uint32_t lastMicros = 0;
  uint32_t elapsedMicros;
  
  if (!captured) {
    elapsedMicros = simulation ? simPulse : micros() - lastMicros; 
    lastMicros = micros(); // elapsed uS between interrupts
    if (captureIndex >= captureSize) captured = true;
    else {
      if (elapsedMicros > minPulse && elapsedMicros < maxPulse) { 
        // potentially valid pulse
        if (elapsedMicros > slBoundary) {
          // long pulse
          if (gotFirstShort) invalidPulse = true; // as only have single short pulse
          else {
            if (encodeFDX) bitBuffer[captureIndex++] = 1; // FDX-B long pulse = logic 1
            else {
              // EM4100 long pulse changes logic level
              level ^= 1;
              bitBuffer[captureIndex++] = level;
            }
          }
        } else {
          // short pulse, FDX-B = logic 0, EM4100 same as previous level
          if (gotFirstShort) {
            // second of two short pulses
            bitBuffer[captureIndex++] = encodeFDX ? 0 : level;
            gotFirstShort = false;
          } else gotFirstShort = true;
        }
        pulses[pulseIndex++] = elapsedMicros; // debugging
      } else invalidPulse = true;
      // restart capture on outliers unless min length achieved
      if (invalidPulse) {
        gotFirstShort = false;
        if (captureIndex < minLength) captureIndex = pulseIndex = 0; // restart capture
        else captured = true; // try current capture if minumum capture length achieved
      }
    }
  }
}

static bool findHeader() {
  // Search for the start of the relevant protocol header
  // Determine if EM4100 tag or FDX-B tag depending on which header expected
  // Header is either:
  // - 10 0-bits in a row for FDX-B (eg 100000000001)
  // - 9 1-bits in a row for em4100 (eg 01111111110)
  // There is no point looking for a header after codelength
  
  uint8_t theBit;
  uint8_t countBit = 0;
  uint8_t countEnd = (encodeFDX) ? 10 : 9;
  
  for (uint16_t i = 0; i <= codeLength; i++) {
    theBit = bitBuffer[i];
    // (re)start count on an opposite bit
    countBit = theBit ? (encodeFDX ? 0 : countBit + 1) : (encodeFDX ? countBit + 1 : 0);
    // if appropriate header found, parse data
    if (countBit == countEnd) {
      if (encodeFDX) return parseBitBufferFDX(i); 
      else return parseBitBufferEM(i);
    }
  }
  LOG_VRB("No %s header found", codeStr[encodeFDX]);
  return false;
}

static bool parseBitBufferEM(uint8_t currIndex) {
  // match data to EM4100 encoding 
  uint8_t theBit;
  // Clear buffers
  for (uint8_t i = 0; i < 4; i++) columnSums[i] = 0;
  uint16_t countryId = 0;
  uint64_t cardTag = 0;

  // Bit currIndex in bitBuffer is the 9th and final header bit (index 8).
  // Verify that bit 64 is 0 (stop bit, index 63))
  uint32_t offset = currIndex - 8; // How many bits were skipped before the header started (ie index of first header bit)
  LOG_VRB("%s header start at: %d", codeStr[encodeFDX], offset);
  headerPos = offset;

  uint32_t index = offset + 63;
  theBit = bitBuffer[index];
  if (theBit) {
    LOG_VRB("EM stopbit not 0");
    return false;
  }

  // Next: 2+8 rows of 4 bits and an even parity bit. MSB first.
  // The first 2 rows are the customer id (or version number/manufacturer code), the last 8 rows the data.
  // Collect the data in 8 bit facility (countryId) and 32 bit cardUID [cardTag] 
  uint8_t nibble;
  for (uint8_t count = 0; count < 2; count ++) {
    if ( !parseNibble(offset + 9 + 5 * count, &nibble)) {
      LOG_VRB(" - Wrong parity for card facility, nibble %d", count);
      return false;
    }
    countryId |= nibble << (4 * (1 - count)); // Most significant nibble first
  }

  for (uint8_t count = 0; count < 8; count ++) {
    if ( ! parseNibble(offset + 19 + 5 * count, &nibble)) {
      LOG_VRB(" EM Wrong parity for card UID, nibble %d", count);
      return false;
    }
    cardTag |= ((long)nibble) << (4 * (7 - count)); // Most significant nibble fir
  }
  // Finally: 4 column parity bits, even
  index = offset + 9 + 10 * 5;
  for (uint8_t i = 0; i < 4; i++) {
    theBit = bitBuffer[index];
    if ((columnSums[i] & 1) != theBit) {
      LOG_VRB(" - Wrong parity for column %d", i);
      return false;
    }
    index++;
  }

  // valid tag found
  currentTag = countryId;
  currentTag = (currentTag<<32) + cardTag; 
  return true;
} // End parseBitBufferEM()

static bool parseNibble(word offset, uint8_t *nibble) {
  // Extracts a nibble from the given offset in bitBuffer and checks for even parity in the 5th bit.
  // returns true if the parity bit matches the data bits.
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

static bool parseBitBufferFDX(uint8_t currIndex) {
  // Match data to FDX-B encoding.
  // If decoding succeeds the results are stored in cardTag and countryId.
  // Once tag data read, rest can be ignored
  uint8_t theBit;
  uint16_t countryId = 0;
  uint64_t cardTag = 0;
  
  // now check that next bit is a 1
  currIndex++;
  theBit = bitBuffer[currIndex];
  if (!theBit) {
    LOG_VRB("FDX-B header not terminated");
    return false;
  } 
  headerPos = currIndex - 10;
  LOG_VRB("%s header start at: %d", codeStr[encodeFDX], headerPos);

  // next 54 bits (38 bits + 10 bits + control bits) are card id 
  // ignore final control bit
  uint8_t tagId[48];
  uint8_t idCntr = 0;
  word count = 0;
  uint8_t offset = currIndex+1;
  for (uint8_t i = offset; i < offset+53; i++) {
    theBit = bitBuffer[i];
    if (count < 8) {
      // save data bit
      tagId[idCntr] = theBit;
      count++;
      idCntr++;
    } else {    
      // check control bit and skip it
      if (!theBit) {
        LOG_VRB("FDX-B control bit wrong");
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
  LOG_VRB("%s", codeStr[encodeFDX]);
  return true;
} // End parseBitBufferFDX()

static void tagStr() {
  // format 64 bit tag into decimal string
  // 13 digits for EM41000
  // 15 digits for FDX-B
  uint64_t thisCardTag;
  uint16_t thisCountryId;
  uint64_t bitmask = 0x00000000FFFFFFFF; // EM4100
  int cardTagBits = 32; // EM4100
  int cardTagLen = 10; // EM4100 
  if (encodeFDX) { // FDX-B
    cardTagBits = 38; 
    cardTagLen = 12;
    bitmask = 0x0000003FFFFFFFFF;
  }
  thisCardTag = (currentTag & bitmask);
  thisCountryId = (uint16_t)(currentTag >> cardTagBits);
 sprintf(currentTagStr, "%03u", thisCountryId);
  // card tag is LSB ordered
  for (size_t i = cardTagLen; i--; thisCardTag /= 10) currentTagStr[i + 3] = '0' + (thisCardTag % 10);  
  currentTagStr[cardTagLen + 3] =  0;
}

static bool getTag() {
  // decode RFID tag 
  if (findHeader()) {
    tagStr();
    LOG_INF("Read tag - %s", currentTagStr);
    return true;
  } else { 
    LOG_VRB("Tag not read");
    currentTag = currentTagStr[0] = 0;
  }
  return false;
}

static void showRawData(bool haveTag) {
  // show captured pulse lengths and decoding for debugging
  if (dbgVerbose) {
    logPrint("%s", "------------------------------------------\n");
    logPrint("pulses: %d\n", pulseIndex);
    logPrint("slBoundary: %d\n", slBoundary);
    logPrint("%s", "pulselen: ");
    for (int i = 0; i < pulseIndex; i++) logPrint("%d/", pulses[i]);
    logPrint("\n%s", "decoded: ");
    for (int i = 0; i < captureIndex; i++) logPrint("%u", bitBuffer[i]);
    if (haveTag) {
      logPrint("\n%s", "tag: ");
      for (int i = 0; i < minLength; i++) logPrint("%u", bitBuffer[headerPos + i]);
    }
    logPrint("%s", "\n------------------------------------------\n\n");
  }
} 

static void demodSim() {
  // simulate pulses from rfid reader using pregenerated pulse sequence
  uint16_t pulseCount = 0;
  
  const char* delimiter = "/";
  // EM4100 tag 0290003174496
  const char* pulseLensEM = "200/310/457/308/203/306/204/307/204/305/206/302/210/300/210/299/211/300/210/299/211/301/210/301/210/558/208/301/466/300/210/301/210/300/210/306/203/306/205/301/210/556/210/300/210/299/211/299/467/300/210/300/210/299/212/298/212/300/211/554/212/297/468/299/213/297/213/298/213/297/213/299/211/298/213/554/213/297/213/297/469/297/213/553/218/292/213/297/216/289/222/294/216/294/215/295/217/294/215/295/471/295/216/295/216/552/213/297/215/291/219/296/470/551/216/292/474/294/216/295/217/290/220/290/220/291/219/290/220/291/220/290/220/290/220/290/220/290/220/550/216/295/472/289/221/290/220/290/220/290/221/290/219/294/217/546/220/290/220/292/219/290/477/289/221/289/222/288/221/289/221/289/222/546/220/290/476/289/222/289/221/289/222/288/222/289/222/288/222/545/222/288/222/288/479/289/225/542/221/289/221/289/221/289/221/290/220/290/221/290/219/291/220/289/477/289/222/289/222/544/222/288/221/289/222/289/478/544";
  // FDX-B tag 945000001554408
  const char* pulseLensFDX = "90/115/100/225/110/110/95/115/105/115/100/120/100/110/95/115/105/115/100/120/100/110/105/105/210/130/95/120/95/115/95/225/110/115/205/225/215/230/210/225/215/125/100/220/210/125/100/225/210/225/215/230/110/110/200/130/95/120/100/115/95/220/110/115/100/115/105/110/100/115/100/115/100/120/100/110/100/120/200/130/95/120/95/115/100/110/105/115/100/120/95/225/105/110/210/125/95/120/100/215/220/125/90/225/210/230/215/120/100/110/105/115/100/120/95/115/100/110/105/115/100/120/95/225/110/105/105/115/100/110/105/115/100/110/105/115/100/115/210/225/105/115/105/110/210/225/110/110/105/110/210/225/215/125/95/225/110/105/105/115/100/115/100/115/205/230/210/125/100/115/95/120/100/115/100/115/100/115/105/110/100/115/100/220/115/110/95/120/100/110/105/115/100/115/95/120/100/110/105/115/210/120/95/120/95/115/105/115/100/110/105/115/95/115/105/115/100/220/110/105/105/115/105/115/95/115/105/110/100/120/100/115/95/115/105/110/100/120/205/120/100/120/95/115/100/225/110/105/210/230/215/225/210/230/215/125/95/220/215/125/100/215/215/230/215/225/110/105/210/130/95/120/95/115/95/225/110/115/100/110/105/110/100/115/105/115/100/115/100/110/100/120/205/130/95/115/95/115";

  char pulseLens[2048]; // as strtok is destructive of original string
  encodeFDX ? strcpy(pulseLens, pulseLensFDX) : strcpy(pulseLens, pulseLensEM);
  // Calculate the number of pulses
  for (int i = 0; pulseLens[i] != '\0'; ++i) if (pulseLens[i] == '/') ++pulseCount;
  
  // get each pulse length and process
  char* token = strtok(pulseLens, delimiter);
  for (int i = 0; i < pulseCount; i++) {
     simPulse = static_cast<uint16_t>(atoi(token));
     demodISR(); // send simulated pulse
    // get next pulse length from string
    token = strtok(nullptr, delimiter);
    if (token == nullptr) break;
  }
  captured = true;
}

void rfidSetup() {
  // set encoding to FDX-B (true) or EM4100 (false)
  codeLength = (encodeFDX) ? 128 : 64; // FDX-B is 128 bits, EM4100 is 64 bits, 
  if (!rfidFreq) rfidFreq = (encodeFDX) ? 134.2 : 125.0; // default antenna clock in Hz
  // short pulse length in uS between minPulse & slBoundary
  // long pulse length in uS between slBoundary & maxPulse
  minPulse = (encodeFDX) ? 85 : 170; 
  maxPulse = (encodeFDX) ? 320 : 560; 
  slBoundary = (encodeFDX) ? 180 : 360; // boundary between short and long pulse time in uS. 

  captureSize = codeLength * 2; // Capture at 2 x codeLength bit periods to catch at a complete tag regardless of start
  LOG_VRB("codeLength %d, minPulse %d, maxPulse %d, slBoundary %d, captureSize %d", codeLength, minPulse, maxPulse, slBoundary, captureSize);
  if (!simulation) {
    // setup connections to RDM6300
    pinMode(rfidDemod, INPUT_PULLUP);
    pinMode(rfidClock, OUTPUT);
    runClock(); // output square wave clock to RDM6300
    attachInterrupt(digitalPinToInterrupt(rfidDemod), demodISR, CHANGE);
    if (clearPin) {
      pinMode(clearPin, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(clearPin), buttonISR, FALLING); 
    }
  }
  sprintf(encodingStr, "%s %0.1fkHz", codeStr[encodeFDX], rfidFreq);
  updateLcd(false);
  LOG_INF("Detect tag type: %s", encodingStr);
}

void rfidRead() {
  // check if tag available to read
  static uint64_t lastTag = 0;
  bool haveTag = false;
  if (!currentTag) lastTag = 0;
  if (captured) {
    haveTag = getTag();
    if (!haveTag && !encodeFDX) {
      // if EM41000, retry after inverting logic
      LOG_VRB("%s", "Retry after logic inversion");
      for (int i = 0; i < bitBufferSize; i++) bitBuffer[i] ^= 1;
      haveTag = getTag(); 
    }
    showRawData(haveTag);
    captureIndex = pulseIndex = 0; // empty buffers ready for next tag
    captured = false;
  } else if (simulation) demodSim();
  if (buttonPressed) {
    updateLcd(false);
    buttonPressed = false;
  }
  if (haveTag) {
    if (currentTag != lastTag) {
      // to avoid display flicker, only update if different tag
      lastTag = currentTag;
      updateLcd(true);
    }
  }
}
