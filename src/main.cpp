#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <EthernetENC.h>
#include <Watchdog.h>
#include "zfont5x8.cpp"

#define ETHERNET_SELECT_PIN 10
#define SERIAL_BAUD 115200
// #define SERIAL_EN
#define DEBUG_SERVER

#define EEPROM_BOOT_COUNT 0x00
//#define SET_BOOT_COUNT 0

#define LOW_NIBBLE 0x0f
#define HIGH_NIBBLE 0xf0
#define MHZ 1000000

#define MX_NOOP 0x00
#define MX_DIG_0 0x01
#define MX_DECODE_MODE 0x09
#define MX_INTENSITY 0x0a
#define MX_SCAN_LIMIT 0x0b
#define MX_SHUT_DOWN 0x0c
#define MX_DISPLAY_TEST 0x0f

#define MX_DISPLAY_TEST_ON 0x01
#define MX_DISPLAY_TEST_OFF 0x00

#define MX_SHUT_DOWN_ON 0x00
#define MX_SHUT_DOWN_NORMAL_OPERATION 0x01

#define MX_NO_DECODE 0x00 
#define MX_DEFAULT_INTENSITY 0xff
#define MX_SCAN_8 0x07

#define MX_BITMASK B00001000 // PD3
#define MX_PORT PORTD
#define MX_DDR DDRD

#define MX_LOW MX_PORT &= ~MX_BITMASK;
#define MX_HIGH MX_PORT |= MX_BITMASK;
#define MX_PULL MX_DDR |= MX_BITMASK;
#define MX_RELEASE MX_DDR &= ~MX_BITMASK;

#define MX_REFRESH_TIME 60000
#define MX_SPI_SPEED_MAX 125000

#define MX_DISPLAY_L1 0
#define MX_DISPLAY_L2 1
#define MX_DISPLAY_COUNT 2
#define MX_DEVICE_COUNT 4
#define MX_CHAR_COUNT 8
#define MX_ROW_COUNT 8
#define MX_COL_COUNT 8
#define MX_NO_PIXELS 0x00
#define MX_SCREEN_COUNT 2
#define MX_SCREEN_DATA 0
#define MX_SCREEN_PROGRAM 1

#define MX_CHAR_EMPTY '\0'
#define MX_CHAR_INIT "."
#define MX_DISPLAY_REVERSE

#define ZFONT_OFFSET 0x20
#define ZFONT_CHAR_SIZE 5
#define ZFONT_TRIM_RIGHT
#define ZFONT_TRIM_LEFT

// #define SET_WS_INIT // use init colors at boot
#define WS_INIT_RED 0xff
#define WS_INIT_GREEN 0xff
#define WS_INIT_BLUE 0xff
#define WS_LED_COUNT 40 

#define WS_T0H 1  //6 3 400 ns // cycles
#define WS_T1H 8  //12 9 800 ns 
#define WS_T0L 6  //13 9 7 850 ns
#define WS_T1L 1   //7 3  450 ns

#define WS_BITMASK B00010000 // PD4
#define WS_PORT PORTD
#define WS_DDR DDRD
#define WS_LOW WS_PORT &= ~WS_BITMASK;
#define WS_HIGH WS_PORT |= WS_BITMASK;
#define WS_PULL WS_DDR |= WS_BITMASK;
#define WS_RELEASE WS_DDR &= ~WS_BITMASK;

#define WS_RESET_TIME 55
#define WS_REFRESH_TIME 30000

#define BTN_BITMASK B00000001
#define BTN_PORT PORTB
#define BTN_DDR DDRB
#define BTN_PIN PINB
#define BTN_READ (BTN_PIN & BTN_BITMASK)
#define BTN_INPUT BTN_DDR &= ~BTN_BITMASK
#define BTN_PULLUP BTN_PORT |= BTN_BITMASK
#define BTN_DEBOUNCE_COUNTDOWN_START 32

#define PROGRAM_ID_INIT 0
#define PROGRAM_ID_MAX 10
#define PROGRAM_ID_EEPROM 0x08
#define PROGRAM_VALUES_EEPROM 0x10
#define PROGRAM_SIZE 4

#define PAGE_PREVIOUS_CHAR_INIT 0x00
#define PAGE_CHAR_INIT 0x00

#define PAGE_CHAR_POS_INIT 0x00
#define PAGE_CHAR_POS_MAX 0xff

#define PAGE_STORE 0x80
#define PAGE_GET 0x40
#define PAGE_BLUE 0x20
#define PAGE_GREEN 0x10
#define PAGE_RED 0x08
#define PAGE_DISPLAY_INTENSITY 0x04
#define PAGE_L2 0x02
#define PAGE_L1 0x01
#define PAGE_INIT 0x00

#define PAGE_SWITCH_PROGRAM 0x20
#define PAGE_SWITCH_INTENSITY 0x10
#define PAGE_SWITCH_TEXT 0x08
#define PAGE_SWITCH_L 0x04
#define PAGE_SWITCH_PATH 0x02
#define PAGE_SWITCH_BLANK_LINE 0x01
#define PAGE_SWITCH_INIT 0x00;

#define SERV_OK 0x08
#define SERV_SERVICE_UNAVAILABLE 0x04
#define SERV_NOT_FOUND 0x02
#define SERV_BAD_REQUEST 0x01
#define SERV_NONE 0x00

#define SERV_ERR 0x01
#define SERV_UNDEFINED 0x00

#define CYCLES_MICROSEC (F_CPU / 1000000)

// See https://stackoverflow.com/a/63468969
template< unsigned N > 
inline static void nops(){
  asm ("nop");
  nops< N - 1 >();
}
template<> inline void nops<0>(){};

const byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};
const IPAddress ip(10, 200, 125, 81);
// const IPAddress ip(192, 168, 1, 181);
EthernetServer server(80);

Watchdog watchdog;

uint8_t wsRed = WS_INIT_RED;
uint8_t wsGreen = WS_INIT_GREEN;
uint8_t wsBlue = WS_INIT_BLUE;

uint8_t mxIntensity = MX_DEFAULT_INTENSITY;
uint8_t programId = PROGRAM_ID_INIT;

uint32_t wsLastRequest = 0;
uint16_t wsInterval = 0;

uint32_t mxLastRequest = 0;
uint16_t mxInterval = 0;

uint8_t mxMatrix[MX_SCREEN_COUNT][MX_DISPLAY_COUNT][MX_DEVICE_COUNT][MX_ROW_COUNT];
char mxChar[MX_CHAR_COUNT] = MX_CHAR_INIT;
uint8_t mxScreen = MX_SCREEN_DATA;

uint32_t bootCount;

uint8_t prevClientChar;
uint8_t clientChar;
uint8_t clientCharPos;
uint8_t pageSwitch;
uint8_t page;
uint8_t serv;

uint8_t btnCount = BTN_DEBOUNCE_COUNTDOWN_START;

const uint8_t* adr;

inline void eepromU32Put(uint16_t location, uint8_t index, uint32_t data){
  EEPROM.put(location + (index * 4), data);
}

inline uint32_t eepromU32Get(uint16_t location, uint8_t index){
  uint32_t data;
  EEPROM.get(location + (index * 4), data);
  return data;
}

inline uint32_t eepromU32Inc(uint16_t location, uint8_t index){
  uint32_t data;
  data = eepromU32Get(location, index);
  data++;
  eepromU32Put(location, index, data);
  return data;
}

inline void wsWriteColor(uint8_t wsColor){
  uint8_t bitMask;
  for (bitMask = 0x80; bitMask; bitMask >>= 1){
    WS_HIGH;
    if (wsColor & bitMask){
      nops<WS_T1H>();
      WS_LOW;
      nops<WS_T1L>();
      continue;
    }
    nops<WS_T0H>();
    WS_LOW;
    nops<WS_T0L>();
  }
}

inline void wsWrite(){
  uint8_t i;

  noInterrupts();
  for (i = 0; i < WS_LED_COUNT; i++){
    wsWriteColor(wsGreen);
    wsWriteColor(wsRed);
    wsWriteColor(wsBlue);
  }
  interrupts();
}

inline void mxSPIBegin(){
  SPI.beginTransaction(SPISettings(MX_SPI_SPEED_MAX, MSBFIRST, SPI_MODE0));
}

inline void mxWriteAll(uint8_t adr, uint8_t data){
  uint8_t iii;

  mxSPIBegin();
  MX_LOW;

  for (iii = 0; iii < MX_DISPLAY_COUNT * MX_DEVICE_COUNT; iii++){
    SPI.transfer(adr);
    SPI.transfer(data);    
  }

  MX_HIGH;
  SPI.endTransaction();
}

inline void mxWriteLines(){
  uint8_t mxDisplay;
  uint8_t mxDevice;
  uint8_t mxRow;

  for (mxRow = 0; mxRow < MX_ROW_COUNT; mxRow++){

    mxSPIBegin();
    MX_LOW;

    for (mxDisplay = 0; mxDisplay < MX_DISPLAY_COUNT; mxDisplay++){
      for (mxDevice = 0; mxDevice < MX_DEVICE_COUNT; mxDevice++){
        SPI.transfer(mxRow + MX_DIG_0);
        SPI.transfer(mxMatrix[mxScreen][mxDisplay][mxDevice][mxRow]);      
      }    
    }

    MX_HIGH;
    SPI.endTransaction();
  }
}

inline void mxWriteMatrixColumn(uint8_t screen, uint8_t display, uint8_t col, uint8_t pixels){
  uint8_t mxRow;
  uint8_t mxDevice = col >> 3;
  uint8_t mxColMask = 0x80 >> (col & 0x07);
  uint8_t pixelBitMask = 0x01;

  for (mxRow = 0; mxRow < MX_ROW_COUNT; mxRow++){
    if (pixels & pixelBitMask){
      mxMatrix[screen][display][mxDevice][mxRow] |= mxColMask;
    } else {
      mxMatrix[screen][display][mxDevice][mxRow] &= ~mxColMask;      
    }
    pixelBitMask <<= 1;
  }
}

inline void mxCalcPixelMatrix(uint8_t screen, uint8_t display){
  uint8_t col;
  uint8_t pixels;
  uint8_t zfPos = ZFONT_CHAR_SIZE;
  uint8_t chPos = strlen(mxChar);
  const uint8_t* zfAdr;
  #ifdef SERIAL_EN
    uint8_t seri;
  #endif

  for (col = MX_DEVICE_COUNT * MX_COL_COUNT; col; col--){

    if (chPos){
      if (zfPos == ZFONT_CHAR_SIZE){
        #ifdef SERIAL_EN
          Serial.print("char ");
          Serial.print(mxChar[chPos - 1]);
          Serial.print(": ");
          Serial.println(mxChar[chPos - 1], HEX);
        #endif

        zfAdr = zfont5x8 + (ZFONT_CHAR_SIZE * (mxChar[chPos - 1] - ZFONT_OFFSET));
      }

      if (zfPos){
        zfPos--;
        pixels = pgm_read_byte_near(zfAdr + zfPos);
      } else {
        // blank col between chars
        pixels = MX_NO_PIXELS;
        chPos--;
        zfPos = ZFONT_CHAR_SIZE;
      }
    } else {
      // blank space on right side
      pixels = MX_NO_PIXELS;
    }

    #ifdef SERIAL_EN
      for (seri = 0x01; seri; seri <<= 1){
        Serial.print(seri & pixels ? '@' : '.');
      }
      Serial.println();
    #endif

    mxWriteMatrixColumn(screen, display, col - 1, pixels);
  }
}

inline void getProgramValuesFromEeprom(){
  uint8_t adr = PROGRAM_VALUES_EEPROM + (programId * 4);
  EEPROM.get(adr, mxIntensity);
  EEPROM.get(adr + 1, wsRed);
  EEPROM.get(adr + 2, wsGreen);
  EEPROM.get(adr + 3, wsBlue);
}

inline void putProgramValuesToEeprom(){
  uint8_t adr = PROGRAM_VALUES_EEPROM + (programId * 4);
  EEPROM.put(adr, mxIntensity);
  EEPROM.put(adr + 1, wsRed);
  EEPROM.put(adr + 2, wsGreen);
  EEPROM.put(adr + 3, wsBlue);
}

inline void getProgramIdFromEeprom(){
  EEPROM.get(PROGRAM_ID_EEPROM, programId);
}

inline void putProgramIdToEeprom(){
  EEPROM.put(PROGRAM_ID_EEPROM, programId);
}

inline void mxCalcProgramPixelMatrix(uint8_t display){
  mxChar[0] = '.';
  mxChar[1] = '.';
  mxChar[2] = '.';
  if (display == MX_DISPLAY_L2){
    mxChar[3] = programId + '0';    
  } else {
    mxChar[3] = '.';
  }
  mxChar[4] = MX_CHAR_EMPTY;

  #ifdef MX_DISPLAY_REVERSE
    if (display == MX_DISPLAY_L1){
      mxCalcPixelMatrix(MX_SCREEN_PROGRAM, MX_DISPLAY_L2);            
    } else {
      mxCalcPixelMatrix(MX_SCREEN_PROGRAM, MX_DISPLAY_L1);
    }
  #else 
    if (display == MX_DISPLAY_L1){
      mxCalcPixelMatrix(MX_SCREEN_PROGRAM, MX_DISPLAY_L1);            
    } else {
      mxCalcPixelMatrix(MX_SCREEN_PROGRAM, MX_DISPLAY_L2);
    }        
  #endif
}

inline void btnRead(){
  if (BTN_READ){
    if (btnCount == BTN_DEBOUNCE_COUNTDOWN_START){
      // trigger to normal operation
      mxScreen = MX_SCREEN_DATA;
      mxInterval = 0;
    }

    if (btnCount){
      btnCount--;
    }
  } else {
    if (!btnCount){
      // trigger "btn on", next program
      programId++;
      if (programId == PROGRAM_ID_MAX){
        programId = 0;
      }
      putProgramIdToEeprom();
      getProgramValuesFromEeprom();
      mxCalcProgramPixelMatrix(MX_DISPLAY_L2);
      mxScreen = MX_SCREEN_PROGRAM;
      mxInterval = 0;
      wsInterval = 0;
    }

    btnCount = BTN_DEBOUNCE_COUNTDOWN_START;
  }
}

void setup() {
  delay(200);
  MX_HIGH;
  MX_PULL;
  WS_LOW;
  WS_PULL;
  BTN_INPUT;
  BTN_PULLUP;
  delay(200);

  Ethernet.init(ETHERNET_SELECT_PIN);
  SPI.begin();
  Ethernet.begin(mac, ip);
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

  delay(200);

  if (Ethernet.hardwareStatus() == EthernetHardwareStatus::EthernetNoHardware) {
#ifdef SERIAL_EN
    Serial.println("ENC28J60 not found.");
#endif
    while(1){
      delay(1);
    }
  }

#ifdef SERIAL_EN
  if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
    Serial.println("Ethernet not connected.");
  } else {
    Serial.println("Ethernet ok.");
  }
#endif 

  server.begin();

#ifdef SET_BOOT_COUNT
  bootCount = SET_BOOT_COUNT;
#else
  bootCount = eepromU32Get(EEPROM_BOOT_COUNT, 0);
  bootCount++;   
#endif

  eepromU32Put(EEPROM_BOOT_COUNT, 0, bootCount);  

#ifdef SERIAL_EN
  Serial.print("Boot count: ");
  Serial.println(bootCount);
#endif

  mxCalcPixelMatrix(MX_SCREEN_DATA, MX_DISPLAY_L1);
  mxCalcPixelMatrix(MX_SCREEN_DATA, MX_DISPLAY_L2);

  getProgramIdFromEeprom();
  getProgramValuesFromEeprom();
  mxCalcProgramPixelMatrix(MX_DISPLAY_L1);
  mxCalcProgramPixelMatrix(MX_DISPLAY_L2);

  delay(200);

  wsLastRequest = millis();
  mxLastRequest = millis();
  wsInterval = 0;
  mxInterval = 0;
  watchdog.enable(Watchdog::TIMEOUT_4S);
}

void loop() {
  uint8_t intensity;
  uint8_t strLen;

  watchdog.reset();

  btnRead();

  if (millis() - wsLastRequest > wsInterval) {
    wsWrite();
    wsInterval = WS_REFRESH_TIME;
    wsLastRequest = millis();
  }

  btnRead();

  if (millis() - mxLastRequest > mxInterval) {

    mxWriteAll(MX_DECODE_MODE, MX_NO_DECODE);
    mxWriteAll(MX_SCAN_LIMIT, MX_SCAN_8);
    mxWriteAll(MX_INTENSITY, mxIntensity >> 4);
    mxWriteAll(MX_DISPLAY_TEST, MX_DISPLAY_TEST_OFF);
    mxWriteAll(MX_SHUT_DOWN, MX_SHUT_DOWN_NORMAL_OPERATION);  
    mxWriteLines();

    mxInterval = MX_REFRESH_TIME;
    mxLastRequest = millis();
  }

  btnRead();

  /**
   * Listen ethernet clients
   */

  Ethernet.maintain();

  EthernetClient client = server.available();

  if (!client) {
    return;
  }

#ifdef SERIAL_EN
  Serial.println("HTTP ");
  Serial.println("client >>");
#endif

  prevClientChar = PAGE_PREVIOUS_CHAR_INIT;
  clientChar = PAGE_CHAR_INIT;
  clientCharPos = PAGE_CHAR_POS_INIT;
  pageSwitch = PAGE_SWITCH_INIT;
  page = PAGE_INIT;
  serv = SERV_UNDEFINED;
  mxChar[0] = MX_CHAR_EMPTY;

  while (client.connected()) {
    if (!client.available()) {
      continue;
    }

    clientChar = client.read();

    if (clientCharPos < PAGE_CHAR_POS_MAX){
      clientCharPos++;      
    }

#ifdef SERIAL_EN
    Serial.print((char) clientChar);
#endif

    if (pageSwitch & PAGE_SWITCH_PATH){

      pageSwitch &= ~PAGE_SWITCH_PATH; 
     
      switch (clientChar){
        case 'l':
        case 'L':
          pageSwitch |= PAGE_SWITCH_L;
          break;
        case 'i':
        case 'I':
          pageSwitch |= PAGE_SWITCH_INTENSITY;
          page |= PAGE_DISPLAY_INTENSITY;
          break;
        case 'r':
        case 'R':
          pageSwitch |= PAGE_SWITCH_INTENSITY;
          page |= PAGE_RED;
          break;
        case 'g':
        case 'G':
          pageSwitch |= PAGE_SWITCH_INTENSITY;
          page |= PAGE_GREEN;
          break;
        case 'b':
        case 'B':
          pageSwitch |= PAGE_SWITCH_INTENSITY;
          page |= PAGE_BLUE;
          break;
        case 'w':
        case 'W':
          pageSwitch |= PAGE_SWITCH_INTENSITY;
          page |= PAGE_RED;
          page |= PAGE_GREEN;
          page |= PAGE_BLUE;
          break;
        case 's': // store program (color intensities)
        case 'S':
          pageSwitch |= PAGE_SWITCH_PROGRAM;
          page |= PAGE_STORE;
          break;
        case 'p': // get program (color intensities)
        case 'P':
          pageSwitch |= PAGE_SWITCH_PROGRAM;
          page |= PAGE_GET;
          break;
        default:
          serv |= SERV_NOT_FOUND;
          break;
      }
      continue;
    } 
    
    if (pageSwitch & PAGE_SWITCH_L){
      switch (clientChar){
        case '1':
          page |= PAGE_L1;
          break;
        case '2':
          page |= PAGE_L2;
          break;
        case '/':
          pageSwitch &= ~PAGE_SWITCH_L;
          pageSwitch |= PAGE_SWITCH_TEXT;
        default:
          pageSwitch &= ~PAGE_SWITCH_L;          
          serv |= SERV_NOT_FOUND;          
          break;
      }
      continue;
    } 
    
    if (pageSwitch & PAGE_SWITCH_TEXT){
      if (clientChar > 0x20 && clientChar <= 0xbf){
        strLen = strlen(mxChar);
        if (strLen < (sizeof(mxChar) - 1)){        
          mxChar[strLen] = clientChar;
          mxChar[strLen + 1] = MX_CHAR_EMPTY;
        }
      } else {
        #ifdef MX_DISPLAY_REVERSE
          if (page & PAGE_L1){
            mxCalcPixelMatrix(MX_SCREEN_DATA, MX_DISPLAY_L2);            
          } else {
            mxCalcPixelMatrix(MX_SCREEN_DATA, MX_DISPLAY_L1);
          }
        #else 
          if (page & PAGE_L1){
            mxCalcPixelMatrix(MX_SCREEN_DATA, MX_DISPLAY_L1);            
          } else {
            mxCalcPixelMatrix(MX_SCREEN_DATA, MX_DISPLAY_L2);
          }        
        #endif 

        serv = SERV_OK;
        pageSwitch &= ~PAGE_SWITCH_TEXT; 
        mxInterval = 0;
      }
      continue;
    } 

    if (pageSwitch & PAGE_SWITCH_INTENSITY){
      intensity = 0xff;
      switch (clientChar){
        case '0' ... '9':
          intensity = clientChar - '0';
          break;
        case 'a' ... 'f':
          intensity = clientChar - 'a' + 10;
          break;
        case 'A' ... 'F':
          intensity = clientChar - 'A' + 10;
          break;
        case '/':
          break;
        default:
          serv |= SERV_NOT_FOUND;
          pageSwitch &= ~PAGE_SWITCH_INTENSITY;
          break;
      }

      if (intensity != 0xff){
        pageSwitch &= ~PAGE_SWITCH_INTENSITY;          
        intensity = intensity << 4 | intensity;
        #ifdef SERIAL_EN
          Serial.print("intesity: ");
          Serial.println(intensity, HEX);
        #endif
        if (page & PAGE_RED){
          wsRed = intensity;
          wsInterval = 0;
        }
        if (page & PAGE_GREEN){
          wsGreen = intensity;
          wsInterval = 0;
        }
        if (page & PAGE_BLUE){
          wsBlue = intensity;
          wsInterval = 0;
        }
        if (page & PAGE_DISPLAY_INTENSITY){
          mxIntensity = intensity;
          mxInterval = 0;
        }
        serv |= SERV_OK;
      }
      continue;   
    }

    if (pageSwitch & PAGE_SWITCH_PROGRAM){
      switch (clientChar){
        case '0' ... '9':
          pageSwitch &= ~PAGE_SWITCH_PROGRAM;
          programId = clientChar - '0';
          putProgramIdToEeprom();
          if (page & PAGE_STORE){
            putProgramValuesToEeprom();
          }
          if (page & PAGE_GET){
            getProgramValuesFromEeprom();
            mxLastRequest = 0;
            wsLastRequest = 0;
          }
          serv |= SERV_OK;
          break;
        case '/':
          break;
        default:
          serv |= SERV_NOT_FOUND;
          pageSwitch &= ~PAGE_SWITCH_PROGRAM;
          break;
      }
      continue;
    }
    
    if (serv == SERV_UNDEFINED){

      if (clientCharPos < 5){
        if (clientChar == "_GET "[clientCharPos]){
          #ifdef SERIAL_EN
            Serial.print(">B5>>> ");
            Serial.println(clientChar);
          #endif
          continue;
        }
      } else if (clientChar == '/'){
          pageSwitch |= PAGE_SWITCH_PATH;
          continue;
      }
      #ifdef SERIAL_EN
        Serial.print(">BadR>>> ");
        Serial.println(clientChar);
      #endif
      serv = SERV_BAD_REQUEST;        
    }

    if (prevClientChar == '\n'){
      pageSwitch |= PAGE_SWITCH_BLANK_LINE;
    } else if (prevClientChar != '\r') {
      pageSwitch &= ~PAGE_SWITCH_BLANK_LINE;
    }

    prevClientChar = clientChar;

    if (clientChar != '\n'){
      continue;
    }    

    if (~pageSwitch & PAGE_SWITCH_BLANK_LINE){
      continue;
    }

    /**
     * Serve page to client
     */

    client.print("HTTP/1.1");
    client.print(" ");

    if (serv & SERV_SERVICE_UNAVAILABLE){
        client.print("503 ");
        client.print("Service ");
        client.print("Unavai");
        client.println("lable");
    } else if (serv & SERV_BAD_REQUEST) {  
        client.print("400 Bad ");
        client.println("Request");     
    } else if (serv & SERV_NOT_FOUND) {
        client.print("404 Not ");
        client.println("Found");
    } else if (serv & SERV_OK){
      client.println("200 OK");

    } else {
        client.print("500 Inter");
        client.print("nal Server ");
        client.println("Error");        
    }

    client.print("Content-");
    client.print("Type: ");
    client.print("text/");
    client.println("plain");
    client.println();

    if (serv & SERV_SERVICE_UNAVAILABLE){
      client.print("503 ");
      client.print("Service ");
      client.print("Unavai");
      client.print("lable");
      break;
    }

    if (serv & SERV_BAD_REQUEST){
      client.print("400 Bad ");
      client.print("Request");
      break;      
    }

    if (serv & SERV_NOT_FOUND){
      client.print("404 Not ");
      client.print("Found");
      break;      
    }

    if (serv & SERV_OK){
      client.print("Ok");
      break;
    }

    client.print("500 Inter");
    client.print("nal Server ");
    client.print("Error");
  }
  // time for client
  delay(1);
  client.stop();
}
