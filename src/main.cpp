#include <Arduino.h>
#include <.env.h>
#include <env_default.h>
#include <SPI.h>
#include <EEPROM.h>
#include <EthernetENC.h>
#include <PubSubClient.h>
#include <Watchdog.h>
#include <zfont5x8.cpp>

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
#define MX_SPI_SPEED_MAX 4000000

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
#define BTN_DEBOUNCE_COUNTDOWN_START 4

#define PROGRAM_ID_INIT 0
#define PROGRAM_ID_MAX 10
#define PROGRAM_ID_EEPROM 0x08
#define PROGRAM_VALUES_EEPROM 0x10
#define PROGRAM_SIZE 4

#define CYCLES_MICROSEC (F_CPU / 1000000)

// See https://stackoverflow.com/a/63468969
template< unsigned N > 
inline static void nops(){
  asm ("nop");
  nops< N - 1 >();
}
template<> inline void nops<0>(){};

const byte mac[] = {0xDE, 0xAD, MAC_4_LAST};
const IPAddress selfIp(SELF_IP);
const IPAddress mqttServerIp(MQTT_SERVER_IP);

EthernetClient eth;
PubSubClient mqttClient(eth);
uint32_t mqttLastReconnectAttempt = 0;
uint32_t lastPresence = 0;
uint32_t lastWaterTemp = 0;
uint32_t lastAirTemp = 0;

Watchdog watchdog;

uint8_t wsRed = WS_INIT_RED;
uint8_t wsGreen = WS_INIT_GREEN;
uint8_t wsBlue = WS_INIT_BLUE;

uint8_t mxIntensity = MX_DEFAULT_INTENSITY;
uint8_t programId = PROGRAM_ID_INIT;

uint32_t wsLastRequest = 0;
uint32_t wsInterval = 0;

uint32_t mxLastRequest = 0;
uint32_t mxInterval = 0;

uint8_t mxMatrix[MX_SCREEN_COUNT][MX_DISPLAY_COUNT][MX_DEVICE_COUNT][MX_ROW_COUNT];
char mxChar[MX_CHAR_COUNT];
uint8_t mxScreen = MX_SCREEN_DATA;

uint8_t btnCount = BTN_DEBOUNCE_COUNTDOWN_START;
uint8_t btnLastRead = BTN_BITMASK;

const uint8_t* adr;

inline void mxCharInit(){
  mxChar[0] = '.';
  mxChar[1] = '\0';
}

inline void mxCharEmpty(){
  mxChar[0] = '.';
  mxChar[1] = '.';
  mxChar[2] = '\0';
}

inline void mxCharNoConnection(){
  mxChar[0] = '.';
  mxChar[1] = '.';
  mxChar[2] = '.';
  mxChar[3] = '\0';
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

inline void mxCalcProgramPixelMatrix(uint8_t display){
  mxChar[0] = '.';
  mxChar[1] = '.';
  mxChar[2] = '.';
  if (display == MX_DISPLAY_L2){
    mxChar[3] = programId + '0';    
  } else {
    mxChar[3] = '.';
  }
  mxChar[4] = '\0';

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
  uint8_t btnNewRead;

  if (btnCount){
    btnCount--;
    return;
  }

  btnNewRead = BTN_READ;

  if (!(btnNewRead^btnLastRead)){
    // no difference
    return;
  }

  btnCount = BTN_DEBOUNCE_COUNTDOWN_START;
  btnLastRead = btnNewRead;
  
  if (btnNewRead){
    // btn release
    // resume normal screen data
    mxScreen = MX_SCREEN_DATA;
    mxInterval = 0;
    return;
  } 

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

bool mqttReconnect() {
  uint8_t iii;
  uint8_t charPos;
  char clientId[10] = CLIENT_ID_PREFIX;
  for (iii = 0; iii < 4; iii++){
    charPos = strlen(clientId);
    clientId[charPos] = '0' + random(0, 10);
    clientId[charPos + 1] = '\0';
  }
  if (mqttClient.connect(clientId)) {
    mqttClient.subscribe(SUB_SET_INTENSITY);
    mqttClient.subscribe(SUB_SET_RED);
    mqttClient.subscribe(SUB_SET_GREEN);
    mqttClient.subscribe(SUB_SET_BLUE);
    mqttClient.subscribe(SUB_SELECT_PROGRAM);
    mqttClient.subscribe(SUB_STORE);
    mqttClient.subscribe(SUB_WATER_TEMP);
    mqttClient.subscribe(SUB_AIR_TEMP);
  }
  return mqttClient.connected();
}

void formatTemp(char* temp, uint8_t len){
  char msg[8];
  uint8_t iii;
  strncpy(msg, temp, len);
  msg[len] = '\0';
  dtostrf(atof(msg), 3, 1, mxChar);
  // does not work with floats in Arduino
  // snprintf(mxChar, sizeof(mxChar), "%.1f", fl);
  if (strcmp(mxChar, "-0.0") == 0){
    strcpy(mxChar, "0.0");
  }
  for (iii = 0; iii < strlen(mxChar); iii++){
    if (mxChar[iii] == '.'){
      mxChar[iii] = ',';
    }
  }
}

inline void mxCalcDataLine1(){
  #ifdef MX_DISPLAY_REVERSE
    mxCalcPixelMatrix(MX_SCREEN_DATA, MX_DISPLAY_L2);            
  #else 
    mxCalcPixelMatrix(MX_SCREEN_DATA, MX_DISPLAY_L1);                
  #endif 
}

inline void mxCalcDataLine2(){
  #ifdef MX_DISPLAY_REVERSE
    mxCalcPixelMatrix(MX_SCREEN_DATA, MX_DISPLAY_L1);            
  #else 
    mxCalcPixelMatrix(MX_SCREEN_DATA, MX_DISPLAY_L2);                
  #endif 
}

void mqttCallback(char* topic, char* payload, unsigned int length) {
  uint8_t iii;

  #ifdef SERIAL_EN
    Serial.print("sub rec: ");
    Serial.print(topic);
    Serial.print(" : ");
    for (iii = 0; iii < length; iii++) {
      Serial.print(payload[iii]);
    }
  #endif

  if (strcmp(topic, SUB_WATER_TEMP) == 0){
    formatTemp(payload, length);
    mxCalcDataLine1();
    mxInterval = 0;
    lastWaterTemp = millis();
    return;
  }

  if (strcmp(topic, SUB_AIR_TEMP) == 0){
    formatTemp(payload, length);
    mxCalcDataLine2();
    mxInterval = 0;  
    lastAirTemp = millis();
    return;
  }

  if (strcmp(topic, SUB_SET_INTENSITY) == 0){
    mxIntensity = atoi(payload);
    mxInterval = 0;
    return;
  }

  if (strcmp(topic, SUB_SET_RED) == 0){
    wsRed = atoi(payload);
    wsInterval = 0;
    return;
  }

  if (strcmp(topic, SUB_SET_GREEN) == 0){
    wsGreen = atoi(payload);
    wsInterval = 0;
    return;
  }

  if (strcmp(topic, SUB_SET_BLUE) == 0){
    wsBlue = atoi(payload);
    wsInterval = 0;
    return;
  }

  if (strcmp(topic, SUB_SELECT_PROGRAM) == 0){
    if (payload[0] < '0'){
      return;
    }
    if (payload[0] > '9'){
      return;
    }
    programId = (uint8_t) (payload[0] - '0');
    putProgramIdToEeprom();
    getProgramValuesFromEeprom();
    wsInterval = 0;
    mxInterval = 0;
    return;
  }

  if (strcmp(topic, SUB_STORE) == 0){
    if (payload[0] < '0'){
      return;
    }
    if (payload[0] > '9'){
      return;
    }
    programId = (uint8_t) (payload[0] - '0');
    putProgramIdToEeprom();    
    putProgramValuesToEeprom();
    wsInterval = 0;
    mxInterval = 0;
    return;
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

  mqttClient.setServer(mqttServerIp, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  Ethernet.init(ETH_CS_PIN);
  SPI.begin();
  Ethernet.begin(mac, selfIp);

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

  mqttReconnect();

  mxCharInit();
  mxCalcDataLine1();
  mxCalcDataLine2();

  getProgramIdFromEeprom();
  getProgramValuesFromEeprom();
  mxCalcProgramPixelMatrix(MX_DISPLAY_L1);
  mxCalcProgramPixelMatrix(MX_DISPLAY_L2);

  delay(200);

  wsLastRequest = millis();
  mxLastRequest = millis();
  wsInterval = 0;
  mxInterval = 0;

  #ifdef WATCHDOG_EN
    watchdog.enable(Watchdog::TIMEOUT_4S);
  #endif
}

void loop() {
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

  Ethernet.maintain();

  btnRead();

  if (mqttClient.connected()) {

    if (millis() - lastPresence > PRESENCE_TIME){
      mqttClient.publish(PUB_PRESENCE, "1");
      lastPresence = millis();
    }

    btnRead();

    if (millis() - lastWaterTemp > WATER_TEMP_VALID_TIME){
      mxCharEmpty();
      mxCalcDataLine1();
      mxInterval = 0;
    }

    btnRead();

    if (millis() - lastAirTemp > AIR_TEMP_VALID_TIME){
      mxCharEmpty();
      mxCalcDataLine2();
      mxInterval = 0;
    }

    btnRead();

    mqttClient.loop();
  } else {

    mxCharNoConnection();
    mxCalcDataLine1();
    mxCalcDataLine2();
    mxInterval = 0;

    if (millis() - mqttLastReconnectAttempt > MQTT_CONNECT_RETRY_TIME) {
      if (mqttReconnect()) {
        #ifdef SERIAL_EN
          Serial.println("connected");
        #endif
        mqttLastReconnectAttempt = 0;
      } else {

        #ifdef SERIAL_EN
          Serial.print("failed, rc=");
          Serial.print(mqttClient.state());
          Serial.print(" try again in ");
          Serial.print(MQTT_CONNECT_RETRY_TIME);
          Serial.println(" ms.");
        #endif
        mqttLastReconnectAttempt = millis();
      }
    }
  }
}
