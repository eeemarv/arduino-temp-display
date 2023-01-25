#include <Arduino.h>

/*
* definitions that 
* can be changed
* in .env.h 
*/

#ifndef MAC_4_LAST
#define MAC_4_LAST 0xbe, 0xef, 0x13, 0x41
#endif
#ifndef SELF_IP
#define SELF_IP 192, 168, 0, 41
#endif
#ifndef ETH_CS_PIN 
#define ETH_CS_PIN 10
#endif
#ifndef MQTT_SERVER_IP
#define MQTT_SERVER_IP 192, 168, 0, 70
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#ifndef MQTT_CONNECT_RETRY_TIME
#define MQTT_CONNECT_RETRY_TIME 5000
#endif
#ifndef SERIAL_BAUD 
#define SERIAL_BAUD 115200
#endif

#ifndef CLIENT_ID_PREFIX
#define CLIENT_ID_PREFIX "d_"
#endif

#ifndef SUB_SET_INTENSITY
#define SUB_SET_INTENSITY "d/i"
#endif
#ifndef SUB_SET_RED
#define SUB_SET_RED "d/red"
#endif
#ifndef SUB_SET_GREEN
#define SUB_SET_GREEN "d/green"
#endif 
#ifndef SUB_SET_BLUE
#define SUB_SET_BLUE "d/blue"
#endif
#ifndef SUB_SELECT_PROGRAM
#define SUB_SELECT_PROGRAM "d/select"
#endif
#ifndef SUB_STORE
#define SUB_STORE "d/store"
#endif
#ifndef SUB_WATER_TEMP
#define SUB_WATER_TEMP "we/water_temp"
#endif
#ifndef SUB_AIR_TEMP
#define SUB_AIR_TEMP "we/air_temp"
#endif
#ifndef PUB_PRESENCE
#define PUB_PRESENCE "d/p"
#endif

#ifndef WATER_TEMP_VALID_TIME
#define WATER_TEMP_VALID_TIME 900000 // 15 min
#endif
#ifndef AIR_TEMP_VALID_TIME
#define AIR_TEMP_VALID_TIME 900000 // 15 min
#endif
#ifndef PRESENCE_TIME 
#define PRESENCE_TIME 2000
#endif
