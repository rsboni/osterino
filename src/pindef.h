#ifndef PINDEF_H
#define PINDEF_H


// ESP32 pins definitions
#define thermoDO      19 //ok
#define thermoCS      23 //ok
#define thermoCLK     5  //ok

#define zcPin         26 //ok
#define brewPin       2  // to brew switch
#define relayPin      3
#define dimmerPin     27 //ok
#define steamPin      6
#define valvePin      17 //changed

#define HX711_sck_1   7
#define HX711_sck_2   8
#define HX711_dout_1  9
#define HX711_dout_2  10

// #define USART_LCD     Serial2 // PA2 & PA3
// #define USART_DEBUG   Serial  // USB-CDC

#define hw_SCL        11
#define hw_SDA        12

#endif
