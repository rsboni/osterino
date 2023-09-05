#ifndef PINDEF_H
#define PINDEF_H

#define SERVICE_UUID "381af1eb-b002-4a8e-b698-458841444945"
#define PRESSURE_UUID "07a62719-c9c0-442f-af6c-336e8839469c" 
#define TARGET_PRESSURE_UUID "202c8717-9005-4eb3-876a-70f977a89c72" 
#define TEMPERATURE_UUID "22cf5e58-b119-4da5-8341-56cc2378f406" 
#define WEIGHT_UUID "8fe6deb9-02f5-4dbd-9bec-1b7291a9ba5a" 
#define BREW_UUID "0ddcee2d-4a38-46a3-9054-04691f5a7e26" 
#define FLOW_UUID "3a19025c-0dc3-492c-ae05-db00dfad91cd"
#define TARGET_WEIGHT_UUID "9414b5a6-fcb2-492d-8023-e34348fa7870"
#define TARGET_TEMPERATURE_UUID "fa62b5ea-6b4e-477e-ba05-7cefbe3d63b5"

// ESP32 pins definitions
#define thermoDO      19 //ok
#define thermoCS      23 //ok
#define thermoCLK     5  //ok

#define zcPin         26 //ok
#define brewPin       2  // to brew switch
#define relayPin      14 //ok
#define dimmerPin     27 //ok
#define steamPin      6
#define valvePin      17 //changed

#define HX711_sck_1   4
#define HX711_sck_2   8
#define HX711_dout_1  16
#define HX711_dout_2  10

// #define USART_LCD     Serial2 // PA2 & PA3
// #define USART_DEBUG   Serial  // USB-CDC

#define hw_SCL        11
#define hw_SDA        12

#endif
