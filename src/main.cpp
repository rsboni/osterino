#include <max6675.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "pressure_sensor.h"
#include <SimpleKalmanFilter.h>
#include "pindef.h"
#include "sensors_state.h"
#include "pump.h"
#include "peripherals.h"
#define ARDUINO_ARCH_ESP32 1

BLEServer *pServer = NULL;
bool isBrewing = false;
int state = 0;
String filterString;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

bool deviceConnected = false;
bool oldDeviceConnected = false;
int rec = 0;

void getTemp();
void toogleBrew()
{
  isBrewing = !isBrewing;
}
void BLETransfer(int16_t);

float temperature = -1000;
float TEMPDIFF = -3;

#define enviornmentService BLEUUID((uint16_t)0x1A02)

BLECharacteristic temperatureCharacteristic(
    BLEUUID((uint16_t)0x1A01),
    BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY);

BLECharacteristic pressureCharacteristic(
    BLEUUID((uint16_t)0x1A02),
    BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY);

BLECharacteristic brewingCharacteristic(
    BLEUUID((uint16_t)0x1A03),
    BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE);

BLECharacteristic targetPressureCharacteristic(
    BLEUUID((uint16_t)0x1A04),
    BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE);

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("Connected was called");
    deviceConnected = true;
    oldDeviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

void checkToReconnect()
{
  // disconnected so advertise
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Disconnected: start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connected so reset boolean control
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    Serial.println("Reconnected");
    oldDeviceConnected = deviceConnected;
  }
}

SimpleKalmanFilter smoothPressure(2.f, 2.f, 0.5f);
int pressureTimer = millis();
int GET_PRESSURE_READ_EVERY = 10;
int SEND_DATA_EVERY = 200;
int dataSendTimer = millis();
float previousSmoothedPressure = 0;
SensorState currentState;

static void sensorsReadPressure(void)
{
  uint32_t elapsedTime = millis() - pressureTimer;

  if (elapsedTime > GET_PRESSURE_READ_EVERY)
  {
    float elapsedTimeSec = elapsedTime / 1000.f;
    previousSmoothedPressure = currentState.smoothedPressure;
    currentState.pressure = getPressure();
    previousSmoothedPressure = currentState.smoothedPressure;
    currentState.smoothedPressure = smoothPressure.updateEstimate(currentState.pressure);
    currentState.pressureChangeSpeed = (currentState.smoothedPressure - previousSmoothedPressure) / elapsedTimeSec;

    pressureTimer = millis();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial Begin");

  Serial.println("Pressure Sensor init");
  pinInit();
  Serial.println("Temp Sensor init");
  adsInit();

  // pump;
  Serial.println("Pump init");
  pumpInit(60, 0.285f);
  closeValve();
  setPumpOff();

  BLEDevice::init("Minibar");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  // BLEService *pEnviornment = pServer->createService(enviornmentService);
  BLEService *pEnviornment = pServer->createService(
      enviornmentService,
      (uint32_t)30,
      (uint8_t)0);
  Serial.println("BLE Service Initialized");

  // Create a BLE Characteristic
  pEnviornment->addCharacteristic(&temperatureCharacteristic);
  pEnviornment->addCharacteristic(&pressureCharacteristic);
  pEnviornment->addCharacteristic(&brewingCharacteristic);
  pEnviornment->addCharacteristic(&targetPressureCharacteristic);

  // Create a BLE Descriptor
  temperatureCharacteristic.addDescriptor(new BLE2902());
  pressureCharacteristic.addDescriptor(new BLE2902());
  brewingCharacteristic.addDescriptor(new BLE2902());
  targetPressureCharacteristic.addDescriptor(new BLE2902());

  BLEDescriptor TemperatureDescriptor(BLEUUID((uint16_t)0x2901));
  TemperatureDescriptor.setValue("Temperature -40-60°C");
  temperatureCharacteristic.addDescriptor(&TemperatureDescriptor);

  BLEDescriptor PressureDescriptor(BLEUUID((uint16_t)0x2901));
  PressureDescriptor.setValue("Pressure in bar");
  pressureCharacteristic.addDescriptor(&PressureDescriptor);

  BLEDescriptor BrewingDescriptor(BLEUUID((uint16_t)0x2901));
  BrewingDescriptor.setValue("If is brewing or not");
  brewingCharacteristic.addDescriptor(&BrewingDescriptor);

  BLEDescriptor TargetPressureDescriptor(BLEUUID((uint16_t)0x2901));
  TargetPressureDescriptor.setValue("Pressure we want to reach");
  targetPressureCharacteristic.addDescriptor(&TargetPressureDescriptor);

  pServer->getAdvertising()->addServiceUUID(enviornmentService);

  // Start the service
  pEnviornment->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

float getTargetPressure(void)
{
  uint8_t *received_data = targetPressureCharacteristic.getData();
  return received_data[0] / 10;
}

void manualFlowControl(void)
{
  if (currentState.isBrewing)
  {
    openValve();
    float flow_reading = getTargetPressure();
    if (flow_reading != currentState.targetPressure)
    {
      currentState.targetPressure = flow_reading;
    }
    setPumpPressure(flow_reading, 0.f, currentState);
  }
}

void loop()
{
  sensorsReadPressure();
  checkToReconnect();
  getTemp();
  getAndResetClickCounter();
  String beaconMsg;
  if (temperature == -1000)
  {
    if (rec > 5)
    {
      beaconMsg = "ESPSNA";
      Serial.println(beaconMsg);
      temperature = 100000;
    }
    return;
  }

  if (deviceConnected)
  {
    uint32_t elapsedTime = millis() - dataSendTimer;

    if (elapsedTime > SEND_DATA_EVERY){

      int value;
      value = (int)(temperature * 100);
      char buffer[20];
      dtostrf(value, 1, 0, buffer);
      temperatureCharacteristic.setValue((char *)&buffer);
      temperatureCharacteristic.notify();

      char buffer1[20];
      dtostrf(currentState.smoothedPressure * 100, 1, 0, buffer1);
      pressureCharacteristic.setValue((char *)&buffer1);
      pressureCharacteristic.notify();
      dataSendTimer = millis();
    }

    uint8_t *received_data = brewingCharacteristic.getData();
    if (received_data[0] == 1 && !currentState.isBrewing)
    {
      currentState.isBrewing = true;
      Serial.println("Started brew");
    }
    if (received_data[0] == 0 && currentState.isBrewing)
    {
      currentState.isBrewing = false;

      Serial.println("Stopped brew");
      setPumpOff();
      closeValve();
    }
    manualFlowControl();
  }
}

// void justDoCoffee(SensorState &currentState, bool brewActive) {
//   int HPWR_LOW = 550 / 5; // hpw/divider
//   static double heaterWave;
//   static bool heaterState;
//   float BREW_TEMP_DELTA;
// Calculating the boiler heating power range based on the below input values
// int HPWR_OUT = mapRange(currentState.temperature, runningCfg.setpoint - 10, runningCfg.setpoint, runningCfg.hpwr, HPWR_LOW, 0);
// HPWR_OUT = constrain(HPWR_OUT, HPWR_LOW, runningCfg.hpwr);  // limits range of sensor values to HPWR_LOW and HPWR
// BREW_TEMP_DELTA = mapRange(currentState.temperature, runningCfg.setpoint, runningCfg.setpoint + TEMP_DELTA(runningCfg.setpoint), TEMP_DELTA(runningCfg.setpoint), 0, 0);
// BREW_TEMP_DELTA = constrain(BREW_TEMP_DELTA, 0, TEMP_DELTA(runningCfg.setpoint));
// lcdTargetState(0); // setting the target mode to "brew temp"

// if (brewActive) {
// Applying the HPWR_OUT variable as part of the relay switching logic
// if (currentState.temperature > runningCfg.setpoint && currentState.temperature < runningCfg.setpoint + 0.25f && !preinfusionFinished ) {
//   if (millis() - heaterWave > HPWR_OUT * runningCfg.brewDivider && !heaterState ) {
//     setBoilerOff();
//     heaterState=true;
//     heaterWave=millis();
//   } else if (millis() - heaterWave > HPWR_LOW * runningCfg.mainDivider && heaterState ) {
//     setBoilerOn();
//     heaterState=false;
//     heaterWave=millis();
//   }
// } else if (currentState.temperature > runningCfg.setpoint - 1.5f && currentState.temperature < runningCfg.setpoint + (runningCfg.brewDeltaState ? BREW_TEMP_DELTA : 0.f) && preinfusionFinished ) {
//   if (millis() - heaterWave > runningCfg.hpwr * runningCfg.brewDivider && !heaterState ) {
//     setBoilerOn();
//     heaterState=true;
//     heaterWave=millis();
//   } else if (millis() - heaterWave > runningCfg.hpwr && heaterState ) {
//     setBoilerOff();
//     heaterState=false;
//     heaterWave=millis();
//   }
// } else if (runningCfg.brewDeltaState && currentState.temperature >= (runningCfg.setpoint + BREW_TEMP_DELTA) && currentState.temperature <= (runningCfg.setpoint + BREW_TEMP_DELTA + 2.5f)  && preinfusionFinished ) {
//   if (millis() - heaterWave > runningCfg.hpwr * runningCfg.mainDivider && !heaterState ) {
//     setBoilerOn();
//     heaterState=true;
//     heaterWave=millis();
//   } else if (millis() - heaterWave > runningCfg.hpwr && heaterState ) {
//     setBoilerOff();
//     heaterState=false;
//     heaterWave=millis();
//   }
// } else if(currentState.temperature <= runningCfg.setpoint - 1.5f) {
//   setBoilerOn();
// } else {
//   setBoilerOff();
// }
// // } else { //if brewState == 0
// //   if (currentState.temperature < ((float)runningCfg.setpoint - 10.00f)) {
// //     setBoilerOn();
// //   } else if (currentState.temperature >= ((float)runningCfg.setpoint - 10.f) && currentState.temperature < ((float)runningCfg.setpoint - 5.f)) {
// //     if (millis() - heaterWave > HPWR_OUT && !heaterState) {
// //       setBoilerOn();
// //       heaterState=true;
// //       heaterWave=millis();
// //     } else if (millis() - heaterWave > HPWR_OUT / runningCfg.brewDivider && heaterState ) {
// //       setBoilerOff();
// //       heaterState=false;
// //       heaterWave=millis();
// //     }
// //   } else if ((currentState.temperature >= ((float)runningCfg.setpoint - 5.f)) && (currentState.temperature <= ((float)runningCfg.setpoint - 0.25f))) {
// //     if (millis() - heaterWave > HPWR_OUT * runningCfg.brewDivider && !heaterState) {
// //       setBoilerOn();
// //       heaterState=!heaterState;
// //       heaterWave=millis();
// //     } else if (millis() - heaterWave > HPWR_OUT / runningCfg.brewDivider && heaterState ) {
// //       setBoilerOff();
// //       heaterState=!heaterState;
// //       heaterWave=millis();
// //     }
// //   } else {
// //     setBoilerOff();
// //   }
// }
// }

void getTemp()
{
  temperature = thermocouple.readCelsius() + TEMPDIFF;
}

// static void fillBoiler(float targetBoilerFullPressure) {
//   static long elapsedTimeSinceStart = millis();
//   lcdSetUpTime((millis() > elapsedTimeSinceStart) ? (int)((millis() - elapsedTimeSinceStart) / 1000) : 0);
//   if (!startupInitFinished && lcdCurrentPageId == 0 && millis() - elapsedTimeSinceStart >= 3000) {
//     unsigned long timePassed = millis() - elapsedTimeSinceStart;

//     if (currentState.smoothedPressure < targetBoilerFullPressure && timePassed <= BOILER_FILL_TIMEOUT) {
//       // lcdShowPopup("Filling boiler!");
//       openValve();
//       setPumpToRawValue(80);
//     } else if (!startupInitFinished) {
//       closeValve();
//       setPumpToRawValue(0);
//       startupInitFinished = true;
//     }
//   }
// }