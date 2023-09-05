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
#include "scales.h"
#include "utils.h"

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
SimpleKalmanFilter smoothPressure(2.f, 2.f, 0.5f);
SimpleKalmanFilter smoothScalesFlow(2.f, 2.f, 0.5f);

SensorState currentState;
BLEServer *pServer = NULL;
BLECharacteristic *pPressureSensorBLEChar = NULL;
BLECharacteristic *pWeightBLEChar = NULL;
BLECharacteristic *pTargetPressureBLEChar = NULL;
BLECharacteristic *pBrewingBLEChar = NULL;
BLECharacteristic *pTemperatureBLEChar = NULL;
BLECharacteristic *pFlowBLEChar = NULL;
BLECharacteristic *pTargetWeightBLEChar = NULL;
BLECharacteristic *pTargetTemperatureBLEChar = NULL;

bool tareDone = false;
bool deviceConnected = false;
bool oldDeviceConnected = false;

int pressureTimer = millis();
int weightTimer = millis();
int tempTimer = millis();
int dataSendTimer = millis();
int GET_PRESSURE_READ_EVERY = 100;
int GET_WEIGHT_READ_EVERY = 200;
int GET_TEMP_READ_EVERY = 250;
int SEND_DATA_EVERY = 250;

float lastTemp;
float lastPressure;
float lastWeight;
float lastFlow = 0;
float SCALE_CALIBRATION = 710.73015873f;
float temperature = -1000;
float TEMPDIFF = -3;
float previousSmoothedPressure = 0;

// Callbacks Setup
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("Connected was called");
    deviceConnected = true;
    oldDeviceConnected = true;
    delay(500);
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

void stopBrew(void)
{
  currentState.isBrewing = false;
  tareDone = false;
  Serial.println("Stopped brew");
  setPumpOff();
  closeValve();
  currentState.targetWeight = 0;
  currentState.targetPressure = 0;
}


class BrewBLECharCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    // std::string value = pCharacteristic->getValue();
    uint8_t *value = pCharacteristic->getData();
    int intValue = value[0];
    Serial.println("Recieved brew char= " + String(intValue));

    if (intValue == 1 && !currentState.isBrewing)
    {
      scalesTare();
      delay(500);
      currentState.isBrewing = true;
      Serial.println("Started brew");
      tareDone = true;
    }
    if (intValue == 0 && currentState.isBrewing)
    {
      stopBrew();
    }
  }
};

class targetWeightBLECharCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    uint8_t *value = pCharacteristic->getData();
    int intValue = value[0];
    // fValue = fValue / 10;
    Serial.println("Received BT Target Weight= " + String(intValue));

    currentState.targetWeight = intValue;
  }
};

class TargetPressureBLECharCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    uint8_t *value = pCharacteristic->getData();

    float fValue = value[0];
    fValue = fValue / 10;
    Serial.println("Received BT Target Pressure= " + String(fValue));

    currentState.targetPressure = fValue;
  }
};

class TargetTemperatureBLECharCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    uint8_t *value = pCharacteristic->getData();

    float fValue = value[0];
    fValue = fValue / 10;
    Serial.println("Received BT Target Temperature= " + String(fValue));

    currentState.targetTemperature = fValue;
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
    Serial.println("Reconnected");
    oldDeviceConnected = deviceConnected;
  }
}

static void sensorsReadPressure(void)
{
  uint32_t elapsedTime = millis() - pressureTimer;

  if (elapsedTime > GET_PRESSURE_READ_EVERY)
  {
    float elapsedTimeSec = elapsedTime / 1000.f;
    previousSmoothedPressure = currentState.smoothedPressure;
    currentState.pressure = fmaxf(0.f, getPressure());
    previousSmoothedPressure = currentState.smoothedPressure;
    currentState.smoothedPressure = smoothPressure.updateEstimate(currentState.pressure);
    currentState.pressureChangeSpeed = (currentState.smoothedPressure - previousSmoothedPressure) / elapsedTimeSec;

    pressureTimer = millis();
  }
}

void manualFlowControl(void)
{
  if (currentState.isBrewing)
  {
    if (!tareDone)
      scalesTare();
    openValve();
    // float flow_reading = getTargetPressure();
    // if (flow_reading != currentState.targetPressure)
    // {
    //   currentState.targetPressure = flow_reading;
    // }
    setPumpPressure(currentState.targetPressure, 0.f, currentState);
    if (currentState.targetWeight > 0 && currentState.weight + (currentState.weightFlow / 9) >= currentState.targetWeight)
    {
      Serial.println("Stopped at: " + String(currentState.weight) + " target was: " + String(currentState.targetWeight));
      stopBrew();

      char brew[20];
      dtostrf(0.f, 1, 0, brew);
      pBrewingBLEChar->setValue((char *)&brew);
      pBrewingBLEChar->notify();
      pTargetWeightBLEChar->setValue((char *)&brew);
      pTargetWeightBLEChar->notify();
    }
  }
}

static void getWeight(void)
{
  uint32_t elapsedTime = millis() - weightTimer;

  if (elapsedTime > GET_WEIGHT_READ_EVERY)
  {
    float elapsedTimeSec = elapsedTime / 1000.f;
    float previousWeight = currentState.weight;
    currentState.weight = scalesGetWeight();
    currentState.weightFlow = fmaxf(0.f, (currentState.weight - previousWeight) / elapsedTimeSec);
    currentState.weightFlow = smoothScalesFlow.updateEstimate(currentState.weightFlow);
    weightTimer = millis();
  }
}

static void getTemp(void)
{
  uint32_t elapsedTime = millis() - tempTimer;

  if (elapsedTime > GET_TEMP_READ_EVERY)
  {
    temperature = thermocouple.readCelsius() + TEMPDIFF;
    currentState.temperature = temperature;
    tempTimer = millis();
  }
}

inline static float TEMP_DELTA(float d) { return (d*0.25f); }
int hpwr = 550;
int brewDivider = 2;
bool brewDeltaState = true;
int mainDivider = 5;
void justDoCoffee(SensorState &currentState, bool brewActive)
{
  int HPWR_LOW = 550 / 5; // hpw/divider
  static double heaterWave;
  static bool heaterState;
  float BREW_TEMP_DELTA;
  bool preinfusionFinished = brewActive && currentState.targetPressure > 3.0f;
  // Calculating the boiler heating power range based on the below input values
  int HPWR_OUT = mapRange(currentState.temperature, currentState.targetTemperature - 10, currentState.targetTemperature, hpwr, HPWR_LOW, 0);
  HPWR_OUT = constrain(HPWR_OUT, HPWR_LOW, hpwr); // limits range of sensor values to HPWR_LOW and HPWR
  BREW_TEMP_DELTA = mapRange(currentState.temperature, currentState.targetTemperature, currentState.targetTemperature + TEMP_DELTA(currentState.targetTemperature), TEMP_DELTA(currentState.targetTemperature), 0, 0);
  BREW_TEMP_DELTA = constrain(BREW_TEMP_DELTA, 0, TEMP_DELTA(currentState.targetTemperature));
  // lcdTargetState(0); // setting the target mode to "brew temp"

  if (brewActive)
  {
    // Applying the HPWR_OUT variable as part of the relay switching logic
    if (currentState.temperature > currentState.targetTemperature && currentState.temperature < currentState.targetTemperature + 0.25f && !preinfusionFinished)
    {
      if (millis() - heaterWave > HPWR_OUT * brewDivider && !heaterState)
      {
        setBoilerOff();
        heaterState = true;
        heaterWave = millis();
      }
      else if (millis() - heaterWave > HPWR_LOW * mainDivider && heaterState)
      {
        setBoilerOn();
        heaterState = false;
        heaterWave = millis();
      }
    }
    else if (currentState.temperature > currentState.targetTemperature - 1.5f && currentState.temperature < currentState.targetTemperature + (brewDeltaState ? BREW_TEMP_DELTA : 0.f) && preinfusionFinished)
    {
      if (millis() - heaterWave > hpwr * brewDivider && !heaterState)
      {
        setBoilerOn();
        heaterState = true;
        heaterWave = millis();
      }
      else if (millis() - heaterWave > hpwr && heaterState)
      {
        setBoilerOff();
        heaterState = false;
        heaterWave = millis();
      }
    }
    else if (brewDeltaState && currentState.temperature >= (currentState.targetTemperature + BREW_TEMP_DELTA) && currentState.temperature <= (currentState.targetTemperature + BREW_TEMP_DELTA + 2.5f) && preinfusionFinished)
    {
      if (millis() - heaterWave > hpwr * mainDivider && !heaterState)
      {
        setBoilerOn();
        heaterState = true;
        heaterWave = millis();
      }
      else if (millis() - heaterWave > hpwr && heaterState)
      {
        setBoilerOff();
        heaterState = false;
        heaterWave = millis();
      }
    }
    else if (currentState.temperature <= currentState.targetTemperature - 1.5f)
    {
      setBoilerOn();
    }
    else
    {
      setBoilerOff();
    }
  }
  else
  { // if brewState == 0
    if (currentState.temperature < ((float)currentState.targetTemperature - 10.00f))
    {
      setBoilerOn();
    }
    else if (currentState.temperature >= ((float)currentState.targetTemperature - 10.f) && currentState.temperature < ((float)currentState.targetTemperature - 5.f))
    {
      if (millis() - heaterWave > HPWR_OUT && !heaterState)
      {
        setBoilerOn();
        heaterState = true;
        heaterWave = millis();
      }
      else if (millis() - heaterWave > HPWR_OUT / brewDivider && heaterState)
      {
        setBoilerOff();
        heaterState = false;
        heaterWave = millis();
      }
    }
    else if ((currentState.temperature >= ((float)currentState.targetTemperature - 5.f)) && (currentState.temperature <= ((float)currentState.targetTemperature - 0.25f)))
    {
      if (millis() - heaterWave > HPWR_OUT * brewDivider && !heaterState)
      {
        setBoilerOn();
        heaterState = !heaterState;
        heaterWave = millis();
      }
      else if (millis() - heaterWave > HPWR_OUT / brewDivider && heaterState)
      {
        setBoilerOff();
        heaterState = !heaterState;
        heaterWave = millis();
      }
    }
    else
    {
      setBoilerOff();
    }
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

  Serial.println("Scale init");
  scalesInit(SCALE_CALIBRATION);

  Serial.println("Pump init");
  pumpInit(60, 0.285f);

  closeValve();
  setPumpOff();

  BLEDevice::init("Minibar");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  // BLEService *pService = pServer->createService(enviornmentService);
  BLEService *pService = pServer->createService(
      BLEUUID(SERVICE_UUID),
      (uint32_t)30,
      (uint8_t)0);
  Serial.println("BLE Service Initialized");

  pPressureSensorBLEChar = pService->createCharacteristic(
      PRESSURE_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("BLE Pressure Sensor Characteristic Created");
  pPressureSensorBLEChar->addDescriptor(new BLE2902());

  pFlowBLEChar = pService->createCharacteristic(
      FLOW_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("BLE Flow Characteristic Created");
  pFlowBLEChar->addDescriptor(new BLE2902());

  pTemperatureBLEChar = pService->createCharacteristic(
      TEMPERATURE_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("BLE Temperature Characteristic Created");
  pTemperatureBLEChar->addDescriptor(new BLE2902());

  pWeightBLEChar = pService->createCharacteristic(
      WEIGHT_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("BLE Weight Characteristic Created");
  pWeightBLEChar->addDescriptor(new BLE2902());

  pTargetPressureBLEChar = pService->createCharacteristic(
      TARGET_PRESSURE_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);
  // BLECharacteristic::PROPERTY_INDICATE);
  Serial.println("BLE Pressure Target Characteristic Created");
  pTargetPressureBLEChar->addDescriptor(new BLE2902());

  pTargetWeightBLEChar = pService->createCharacteristic(
      TARGET_WEIGHT_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);
  // BLECharacteristic::PROPERTY_INDICATE);
  Serial.println("BLE Pressure Target Weight Created");
  pTargetWeightBLEChar->addDescriptor(new BLE2902());

  pTargetTemperatureBLEChar = pService->createCharacteristic(
      TARGET_TEMPERATURE_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);
  // BLECharacteristic::PROPERTY_INDICATE);
  Serial.println("BLE Pressure Target Temperature Created");
  pTargetTemperatureBLEChar->addDescriptor(new BLE2902());

  pBrewingBLEChar = pService->createCharacteristic(
      BREW_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);
  // BLECharacteristic::PROPERTY_INDICATE);
  Serial.println("BLE Brewing Characteristic Created");
  pBrewingBLEChar->addDescriptor(new BLE2902());

  pBrewingBLEChar->setCallbacks(new BrewBLECharCallbacks());
  pTargetPressureBLEChar->setCallbacks(new TargetPressureBLECharCallbacks());
  pTargetWeightBLEChar->setCallbacks(new targetWeightBLECharCallbacks());
  pTargetTemperatureBLEChar->setCallbacks(new TargetTemperatureBLECharCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x12);

  // Start advertising
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop()
{
  sensorsReadPressure();
  checkToReconnect();
  getWeight();
  getTemp();
  // getAndResetClickCounter();

  if (deviceConnected)
  {
    uint32_t elapsedTime = millis() - dataSendTimer;
    if (elapsedTime > SEND_DATA_EVERY)
    {
      if (currentState.temperature != lastTemp)
      {
        char TempBuffer[20];
        dtostrf(currentState.temperature * 100, 1, 0, TempBuffer);
        pTemperatureBLEChar->setValue((char *)&TempBuffer);
        pTemperatureBLEChar->notify();
        lastTemp = currentState.temperature;
      }

      if (currentState.smoothedPressure != lastPressure)
      {
        char pressureBuffer[20];
        dtostrf(currentState.smoothedPressure * 100, 1, 0, pressureBuffer);
        pPressureSensorBLEChar->setValue((char *)&pressureBuffer);
        pPressureSensorBLEChar->notify();
        lastPressure = currentState.smoothedPressure;
      }
      if (currentState.weight != lastWeight)
      {
        char buffer2[20];
        dtostrf(currentState.weight * 100, 1, 0, buffer2);
        pWeightBLEChar->setValue((char *)&buffer2);
        pWeightBLEChar->notify();
        lastWeight = currentState.weight;
      }
      if (currentState.weightFlow != lastFlow)
      {
        char buffer3[20];
        dtostrf(currentState.weightFlow * 100, 1, 0, buffer3);
        pFlowBLEChar->setValue((char *)&buffer3);
        pFlowBLEChar->notify();
        lastFlow = currentState.weightFlow;
      }
      if (currentState.isBrewing)
      {
        Serial.println("Pressure: " + String(currentState.pressure) + "Weight: " + String(currentState.weight) + "TargetPressure: " + String(currentState.targetPressure));
      }

      dataSendTimer = millis();
    }
    // uint8_t *received_data = brewingCharacteristic.getData();

    manualFlowControl();
    justDoCoffee(currentState, currentState.isBrewing);
  }
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