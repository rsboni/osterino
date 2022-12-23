
#include <max6675.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "pressure_sensor.h"
#include "sensorState.h"
#include <SimpleKalmanFilter.h>

BLEServer *pServer = NULL;
bool isBrewing = false;
int thermoDO = 19;
int thermoCS = 23;
int thermoCLK = 5;
int RELAY_PIN = 2;
int state = 0;
String filterString;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// BLECharacteristic *pCharacteristic;

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
// BLEDescriptor tempDescriptor(BLEUUID((uint16_t)0x2901));

BLECharacteristic pressureCharacteristic(
    BLEUUID((uint16_t)0x1A02),
    BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY);

BLECharacteristic brewingCharacteristic(
    BLEUUID((uint16_t)0x1A03),
    BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE);

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("Connected was called");
    deviceConnected = true;
  };

  void onDisconnect()
  {
    deviceConnected = false;
  }

  // void onWrite(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
  //                                         uint8_t* pData, size_t length, bool isNotify) {
  //   //store temperature value
  //   Serial.println((char*)pData);
  //   // newTemperature = true;
  // }

  void onWrite(BLECharacteristic *pCharacteristic)
  {
    Serial.println("notifyed");
    std::string rxValue = pCharacteristic->getValue();

    filterString = rxValue.c_str(); // Convert to standard c string format
    Serial.println(filterString);
    if (filterString.substring(0, 1) == "t")
    { // Check for header
      if (filterString.substring(2) == "0")
      {
        state = 1;
      }
      else
      {
        state = 0;
      }
    }
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
float previousSmoothedPressure = 0;
SensorState currentState;

static void sensorsReadPressure(void)
{
  if (millis() > pressureTimer)
  {
    previousSmoothedPressure = currentState.smoothedPressure;
    currentState.pressure = getPressure();
    currentState.isPressureRising = isPressureRaising();
    currentState.isPressureRisingFast = currentState.smoothedPressure >= previousSmoothedPressure + 0.06f;
    currentState.isPressureFalling = isPressureFalling();
    currentState.isPressureFallingFast = isPressureFallingFast();
    currentState.smoothedPressure = smoothPressure.updateEstimate(currentState.pressure);
    pressureTimer = millis() + GET_PRESSURE_READ_EVERY;
  }
}

void setup()
{
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("Osterino");

  adsInit();

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pEnviornment = pServer->createService(enviornmentService);

  // Create a BLE Characteristic
  pEnviornment->addCharacteristic(&temperatureCharacteristic);
  pEnviornment->addCharacteristic(&pressureCharacteristic);
  pEnviornment->addCharacteristic(&brewingCharacteristic);

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  temperatureCharacteristic.addDescriptor(new BLE2902());
  pressureCharacteristic.addDescriptor(new BLE2902());
  brewingCharacteristic.addDescriptor(new BLE2902());

  BLEDescriptor TemperatureDescriptor(BLEUUID((uint16_t)0x2901));
  TemperatureDescriptor.setValue("Temperature -40-60°C");
  temperatureCharacteristic.addDescriptor(&TemperatureDescriptor);

  BLEDescriptor PressureDescriptor(BLEUUID((uint16_t)0x2901));
  PressureDescriptor.setValue("Pressure in bar");
  pressureCharacteristic.addDescriptor(&PressureDescriptor);

  BLEDescriptor BrewingDescriptor(BLEUUID((uint16_t)0x2901));
  BrewingDescriptor.setValue("If is brewing or not");
  brewingCharacteristic.addDescriptor(&BrewingDescriptor);

  pServer->getAdvertising()->addServiceUUID(enviornmentService);

  // Start the service
  pEnviornment->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop()
{
  sensorsReadPressure();
  checkToReconnect();
  getTemp();
  // Serial.println((String)temperature + " Pressure: " + (String)currentState.smoothedPressure);
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

    delay(200);
  }
}

void BLETransfer(int16_t val)
{
  temperatureCharacteristic.setValue((uint8_t *)&val, 2);
  temperatureCharacteristic.notify();
}

void getTemp()
{
  temperature = thermocouple.readCelsius() + TEMPDIFF;

  if (deviceConnected)
  {
    // Serial.println(state);
    digitalWrite(RELAY_PIN, state);
  }
  else
  {
    digitalWrite(RELAY_PIN, 0);
  }
}
