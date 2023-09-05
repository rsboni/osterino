#ifndef SENSORS_STATE_H
#define SENSORS_STATE_H

struct SensorState {
  float targetTemperature = 104;
  float targetSteamTemperature = 135;
  float temperature;
  bool deviceConnected;
  bool isBrewing;
  float targetPressure;
  float targetWeight;
  float pressure;
  float smoothedPressure;
  float pressureChangeSpeed;
  // bool isPressureFalling;
  // bool isPressureFallingFast;
  // bool isPressureRising;
  // bool isPressureRisingFast;
  // bool isPumpFlowRisingFast;
  // bool isPumpFlowFallingFast;
  float pumpFlow;
  float smoothedPumpFlow;
  float liquidPumped;
  float smoothedFlow;
  float weightFlow;
  float weight;
  float shotWeight;
};

#endif
