#ifndef SENSORS_STATE_H
#define SENSORS_STATE_H

struct SensorState {
  float temperature;
  bool deviceConnected;
  bool isBrewing;
  float targetPressure;
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
  float weightFlow;
  float weight;
  float shotWeight;
};

#endif
