#include "scales.h"
#include "pindef.h"
#include <HX711.h>
HX711 LoadCell_1; //HX711 1
// HX711 LoadCell_2; //HX711 2

bool scalesPresent;
unsigned char scale_clk = OUTPUT_OPEN_DRAIN;

void scalesInit(float scalesF1) {

    LoadCell_1.begin(HX711_dout_1, HX711_sck_1);
    // LoadCell_2.begin(HX711_dout_2, HX711_sck_2);
    LoadCell_1.set_scale(scalesF1); // calibrated val1
    // LoadCell_2.set_scale(scalesF2); // calibrated val2

    if (LoadCell_1.wait_ready_timeout(700, 20)) {
      scalesPresent = true;
      LoadCell_1.tare();
      // LoadCell_2.tare();
    }
}

void scalesTare(void) {
    if (LoadCell_1.wait_ready_timeout(100, 20)) {
      LoadCell_1.tare();
      // LoadCell_2.tare(2);
    }
}

float scalesGetWeight(void) {
  float currentWeight = 0.f;
    if (LoadCell_1.wait_ready_timeout(100, 20)) {
      currentWeight = LoadCell_1.get_units();
    }
  return currentWeight;
}

bool scalesIsPresent(void) {
  return scalesPresent;
}

float scalesDripTrayWeight() {
  long value[2];
    value[0] = LoadCell_1.read_average(4);
    // value[1] = LoadCell_2.read_average(4);
  return ((float)value[0]);
}
