/** G474 current sense tests
Generic Sense - defaults - 150us per channel
Generic Sense - added -O3 - 150us per channel
Generic Sense - tried PA0 - 150us per channel
Generic Sense - tried PA_0_ALT1(ADC2) - 150us per channel
Generic Sense - -DADC_SAMPLINGTIME0 - 150us per channel
Generic Sense - Canadas1 adc_rebase - 150us per channel
Generic Sense - -DHAL_ADC_MODULE_ONLY - failed
Generic Sense - -DHAL_ADC_MODULE_ENABLED - 150us per channel
Inline Sense - defaults-Canadas adc_rebase - 2-3us for 2 channel
Inline Sense - defaults/test - 300us for 2 channel
*/


#include <SimpleFOC.h>
// user defined function for reading the phase currents
// returning the value per phase in amps

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, A0, A2);
PhaseCurrent_s current1;

float loop_count = 0;
unsigned long start;
unsigned long finish;
unsigned long looptime;
int loopcounter = 0;
int loopiter = 100;

void setup() {
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  // initialise the current sensing
  current_sense.init();
}

void loop() {
  if (loopcounter == loopiter){
    start = micros();
  }
  
  current1 = current_sense.getPhaseCurrents();

  if (loopcounter == loopiter){
    finish = micros();
    looptime = (finish - start);
    loopcounter = 0;
  }
  loopcounter++;
}