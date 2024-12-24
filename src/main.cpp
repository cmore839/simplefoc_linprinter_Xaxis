/**
 * An example code for the generic current sensing implementation
*/
#include <SimpleFOC.h>


// user defined function for reading the phase currents
// returning the value per phase in amps

PhaseCurrent_s readCurrentSense(){
  PhaseCurrent_s c;
  // dummy example only reading analog pins
  c.a = analogRead(A0);
  //c.b = analogRead(A1);
  //c.c = analogRead(0); // if no 3rd current sense set it to 0
  return(c);

}
PhaseCurrent_s current1;
// user defined function for intialising the current sense
// it is optional and if provided it will be called in current_sense.init()
void initCurrentSense(){
  pinMode(A0,INPUT);
  //pinMode(A1,INPUT);
  //pinMode(A2,INPUT);
}


// GenericCurrentSense class constructor
// it receives the user defined callback for reading the current sense
// and optionally the user defined callback for current sense initialisation
GenericCurrentSense current_sense = GenericCurrentSense(readCurrentSense, initCurrentSense);

float loop_count = 0;
unsigned long start;
unsigned long finish;
unsigned long looptime;
int loopcounter = 0;
int loopiter = 10;

void setup() {

  // use monitoring with serial 
  //Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  //SimpleFOCDebug::enable(&Serial);

  // initialise the current sensing
  current_sense.init();

  // for SimpleFOCShield v2.01/v2.0.2
  current_sense.gain_b *= -1;
}

void loop() {
  if (loopcounter == loopiter){
    start = micros();
  }
  current1 = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();
  if (loopcounter == loopiter){
    finish = micros();
    looptime = (finish - start);
    loopcounter = 0;
  }
  loopcounter++;
}