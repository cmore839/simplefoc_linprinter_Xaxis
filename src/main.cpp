#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
BLDCMotor motor = BLDCMotor(2); //Probably leave as 2 for every linear motor combo
BLDCDriver3PWM driver = BLDCDriver3PWM(PB6, PB4, PB10, PA9);
Encoder encoder = Encoder(PC8, PC6, 1120); //1120 was found via trial and error.
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
PhaseCurrent_s current;
// angle set point variable
float received_angle = 0;
float actual_distance_mm = 0;
float set_distance_mm = 0;
float actual_distance_velocity = 0;
float position_error = 0;
float loop_count = 0;
// instantiate the commander
//Commander command = Commander(Serial);
//void onMotor(char* cmd){ command.motor(&motor,cmd); }
InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 50, A0, A2, _NC);
//InlineCurrentSense current_sense  = InlineCurrentSense(132, A1, A3, _NC);
StepDirListener step_dir = StepDirListener(PA15, PC12, 0.0014); // Comment out if want to use commander
void onStep() { step_dir.handle(); } // Comment out if want to use commander

void setup() {
  SimpleFOC_CORDIC_Config();
  //command.add('M',onMotor,"LinMot");

  // setting the limits
  motor.velocity_limit = 9999999;
  motor.voltage_limit = 4;
  motor.current_limit = 0.5;
  current_sense.skip_align  = true; // default false, skip for open loop

  encoder.quadrature = Quadrature::ON;
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);

  // driver config
  driver.pwm_frequency = 50000;
  driver.voltage_power_supply = 24;
  
  driver.init();
  current_sense.linkDriver(&driver);
  motor.linkDriver(&driver);
    // initialize motor
  motor.init();
  // init current sense
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  
  motor.voltage_sensor_align = 3;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle_openloop;
  motor.torque_controller = TorqueControlType::foc_current;

  // velocity PID controller parameters
  motor.PID_velocity.P = 0.6; //0.6
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 0;
  motor.LPF_velocity.Tf = 0.001;
   
  // angle PID controller 
  motor.P_angle.P = 100; //200 
  motor.P_angle.I = 0; //0.4
  motor.P_angle.D = 0; //2.0
  motor.P_angle.output_ramp = 0;
  motor.LPF_angle.Tf = 0;

  // foc current control parameters
  motor.PID_current_q.P = 1;
  motor.PID_current_q.I= 0.1;
  motor.PID_current_d.P= 1;
  motor.PID_current_d.I = 0.1;
  motor.LPF_current_q.Tf = 0.01; 
  motor.LPF_current_d.Tf = 0.01; 
  // motor.phase_resistance = 6.0;
  // motor.phase_inductance = 0.006;

  //motor.motion_downsample = 0;
  //motor.useMonitoring(Serial);
  //Serial.begin(115200);

  //motor.initFOC(); //skip for open loop
  step_dir.init();// Comment out if want to use commander
  step_dir.enableInterrupt(onStep);// Comment out if want to use commander
  step_dir.attach(&received_angle);// Comment out if want to use commander
  //delay(10000);

}

void loop() {
  motor.loopFOC();
  motor.move(received_angle);
  //motor.move();
  //motor.monitor();
  //command.run();

  //MCU Viewer Variables
  current = current_sense.getPhaseCurrents();
  set_distance_mm = received_angle * 12.732395;
  actual_distance_mm = motor.shaft_angle * 12.732395;
  actual_distance_velocity = motor.shaft_velocity * 12.732395;
  position_error = set_distance_mm-actual_distance_mm;

}