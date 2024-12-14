#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

//Motor & Encoder Setup
BLDCMotor M1 = BLDCMotor(2); //Probably leave as 2 for every linear motor combo
BLDCMotor M2 = BLDCMotor(2);
BLDCDriver3PWM DR1 = BLDCDriver3PWM(PB6, PB4, PB10, PA9); //M1 - Lower
BLDCDriver3PWM DR2 = BLDCDriver3PWM(PC7, PB3, PA5, PA8); //M2 - Upper
Encoder E1 = Encoder(PC6, PC8, 1110); //M1 - Lower, X1=1108-1109 9/12/24
Encoder E2 = Encoder(PB1, PB2, 1110); //M2 - Upper, X2=1110, 9/12/24
void doA1(){E1.handleA();}
void doB1(){E1.handleB();}
void doA2(){E2.handleA();}
void doB2(){E2.handleB();}

// Variables
PhaseCurrent_s current1;
PhaseCurrent_s current2;
float received_angle = 0; // angle set point variable
float actual_distance_mm = 0;
float set_distance_mm = 0;
float actual_distance_velocity = 0;
float position_error = 0;
float loop_count = 0;
unsigned long start;
unsigned long finish;
unsigned long looptime;
int loopcounter = 0;
int loopiter = 10;

float voltage_set = 12;
float M_angle_P = 50;
float M_velocity_P = 0.5;

//Inline sense and Step/Dir
InlineCurrentSense CS1  = InlineCurrentSense(0.01, 50, A0, A2, _NC);
InlineCurrentSense CS2  = InlineCurrentSense(0.01, 50, A1, A3, _NC);
StepDirListener SD1 = StepDirListener(PA15, PC12, 0.0014);
void onStep() { SD1.handle(); } 

void setup() {
  SimpleFOC_CORDIC_Config();

  //Motor 1
  E1.quadrature = Quadrature::ON;
  E1.init();
  E1.enableInterrupts(doA1, doB1);
  E2.quadrature = Quadrature::ON;
  E2.init();
  E2.enableInterrupts(doA2, doB2);
  M1.linkSensor(&E1);

  // driver config
  DR1.pwm_frequency = 50000;
  DR1.voltage_power_supply = voltage_set;
  DR1.init();
  CS1.linkDriver(&DR1);
  M1.linkDriver(&DR1);
  M1.init();
  CS1.init();
  M1.linkCurrentSense(&CS1);
  M1.voltage_sensor_align = 12;
  M1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  M1.controller = MotionControlType::angle;
  M1.torque_controller = TorqueControlType::foc_current;

  // setting the limits
  M1.velocity_limit = 20;
  M1.voltage_limit = voltage_set;
  M1.current_limit = 1;

  // velocity PID controller parameters
  M1.PID_velocity.P = M_velocity_P; //0.6
  M1.PID_velocity.I = 0;
  M1.PID_velocity.D = 0;
  M1.PID_velocity.output_ramp = 0;
  M1.LPF_velocity.Tf = 0.001;
   
  // angle PID controller 
  M1.P_angle.P = M_angle_P; //100 
  M1.P_angle.I = 0; //0.4
  M1.P_angle.D = 0; //2.0
  M1.P_angle.output_ramp = 0;
  M1.LPF_angle.Tf = 0;

  // foc current control parameters
  M1.PID_current_q.P = 1;
  M1.PID_current_q.I= 0.1;
  M1.PID_current_d.P= 1;
  M1.PID_current_d.I = 0.1;
  M1.LPF_current_q.Tf = 0.01; 
  M1.LPF_current_d.Tf = 0.01; 
  
  //Motor 2
  M2.linkSensor(&E2);

  // driver config
  DR2.pwm_frequency = 50000;
  DR2.voltage_power_supply = voltage_set;
  DR2.init();
  CS2.linkDriver(&DR2);
  M2.linkDriver(&DR2);
  M2.init();
  CS2.init();
  M2.linkCurrentSense(&CS2);
  M2.voltage_sensor_align = 12;
  M2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  M2.controller = MotionControlType::angle;
  M2.torque_controller = TorqueControlType::foc_current;

  // setting the limits
  M2.velocity_limit = 20;
  M2.voltage_limit = voltage_set;
  M2.current_limit = 1;

  // velocity PID controller parameters
  M2.PID_velocity.P = M_velocity_P; //0.6
  M2.PID_velocity.I = 0;
  M2.PID_velocity.D = 0;
  M2.PID_velocity.output_ramp = 0;
  M2.LPF_velocity.Tf = 0.001;
   
  // angle PID controller 
  M2.P_angle.P = M_angle_P; //100 
  M2.P_angle.I = 0; //0.4
  M2.P_angle.D = 0; //2.0
  M2.P_angle.output_ramp = 0;
  M2.LPF_angle.Tf = 0;

  // foc current control parameters
  M2.PID_current_q.P = 1;
  M2.PID_current_q.I= 0.1;
  M2.PID_current_d.P= 1;
  M2.PID_current_d.I = 0.1;
  M2.LPF_current_q.Tf = 0.01; 
  M2.LPF_current_d.Tf = 0.01; 

  M1.initFOC(); //skip for open loop
  M2.initFOC(); //skip for open loop

  // 655us with SD
  SD1.init();// Comment out if want to use commander
  SD1.enableInterrupt(onStep);// Comment out if want to use commander
  SD1.attach(&received_angle);// Comment out if want to use commander
}

void loop() {
  if (loopcounter == loopiter){
    start = micros();
  }
  M1.loopFOC();
  M2.loopFOC(); //time halfs with 1 motor
  M1.move(received_angle);
  M2.move(received_angle);
  if (loopcounter == loopiter){
    finish = micros();
    looptime = (finish - start);
    //MCU Viewer Variables 650us without
    current1 = CS1.getPhaseCurrents();
    current2 = CS2.getPhaseCurrents();
    set_distance_mm = received_angle * 12.732395;
    actual_distance_mm = M1.shaft_angle * 12.732395;
    actual_distance_velocity = M1.shaft_velocity * 12.732395;
    position_error = set_distance_mm-actual_distance_mm;
    loopcounter = 0;
  }
  loopcounter++;

}