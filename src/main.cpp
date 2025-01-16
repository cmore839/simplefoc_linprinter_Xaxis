// 02-01-24 Using canadas CS STM32
// Motor 2 tested fine, close loop FOC and open loop.
// Motor 1 Oen loop and closed loop okay.
// 68us without CORDIC, 64us with CORDIC.

#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

//Motor & Encoder Setup
BLDCMotor M1 = BLDCMotor(2); //Probably leave as 2 for every linear motor combo
BLDCMotor M2 = BLDCMotor(2);
BLDCDriver3PWM DR1 = BLDCDriver3PWM(PC9, PB4, PC7, PA9); //M1 - Lower
BLDCDriver3PWM DR2 = BLDCDriver3PWM(PB10, PB3, PA5, PA8); //M2 - Upper
// BLDCDriver3PWM DR1 = BLDCDriver3PWM(PC9, PB5, PC6, PA9); //M1 - Lower
// BLDCDriver3PWM DR2 = BLDCDriver3PWM(5, 9, 6, 8); //M2 - Upper
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
float position_error1 = 0;
float position_error2 = 0;
float loop_count = 0;
unsigned long start;
unsigned long finish;
unsigned long looptime;
int loopcounter = 0;
int loopiter = 10;
int t = 0;
unsigned int timestamp = micros();
float phase_resistance = 6.80;
float d_phase_inductance = 2.40/1000;
float q_phase_inductance = 3.30/1000;

float voltage_set = 31;
float M_angle_P = 50;
float M_velocity_P = 0.2;
float M_velocity_I = 0.0;
float current_bandwidth = 200;

//Inline sense and Step/Dir
LowsideCurrentSense CS1  = LowsideCurrentSense(0.01, 50, A2, A0, _NC);
LowsideCurrentSense CS2  = LowsideCurrentSense(0.01, 50, A3, A1, _NC);
// LowsideCurrentSense CS1  = LowsideCurrentSense(0.01, 50, A3, A1, _NC);
// LowsideCurrentSense CS2  = LowsideCurrentSense(0.01, 50, A2, A0, _NC);
StepDirListener SD1 = StepDirListener(PA15, PC12, 0.0014);
void onStep() { SD1.handle(); } 

// // characterisation voltage set point variable
// float characteriseVolts = 0.0f;

// // instantiate the commander
// Commander command = Commander(Serial);
// void onMotor(char* cmd){command.motor(&M2,cmd);}
// void characterise(char* cmd) { 
//   command.scalar(&characteriseVolts, cmd); 
//   M2.characteriseMotor(characteriseVolts);
// }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable();
  SimpleFOC_CORDIC_Config();

  //Motor 1
  E1.quadrature = Quadrature::ON;
  E1.init();
  E1.enableInterrupts(doA1, doB1);
  E2.quadrature = Quadrature::ON;
  E2.init();
  E2.enableInterrupts(doA2, doB2);
  M1.linkSensor(&E1);

  // setting the limits
  M1.velocity_limit = 200;
  M1.voltage_limit = voltage_set;
  M1.current_limit = 99999;
  DR1.pwm_frequency = 50000;
  DR1.voltage_power_supply = voltage_set;
  M1.voltage_sensor_align = 8;
  E1.min_elapsed_time = 0.000050; //20kHz sensor update

  // velocity PID controller parameters
  M1.PID_velocity.P = M_velocity_P;
  M1.PID_velocity.I = M_velocity_I;
  M1.PID_velocity.D = 0;
  M1.PID_velocity.output_ramp = 0;
  M1.LPF_velocity.Tf = 0.001;
   
  // angle PID controller 
  M1.P_angle.P = M_angle_P;
  M1.P_angle.I = 0;
  M1.P_angle.D = 0;
  M1.P_angle.output_ramp = 0;
  M1.LPF_angle.Tf = 0;

  // foc current control parameters
  M1.PID_current_q.P = q_phase_inductance*current_bandwidth*_2PI;
  M1.PID_current_q.I= M1.PID_current_q.P*phase_resistance/q_phase_inductance;
  M1.PID_current_d.P= d_phase_inductance*current_bandwidth*_2PI;
  M1.PID_current_d.I = M1.PID_current_d.P*phase_resistance/d_phase_inductance;
  M1.LPF_current_q.Tf = 1/(_2PI*1.0f*current_bandwidth); 
  M1.LPF_current_d.Tf = 1/(_2PI*1.0f*current_bandwidth);
  M1.motion_downsample = 0; // - times (default 0 - disabled)

  // init
  DR1.init();
  CS1.linkDriver(&DR1);
  M1.linkDriver(&DR1);
  CS1.init();
  CS1.gain_a *= -1;
  M1.linkCurrentSense(&CS1);
  M1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  M1.controller = MotionControlType::angle;
  M1.torque_controller = TorqueControlType::foc_current;
  M1.init();
  M1.initFOC(); //skip for open loop
  Serial.println("***M1 Init***");
  delay(1000);

  //Motor 2
  M2.linkSensor(&E2);

  // setting the limits
  M2.velocity_limit = M1.velocity_limit;
  M2.voltage_limit = M1.voltage_limit;
  M2.current_limit = M1.current_limit;
  M2.voltage_sensor_align = M1.voltage_sensor_align;
  DR2.pwm_frequency = DR1.pwm_frequency;
  DR2.voltage_power_supply = DR1.voltage_power_supply;
  E2.min_elapsed_time = E1.min_elapsed_time;

  // velocity PID controller parameters
  M2.PID_velocity.P = M1.PID_velocity.P;
  M2.PID_velocity.I = M1.PID_velocity.I;
  M2.PID_velocity.D = 0;
  M2.PID_velocity.output_ramp = 0;
  M2.LPF_velocity.Tf = 0.001;
   
  // angle PID controller 
  M2.P_angle.P = M1.P_angle.P;
  M2.P_angle.I = 0;
  M2.P_angle.D = 0; 
  M2.P_angle.output_ramp = 0;
  M2.LPF_angle.Tf = 0;

  // foc current control parameters
  M2.PID_current_q.P = M1.PID_current_q.P;
  M2.PID_current_q.I= M1.PID_current_q.I;
  M2.PID_current_d.P= M1.PID_current_d.P;
  M2.PID_current_d.I = M1.PID_current_d.I;
  M2.LPF_current_q.Tf = M1.LPF_current_q.Tf; 
  M2.LPF_current_d.Tf = M1.LPF_current_d.Tf; 
  M2.motion_downsample = 0; // - times (default 0 - disabled)

  // init
  DR2.init();
  CS2.linkDriver(&DR2);
  M2.linkDriver(&DR2);
  CS2.init();
  //CS2.gain_a *= -1;
  M2.linkCurrentSense(&CS2);
  M2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  M2.controller = MotionControlType::angle;
  M2.torque_controller = TorqueControlType::foc_current;
  M2.init();
  M2.initFOC(); //skip for open loop
  Serial.println("***M2 Init***");
  delay(1000);

  SD1.init();// Comment out if want to use commander
  SD1.enableInterrupt(onStep);// Comment out if want to use commander
  SD1.attach(&received_angle);// Comment out if want to use commander
  // command.add('M',&onMotor,"Control motor");
  // command.add('L', characterise, "Characterise motor L & R with the given voltage");

  // M1.disable();
  // M2.disable();

  // Serial.println(F("Motor disabled and ready."));
  // Serial.println(F("Control the motor and measure the inductance using the terminal. Type \"?\" for available commands:"));
  // _delay(1000);

}

void loop() {
  // if(t++> 10000){ // 10k analog reads
  //    t = 0;
  //    Serial.println(micros() - timestamp);
  //    timestamp = micros();
  // }
  //command.run();
  if (loopcounter == loopiter){
    start = micros();
  }
  M1.loopFOC();
  M2.loopFOC(); //time halfs with 1 motor
  M1.move(received_angle);
  M2.move(received_angle);
  // M1.move();
  // M2.move();
  if (loopcounter == loopiter){
    finish = micros();
    looptime = (finish - start);
    current1 = CS1.getPhaseCurrents();
    current2 = CS2.getPhaseCurrents();
    set_distance_mm = received_angle * 12.732395;
    actual_distance_mm = M1.shaft_angle * 12.732395;
    actual_distance_velocity = M1.shaft_velocity * 12.732395;
    //position_error = set_distance_mm-actual_distance_mm;
    position_error1 = received_angle-M1.shaft_angle;
    position_error2 = received_angle-M2.shaft_angle;
    //Re apply global vars
    M1.voltage_limit = voltage_set;
    M1.PID_velocity.P = M_velocity_P;
    M1.P_angle.P = M_angle_P;
    M2.velocity_limit = M1.velocity_limit;
    M2.voltage_limit = M1.voltage_limit;
    M2.current_limit = M1.current_limit;
    M2.voltage_sensor_align = M1.voltage_sensor_align;
    M2.PID_velocity.P = M1.PID_velocity.P;
    M2.PID_velocity.I = M1.PID_velocity.I;
    M2.P_angle.P = M1.P_angle.P;
    loopcounter = 0;
  }
  loopcounter++;

}