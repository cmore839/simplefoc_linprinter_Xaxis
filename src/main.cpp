#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#include "chirp_profile.h"
#include "communication/HWStepDirListener.h" // Your new hardware listener class

// ==============================================================================
// 1. CUBEMX HARDWARE INJECTION
// ==============================================================================

// Global timer handle[cite: 7]
TIM_HandleTypeDef htim1;

// Low-level hardware clock and pin initialization (From stm32g4xx_hal_msp.c)[cite: 8]
// WRAPPED IN extern "C" SO ARDUINO/C++ CAN LINK IT PROPERLY
extern "C" void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_encoder->Instance==TIM1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /**TIM1 GPIO Configuration
    PC0     ------> TIM1_CH1
    PC1     ------> TIM1_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

// Timer mode and register configuration (From main.c)[cite: 7]
static void MX_TIM1_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_CLOCKPLUSDIRECTION_X1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

// ==============================================================================
// 2. SIMPLE FOC SETUP & VARIABLES
// ==============================================================================

// Motor & Encoder Setup
BLDCMotor M1 = BLDCMotor(2); 
BLDCMotor M2 = BLDCMotor(2);
BLDCDriver3PWM DR1 = BLDCDriver3PWM(PC9, PB4, PC7, PA9); //M1 - Lower
BLDCDriver3PWM DR2 = BLDCDriver3PWM(PB10, PB3, PA5, PA8); //M2 - Upper
Encoder E1 = Encoder(PC6, PC8, 1110); 
Encoder E2 = Encoder(PB1, PB2, 1110); 
void doA1(){E1.handleA();}
void doB1(){E1.handleB();}
void doA2(){E2.handleA();}
void doB2(){E2.handleB();}

// Variables
PhaseCurrent_s current1;
PhaseCurrent_s current2;
float received_angle = 0; 
float actual_distance1_mm = 0;
float actual_distance2_mm = 0;
float set_distance_mm = 0;
float actual_distance1_velocity = 0;
float actual_distance2_velocity = 0;
float position_error1 = 0;
float position_error2 = 0;
float received_velocity = 0.0f;
float Aset_velocity = 0.0f;
float loop_count = 0;
unsigned long start;
unsigned long finish;
unsigned long looptime;
int loopcounter = 0;
int followerrorcount = 0;
int loopiter = 10;
int t = 0;
int enableKLIP = 0;
int enablecount = 0;
int enablelatch = 0;
int startupcount = 0;
unsigned int timestamp = micros();
float E1_angle_temp = 0.0;
float E2_angle_temp = 0.0;

unsigned long error_timer_start_1 = 0;
bool error_active_1 = false;
unsigned long error_timer_start_2 = 0;
bool error_active_2 = false;
const unsigned long ERROR_TIMEOUT_MS = 2000; 
const float SLOW_ERROR_LIMIT = 5.0; 

float phase_resistance = 6.80;
float d_phase_inductance = 2.40/1000;
float q_phase_inductance = 3.30/1000;
float motor_enable_offset = 0.0f;
float current_bandwidth = 330*0.9; 
float Apos_ref = 0.0f;
float enable_signal = 0.0;

ChirpProfile chirp;

// Inline sense and Hardware Step/Dir
LowsideCurrentSense CS1  = LowsideCurrentSense(0.01, 50, A2, A0, _NC);
LowsideCurrentSense CS2  = LowsideCurrentSense(0.01, 50, A3, A1, _NC);

// Hardware Listener Object (Linked to TIM1)
HWStepDirListener SD1 = HWStepDirListener(&htim1, 0.0014);

// Low pass filter for the feedforward velocity (5ms time constant)
LowPassFilter v_ff_filter = LowPassFilter(0.005);

void startup(){
  while (startupcount < 3000000){
    received_angle = (M1.shaft_angle + M2.shaft_angle) / 2.0;
    startupcount = micros();
    M1.velocity_limit = 6;
    M2.velocity_limit = 6;
    M1.loopFOC();
    M2.loopFOC();
    M1.move(0);
    M2.move(0);
    M1.sensor_offset = M1.sensor_direction * M1.sensor->getAngle();
    M2.sensor_offset = M2.sensor_direction * M2.sensor->getAngle();
  }
  M1.velocity_limit = 999;
  M2.velocity_limit = 999; 
  M1.disable();
  M2.disable();
}

void setup() {
  chirp.begin(5, 50.0, 8.0, 0.05);  

  Serial.begin(115200);
  SimpleFOCDebug::enable();
  SimpleFOC_CORDIC_Config();

  // Initialize TIM1 Hardware BEFORE the listener attaches to it
  MX_TIM1_Init();

  //Motor 1
  E1.quadrature = Quadrature::ON;
  E1.init();
  E1.enableInterrupts(doA1, doB1);
  E2.quadrature = Quadrature::ON;
  E2.init();
  E2.enableInterrupts(doA2, doB2);
  M1.linkSensor(&E1);

  // setting the limits
  M1.velocity_limit = 99999;
  M1.voltage_limit = 31;
  M1.current_limit = 3.0;
  DR1.pwm_frequency = 20000;
  DR1.voltage_power_supply = M1.voltage_limit;
  M1.voltage_sensor_align = 12;
  E1.min_elapsed_time = 0.000050; //20kHz sensor update

  // velocity PID controller parameters
  M1.PID_velocity.P = 0.0858*0.9; //164Hz Bandwidth
  M1.PID_velocity.I = 0;//
  M1.PID_velocity.D = 0;
  M1.PID_velocity.output_ramp = 0;
  M1.LPF_velocity.Tf = (1/(820.0*0.9));
   
  // angle PID controller 
  M1.P_angle.P = 550.0*0.9;
  M1.P_angle.I = 0;
  M1.P_angle.D = 0;
  M1.P_angle.output_ramp = 0;
  M1.LPF_angle.Tf = 0;

  // foc current control parameters
  M1.PID_current_q.P = q_phase_inductance*current_bandwidth*_2PI;
  M1.PID_current_q.I= M1.PID_current_q.P*phase_resistance/q_phase_inductance;
  M1.PID_current_d.P= d_phase_inductance*current_bandwidth*_2PI;
  M1.PID_current_d.I = M1.PID_current_d.P*phase_resistance/d_phase_inductance;
  M1.LPF_current_q.Tf = 1/(5.0*current_bandwidth); 
  M1.LPF_current_d.Tf = 1/(5.0*current_bandwidth);
  M1.motion_downsample = 0; // - times (default 0 - disabled)
  //M1.sensor_direction = Direction::CCW; //Y Axis
  M1.sensor_direction = Direction::CW; //X Axis

  // init
  DR1.init();
  CS1.linkDriver(&DR1);
  M1.linkDriver(&DR1);
  CS1.skip_align = false; //true to skip current sense alignment
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
  M2.PID_velocity.D = M1.PID_velocity.D;
  M2.PID_velocity.output_ramp = M1.PID_velocity.output_ramp;
  M2.LPF_velocity.Tf = M1.LPF_velocity.Tf;
   
  // angle PID controller 
  M2.P_angle.P = M1.P_angle.P;
  M2.P_angle.I = M1.P_angle.I;
  M2.P_angle.D = M1.P_angle.D; 
  M2.P_angle.output_ramp = 0;
  M2.LPF_angle.Tf = M1.LPF_angle.Tf;

  // foc current control parameters
  M2.PID_current_q.P = M1.PID_current_q.P;
  M2.PID_current_q.I= M1.PID_current_q.I;
  M2.PID_current_d.P= M1.PID_current_d.P;
  M2.PID_current_d.I = M1.PID_current_d.I;
  M2.LPF_current_q.Tf = M1.LPF_current_q.Tf; 
  M2.LPF_current_d.Tf = M1.LPF_current_d.Tf; 
  M2.motion_downsample = M1.motion_downsample;
  M2.sensor_direction = M1.sensor_direction;

  // init
  DR2.init();
  CS2.linkDriver(&DR2);
  M2.linkDriver(&DR2);
  CS2.skip_align = false; //true to skip current sense alignment
  CS2.init();
  CS2.gain_a *= -1;
  M2.linkCurrentSense(&CS2);
  M2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  M2.controller = MotionControlType::angle;
  M2.torque_controller = TorqueControlType::foc_current;
  M2.init();
  M2.initFOC(); //skip for open loop
  Serial.println("***M2 Init***");
  delay(1000);

  // Hardware Step/Dir initialization
  SD1.init(); 
  SD1.attach(&received_angle, &received_velocity);
  
  pinMode(PB7, INPUT); // X axis klipper enable pin
  startup();
}

void loop() {
  
  // Update hardware listener (replaces the interrupt handle)
  SD1.update();

  // X axis klipper enable
  if (enablecount == 1000){
    if (enableKLIP == 0){
      M1.disable();
      M2.disable();
      enablelatch = 0;
      error_active_1 = false;
      error_active_2 = false;
    }
    if (enableKLIP == 1 && enablelatch == 0){
      M1.enable();
      M2.enable();
      enablelatch = 1;
    }
    enablecount = 0;
  }
  
  // Loop time start
  if (loopcounter == loopiter){
    start = micros();
  }

  if (enableKLIP == 1){
    // Apply low-pass filter to finite-differenced velocity 
    float filtered_velocity = v_ff_filter(received_velocity);
    
    M1.feed_forward_velocity = filtered_velocity;
    M2.feed_forward_velocity = filtered_velocity;
    M1.loopFOC();
    M2.loopFOC();
    M1.move(received_angle);
    M2.move(received_angle);
  }

  // --- CALCULATION AND SAFETY BLOCK ---
  // Runs every 'loopiter' (approx every 10 loops)
  if (loopcounter == loopiter){
    finish = micros();
    looptime = (finish - start);
    
    // update currents
    current1 = CS1.getPhaseCurrents();
    current2 = CS2.getPhaseCurrents();
    
    // update positions
    set_distance_mm = received_angle * 12.732395;
    actual_distance1_mm = M1.shaft_angle * 12.732395;
    actual_distance1_velocity = M1.shaft_velocity * 12.732395;
    actual_distance2_mm = M2.shaft_angle * 12.732395;
    actual_distance2_velocity = M2.shaft_velocity * 12.732395;
    
    // Errors calculated here
    position_error1 = set_distance_mm - actual_distance1_mm;
    position_error2 = set_distance_mm - actual_distance2_mm;

    if (enableKLIP == 1) { 
        // 1. HARD FAULT (Immediate Kill > 50mm)
        if (abs(position_error1) > 50.0 || abs(position_error2) > 50.0) {
            M1.disable();
            M2.disable();
            Serial.println("CRITICAL: Hard Following Error Exceeded! System Halted.");
            while(1); 
        }

        // 2. TIMED "SLOW" FOLLOWING ERROR (Motor 1)
        if (abs(position_error1) > SLOW_ERROR_LIMIT) {
            if (!error_active_1) {
                // Error just started, start the clock
                error_timer_start_1 = millis();
                error_active_1 = true;
            } else if (millis() - error_timer_start_1 > ERROR_TIMEOUT_MS) {
                // Timer exceeded 2 seconds
                M1.disable();
                M2.disable();
                Serial.println("CRITICAL: Mot 1 Timed Following Error! System Halted.");
                while(1);
            }
        } else {
            // Error is within limits, reset timer flag
            error_active_1 = false;
        }

        // 3. TIMED "SLOW" FOLLOWING ERROR (Motor 2)
        if (abs(position_error2) > SLOW_ERROR_LIMIT) {
            if (!error_active_2) {
                error_timer_start_2 = millis();
                error_active_2 = true;
            } else if (millis() - error_timer_start_2 > ERROR_TIMEOUT_MS) {
                M1.disable();
                M2.disable();
                Serial.println("CRITICAL: Mot 2 Timed Following Error! System Halted.");
                while(1);
            }
        } else {
            error_active_2 = false;
        }
    }
    // ----------------------------------
    //Re apply global vars for M1 & M2
    M2.velocity_limit = M1.velocity_limit;
    M2.voltage_limit = M1.voltage_limit;
    M2.current_limit = M1.current_limit;
    M2.voltage_sensor_align = M1.voltage_sensor_align;
    DR2.pwm_frequency = DR1.pwm_frequency;
    DR2.voltage_power_supply = DR1.voltage_power_supply;
    E2.min_elapsed_time = E1.min_elapsed_time;
    M2.PID_velocity.P = M1.PID_velocity.P;
    M2.PID_velocity.I = M1.PID_velocity.I;
    M2.PID_velocity.D = M1.PID_velocity.D;
    M2.PID_velocity.output_ramp = M1.PID_velocity.output_ramp;
    M2.LPF_velocity.Tf = M1.LPF_velocity.Tf;
    M2.P_angle.P = M1.P_angle.P;
    M2.P_angle.I = M1.P_angle.I;
    M2.P_angle.D = M1.P_angle.D; 
    M2.P_angle.output_ramp = 0;
    M2.LPF_angle.Tf = M1.LPF_angle.Tf;
    M2.PID_current_q.P = M1.PID_current_q.P;
    M2.PID_current_q.I= M1.PID_current_q.I;
    M2.PID_current_d.P= M1.PID_current_d.P;
    M2.PID_current_d.I = M1.PID_current_d.I;
    M2.LPF_current_q.Tf = M1.LPF_current_q.Tf; 
    M2.LPF_current_d.Tf = M1.LPF_current_d.Tf; 
    M2.motion_downsample = M1.motion_downsample;
    E1.update();
    E2.update();
    E1_angle_temp = E1.getSensorAngle();
    E2_angle_temp = E2.getSensorAngle();
    
    //Read klipper enable pin X Axis
    enableKLIP = digitalRead(PB7);
    loopcounter = 0;
  }

  followerrorcount++;
  loopcounter++;
  enablecount++;
}