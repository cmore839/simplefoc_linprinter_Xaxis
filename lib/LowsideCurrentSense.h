#ifndef LOWSIDE_CS_LIB_H
#define LOWSIDE_CS_LIB_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "../common/base_classes/CurrentSense.h"
#include "hardware_api.h"
#include "stm32f4xx_hal.h"

class LowsideCurrentSense: public CurrentSense{
  public:
    // Constructors
    LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC, int pinVbus, float vbus_gain);
    LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);
    LowsideCurrentSense(float mVpA, int pinA, int pinB, int pinC = _NC);

    // Core functions
    int init() override;
    PhaseCurrent_s getPhaseCurrents() override;
    float getVbusVoltage();

    // Feature initialization
    void initBrakeResistorPWM(int pin, float target_voltage, float p_gain, float i_gain);
    void initFETTempSensors(int pin_m0, int pin_aux, float beta, float nom_res, float ser_res);

    // Update functions
    void updateBrakeResistor();
    void updateTemperatures();
    // Public members for monitoring
    float brake_duty_cycle = 0.0f;
    float fet_temp_m0 = 0.0f;
    float fet_temp_aux = 0.0f;

  private:
    void calibrateOffsets();
    

    // Member variables
    int pinA, pinB, pinC;
    float shunt_resistor, amp_gain, volts_to_amps_ratio;
    
    int pinVbus = NOT_SET, vbus_rank = -1;
    float vbus_gain = 1.0f;

    int pin_temp_m0 = NOT_SET, temp_m0_rank = -1;
    int pin_temp_aux = NOT_SET, temp_aux_rank = -1;
    float temp_beta_value, temp_nominal_resistance, temp_series_resistance;
    
    int pin_brake_resistor = NOT_SET;
    TIM_HandleTypeDef brake_timer_handle;
    float brake_target_voltage = 0, brake_p_gain = 0, brake_i_gain = 0, brake_integrator = 0;
};

#endif