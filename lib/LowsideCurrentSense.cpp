#include "LowsideCurrentSense.h"
#include "communication/SimpleFOCDebug.h"
#include "hardware_specific/stm32/stm32_mcu.h"

// Constructor remains the same
LowsideCurrentSense::LowsideCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC, int _pinVbus, float _vbus_gain) : LowsideCurrentSense(_shunt_resistor, _gain, _pinA, _pinB, _pinC) {
    pinVbus = _pinVbus;
    vbus_gain = _vbus_gain;
}
LowsideCurrentSense::LowsideCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC){
    pinA = _pinA; pinB = _pinB; pinC = _pinC;
    shunt_resistor = _shunt_resistor; amp_gain  = _gain;
    volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain;
    gain_a = volts_to_amps_ratio; gain_b = volts_to_amps_ratio; gain_c = volts_to_amps_ratio;
}
LowsideCurrentSense::LowsideCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC){
    pinA = _pinA; pinB = _pinB; pinC = _pinC;
    volts_to_amps_ratio = 1000.0f / _mVpA;
    gain_a = volts_to_amps_ratio; gain_b = volts_to_amps_ratio; gain_c = volts_to_amps_ratio;
}

int LowsideCurrentSense::init(){
    if (driver==nullptr) return 0;

    // Pass ALL pins to the hardware config function at once
    params = _configureADCLowSide(driver->params, pinA, pinB, pinC, pinVbus, pin_temp_m0, pin_temp_aux);
    if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0;

    // The ADC engine is now fully configured. Sync it with the driver.
    void* r = _driverSyncLowSide(driver->params, params);
    if(r == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0;

    calibrateOffsets();
    initialized = (params!=SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
    return 1;
}

void LowsideCurrentSense::calibrateOffsets(){
    const int calibration_rounds = 10000;
    offset_ia=0; offset_ib=0; offset_ic=0;
    for (int i = 0; i < calibration_rounds; i++) {
        if(_isset(pinA)) offset_ia += (_readADCVoltageLowSide(pinA, params));
        if(_isset(pinB)) offset_ib += (_readADCVoltageLowSide(pinB, params));
        if(_isset(pinC)) offset_ic += (_readADCVoltageLowSide(pinC, params));
        _delay(1);
    }
    if(_isset(pinA)) offset_ia /= calibration_rounds;
    if(_isset(pinB)) offset_ib /= calibration_rounds;
    if(_isset(pinC)) offset_ic /= calibration_rounds;
}

PhaseCurrent_s LowsideCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current;
    current.a = (!_isset(pinA)) ? 0 : (_readADCVoltageLowSide(pinA, params) - offset_ia)*gain_a;
    current.b = (!_isset(pinB)) ? 0 : (_readADCVoltageLowSide(pinB, params) - offset_ib)*gain_b;
    current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageLowSide(pinC, params) - offset_ic)*gain_c;
    return current;
}

float LowsideCurrentSense::getVbusVoltage() {
  if (!_isset(pinVbus)) return 0.0f;
  // The ADC engine reads this in the background via DMA. We just grab the latest value.
  float adc_voltage = _readADCVoltageLowSide(pinVbus, params);
  return adc_voltage * vbus_gain;
}

void LowsideCurrentSense::initFETTempSensors(int pin_m0, int pin_aux, float beta, float nom_res, float ser_res) {
    pin_temp_m0 = pin_m0;
    pin_temp_aux = pin_aux;
    temp_beta_value = beta;
    temp_nominal_resistance = nom_res;
    temp_series_resistance = ser_res;
}

void LowsideCurrentSense::updateTemperatures() {
    if (_isset(pin_temp_m0)) {
        float adc_voltage = _readADCVoltageLowSide(pin_temp_m0, params);
        if (adc_voltage > 0.01f && adc_voltage < 3.29f) {
            float resistance = temp_series_resistance * (adc_voltage / (3.3f - adc_voltage));
            float steinhart = log(resistance / temp_nominal_resistance) / temp_beta_value + 1.0f / (25.0f + 273.15f);
            fet_temp_m0 = (1.0f / steinhart) - 273.15f;
        } else {
            fet_temp_m0 = -273.15f;
        }
    }
    if (_isset(pin_temp_aux)) {
        float adc_voltage = _readADCVoltageLowSide(pin_temp_aux, params);
        if (adc_voltage > 0.01f && adc_voltage < 3.29f) {
            float resistance = temp_series_resistance * (adc_voltage / (3.3f - adc_voltage));
            float steinhart = log(resistance / temp_nominal_resistance) / temp_beta_value + 1.0f / (25.0f + 273.15f);
            fet_temp_aux = (1.0f / steinhart) - 273.15f;
        } else {
            fet_temp_aux = -273.15f;
        }
    }
}


void LowsideCurrentSense::initBrakeResistorPWM(int pin, float target_voltage, float p_gain, float i_gain) {
    pin_brake_resistor = pin;
    brake_target_voltage = target_voltage;
    brake_p_gain = p_gain;
    brake_i_gain = i_gain;
    if (!_isset(pin_brake_resistor)) return;
    __HAL_RCC_TIM2_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    brake_timer_handle.Instance = TIM2;
    brake_timer_handle.Init.Prescaler = 0;
    brake_timer_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    brake_timer_handle.Init.Period = 1023;
    brake_timer_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&brake_timer_handle);
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&brake_timer_handle, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&brake_timer_handle, TIM_CHANNEL_4);
}

void LowsideCurrentSense::updateBrakeResistor() {
    if (!_isset(pin_brake_resistor) || brake_target_voltage == 0) return;
    float vbus = getVbusVoltage();
    float duty_cycle = 0;
    if (vbus > brake_target_voltage) {
        float error = vbus - brake_target_voltage;
        duty_cycle = error * brake_p_gain;
        brake_integrator += error * brake_i_gain;
        brake_integrator = _constrain(brake_integrator, 0.0f, 1.0f);
        duty_cycle += brake_integrator;
    } else {
        brake_integrator = 0;
        duty_cycle = 0;
    }
    duty_cycle = _constrain(duty_cycle, 0.0f, 0.95f);
    brake_duty_cycle = duty_cycle;
    __HAL_TIM_SET_COMPARE(&brake_timer_handle, TIM_CHANNEL_4, (uint32_t)(brake_duty_cycle * 1023));
}