#ifndef HARDWARE_UTILS_CURRENT_H
#define HARDWARE_UTILS_CURRENT_H

#include "../common/foc_utils.h"
#include "../common/time_utils.h"

// flag returned if current sense init fails
#define SIMPLEFOC_CURRENT_SENSE_INIT_FAILED ((void*)-1)

// generic implementation of the hardware specific structure
typedef struct GenericCurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
} GenericCurrentSenseParams;

float _readADCVoltageInline(const int pinA, const void* cs_params);
void* _configureADCInline(const void *driver_params, const int pinA,const int pinB,const int pinC = NOT_SET);

// UPDATED function signature
void* _configureADCLowSide(const void *driver_params, const int pinA,const int pinB,const int pinC, const int vbus_pin, const int temp_m0_pin, const int temp_aux_pin);

void _startADC3PinConversionLowSide();
float _readADCVoltageLowSide(const int pinA, const void* cs_params);
void* _driverSyncLowSide(void* driver_params, void* cs_params);

#endif