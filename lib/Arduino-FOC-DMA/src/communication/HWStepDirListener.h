#ifndef HWSTEPDIR_H
#define HWSTEPDIR_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "stm32g4xx_hal.h" 

class HWStepDirListener
{
  public:
    /**
     * Constructor for hardware step/direction interface
     * @param timer_instance    - The hardware timer to use (e.g., TIM1)
     * @param counter_to_value  - step counter to value
     */
    HWStepDirListener(TIM_TypeDef* timer_instance, float counter_to_value = 1);

    void init();
    float getValue();
    void attach(float* pos_var, float* vel_var = nullptr);
    void update();

    TIM_HandleTypeDef htim;  // Stored as an object now, not a pointer
    float counter_to_value;  

  private:
    volatile float* attached_position = nullptr; 
    volatile float* attached_velocity = nullptr; 
    
    int32_t prev_update_time;
    int16_t prev_count; 
};

#endif