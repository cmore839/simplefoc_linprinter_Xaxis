#ifndef HWSTEPDIR_H
#define HWSTEPDIR_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "stm32g4xx_hal.h" // Required for the TIM_HandleTypeDef

class HWStepDirListener
{
  public:
    /**
     * Constructor for hardware step/direction interface
     *  @param htim              - Pointer to the hardware timer handle (e.g., &htim1)
     *  @param counter_to_value  - step counter to value
     */
    HWStepDirListener(TIM_HandleTypeDef* htim, float counter_to_value = 1);

    /**
     * Start the hardware encoder interface
     */
    void init();

    /**
     * Get absolute position directly from the hardware timer
     */
    float getValue();

    /**
     * Attach the variables to be updated
     */
    void attach(float* pos_var, float* vel_var = nullptr);

    /**
     * Poll this in the main control loop to update the attached variables
     * and calculate velocity via finite differencing.
     **/
    void update();

    TIM_HandleTypeDef* htim; //!< Hardware timer handle
    float counter_to_value;  //!< step counter to value 

  private:
    volatile float* attached_position = nullptr; //!< pointer to the attached position 
    volatile float* attached_velocity = nullptr; //!< pointer to the attached velocity 
    
    int32_t prev_update_time;
    int16_t prev_count; // 16-bit to match the timer
};

#endif