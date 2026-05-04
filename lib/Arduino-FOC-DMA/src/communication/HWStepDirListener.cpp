#include "HWStepDirListener.h"
#include "common/time_utils.h"

HWStepDirListener::HWStepDirListener(TIM_HandleTypeDef* _htim, float _counter_to_value) {
    htim = _htim;
    counter_to_value = _counter_to_value;
}

void HWStepDirListener::init() {
    // Start the STM32 timer in Encoder Mode
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    
    // Reset timer to 0 for homing
    htim->Instance->CNT = 0; 
    
    prev_count = 0;
    prev_update_time = _micros();
}

float HWStepDirListener::getValue() {
    // Cast the 16-bit register to a signed 16-bit integer
    // This allows seamless tracking of negative travel below 0
    int16_t hw_count = (int16_t)htim->Instance->CNT;
    return (float)hw_count * counter_to_value;
}

void HWStepDirListener::attach(float* pos_var, float* vel_var) {
    attached_position = pos_var;
    attached_velocity = vel_var;
}

void HWStepDirListener::update() {
    // 1. Read the hardware register
    int16_t current_count = (int16_t)htim->Instance->CNT;
    int32_t current_time = _micros();

    // 2. Update attached position
    if (attached_position) {
        *attached_position = (float)current_count * counter_to_value;
    }

    // 3. Calculate and update velocity
    if (attached_velocity) {
        int32_t dt_micros = current_time - prev_update_time;
        
        if (dt_micros > 0) {
            // Delta position (works correctly across 16-bit rollovers due to signed math)
            int16_t delta_steps = current_count - prev_count; 
            float delta_pos = (float)delta_steps * counter_to_value;
            
            // v = dp / dt (convert dt back to seconds)
            *attached_velocity = delta_pos / ((float)dt_micros * 1e-6f);
        }
    }

    // 4. Save state for next loop
    prev_count = current_count;
    prev_update_time = current_time;
}