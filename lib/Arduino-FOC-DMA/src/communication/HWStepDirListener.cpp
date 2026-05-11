#include "HWStepDirListener.h"
#include "common/time_utils.h"

// ==============================================================================
// LOW-LEVEL HARDWARE INJECTION (Hidden from main.cpp)
// ==============================================================================
extern "C" void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  // If you ever add TIM2, TIM3, etc., you just add 'else if' blocks here
  if(htim_encoder->Instance == TIM1) 
  {
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
// ==============================================================================

HWStepDirListener::HWStepDirListener(TIM_TypeDef* timer_instance, float _counter_to_value) {
    htim.Instance = timer_instance; // Assign TIM1 (or others) here
    counter_to_value = _counter_to_value;
}

void HWStepDirListener::init() {
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    // Configure the Timer parameters
    htim.Init.Prescaler = 0;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = 65535;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.RepetitionCounter = 0;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    sConfig.EncoderMode = TIM_ENCODERMODE_CLOCKPLUSDIRECTION_X1;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 4;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 4;
    
    if (HAL_TIM_Encoder_Init(&htim, &sConfig) != HAL_OK) {
        Serial.println("ERR: TIM Encoder Init Failed");
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    
    if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK) {
        Serial.println("ERR: TIM Sync Failed");
    }

    // Start the STM32 timer in Encoder Mode
    HAL_TIM_Encoder_Start(&htim, TIM_CHANNEL_ALL);
    
    // Reset timer to 0 for homing
    htim.Instance->CNT = 0; 
    
    prev_count = 0;
    prev_update_time = _micros();
}

float HWStepDirListener::getValue() {
    int16_t hw_count = (int16_t)htim.Instance->CNT; // Use object '.' not pointer '->'
    return (float)hw_count * counter_to_value;
}

void HWStepDirListener::attach(float* pos_var, float* vel_var) {
    attached_position = pos_var;
    attached_velocity = vel_var;
}

void HWStepDirListener::update() {
    int16_t current_count = (int16_t)htim.Instance->CNT;
    int32_t current_time = _micros();

    if (attached_position) {
        *attached_position = (float)current_count * counter_to_value;
    }

    if (attached_velocity) {
        int32_t dt_micros = current_time - prev_update_time;
        if (dt_micros > 0) {
            int16_t delta_steps = current_count - prev_count; 
            float delta_pos = (float)delta_steps * counter_to_value;
            *attached_velocity = delta_pos / ((float)dt_micros * 1e-6f);
        }
    }

    prev_count = current_count;
    prev_update_time = current_time;
}