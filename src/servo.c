#include "servo.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

void GPIO_init(void)
{
    RCC->AHB1ENR |= 1; //Enable GPIOA clock
         GPIOA->AFR[0] |= 0x00100000; // Select the PA5 pin in alternate function mode
        // GPIOA->MODER &=~ GPIO_MODER_MODER6_0; //Set the PA5 pin alternate function
         GPIOA->MODER |= 0x00000800;
          RCC->CFGR |= 0<<10; // set APB1 = 16 MHz

          TIM2_init();
                    TIM2->CR1 |= 1;
}

void TIM2_init(void)
{
// Implement PWM
    RCC->APB1ENR |=1;
    TIM2->PSC = 16-1; //Setting the clock frequency to 1MHz.
    TIM2->ARR = 20000; // Total period of the timer
    TIM2->CNT = 0;
    TIM2->CCMR1 = 0x0060; //PWM mode for the timer
    TIM2->CCER |= 1; //Enable channel 1 as output
    TIM2->CCR1 = 500; // Pulse width for PWM
}

void TIM4_ms_Delay(int delay)
{
    RCC->APB1ENR |= 1<<2; //Start the clock for the timer peripheral
    TIM4->PSC = 16000-1; //Setting the clock frequency to 1kHz.
    TIM4->ARR = (delay); // Total period of the timer
    TIM4->CNT = 0;
    TIM4->CR1 |= 1; //Start the Timer
    while(!(TIM4->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
    TIM4->SR &= ~(0x0001); //Reset the update interrupt flag
}

void SERVO_INIT(void)
{
	RCC->CFGR |= 0<<10; // set APB1 = 16 MHz
    GPIO_init();
    TIM2_init();
    TIM2->CR1 |= 1;

}

void SERVO (double delay1 )
{


        
             TIM2->CCR1=20000;
            TIM4_ms_Delay(delay1);
            TIM2->CCR1=0;
                     TIM4_ms_Delay(1);
        
    
} 
