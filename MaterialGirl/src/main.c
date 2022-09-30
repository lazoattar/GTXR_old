#define STM32F767xx

#include <stdint.h>
#include "stm32f7xx.h"

#define STACK_TOP   ((uint32_t)0x20000800)

static void NMI_Handler(void);
static void HardFault_Handler(void);
static void Default_Handler(void);
static void GPIO_TogglePin(GPIO_TypeDef *GPIOPort, int GPIOPin);
int main(void);

// Vector Table
uint32_t *myvectors[47] __attribute__ ((section("vectors"))) = {
    (uint32_t *) STACK_TOP,             // Stack pointer
    (uint32_t *) main,                  // Code entry point
    (uint32_t *) NMI_Handler,           // NMI Handler
    (uint32_t *) HardFault_Handler,     // Hard fault handler
    (uint32_t *) Default_Handler,       // Memory management
    (uint32_t *) Default_Handler,       // Pre-fetch fault, memory access fault
    (uint32_t *) Default_Handler,       // Undefined instruction or illegal state
    (uint32_t *) Default_Handler,       // Reserved
    (uint32_t *) Default_Handler,       // Reserved
    (uint32_t *) Default_Handler,       // Reserved
    (uint32_t *) Default_Handler,       // Reserved
    (uint32_t *) Default_Handler,       // System service call via SWI instruction
    (uint32_t *) Default_Handler,       // Debug Monitor
    (uint32_t *) Default_Handler,       // Reserved
    (uint32_t *) Default_Handler,       // Pendable request for system service
    (uint32_t *) Default_Handler,       // System tick timer
    (uint32_t *) Default_Handler,       // Window Watchdog interrupt
    (uint32_t *) Default_Handler,       // PVD through EXTI line detection interrupt
    (uint32_t *) Default_Handler,       // Tamper and TimeStamp interrupts through the EXTI line
    (uint32_t *) Default_Handler,       // RTC Wakeup interrupt through the EXTI line
    (uint32_t *) Default_Handler,       // Flash global interrupt
    (uint32_t *) Default_Handler,       // RCC global interrupt
    (uint32_t *) Default_Handler,       // EXTI Line0 interrupt
    (uint32_t *) Default_Handler,       // EXTI Line1 interrupt
    (uint32_t *) Default_Handler,       // EXTI Line2 interrupt
    (uint32_t *) Default_Handler,       // EXTI Line3 interrupt
    (uint32_t *) Default_Handler,       // EXTI Line4 interrupt
    (uint32_t *) Default_Handler,       // DMA1 Stream0 global interrupt
    (uint32_t *) Default_Handler,       // DMA1 Stream1 global interrupt
    (uint32_t *) Default_Handler,       // DMA1 Stream2 global interrupt
    (uint32_t *) Default_Handler,       // DMA1 Stream3 global interrupt
    (uint32_t *) Default_Handler,       // DMA1 Stream4 global interrupt
    (uint32_t *) Default_Handler,       // DMA1 Stream5 global interrupt
    (uint32_t *) Default_Handler,       // DMA1 Stream6 global interrupt
    (uint32_t *) Default_Handler,       // ADC1, ADC2 and ADC3 global interrupts
    (uint32_t *) Default_Handler,       // CAN1 TX interrupts
    (uint32_t *) Default_Handler,       // CAN1 RX0 interrupts
    (uint32_t *) Default_Handler,       // CAN1 RX1 interrupt
    (uint32_t *) Default_Handler,       // CAN1 SCE interrupt
    (uint32_t *) Default_Handler,       // EXTI Line[9:5] interrupts
    (uint32_t *) Default_Handler,       // TIM1 Break interrupt and TIM9 global interrupt
    (uint32_t *) Default_Handler,       // TIM1 Update interrupt and TIM10 global interrupt
    (uint32_t *) Default_Handler,       // TIM1 Trigger and Commutation interrupts and TIM11 global interrupt
    (uint32_t *) Default_Handler,       // TIM1 Capture Compare interrupt
    (uint32_t *) Default_Handler,       // TIM2 global interrupt
    (uint32_t *) Default_Handler,       // TIM3 global interrupt
    (uint32_t *) Default_Handler        // TIM4 global interrupt
};

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // setup clock for GPIOB

	GPIOB->MODER |= GPIO_MODER_MODER7_0;
	GPIOB->MODER &= ~GPIO_MODER_MODER7_1;

    while(1)
    {
        GPIOB->ODR |= GPIO_ODR_OD7;
		//delay for a little bit
		for (volatile int i=0; i<1000000; i++);
		//set PB7 back to zero
		GPIOB->ODR &= ~GPIO_ODR_OD7;
		//delay for a little bit
		for (volatile int i=0; i<1000000; i++);
    }
}

void NMI_Handler(void)
{
    for(;;);
}

void HardFault_Handler(void)
{
    for(;;);
}

void Default_Handler(void)
{
    for(;;);
}

void GPIO_TogglePin(GPIO_TypeDef *GPIOPort, int GPIOPin)
{
    uint32_t currentButtonState;
    currentButtonState = GPIOPort->ODR & ((uint32_t)0x01<<GPIOPin);

    if(currentButtonState != 0)
    {
        GPIOPort->ODR = GPIOPort->ODR & ~((uint32_t)0x01<<GPIOPin);
    }
    else
    {
        GPIOPort->ODR = GPIOPort->ODR | ((uint32_t)0x01<<GPIOPin);
    }
}

