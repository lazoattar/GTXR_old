#define STM32F767xx

#include <stdint.h>
#include "stm32f7xx.h"

#define STACK_TOP   ((uint32_t)0x20000800)
#define AF07        (0x7UL)

static void NMI_Handler(void);
static void HardFault_Handler(void);
static void Default_Handler(void);
static void USART3_Write(unsigned char x);
static unsigned char USART3_Read(void);
static int USART3_Available(void);
int main(void);

// Vector Table
uint32_t *myvectors[52] __attribute__ ((section("vectors"))) = {
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
    (uint32_t *) Default_Handler,       // TIM4 global interrupt
    (uint32_t *) Default_Handler,       // I2C1 event interrupt
    (uint32_t *) Default_Handler,       // I2C1 error interrupt
    (uint32_t *) Default_Handler,       // I2C2 event interrupt
    (uint32_t *) Default_Handler,       // I2C2 error interrupt
    (uint32_t *) Default_Handler        // SPI1 global interrupt
};

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;           // enable GPIOD clock
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;          // enable USART3 clock

    GPIOD->MODER |= GPIO_MODER_MODER8_1;
    GPIOD->MODER |= GPIO_MODER_MODER9_1;

    GPIOD->AFR[1] |= (AF07 << 0);
    GPIOD->AFR[1] |= (AF07 << 4);

    USART3->BRR = 0x008B;
    USART3->CR1 = 0;
    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);

    while(1)
    {
        if (USART3_Available())
        {
            USART3_Write(USART3_Read());
        }
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

void USART3_Write(unsigned char x)
{
    USART3->TDR = (x);
    while(!((USART3->ISR) & USART_ISR_TC)){;}
}

unsigned char USART3_Read(void)
{
    return USART3->RDR;
}

int USART3_Available(void)
{
    if ((USART3->ISR) & USART_ISR_RXNE)
    {
        return 1;
    }

    else
    {
        return 0;
    }
}

