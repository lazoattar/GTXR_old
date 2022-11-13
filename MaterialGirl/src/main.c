#define STM32H723xx
#define ARM_MATH_CM7

#include <stdint.h>
#include "stm32h7xx.h"
#include "stm32h7xx_ll_dma.h"
#include "arm_math.h"
#include "util/log.h"

#define STACK_TOP   ((uint32_t)0x20000800)
#define AF07        (0x7UL)
#define AF08        (0x8UL)

#define BUFFER_EMPTY    (0x0UL)
#define BUFFER_FULL     (0x1UL)

static void NMI_Handler(void);
static void HardFault_Handler(void);
static void Default_Handler(void);
static void USART3_DMA1_Stream3_Write(char *data, uint16_t length);
static void USART3_DMA1_Stream1_Read(uint8_t *buffer, uint16_t length);
static void DMA1_Stream1_IRQHandler(void);
static void DMA1_Stream3_IRQHandler(void);
static void UART8_DMA1_Stream6_Read(uint8_t *buffer, uint16_t length);
static void UART8_DMA1_Stream0_Write(char *data, uint16_t length);
static void DMA1_Stream6_IRQHandler(void);
static void DMA1_Stream0_IRQHandler(void);
static uint8_t Is_USART3_Buffer_Full(void);
static uint8_t Is_UART8_Buffer_Full(void);
int main(void);

uint8_t usart3_tx_finished;
uint8_t usart3_rx_finished;
uint8_t uart8_tx_finished;
uint8_t uart8_rx_finished;
char uart_tx_data[40];
char uart_rx_data[20];
char uart8_rx_data[1024];
char uart8_tx_data[128];
uint8_t counter = 0;
uint8_t length;

// Vector Table
uint32_t *myvectors[56] __attribute__ ((section("vectors"))) = {
    (uint32_t *) STACK_TOP,                                                 // Stack pointer
    (uint32_t *) main,                                                      // Code entry point
    (uint32_t *) NMI_Handler,                                               // NMI Handler
    (uint32_t *) HardFault_Handler,                                         // Hard fault handler
    (uint32_t *) Default_Handler,                                           // Memory management
    (uint32_t *) Default_Handler,                                           // Pre-fetch fault, memory access fault
    (uint32_t *) Default_Handler,                                           // Undefined instruction or illegal state
    (uint32_t *) Default_Handler,                                           // Reserved
    (uint32_t *) Default_Handler,                                           // Reserved
    (uint32_t *) Default_Handler,                                           // Reserved
    (uint32_t *) Default_Handler,                                           // Reserved
    (uint32_t *) Default_Handler,                                           // System service call via SWI instruction
    (uint32_t *) Default_Handler,                                           // Debug Monitor
    (uint32_t *) Default_Handler,                                           // Reserved
    (uint32_t *) Default_Handler,                                           // Pendable request for system service
    (uint32_t *) Default_Handler,                                           // System tick timer
    (uint32_t *) Default_Handler,                                           // Window Watchdog interrupt
    (uint32_t *) Default_Handler,                                           // PVD through EXTI line detection interrupt
    (uint32_t *) Default_Handler,                                           // Tamper and TimeStamp interrupts through the EXTI line
    (uint32_t *) Default_Handler,                                           // RTC Wakeup interrupt through the EXTI line
    (uint32_t *) Default_Handler,                                           // Flash global interrupt
    (uint32_t *) Default_Handler,                                           // RCC global interrupt
    (uint32_t *) Default_Handler,                                           // EXTI Line0 interrupt
    (uint32_t *) Default_Handler,                                           // EXTI Line1 interrupt
    (uint32_t *) Default_Handler,                                           // EXTI Line2 interrupt
    (uint32_t *) Default_Handler,                                           // EXTI Line3 interrupt
    (uint32_t *) Default_Handler,                                           // EXTI Line4 interrupt
    (uint32_t *) DMA1_Stream0_IRQHandler,                                   // DMA1 Stream0 global interrupt
    (uint32_t *) DMA1_Stream1_IRQHandler,                                   // DMA1 Stream1 global interrupt
    (uint32_t *) Default_Handler,                                           // DMA1 Stream2 global interrupt
    (uint32_t *) DMA1_Stream3_IRQHandler,                                   // DMA1 Stream3 global interrupt
    (uint32_t *) Default_Handler,                                           // DMA1 Stream4 global interrupt
    (uint32_t *) Default_Handler,                                           // DMA1 Stream5 global interrupt
    (uint32_t *) DMA1_Stream6_IRQHandler,                                   // DMA1 Stream6 global interrupt
    (uint32_t *) Default_Handler,                                           // ADC1, ADC2 and ADC3 global interrupts
    (uint32_t *) Default_Handler,                                           // CAN1 TX interrupts
    (uint32_t *) Default_Handler,                                           // CAN1 RX0 interrupts
    (uint32_t *) Default_Handler,                                           // CAN1 RX1 interrupt
    (uint32_t *) Default_Handler,                                           // CAN1 SCE interrupt
    (uint32_t *) Default_Handler,                                           // EXTI Line[9:5] interrupts
    (uint32_t *) Default_Handler,                                           // TIM1 Break interrupt and TIM9 global interrupt
    (uint32_t *) Default_Handler,                                           // TIM1 Update interrupt and TIM10 global interrupt
    (uint32_t *) Default_Handler,                                           // TIM1 Trigger and Commutation interrupts and TIM11 global interrupt
    (uint32_t *) Default_Handler,                                           // TIM1 Capture Compare interrupt
    (uint32_t *) Default_Handler,                                           // TIM2 global interrupt
    (uint32_t *) Default_Handler,                                           // TIM3 global interrupt
    (uint32_t *) Default_Handler,                                           // TIM4 global interrupt
    (uint32_t *) Default_Handler,                                           // I2C1 event interrupt
    (uint32_t *) Default_Handler,                                           // I2C1 error interrupt
    (uint32_t *) Default_Handler,                                           // I2C2 event interrupt
    (uint32_t *) Default_Handler,                                           // I2C2 error interrupt
    (uint32_t *) Default_Handler,                                           // SPI1 global interrupt
    (uint32_t *) Default_Handler,                                           // SPI2 global interrupt
    (uint32_t *) Default_Handler,                                           // USART1 global interrupt
    (uint32_t *) Default_Handler,                                           // USART2 global interrupt
    (uint32_t *) Default_Handler                                            // USART3 global interrupt
};

const float32_t A_f32[16] = {
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
};

float32_t AI_f32[4 * 4];

int main(void)
{
    LOG_FATAL("Hello %d", 123);

    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;                                    // enable GPIOE clock
    RCC->APB1LENR |= RCC_APB1LENR_UART8EN;                                    // enable UART8 clock
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;                                    // enable GPIOD clock
    RCC->APB1LENR |= RCC_APB1LENR_USART3EN;                                   // enable USART3 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;                                     // enable DMA1 clock

    GPIOD->MODER &= ~(GPIO_MODER_MODE8);
    GPIOD->MODER &= ~(GPIO_MODER_MODE9);

    GPIOD->MODER |= GPIO_MODER_MODE8_1;                                    // set PD8 to AF mode
    GPIOD->MODER |= GPIO_MODER_MODE9_1;                                    // set PD9 to AF mode

    GPIOE->MODER |= GPIO_MODER_MODE0_1;                                    // set PE0 to AF mode
    GPIOE->MODER |= GPIO_MODER_MODE1_1;                                    // set PE1 to AF mode

    GPIOD->AFR[1] |= (AF07 << 0);                                           // set PD8 to AF7 (USART3_TX)
    GPIOD->AFR[1] |= (AF07 << 4);                                           // set PD9 to AF8 (USART3_RX)

    GPIOE->AFR[0] |= (AF08 << 0);                                           // set PE0 to AF8 (UART8_RX)
    GPIOE->AFR[0] |= (AF08 << 4);                                           // set PE1 to AF8 (UART8_TX)

    USART3->BRR = 0x0683;
    USART3->CR1 = 0;
    USART3->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);
    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);

    UART8->BRR = 0x0683;
    UART8->CR1 = 0;
    UART8->CR3 |= (USART_CR3_DMAT |USART_CR3_DMAR);
    UART8->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);

    DMA1_Stream1->CR &= ~(DMA_SxCR_EN);
    while((DMA1_Stream1->CR & (DMA_SxCR_EN)));

    DMA1_Stream3->CR &=~(DMA_SxCR_EN);
	while((DMA1_Stream3->CR &(DMA_SxCR_EN)));

    DMA1_Stream1->CR |= ((DMA_SxCR_MINC)
                        |(DMA_SxCR_TCIE)
                        |(DMA_SxCR_CIRC));

    DMA1_Stream3->CR |= ((DMA_SxCR_MINC)
                        |(DMA_SxCR_DIR_0)
                        |(DMA_SxCR_TCIE));

    DMA1_Stream1->PAR = (uint32_t) &USART3->RDR;
    DMA1_Stream3->PAR = (uint32_t) &USART3->TDR;

    // Disable RTC write protection
    RTC->WPR = 0xCAU;
    RTC->WPR = 0x53U;

    // Set initialiation mode
    RTC->ISR = RTC_ISR_INIT_Msk;

    // Wait until initialization mode entered
    while ((RTC->ISR & RTC_ISR_INITF) == 0U);

    RTC->CR &= ~(RTC_CR_FMT | RTC_CR_OSEL | RTC_CR_POL);

    // Configure 24 hour time, no output, high output polarity
    RTC->CR |= RTC_CR_POL;

    LL_DMA_SetPeriphRequest(DMA1, 3, 46U);
    LL_DMA_SetPeriphRequest(DMA1, 1, 45U);

//    DMA1_Stream6->CR &= ~(DMA_SxCR_EN);
//    while((DMA1_Stream6->CR & (DMA_SxCR_EN)));
//
//    DMA1_Stream0->CR &=~(DMA_SxCR_EN);
//	while((DMA1_Stream0->CR &(DMA_SxCR_EN)));
//
//    DMA1_Stream6->CR |= ((DMA_SxCR_MINC)
//                        |(DMA_SxCR_TCIE)
//                        |(DMA_SxCR_CIRC));
//
//    DMA1_Stream0->CR |= ((DMA_SxCR_MINC)
//                        |(DMA_SxCR_DIR_0)
//                        |(DMA_SxCR_TCIE));
//
//    DMA1_Stream6->PAR = (uint32_t) &UART8->RDR;
//    DMA1_Stream0->PAR = (uint32_t) &UART8->TDR;


    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

//    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
//    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

//    UART8_DMA1_Stream6_Read(&uart8_rx_data, 1024);



    USART3_DMA1_Stream3_Write("hello\n", 6);
    USART3_DMA1_Stream1_Read(&uart_rx_data, 19);

    arm_matrix_instance_f32 A, AI;
    arm_status status;

    arm_mat_init_f32(&A, 4, 4, (float32_t *)A_f32);
    arm_mat_init_f32(&AI, 4, 4, (float32_t *)AI_f32);

    status = arm_mat_inverse_f32(&A, &AI);

    if (status == ARM_MATH_SUCCESS) {
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN; // Enable GPIOB clock
        GPIOB->MODER |= GPIO_MODER_MODE0_1; // Set to output mode
	    GPIOB->ODR |= GPIO_ODR_OD0; // Turn on pin
    }

    while(1)
    {
        if (Is_USART3_Buffer_Full())
        {
            USART3_DMA1_Stream3_Write(uart_rx_data, 19);
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

void USART3_DMA1_Stream3_Write(char *data, uint16_t length)
{
    DMA1_Stream3->M0AR = data;
    DMA1_Stream3->NDTR = length;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
    while(usart3_tx_finished == 0);
    usart3_tx_finished = 0;
}

void UART8_DMA1_Stream0_Write(char *data, uint16_t length)
{
    DMA1_Stream0->M0AR = data;
    DMA1_Stream0->NDTR = length;
    DMA1_Stream0->CR |= DMA_SxCR_EN;
    uart8_tx_finished = 0;
}

void USART3_DMA1_Stream1_Read(uint8_t *buffer, uint16_t length)
{
    DMA1_Stream1->M0AR = buffer;
    DMA1_Stream1->NDTR = length;
    DMA1_Stream1->CR |= DMA_SxCR_EN;
}

void UART8_DMA1_Stream6_Read(uint8_t *buffer, uint16_t length)
{
    DMA1_Stream6->M0AR = buffer;
    DMA1_Stream6->NDTR = length;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

void DMA1_Stream3_IRQHandler(void)
{
	if(DMA1->LISR & DMA_LISR_TCIF3)
	{
		usart3_tx_finished = 1;
		DMA1->LIFCR = DMA_LIFCR_CTCIF3;
	}
}

void DMA1_Stream0_IRQHandler(void)
{
	if(DMA1->LISR & DMA_LISR_TCIF0)
	{
		uart8_tx_finished = 1;
		DMA1->LIFCR = DMA_LIFCR_CTCIF0;
	}
}

void DMA1_Stream1_IRQHandler(void)
{
    if(DMA1->LISR & DMA_LISR_TCIF1)
    {
        usart3_rx_finished = 1;
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;

    }
}

void DMA1_Stream6_IRQHandler(void)
{
    if(DMA1->HISR & DMA_HISR_TCIF6)
    {
        uart8_rx_finished = 1;
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;
    }
}

uint8_t Is_USART3_Buffer_Full(void)
{
    if (usart3_rx_finished == 0)
    {
        return BUFFER_EMPTY;
    }
    else
    {
        usart3_rx_finished = 0;
        return BUFFER_FULL;
    }
}

uint8_t Is_UART8_Buffer_Full(void)
{
    if (uart8_rx_finished == 0)
    {
        return BUFFER_EMPTY;
    }
    else
    {
        uart8_rx_finished = 0;
        return BUFFER_FULL;
    }
}