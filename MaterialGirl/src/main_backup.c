#define STM32F767xx

#include <stdint.h>
#include "stm32f7xx.h"

#define STACK_TOP   ((uint32_t)0x20000800)
#define AF07        (0x7UL)
#define CH04        (0x4UL)

#define BUFFER_EMPTY    0
#define BUFFER_FULL     1

static void NMI_Handler(void);
static void HardFault_Handler(void);
static void Default_Handler(void);
static void USART3_DMA1_Stream1_Read(uint8_t *buffer, uint16_t length);
static void USART3_DMA1_Stream3_Write(char *data, uint16_t length);
static void DMA1_Stream1_IRQHandler(void);
static void DMA1_Stream3_IRQHandler(void);
static uint8_t Is_Buffer_Full(void);
static void Delay(int ms);
int main(void);

uint8_t tx_finished;
uint8_t rx_finished;
char uart_tx_data[40];
char uart_rx_data[20];
uint8_t counter = 0;
uint8_t length;

// Vector Table
uint32_t *myvectors[56] __attribute__ ((section("vectors"))) = {
    (uint32_t *) STACK_TOP,                 // Stack pointer
    (uint32_t *) main,                      // Code entry point
    (uint32_t *) NMI_Handler,               // NMI Handler
    (uint32_t *) HardFault_Handler,         // Hard fault handler
    (uint32_t *) Default_Handler,           // Memory management
    (uint32_t *) Default_Handler,           // Pre-fetch fault, memory access fault
    (uint32_t *) Default_Handler,           // Undefined instruction or illegal state
    (uint32_t *) Default_Handler,           // Reserved
    (uint32_t *) Default_Handler,           // Reserved
    (uint32_t *) Default_Handler,           // Reserved
    (uint32_t *) Default_Handler,           // Reserved
    (uint32_t *) Default_Handler,           // System service call via SWI instruction
    (uint32_t *) Default_Handler,           // Debug Monitor
    (uint32_t *) Default_Handler,           // Reserved
    (uint32_t *) Default_Handler,           // Pendable request for system service
    (uint32_t *) Default_Handler,           // System tick timer
    (uint32_t *) Default_Handler,           // Window Watchdog interrupt
    (uint32_t *) Default_Handler,           // PVD through EXTI line detection interrupt
    (uint32_t *) Default_Handler,           // Tamper and TimeStamp interrupts through the EXTI line
    (uint32_t *) Default_Handler,           // RTC Wakeup interrupt through the EXTI line
    (uint32_t *) Default_Handler,           // Flash global interrupt
    (uint32_t *) Default_Handler,           // RCC global interrupt
    (uint32_t *) Default_Handler,           // EXTI Line0 interrupt
    (uint32_t *) Default_Handler,           // EXTI Line1 interrupt
    (uint32_t *) Default_Handler,           // EXTI Line2 interrupt
    (uint32_t *) Default_Handler,           // EXTI Line3 interrupt
    (uint32_t *) Default_Handler,           // EXTI Line4 interrupt
    (uint32_t *) Default_Handler,           // DMA1 Stream0 global interrupt
    (uint32_t *) DMA1_Stream1_IRQHandler,   // DMA1 Stream1 global interrupt
    (uint32_t *) Default_Handler,           // DMA1 Stream2 global interrupt
    (uint32_t *) DMA1_Stream3_IRQHandler,   // DMA1 Stream3 global interrupt
    (uint32_t *) Default_Handler,           // DMA1 Stream4 global interrupt
    (uint32_t *) Default_Handler,           // DMA1 Stream5 global interrupt
    (uint32_t *) Default_Handler,           // DMA1 Stream6 global interrupt
    (uint32_t *) Default_Handler,           // ADC1, ADC2 and ADC3 global interrupts
    (uint32_t *) Default_Handler,           // CAN1 TX interrupts
    (uint32_t *) Default_Handler,           // CAN1 RX0 interrupts
    (uint32_t *) Default_Handler,           // CAN1 RX1 interrupt
    (uint32_t *) Default_Handler,           // CAN1 SCE interrupt
    (uint32_t *) Default_Handler,           // EXTI Line[9:5] interrupts
    (uint32_t *) Default_Handler,           // TIM1 Break interrupt and TIM9 global interrupt
    (uint32_t *) Default_Handler,           // TIM1 Update interrupt and TIM10 global interrupt
    (uint32_t *) Default_Handler,           // TIM1 Trigger and Commutation interrupts and TIM11 global interrupt
    (uint32_t *) Default_Handler,           // TIM1 Capture Compare interrupt
    (uint32_t *) Default_Handler,           // TIM2 global interrupt
    (uint32_t *) Default_Handler,           // TIM3 global interrupt
    (uint32_t *) Default_Handler,           // TIM4 global interrupt
    (uint32_t *) Default_Handler,           // I2C1 event interrupt
    (uint32_t *) Default_Handler,           // I2C1 error interrupt
    (uint32_t *) Default_Handler,           // I2C2 event interrupt
    (uint32_t *) Default_Handler,           // I2C2 error interrupt
    (uint32_t *) Default_Handler,           // SPI1 global interrupt
    (uint32_t *) Default_Handler,           // SPI2 global interrupt
    (uint32_t *) Default_Handler,           // USART1 global interrupt
    (uint32_t *) Default_Handler,           // USART2 global interrupt
    (uint32_t *) Default_Handler            // USART3 global interrupt
};

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;            // enable GPIOD clock
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;           // enable USART3 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;             // enable DMA1 clock

    GPIOD->MODER |= GPIO_MODER_MODER8_1;
    GPIOD->MODER |= GPIO_MODER_MODER9_1;

    GPIOD->AFR[1] |= (AF07 << 0);
    GPIOD->AFR[1] |= (AF07 << 4);

    USART3->BRR = 0x008B;
    USART3->CR1 = 0;
    USART3->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);
    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);

    DMA1_Stream1->CR &= ~(DMA_SxCR_EN);
    while((DMA1_Stream1->CR & (DMA_SxCR_EN)));

    DMA1_Stream3->CR &=~(DMA_SxCR_EN);
	while((DMA1_Stream3->CR &(DMA_SxCR_EN)));

    DMA1_Stream1->CR |= ((CH04 << 25)
                        |(DMA_SxCR_MINC)
                        |(DMA_SxCR_TCIE)
                        |(DMA_SxCR_CIRC));

    DMA1_Stream3->CR |= ((CH04 << 25)
                        |(DMA_SxCR_MINC)
                        |(DMA_SxCR_DIR_0)
                        |(DMA_SxCR_TCIE));

    DMA1_Stream1->PAR = (uint32_t) &USART3->RDR;
    DMA1_Stream3->PAR = (uint32_t) &USART3->TDR;

    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    char *hi = "hii\r\n";
    USART3_DMA1_Stream1_Read(&uart_rx_data, 20);

    while(1)
    {
        if (Is_Buffer_Full())
        {
            USART3_DMA1_Stream3_Write(uart_rx_data,20);
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
    while(tx_finished == 0);
    tx_finished = 0;

}

void USART3_DMA1_Stream1_Read(uint8_t *buffer, uint16_t length)
{
    DMA1_Stream1->M0AR = buffer;
    DMA1_Stream1->NDTR = length;
    DMA1_Stream1->CR |= DMA_SxCR_EN;
}

void DMA1_Stream3_IRQHandler(void)
{
	if(DMA1->LISR & DMA_LISR_TCIF3)
	{
		tx_finished = 1;
		DMA1->LIFCR = DMA_LIFCR_CTCIF3;
	}
}

void DMA1_Stream1_IRQHandler(void)
{
    if(DMA1->LISR & DMA_LISR_TCIF1)
    {
        rx_finished = 1;
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;

    }
}

uint8_t Is_Buffer_Full(void)
{
    if (rx_finished == 0)
    {
        return 0;
    }
    else
    {
        rx_finished = 0;
        return 1;
    }
}

void Delay(int ms)
{
	int i;
	SysTick->LOAD = 16000 - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x5;
	for (i = 0; i < ms; i++)
	{
		while(!(SysTick->CTRL & 0x10000)){}
	}
	SysTick->CTRL = 0;
}

