//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#include <stdio.h>
#include <string.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#define MAX_ADC_VALUE ((float) 0xFFF)
#define MAX_RESISTANCE_VALUE (5000.0)

#define myTIM2_PRESCALER ((uint16_t) 0x0000) // No prescaling
#define myTIM2_PERIOD ((uint32_t) 0xFFFFFFFF) // Max for overflow

#define myTIM3_PRESCALER (48000)
#define myTIM16_PRESCALER ((uint16_t)((SystemCoreClock - 1) / 1000))

#define LCD_LCK_PIN (GPIO_Pin_4) // Configure PB4 as latch clock LCK for 74HC595
#define LCD_UPDATE_PERIOD ((uint32_t) 500) // 500 ms

/* Prototypes */
void myGPIOA_Init();
void myADC_Init();
void myDAC_Init();
void myEXTI_Init();
void myTIM2_Init();

void EXTI0_1_IRQHandler();
void TIM2_IRQHandler();

uint32_t getADCValue();

void myGPIOB_Init();
void mySPI_Init(void);
void myLCD_Init(void);
void myTIM16_Init();
void myTIM3_Init();

void TIM16_IRQHandler();
void wait(uint32_t delay);

void LCD_UpdateFreq(void);
void LCD_UpdateResistance();
void LCD_SetAddress(uint8_t row, uint8_t col);
void HC595_Write(uint8_t word);
void LCD_Write(char c, int isCommand);
void LCD_WriteString(char * str);
void LCD_WriteCommand(char cmd);

/* Global variables */
float globalFreq = 0.0;
float globalResistance = 0.0;
unsigned int firstRisingEdge = 0;
unsigned int countRegister = 0;

int main(int argc, char* argv[]) {

	myGPIOA_Init(); /* Port for ADC, DAC, and NE555 timer input */
	myADC_Init();
	myDAC_Init();
	myEXTI_Init();
	myTIM2_Init(); /* Timer for measuring PWM period, freq */

	myGPIOB_Init(); /* Port for SPI */
	mySPI_Init();
	myTIM3_Init(); /* Timer for wait() function */
	myLCD_Init();	/* Initialize LCD settings (i.e. data length, number of lines, etc) */
	myTIM16_Init(); /* Timer for updating LCD every 0.5s */

	while (1) {

		uint32_t adcVal = getADCValue();

		/* Display converted data on the LCD */
		// DHR12R1: Data Holding Register, 12b, Right aligned, Channel 1
		DAC->DHR12R1 = adcVal;

		/* Convert ADC to resistance value */
		globalResistance = ((float) adcVal) / MAX_ADC_VALUE * MAX_RESISTANCE_VALUE;

        trace_printf("Resistance: %u Oh\n", (unsigned int) globalResistance);

	}
	return 0;
}

void myGPIOA_Init() {
	/* Enable clock for GPIOA peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA0 as analog input to ADC for resistance */
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1); /* No pull up/down */

	/* Configure PA1 as input */
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1); /* No pull up/down */

	/* Configure PA4 as analog out from DAC for optocoupler*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER4); /* Analog output */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4); /* No pull up/down */
}

void myGPIOB_Init() {
	// Turn on the GPIOB clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	// PB3 --AF0-> SPI MOSI
	// PB5 --AF0-> SPI SCK
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Configure the LCK pin for manual control in HC595_Write
	GPIO_InitStruct.GPIO_Pin = LCD_LCK_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void myADC_Init() {
	/* Enable clock for ADC */
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	// Make ADC self-calibrate
	ADC1->CR = ADC_CR_ADCAL;
	while (ADC1->CR == ADC_CR_ADCAL) {}

	/* ADC Configuration: Continuous conversion an Overrun mode */
	ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);

	/* Select channel, PA0 needs Channel 0 */
	ADC1->CHSELR = ADC_CHSELR_CHSEL0;

	/* Enable ADC and wait for it to have ready status */
	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}
}

void myDAC_Init() {
	/* Enable clock for DAC */
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	/* Enable DAC */
	DAC->CR |= DAC_CR_EN1;
}

void myEXTI_Init() {
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);

}

void EXTI0_1_IRQHandler() {

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0) {

		// 1. If this is the first edge:
		if (firstRisingEdge == 0) {
			firstRisingEdge = 1;

			// Clear count register (TIM2->CNT).
			TIM2->CNT = 0x00000000;

			// Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;

		// Else this is the second edge
		} else if(firstRisingEdge == 1){
			// Else this is the second edge
			firstRisingEdge = 0;

			// Stop timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_UDIS;

			// Read out count register (TIM2->CNT).
			countRegister = TIM2->CNT;

			// Calculate signal period and frequency.
			globalFreq = ((float) SystemCoreClock) / ((float) countRegister);

			trace_printf("Frequency: %f Hz\n", globalFreq);

		}

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR |= EXTI_PR_PR1;
	}
}

void myTIM2_Init() {
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t) 0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t) 0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
}

void TIM2_IRQHandler() {
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0) {
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

void myTIM3_Init() {
	// Enable timer clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/* Configure TIM3: buffer auto-reload, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM3->CR1 = ((uint16_t) 0x8C);

	TIM3->PSC = myTIM3_PRESCALER;
	TIM3->ARR = 100;

	// Update timer registers.
	TIM3->EGR |= 0x0001;
}

void myTIM16_Init() {
	/* Enable clock for TIM16 peripheral */
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

	/* Configure TIM16: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM16->CR1 = ((uint16_t) 0x008C);

	/* Set clock prescaler value */
	TIM16->PSC = myTIM16_PRESCALER;

	/* Set auto-reloaded delay */
	TIM16->ARR = LCD_UPDATE_PERIOD;

	/* Update timer registers */
	TIM16->EGR = ((uint16_t) 0x0001);

	/* Assign TIM16 interrupt priority = 1 in NVIC */
	NVIC_SetPriority(TIM16_IRQn, 1);

	/* Enable TIM16 interrupts in NVIC */
	NVIC_EnableIRQ(TIM16_IRQn);

	/* Enable update interrupt generation */
	TIM16->DIER |= TIM_DIER_UIE;

	/* Activate timer! */
	TIM16->CR1 |= TIM_CR1_CEN;
}

/* Update the LCD every 500ms */
void TIM16_IRQHandler() {
	/* Check if interrupt flag is indeed set */
	if ((TIM16->SR & TIM_SR_UIF) != 0) {

		LCD_UpdateResistance();
		LCD_UpdateFreq();

		/* Clear UIF */
		TIM16->SR &= ~(TIM_SR_UIF);

		/* Restart timer */
		TIM16->CR1 |= TIM_CR1_CEN;
	}

}

uint32_t getADCValue() {
	/* Start ADC conversion */
	ADC1->CR |= ADC_CR_ADSTART;

	/* Wait for End Of Conversion flag to be set */
	while (!(ADC1->ISR & ADC_ISR_EOC)) {}

	/* Reset End Of Conversion flag */
	ADC1->ISR &= ~(ADC_ISR_EOC);

	/* Apply the data mask to the data register */
	return ((ADC1->DR) & ADC_DR_DATA);
}

void mySPI_Init() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_InitTypeDef SPI_InitStruct;
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE);
}

void myLCD_Init() {
	LCD_WriteCommand(0x2); /* Set LCD to 4-bit mode */
	wait(1);

	LCD_WriteCommand(0x28); // DL=0, N=1, F=0
	LCD_WriteCommand(0x0C); // D=1, C=0, B=0
	LCD_WriteCommand(0x06); // I/D=1, S=0
	LCD_WriteCommand(0x1);  // Clear LCD
	wait(1);

	LCD_SetAddress(1, 7);
	LCD_WriteString("Hz");

	LCD_SetAddress(2, 7);
	LCD_WriteString("Oh");
}

/* Update the frequency value on the LCD */
void LCD_UpdateFreq() {
	if (globalFreq < 1) {
		LCD_SetAddress(1, 1);
		LCD_WriteString("F:<1 ");
	} else {
		char * suffix = "";
		// Changed suffixes
		if (globalFreq < 10) {
			suffix = "   ";
		} else if (globalFreq < 100) {
			suffix = "  ";
		} else if (globalFreq < 1000) {
			suffix = " ";
		}

		char* format = "F:%.0f%s";

		char finalString[6]; // Was 5, changed to 6
		sprintf(finalString, format, globalFreq, suffix);

		LCD_SetAddress(1, 1);
		LCD_WriteString(finalString);
		//LCD_WriteString(suffix); Commented out
	}
}

/* Update the resistance value on the LCD */
void LCD_UpdateResistance() {
	if (globalResistance < 1) {
		LCD_SetAddress(2, 1);
		LCD_WriteString("R:<1 ");
	} else {
		char * suffix = "";

		if (globalResistance < 10) {
			suffix = "   ";
		} else if (globalResistance < 100){
			suffix = "  ";
		} else if (globalResistance < 1000){
			suffix = " ";
		}

		char* format = "R:%.0f%s";

		char finalString[6];
		sprintf(finalString, format, globalResistance, suffix);

		LCD_SetAddress(2, 1);
		LCD_WriteString(finalString);
	}
}

/* Set DDRAM address that we want to write data to */
/* Rows are 1,2 and cols are 1-8 */
void LCD_SetAddress(uint8_t row, uint8_t col) {
	uint8_t rowAddr = (row==1) ? 0x0 : 0x40;
	uint8_t colAddr = col - 1;

	uint8_t moveCursorCommand = 0x80 | rowAddr | colAddr;
	LCD_WriteCommand(moveCursorCommand);
}

/* Send a byte of data to the shift register via SPI */
void HC595_Write(uint8_t data) {
	GPIOB->BRR = LCD_LCK_PIN;

	// Poll SPI until its ready to receive data
	while ((SPI1->SR & SPI_SR_BSY) != 0) {
	}

	SPI_SendData8(SPI1, data);

	// Poll SPI until its no longer busy transmitting data
	while ((SPI1->SR & SPI_SR_BSY) != 0) {
	}

	/* Force LCK signal to 1 */
	GPIOB->BSRR = LCD_LCK_PIN;
}

/* Wait for delay milliseconds */
void wait(uint32_t delay) {
	// Clear timer
	TIM3->CNT |= 0x0;

	// Set timeout
	TIM3->ARR = delay;

	// Update timer registers
	TIM3->EGR |= 0x0001;

	// Start the timer
	TIM3->CR1 |= TIM_CR1_CEN;

	// Wait until update interrupt flag is set
	while (!(TIM3->SR & 0x0001)) { }

	// Stop the timer
	TIM3->CR1 &= ~(TIM_CR1_CEN);

	// Reset the interrupt flag
	TIM3->SR &= ~(TIM_SR_UIF);
}

/* This is called by LCD_WriteString and LCD_WriteCommand to write data to the LCD */
/* Sends a byte of data to the shift register 4 bits at a time */
void LCD_Write(char c, int isCommand){
	char disable = 0x00;
	char enable = 0x80;
	char RS = (isCommand) ? 0x00 : 0x40;

	char low = c & 0x0F;
	char high = (c >> 4) & 0x0F;

	HC595_Write(disable | RS | high);
	HC595_Write(enable | RS | high);
	HC595_Write(disable | RS | high);

	HC595_Write(disable | RS | low);
	HC595_Write(enable | RS | low);
	HC595_Write(disable | RS | low);
}

/* Write a string to display on the LCD one byte at a time */
void LCD_WriteString(char * str) {
	while (*str) {
		LCD_Write(*(str++), 0);
	}
}

/* Write a command to the LCD */
void LCD_WriteCommand(char cmd) {
	LCD_Write(cmd, 1);
}

#pragma GCC diagnostic pop
