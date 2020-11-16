//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myGPIOC_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void writeLCD(uint16_t);
void printResistance(char*);
void printFrequency(char*);
// Declare/initialize your global variables here...
// NOTE: You'll need at least one global variable
// (say, timerTriggered = 0 or 1) to indicate
// whether TIM2 has started counting or not.
int timerTriggered = 0;
float frequency;
char resistanceArray[4]; //allocated for 4 digits
char frequencyArray[4]; //allocated for 4 digits

int
main(int argc, char* argv[])
{

	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		//Initialize I/O port PA
	myGPIOB_Init();		//Initialize I/O port PB
	myGPIOC_Init();		//Initialize I/O port PC
	myTIM2_Init();		//Initialize timer TIM2
	myEXTI_Init();		//Initialize EXTI
	myADC_Init();		//Initialize ADC
	myDAC_Init();		//Initialize DAC

	//Initialize LCD
	//DDRAM access is performed using 8-bit interface
	//2 lines of 8 bit characters are displayed

	writeLCD(0x3800);
	//Display is on, no cursor, no blinking
	writeLCD(0x0C00);
	//DDRAM address incremented after each access, no shifting
	writeLCD(0x0600);
	//Clear display
	writeLCD(0x0100);

	while (1)
	{
		while(!(ADC1->ISR & ADC_ISR_EOC)){}; //wait until conversion is finished

		ADC1->ISR &= ~(ADC_ISR_EOC); //clear EOC flag

		uint32_t ADCVAL = ADC1->DR & 0xFFF; //masked ADC reg value to get right hand alignment
		float Resistance = ((float)(ADCVAL)-50)*1.2658; //calculate resistance from potentiometer voltage
		sprintf(resistanceArray, "%d", (int)Resistance);
		printResistance(resistanceArray);
		sprintf(frequencyArray, "%d", (int)frequency);
		printFrequency(frequencyArray);
		//trace_printf("ADC Value: %d\n", (uint32_t)Resistance);

		DAC->DHR12R1 = ADCVAL; //pass ADC input directly to DAC

		//Final
		//Read ADC Data register
		//Perform conversion calculation
		//Call LCD helper with read value
		//Write to DAC
	}

	return 0;

}


void myGPIOA_Init()
{
	// Enable clock for GPIOA peripheral
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA1 as input
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	// Ensure no pull-up/pull-down for PA1
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	//Configure PA4 as analog output
	GPIOA->MODER |= GPIO_MODER_MODER4;

	// Ensure no pull-up/pull-down for PA4
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
}

void myGPIOB_Init()
{
	// Enable clock for GPIOB peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	//Configure PB[15:8, 6:4] as Output, PB7 as input
	GPIOB->MODER |= 0x55551500;
	GPIOB->MODER &= ~(0xAAAAEA00);

	//Ensure no pull-up/pull-down for PB[15:4]
	GPIOB->PUPDR &= ~(0xFFFFF00);
}

void myGPIOC_Init(){
	// Enable clock for GPIOC peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	//Configure PC1 as analog
	GPIOC->MODER |= GPIO_MODER_MODER1;

	//Ensure no pull-up/pull-down for PC1
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
}


void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload bit 7 = 1, count up bit 4 = 0, stop on overflow bit 3 = 1,
	 * enable update events, interrupt on overflow only bit 2 = 1 */
	// Relevant register: TIM2->CR1
	//enable update events implicitly enabled in bit 2, but just on overflow/underflow
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Loads all our configured timer settings when this bit is written
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	// Do this after NVIC interrupts are configured
	TIM2->DIER |= TIM_DIER_UIE;
}


void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	// force bits 4-6 to zero, rest is unchanged
	SYSCFG->EXTICR[0] &= SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI2 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI2 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	// Declare/initialize your local variables here...
	float period;
	int currentTimer;

	/* Check if EXTI2 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//
		// 1. If this is the first edge: ie. timer has not been triggered
		if(!timerTriggered)
		{
			//	- Clear count register (TIM2->CNT).
			TIM2->CNT &= (uint32_t)0x00000000;
			//	- Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;
			timerTriggered = 1;
		}
		//    Else (this is the second edge):
		else
		{
			//	- Stop timer (TIM2->CR1).
			TIM2->CR1 &= ~(TIM_CR1_CEN);
			//	- Read out count register (TIM2->CNT).
			//	- Calculate signal period and frequency.
			currentTimer = TIM2->CNT;
			period = currentTimer/(float)48000000;
			frequency = 1/period;
			//	- Print calculated values to the console.
			timerTriggered = 0;
			//	  NOTE: Function trace_printf does not work
			//	  with floating-point numbers: you must use
			//	  "unsigned int" type to print your signal
			//	  period and frequency.

			//printFrequency(frequencyArray);
			//trace_printf("Frequency: %0.6f\n", frequency);
		}
		//
		// 2. Clear EXTI2 interrupt pending flag (EXTI->PR).
		// NOTE: A pending register (PR) bit is cleared
		// by writing 1 to it.
		EXTI->PR |= EXTI_PR_PR2;
	}
}

void myADC_Init()
{
	// Enable clock for ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	// Enable port 11 of the ADC, mapped to PC1
	ADC1->CHSELR |= ADC_CHSELR_CHSEL11;

	//Select maximum sampling time
	ADC1->SMPR |= ADC_SMPR_SMP;

	//Continuous conversion | Overrun mode
	ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);

	//Maximum resolution | Right alignment
	ADC1->CFGR1 &= ~(ADC_CFGR1_RES | ADC_CFGR1_ALIGN);

	//Enable ADC
	ADC1->CR |= ADC_CR_ADEN;

	//Wait until ADC is ready
	while(!(ADC1->ISR & ADC_ISR_ADRDY)){};

	//Start ADC, values can now be read from ADC_DR
	ADC1->CR |= ADC_CR_ADSTART;
}

void myDAC_Init(){
	//Enable clock for DAC
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	//Enable DAC
	DAC->CR |= DAC_CR_EN1;
}

void writeLCD(uint16_t inputValue){//send inputValue to LCD, assumed that any ascii conversion has been completed
	//send the 4 sequential instructions to LCD
	//clear port B[15:4]
	GPIOB->ODR &= ~(0xFFF0);
	//write first instruction to port B
	GPIOB->ODR |= inputValue;
	//assert enable
	GPIOB->ODR |= 0x10;
	//wait for LCD done flag to be asserted
	while(!(GPIOB->IDR & 0x80)){};
	//deassert enable
	GPIOB->ODR &= ~(0x10);
	//wait for LCD done flag to be deasserted
	while(GPIOB->IDR & 0x80){};
}

void printResistance(char output[]){
	//set second row and first column
	writeLCD(0xC000);
	//Write R
	writeLCD(0x5220);
	//write :
	writeLCD(0x3A20);
	for(int i = 0; i < sizeof(output); i++)
	{
		writeLCD((output[i]<<8) | 0x20);
	}
}

void printFrequency(char output[]){
	NVIC_DisableIRQ(EXTI0_1_IRQn);
	//set first row and column
	writeLCD(0x8000);
	//Write F
	writeLCD(0x4620);
	//write :
	writeLCD(0x3A20);
	for(int i = 0; i < sizeof(output); i++)
	{
		writeLCD((output[i]<<8) | 0x20);
	}
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
