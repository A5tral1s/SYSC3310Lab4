#include "msp.h"

#include <stdio.h>

#define LED_OFF 0
#define LED_ON 1
#define RED_LED 0
#define RGB_LED 1

static uint8_t state = LED_OFF;
static int led = RED_LED;
static int index = 0;
static int active = 0;
static uint8_t overflow = ((1<<0)|(1<<1)|(1<<2));
uint16_t timer_values[4] = {24576, 49152, 0, 8192};

int main(){
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
	P1SEL0 &= (uint8_t)(~((1 << 4) | (1 << 1) | (1<<0)));
	P1SEL1 &= (uint8_t)(~((1 << 4) | (1 << 1) | (1<<0)));
	P2SEL0 &= (uint8_t)(~((1<<2)| (1<<1)| (1<<0)));
	P2SEL1 &= (uint8_t)(~((1<<2)| (1<<1)| (1<<0)));
	P1DIR &= (uint8_t)(~((1<<4) |(1<<1)));
	P1DIR |= (uint8_t)(1<<0);
	P2DIR |= (uint8_t)(((1<<2) |(1<<1) |(1<<0)));
	P1OUT |= (uint8_t)((1<<4)|(1<<1));
	P1OUT &= ~(uint8_t)(1<<0);
	P2OUT &= ~(uint8_t)(((1<<2) |(1<<1) |(1<<0)));
	P1REN |= (uint8_t)((1<<4)| (1<<1));
	P1IES |= (uint8_t)((1<<1)|(1<<4));
	P1IFG &= (uint8_t)(~((1<<1)|(1<<4)));
	P1IE |= (uint8_t)((1<<1)|(1<<4));
	TA0CTL &= (uint16_t)(~((1<<5)|(1<<4)));
	TA0CTL &= (uint16_t)(~(1<<0));
	TA0CCR0 = timer_values[0];
	TA0CTL |= (uint16_t)((1<<1));
	TA0CTL |= (uint16_t)((1<<8)|(1<<6));
	TA0CTL |= (uint16_t)((1<<4));
  	NVIC_SetPriority(PORT1_IRQn, 2);
  	NVIC_EnableIRQ(PORT1_IRQn);
  	NVIC_ClearPendingIRQ(PORT1_IRQn);
  	__ASM("CPSIE I");
  	NVIC_SetPriority(TA0_N_IRQn, 2);
  	NVIC_EnableIRQ(TA0_N_IRQn);
  	NVIC_ClearPendingIRQ(TA0_N_IRQn);

	
  	while(1){}
	return 0;
}

void TA0_N_IRQHandler(void){
	TA0CTL &= ~(uint16_t)((1<<0));
	if(active == 1){
		if(led == RED_LED){
			P1->OUT ^= (uint8_t)(1<<0);
		} else {
			P2->OUT &= ~overflow;
			state++;
			state &= overflow;
			P2->OUT |= state;
		}
	}
}

void PORT1_IRQHandler(void){
	active = 0;
	TAOCCR0 = 0;
	static int DELAY_VAL;
	DELAY_VAL = 5000;
	while(DELAY_VAL > 0){DELAY_VAL--;}
	if((P1IFG & (uint8_t)(1<<4)) != 0){
		P1IFG &= ~(uint8_t)(1<<4);
		index++;
		index = index%4;
		if(index == 2){
			P2->OUT &= ~(uint8_t)((1<<0)|(1<<1)|(1<<2));
			P1->OUT &= ~(uint8_t)(1<<0);
		} else{
			TA0CTL |= (uint16_t)(1<<4);
			active = 1;
		}
	}
	if((P1IFG & (uint8_t)(1<<1)) != 0){
		P1IFG &= ~(uint8_t)(1<<1);
		switch(led){
			case RED_LED:
				led = RGB_LED;
				break;
			case RGB_LED:
				led = RED_LED;
				break;
		}
		active = 1;
	}
	TA0CCR0 = timer_values[index];
}
