#include "Arduino.h"
#include "zeropi.h"


SLOT ZeroPi::slots[NUM_SLOTS]={
	{
		SLOT_NULL,
		{44,45,46,47,4,3}
	},
	{
		SLOT_NULL,
		{48,49,50,51,6,7}
	},
	{
		SLOT_NULL,
		{52,53,54,55,12,36}
	},
	{
		SLOT_NULL,
		{11,5,2,38,8,9}
	}
};

EXTIO ZeroPi::extios[NUM_EXTIO]={
	{EXT_NULL, A3, 0},
	{EXT_NULL, A2, 0},
	{EXT_NULL, A1, 0},
	{EXT_NULL, A0, 0},
	{EXT_NULL, MO, 0},
	{EXT_NULL, MI, 0},
	{EXT_NULL, SCK, 0},
	{EXT_NULL, SDA, 0},
	{EXT_NULL, SCL, 0},
	{EXT_NULL, 0, 0},
	{EXT_NULL, 1, 0}
};


ZeroPi::ZeroPi(){
	timerSetup();
}

ZeroPi::~ZeroPi(){


}

// timer setup for pwm output
void ZeroPi::timerSetup(void){
	uint32_t tmp;
	
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 )) ;
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	// Timer for servo pulse output
	tmp = BASEFREQ/SERVO_CLOCK_FREQ;
	SERVO_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
	SERVO_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
	SERVO_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	SERVO_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
	SERVO_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	NVIC_EnableIRQ( SERVO_TIMER_IRQ ) ;

	// Timer for dc motor pwm output
	tmp = BASEFREQ/PWM_CLOCK_FREQ;
	PWM_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
	PWM_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
	PWM_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	PWM_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
	PWM_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	NVIC_EnableIRQ( PWM_TIMER_IRQ ) ;
	

}



void ZeroPi::slotSetup(int slot, int fun){
	if(fun==SLOT_STEPPER){
		stepperInit(slot);
	}else if(fun==SLOT_MOTOR){
		//motorInit(slot);
	}

}



/******* STEPPERS *******************/ 
void ZeroPi::stepperInit(int slot){
	int n;
	for(n=0;n<SLOT_NUM_PINS;n++){
		SET_OUTPUT(slots[slot].pin[n]);
	}
	stepperEnable(slot,0);
	slots[slot].function = SLOT_STEPPER;
}

void ZeroPi::stepperEnable(int slot,int en){
	if(en){
		WRITE(slots[slot].pin[STEP_EN],1);
	}else{
		WRITE(slots[slot].pin[STEP_EN],0);
	}
}

void ZeroPi::stepperSetResolution(int s, int res)
{
	if(res==2){ // half
		WRITE(slots[s].pin[STEP_MS1],0);
		WRITE(slots[s].pin[STEP_MS2],1);
		WRITE(slots[s].pin[STEP_MS3],1);
	}else if(res==4){ // quarter
		WRITE(slots[s].pin[STEP_MS1],0);
		WRITE(slots[s].pin[STEP_MS2],1);
		WRITE(slots[s].pin[STEP_MS3],0);
	}else if(res==8){ // 8th
		WRITE(slots[s].pin[STEP_MS1],1);
		WRITE(slots[s].pin[STEP_MS2],1);
		WRITE(slots[s].pin[STEP_MS3],0);
	}else if(res==16){ // 16th
		WRITE(slots[s].pin[STEP_MS1],1);
		WRITE(slots[s].pin[STEP_MS2],1);
		WRITE(slots[s].pin[STEP_MS3],1);
	}else{
		WRITE(slots[s].pin[STEP_MS1],0);
		WRITE(slots[s].pin[STEP_MS2],0);
		WRITE(slots[s].pin[STEP_MS3],0);
	}
}

void ZeroPi::stepperMove(int s, int dir)
{
	if(dir>0){
		WRITE(slots[s].pin[STEP_DIR],1);
		WRITE(slots[s].pin[STEP_STP],1);
		delayMicroseconds(1);
		WRITE(slots[s].pin[STEP_STP],0);
	}else{
		WRITE(slots[s].pin[STEP_DIR],0);
		WRITE(slots[s].pin[STEP_STP],1);
		delayMicroseconds(1);
		WRITE(slots[s].pin[STEP_STP],0);
	}
}


/******* DC MOTORS *******************/ 

/******* EXT IO ***********************/
void ZeroPi::extInit(int index, int type){


}

int ZeroPi::extRead(int index){


}

void ZeroPi::extWrite(int index, int value){


}



/* Timer IRQ */
/**
This timer is called 3906 times per second. It is used to update
pwm values for heater and some other frequent jobs. 
*/
void PWM_TIMER_VECTOR ()
{
    PWM_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;


}



// Servo timer Interrupt handler
void SERVO_COMPA_VECTOR ()
{
    SERVO_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;




}


