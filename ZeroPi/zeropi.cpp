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

}

ZeroPi::~ZeroPi(){


}

// timer setup for pwm output
void ZeroPi::begin(void){
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
void ZeroPi::motorInit(int slot){
	int n;
	for(n=0;n<SLOT_NUM_PINS;n++){
		SET_OUTPUT(slots[slot].pin[n]);
	}
	stepperEnable(slot,0);
	slots[slot].function = SLOT_MOTOR;
}

void ZeroPi::motorRun(int slot, int pwm1, int pwm2){
	pwm1 = constrain(pwm1, -255, 255);
	pwm2 = constrain(pwm2, -255, 255);
	
	if(pwm1>0){
		WRITE(slots[slot].pin[MOTOR_AIN1],1);
		WRITE(slots[slot].pin[MOTOR_AIN2],0);
	}else{
		WRITE(slots[slot].pin[MOTOR_AIN1],0);
		WRITE(slots[slot].pin[MOTOR_AIN2],1);
	}

	if(pwm2>0){
		WRITE(slots[slot].pin[MOTOR_BIN1],1);
		WRITE(slots[slot].pin[MOTOR_BIN2],0);
	}else{
		WRITE(slots[slot].pin[MOTOR_BIN1],0);
		WRITE(slots[slot].pin[MOTOR_BIN2],1);
	}
	slots[slot].value1 = abs(pwm1);
	slots[slot].value2 = abs(pwm2);
}



/******* EXT IO ***********************/
void ZeroPi::extInit(int index, int type){
	if(EXT_INPUT == type){
		SET_INPUT(extios[index].pin);
		extios[index].function = EXT_INPUT;
	}else if(EXT_OUTPUT == type){
		SET_OUTPUT(extios[index].pin);
		extios[index].function = EXT_OUTPUT;
	}else if(EXT_SERVO== type){
		SET_OUTPUT(extios[index].pin);
		extios[index].function = EXT_SERVO;
	}
}

int ZeroPi::extRead(int index){
	return READ(extios[index].pin);
}

void ZeroPi::extWrite(int index, int value){
	WRITE(extios[index].pin,value);
	extios[index].value = value;
}

void ZeroPi::extWriteUs(int index, int value){
	value = constrain(value,500,2500);
	extios[index].value = 	((BASEFREQ / 1000000) * value);
}

/***** Temperature Sensor ************/
void ZeroPi::tempInit(int index){

}

uint32_t ZeroPi::tempRead(int index){
	return analogRead(index);
}



/* Timer IRQ */
/**
This timer is called 3906 times per second. It is used to update
pwm values for heater and some other frequent jobs. 
*/
int test=0;
void PWM_TIMER_VECTOR ()
{
    static uint8_t pwm_count = 0;
	int n;
    PWM_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
	if(test){
		WRITE(A0,1);test=0;
	}else{
		WRITE(A0,0);test=1;
	}

	if(pwm_count==0){
		for(n=0;n<NUM_SLOTS;n++){
			if(ZeroPi::slots[n].function == SLOT_MOTOR){
				if(ZeroPi::slots[n].value1!=0)
					WRITE(ZeroPi::slots[n].pin[MOTOR_PWMA],1);
				if(ZeroPi::slots[n].value2!=0)
					WRITE(ZeroPi::slots[n].pin[MOTOR_PWMB],1);
			}
		}
	}

	for(n=0;n<NUM_SLOTS;n++){
		if(ZeroPi::slots[n].function == SLOT_MOTOR){
			if(ZeroPi::slots[n].value1==pwm_count)
				WRITE(ZeroPi::slots[n].pin[MOTOR_PWMA],0);
			if(ZeroPi::slots[n].value2==pwm_count)
				WRITE(ZeroPi::slots[n].pin[MOTOR_PWMB],0);
		}
	}

	pwm_count++; // max to 255
}



// Servo timer Interrupt handler
static uint8_t servoIndex = 0;
void SERVO_COMPA_VECTOR ()
{
    static uint32_t interval;
	int extPinIndex = servoIndex/2;
	int servoStage = servoIndex%2;
    SERVO_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;

	if(servoStage==0){
		if(ZeroPi::extios[extPinIndex].value>0){
			if(ZeroPi::extios[extPinIndex].function == EXT_SERVO) WRITE(ZeroPi::extios[extPinIndex].pin,1);
			interval = ZeroPi::extios[extPinIndex].value;
		}else{
			interval = SERVO2500US;
		}		
		SERVO_TIMER->COUNT16.CC[0].reg = interval;
	}else{
		if(ZeroPi::extios[extPinIndex].function == EXT_SERVO) WRITE(ZeroPi::extios[extPinIndex].pin,0);
		SERVO_TIMER->COUNT16.CC[0].reg = (SERVO5000US - interval);
	}
	
	servoIndex++;
	if(servoIndex>(NUM_EXTIO*2-1)) servoIndex = 0;
}


