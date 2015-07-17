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

void ZeroPi::stepperSetResolution(int s, int res, DRIVER_t type)
{
	switch (type)
	{
	case A4982://only MS1 MS2
		switch (res)
		{
		case 1:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],0);
			break;
		case 2:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],0);
			break;
		case 4:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],1);
			break;
		case 16:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],1);
			break;
		default:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],0);
			break;
		}
		break;
	case A4988:
		switch (res)
		{
		case 1:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],0);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		case 2:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],0);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		case 4:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],1);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		case 8:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],1);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		case 16:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],1);
			WRITE(slots[s].pin[STEP_MS3],1);
			break;
		default:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],0);
			WRITE(slots[s].pin[STEP_MS3],0);		
			break;
		}
		break;
	case DRV8825:
		switch (res)
		{
		case 1:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],0);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		case 2:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],0);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		case 4:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],1);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		case 8:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],1);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		case 16:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],0);
			WRITE(slots[s].pin[STEP_MS3],1);
			break;
		case 32:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],1);
			WRITE(slots[s].pin[STEP_MS3],1);
			break;
		default:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],0);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		}
		break;
	case TB67S269:
	case TB67S109:
		switch (res)
		{
		case 1:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],0);
			WRITE(slots[s].pin[STEP_MS3],1);
			break;
		case 2://Half step resolution(Type (A))
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],1);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		case 4:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],1);
			WRITE(slots[s].pin[STEP_MS3],1);
			break;
		case 8:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],0);
			WRITE(slots[s].pin[STEP_MS3],1);
			break;
		case 16:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],1);
			WRITE(slots[s].pin[STEP_MS3],0);
			break;
		case 32:
			WRITE(slots[s].pin[STEP_MS1],1);
			WRITE(slots[s].pin[STEP_MS2],1);
			WRITE(slots[s].pin[STEP_MS3],1);
			break;
		default:
			WRITE(slots[s].pin[STEP_MS1],0);
			WRITE(slots[s].pin[STEP_MS2],0);
			WRITE(slots[s].pin[STEP_MS3],1);
			break;
		}
		break;
	default:
		break;
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
//	stepperEnable(slot,0);
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
	pwm1 = abs(pwm1);
	pwm2 = abs(pwm2);
	
	analogWrite(slots[slot].pin[MOTOR_PWMA],pwm1);
	analogWrite(slots[slot].pin[MOTOR_PWMB],pwm2);
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
//	value = constrain(value,500,2500);
//	extios[index].value = 	((BASEFREQ / 1000000) * value);
}

/***** Temperature Sensor ************/
void ZeroPi::tempInit(int index){

}

uint32_t ZeroPi::tempRead(int index){
	return analogRead(index);
}
