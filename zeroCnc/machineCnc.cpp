/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "machineCnc.h"
#include "serialManager.h"
#include "ZeroPi.h"

ZeroPi pi;

#define COORD_ABSOLUTE 0x0
#define COORD_RELATIVE 0x1
#define SEGLEN 0.5f

//porting from 3d printer frimware
typedef struct _move{
	float totalTime;
	float accFactor;
	float accTime; // accelerate until
	float decTime; // decelerate at
	int dStepX,dStepY,dStepZ,dStepE;
	int8_t motoXdir,motoYdir,motoZdir,motoEdir;
	uint32_t stepDelayX,stepDelayY,stepDelayZ,stepDelayE;
	unsigned char occupied;
	unsigned char checkEndStop;
	unsigned char disableMotor; // disable motor when move finished
	int index;	
	void * last;
	void * next;
}Move;

struct _cncMachine{
	uint8_t type;
	uint8_t cordType;
	uint8_t isMoving;
	float feedRate;
	float temperature;
	float tarTemp;
	float stpPerUnitX;
	float stpPerUnitY;
	float stpPerUnitZ;
	float stpPerUnitE;
	int8_t xDir;
	int8_t yDir;
	int8_t zDir;
	int8_t eDir;
	uint8_t xLimit;
	uint8_t yLimit;
	uint8_t zLimit;
	float tarX,tarY,tarZ,tarE;
	float curX,curY,curZ,curE;
	int32_t curStepX,curStepY,curStepZ,curStepE;
	float maxFeedRateX;
	float maxFeedRateY;
	float maxFeedRateZ;
	float maxFeedRateE;
	float jerkX;
	float jerkY;
	float jerkZ;
	float jerkE;
	float tempP;
	float tempI;
	float tempD;
	float deltaRadius;
	float rodLen;
}cnc;


#define LENMOVELIST 16
Move move[LENMOVELIST+1]; // the last slot for keep mem safe
Move * curMove;
Move * writeMove;

static void initMoveChain()
{
	int i;
	for(i=0;i<LENMOVELIST;i++){
		memset(&move[i],0,sizeof(Move));
		move[i].index = i;
		move[i].next = &move[i+1];
		move[i+1].last = &move[i];
	}
	move[0].last=&move[LENMOVELIST-1];
	move[LENMOVELIST-1].next=&move[0];
	curMove=&move[0];
	writeMove = &move[1];
}

void enableSteppers(int en)
{
	pi.stepperEnable(STP_X, en);
	pi.stepperEnable(STP_Y, en);
	pi.stepperEnable(STP_Z, en);
}

void updateStepperDelays(Move * move)
{
	if(move->stepDelayX){
		STP_X_TIMER->COUNT16.CC[0].reg = move->stepDelayX*6; // timer prescale to 6MHz
		STP_X_TIMER->COUNT16.COUNT.reg = 0;
	}
	if(move->stepDelayY){
		STP_Y_TIMER->COUNT16.CC[0].reg = move->stepDelayY*6;
		STP_Y_TIMER->COUNT16.COUNT.reg = 0;
	}
	if(move->stepDelayZ){
		STP_Z_TIMER->COUNT16.CC[0].reg = move->stepDelayZ*6;
		STP_Z_TIMER->COUNT16.COUNT.reg = 0;
	}
}

void checkMoveComlete()
{
	if(curMove->dStepX ==0 && curMove->dStepY ==0 && curMove->dStepZ ==0 && curMove->dStepE ==0){
		Move * next;
		if(curMove->disableMotor) enableSteppers(0);
		curMove->occupied=0;
		next = (Move*)curMove->next;
		if(next!=writeMove){
			if(next->occupied){
				updateStepperDelays(next);
			}
			//printf("#%d\r\n",next->index);
			COMMPORT.println(next->index);
			curMove = next;
		}
	}
}

void setTimers(int en)
{
	if(en){
		STP_X_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
		STP_Y_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
		STP_Z_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	}else{
		STP_X_TIMER->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
		STP_Y_TIMER->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
		STP_Z_TIMER->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;		
	}
}

//full advance, timer for each stepper
void initStepperTimer()
{
	uint32_t tmp;
	tmp = 48000000/8/1000; // 1k timer
	
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 )) ;
	while (GCLK->STATUS.bit.SYNCBUSY);

	STP_X_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
	STP_X_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
	STP_X_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	STP_X_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
	//STP_X_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	NVIC_EnableIRQ( STP_X_TIMER_IRQ) ;

	STP_Y_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
	STP_Y_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
	STP_Y_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	STP_Y_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
	//STP_Y_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;	
	NVIC_EnableIRQ( STP_Y_TIMER_IRQ) ;

	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 )) ;
	while (GCLK->STATUS.bit.SYNCBUSY);
	// todo: how to enable timer 6 7 in arduino ide
	STP_Z_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
	STP_Z_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
	STP_Z_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	STP_Z_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
	//STP_Z_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;	
	NVIC_EnableIRQ( STP_Z_TIMER_IRQ) ;

}


// X
void STP_X_TIMER_VECTOR(void){
    STP_X_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
	if(cnc.isMoving && curMove->dStepX>0){
		pi.stepperMove(STP_X,curMove->motoXdir);
		curMove->dStepX-=1;
	}
	checkMoveComlete();
}

// y
void STP_Y_TIMER_VECTOR(void){
    STP_Y_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
	if(cnc.isMoving && curMove->dStepY>0){
		pi.stepperMove(STP_Y,curMove->motoYdir);
		curMove->dStepY-=1;
	}
	checkMoveComlete();
}

// z
void STP_Z_TIMER_VECTOR(void){
    STP_Z_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
	if(cnc.isMoving && curMove->dStepZ>0){
		pi.stepperMove(STP_Z,curMove->motoZdir);
		curMove->dStepZ-=1;
	}
	checkMoveComlete();
}


void initMove(Move * move)
{
	memset(move,0,sizeof(Move)-12);
}

void startMoving(void)
{
	if(cnc.isMoving==1) return;
	enableSteppers(1);
	cnc.isMoving = 1;
	updateStepperDelays(curMove);
	setTimers(1);
}


void prepareMove(float tarX, float tarY, float tarZ, float tarE, float feedRate)
{
	int i,cnt;
	Move * next;
	int32_t numSeg;
	int32_t a,b,c,e;
	int32_t dxStep,dyStep,dzStep,deStep;
	float dx,dy,dz,de;
	float dTmax, distance;
	float dTx,dTy,dTz,dTe;
	float minStpSpdX,minStpSpdY,minStpSpdZ,minStpSpdE;

	if(!isnan(feedRate))
		cnc.feedRate = feedRate;
	cnc.tarX = cnc.curX;
	cnc.tarY = cnc.curY;
	cnc.tarZ = cnc.curZ;
	cnc.tarE = cnc.curE;
	if(cnc.cordType == COORD_RELATIVE){
		if(!isnan(tarX)) cnc.tarX = cnc.curX+tarX;
		if(!isnan(tarY)) cnc.tarY = cnc.curY+tarY;
		if(!isnan(tarZ)) cnc.tarZ = cnc.curZ+tarZ;
		if(!isnan(tarE)) cnc.tarE = cnc.curE+tarE;
	}else{
		if(!isnan(tarX)) cnc.tarX = tarX;
		if(!isnan(tarY)) cnc.tarY = tarY;
		if(!isnan(tarZ)) cnc.tarZ = tarZ;
		if(!isnan(tarE)) cnc.tarE = tarE;
	}
	dx = cnc.tarX - cnc.curX;
	dy = cnc.tarY - cnc.curY;
	dz = cnc.tarZ - cnc.curZ;
	de = cnc.tarE - cnc.curE;
	// segmenlize delta moves
	distance = sqrtf(dx*dx+dy*dy+dz*dz);
	numSeg = (int)(distance/SEGLEN)+1;
	dx = dx/numSeg;
	dy = dy/numSeg;
	dz = dz/numSeg;
	de = de/numSeg;
	//printf("\r\n---M %f %d---\r\n",distance,numSeg);
	COMMPORT.print("M ");COMMPORT.print(distance);COMMPORT.print(" ");COMMPORT.println(numSeg);

	for(i=0;i<numSeg;i++){

		a = (cnc.curX+dx)*cnc.stpPerUnitX;
		b = (cnc.curY+dy)*cnc.stpPerUnitY;
		c = (cnc.curZ+dz)*cnc.stpPerUnitZ;
		e = (cnc.curE+de)*cnc.stpPerUnitE;
		// calc steps
		dxStep = a-cnc.curStepX;
		dyStep = b-cnc.curStepY;
		dzStep = c-cnc.curStepZ;
		deStep = e-cnc.curStepE;
		
		minStpSpdX = min(cnc.feedRate,cnc.maxFeedRateX)*cnc.stpPerUnitX;
		minStpSpdY = min(cnc.feedRate,cnc.maxFeedRateY)*cnc.stpPerUnitY;
		minStpSpdZ = min(cnc.feedRate,cnc.maxFeedRateZ)*cnc.stpPerUnitZ;
		minStpSpdE = min(cnc.feedRate,cnc.maxFeedRateE)*cnc.stpPerUnitE;
		dTx = dxStep/minStpSpdX;
		dTy = dyStep/minStpSpdY;
		dTz = dzStep/minStpSpdZ;
		dTe = deStep/minStpSpdE;
		
		dTmax = max(max(abs(dTx),abs(dTy)),max(abs(dTz),abs(dTe)))*1000000; // based on us
		initMove(writeMove);
	
		writeMove->dStepX = abs(dxStep);
		writeMove->dStepY = abs(dyStep);
		writeMove->dStepZ = abs(dzStep);
		writeMove->dStepE = abs(deStep);
		writeMove->motoXdir = dxStep>0?cnc.xDir:-cnc.xDir;
		writeMove->motoYdir = dyStep>0?cnc.yDir:-cnc.yDir;
		writeMove->motoZdir = dzStep>0?cnc.zDir:-cnc.zDir;
		writeMove->motoEdir = deStep>0?cnc.eDir:-cnc.eDir;
		
		// sync move
		// us between steps
		writeMove->stepDelayX = dTmax/writeMove->dStepX;
		writeMove->stepDelayY = dTmax/writeMove->dStepY;
		writeMove->stepDelayZ = dTmax/writeMove->dStepZ;
		writeMove->stepDelayE = dTmax/writeMove->dStepE;
		
		writeMove->occupied = 1;
		cnc.curStepX = a;
		cnc.curStepY = b;
		cnc.curStepZ = c;
		cnc.curStepE = e;
		cnc.curX+=dx;
		cnc.curY+=dy;
		cnc.curZ+=dz;
		cnc.curE+=de;
		
		//printf("%d %d %d\r\n",writeMove->index,writeMove->dStepE,writeMove->stepDelayE);
		next = (Move*)writeMove->next;

		while(next==curMove){
			startMoving();
			//vTaskDelay(1/portTICK_PERIOD_MS);
		}
		writeMove = next;
	}
	if(cnc.isMoving==0){
		startMoving();
	}
}



void cncEchoVersion(void){
	COMMPORT.println("ZeroPI CNC BETA 0.1");

}

void cncInit(void){

	// setup on board resource
	pi.begin();
	pi.slotSetup(STP_X,SLOT_STEPPER); // X
	pi.slotSetup(STP_Y,SLOT_STEPPER); // Y
	pi.slotSetup(STP_Z,SLOT_STEPPER); // Z
	pi.slotSetup(SPINDDLE,SLOT_MOTOR); // spinddle

	pi.stepperSetResolution(STP_X,16,DRV8825);
	pi.stepperSetResolution(STP_Y,16,DRV8825);
	pi.stepperSetResolution(STP_Z,16,DRV8825);

	// init data struct
	initMoveChain();
	initStepperTimer();

	// init machine parameters
	cnc.feedRate = 30;
	cnc.xDir = 1;
	cnc.yDir = 1;
	cnc.zDir = 1;
	cnc.eDir = 1;
	cnc.maxFeedRateX = 50.0f;
	cnc.maxFeedRateY = 50.0f;
	cnc.maxFeedRateZ = 50.0f;
	cnc.maxFeedRateE = 45.0f;
	
	cnc.stpPerUnitX = 80;
	cnc.stpPerUnitY = 80;
	cnc.stpPerUnitZ = 80;
	cnc.stpPerUnitE = 95;
	
	
}




