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
	uint16_t stepDelayX,stepDelayY,stepDelayZ,stepDelayE;
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

void enableSteppers(int en)
{


}

void setTimers(int en)
{
	if(en){
		
	}else{
		
	}
}

//full advance, timer for each stepper
void initStepperTimer()
{
	uint32_t tmp;
	tmp = 48000000/8/8000; // 8k timer
	
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 )) ;
	while (GCLK->STATUS.bit.SYNCBUSY);

	STP_X_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
	STP_X_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
	STP_X_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	STP_X_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
	STP_X_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

	STP_Y_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
	STP_Y_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
	STP_Y_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	STP_Y_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
	STP_Y_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC6_TC7 ));
	while (GCLK->STATUS.bit.SYNCBUSY);
	// todo: how to enable timer 6 7 in arduino ide
	STP_Z_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
	STP_Z_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
	STP_Z_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	STP_Z_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
	STP_Z_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

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

	if(feedRate!=NAN)
		cnc.feedRate = feedRate;
	cnc.tarX = cnc.curX;
	cnc.tarY = cnc.curY;
	cnc.tarZ = cnc.curZ;
	cnc.tarE = cnc.curE;
	if(cnc.cordType == COORD_RELATIVE){
		if(tarX!=NAN) cnc.tarX = cnc.curX+tarX;
		if(tarY!=NAN) cnc.tarY = cnc.curY+tarY;
		if(tarZ!=NAN) cnc.tarZ = cnc.curZ+tarZ;
		if(tarE!=NAN) cnc.tarE = cnc.curE+tarE;
	}else{
		if(tarX!=NAN) cnc.tarX = tarX;
		if(tarY!=NAN) cnc.tarY = tarY;
		if(tarZ!=NAN) cnc.tarZ = tarZ;
		if(tarE!=NAN) cnc.tarE = tarE;
	}
	dx = cnc.tarX - cnc.curX;
	dy = cnc.tarY - cnc.curY;
	dz = cnc.tarZ - cnc.curZ;
	de = cnc.tarE - cnc.curE;
	// segmenlize delta moves
	distance = sqrt(dx*dx+dy*dy+dz*dz);
	numSeg = (int)(distance/SEGLEN)+1;
	dx = dx/numSeg;
	dy = dy/numSeg;
	dz = dz/numSeg;
	de = de/numSeg;
	//printf("\r\n---M %f %d---\r\n",distance,numSeg);

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
	pi.begin();
	pi.slotSetup(STP_X,SLOT_STEPPER); // X
	pi.slotSetup(STP_Y,SLOT_STEPPER); // Y
	pi.slotSetup(STP_Z,SLOT_STEPPER); // Z

	pi.stepperSetResolution(0,16,DRV8825);
	pi.stepperSetResolution(1,16,DRV8825);


	
}




