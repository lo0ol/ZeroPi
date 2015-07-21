#include "Arduino.h"

#include "serialManager.h"
#include "machineCnc.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern QueueHandle_t macQ;

char rxBuf[64];
char msgBuf[64];
int rxIndex=0;
Primitive prim;


static void serialThread( void *pvParameters ){
	char c;
	cncEchoVersion();
	while(1){
		vTaskDelay(1/portTICK_PERIOD_MS);
		while(COMMPORT.available()){
			c =COMMPORT.read();
			rxBuf[rxIndex++] = c;
			if(c==0x0a){
				memcpy(msgBuf,rxBuf,rxIndex); // protect rxbuf
				prim.len = rxIndex;
				prim.msg = msgBuf;
				xQueueSend( macQ, &prim, NULL );
				memset(rxBuf,0,64);
				rxIndex = 0;			
			}
			if(rxIndex>=64){
				memset(rxBuf,0,64);
				rxIndex = 0;
			}
		}
	}
}


void serialInit(void){
	COMMPORT.begin(BAUDRATE);
	// start serial thread
	xTaskCreate( serialThread, "SERIAL", configMINIMAL_STACK_SIZE, NULL, 0, NULL );
	
}








