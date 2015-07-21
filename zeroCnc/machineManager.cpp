/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include <stdlib.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "serialManager.h"
#include "machineManager.h"
#include "machineCnc.h"

QueueHandle_t macQ;


void parseCordinate(char * cmd)
{
  float feedRate=NAN;
  float tarX=NAN,tarY=NAN,tarZ=NAN,tarE=NAN;
  char * tmp;
  char * str;
	
  str = strtok_r(cmd, " ", &tmp);
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='X'){
      tarX = atof(str+1);
    }else if(str[0]=='Y'){
      tarY = atof(str+1);
    }else if(str[0]=='Z'){
      tarZ = atof(str+1);
    }else if(str[0]=='E'){
      tarE = atof(str+1);
    }else if(str[0]=='F'){
      feedRate = atof(str+1);
			feedRate/=60.0f; // change to mm/sec
    }
  }
  prepareMove(tarX,tarY,tarZ,tarE,feedRate);
  
}


void parseGcode(char * cmd)
{
  int code;
  code = atoi(cmd);
  switch(code){
	case 0:
    case 1: // xyz move
		parseCordinate(cmd);
		break;
    case 28: // home

		break;
	case 90: // absolute position

		break;
	case 91: // relative position

		break;
	case 92: // set length

		break;
	default:

		break;
  }
}

void parseMcode(char * cmd)
{
	int code;
	code = atoi(cmd);
	switch(code){
		default:

			break;
	}
}



void parseCmd(char * cmd)
{
  COMMPORT.print(">> ");
  COMMPORT.println(cmd);
  if(cmd[0]=='N'){
	cmd = strstr(cmd,"G"); // remove code index
  }
	
  if(cmd[0]=='G'){ // gcode
    parseGcode(cmd+1);
  }else if(cmd[0]=='M'){ // mcode
    parseMcode(cmd+1);    
  }
  COMMPORT.print("ok\r\n");
}

static void macThread( void *pvParameters )
{
	int i;
	Primitive prim;
	
	for(;;){
		while( xQueueReceive( macQ, &prim, portMAX_DELAY ) != pdPASS );
		parseCmd((char*)prim.msg);
	}
}


void machineInit(){
	cncInit();
	
	macQ = xQueueCreate( 20, sizeof(Primitive));
	xTaskCreate( macThread, "MAC", configMINIMAL_STACK_SIZE, NULL, 0, NULL );


}









