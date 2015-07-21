#ifndef MACHINE_CNC_H
#define MACHINE_CNC_H

// config for cnc
#define BAUDRATE 115200
#define STP_X 0 
#define STP_Y 1
#define STP_Z 2
#define SPINDDLE 4

#define STP_X_TIMER TC4
#define STP_Y_TIMER TC5
#define STP_Z_TIMER TC6



// data struct
typedef struct{
	uint8_t type;
	uint8_t len;
	void * msg;
}Primitive;


void cncInit(void);
void cncEchoVersion(void);
void prepareMove(float tarX, float tarY, float tarZ, float tarE, float feedRate);



#endif

