#ifndef MACHINE_CNC_H
#define MACHINE_CNC_H

#define BASEFREQ (48000000/8)
// config for cnc
#define BAUDRATE 115200
#define STP_X 0 
#define STP_Y 1
#define STP_Z 2
#define SPINDDLE 3

#define STP_X_TIMER TC4
#define STP_X_TIMER_IRQ           TC4_IRQn
#define STP_X_TIMER_VECTOR        TC4_Handler

#define STP_Y_TIMER TC5
#define STP_Y_TIMER_IRQ           TC5_IRQn
#define STP_Y_TIMER_VECTOR        TC5_Handler

#define STP_Z_TIMER TC3
#define STP_Z_TIMER_IRQ           TC3_IRQn
#define STP_Z_TIMER_VECTOR        TC3_Handler




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

