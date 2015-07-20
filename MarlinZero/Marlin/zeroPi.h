#ifndef ZEROPI_H
#define ZEROPI_H

#include "arduino.h"

#define NUM_SLOTS 4
#define SLOT_NUM_PINS 6
#define NUM_EXTIO 13

#define SLOT_NULL 0x0
#define SLOT_STEPPER 0x1
#define SLOT_MOTOR 0x2

// stepper functin define
#define STEP_EN 0x0
#define STEP_MS1 0x1
#define STEP_MS2 0x2
#define STEP_MS3 0x3
#define STEP_STP 0x4
#define STEP_DIR 0x5



#define  Bit_RESET 	0
#define  Bit_SET 	1

#define TEMPERATURE_TIMER	TC5
#define TEMPERATURE_TIMER_IRQ	TC5_IRQn
#define TEMPERATURE_TIMER_ISR()	void TC5_Handler( void )


typedef struct _extIO {
  EPortType port;
  uint16_t pin;
} GPIO_t;

typedef struct _slot {
  int function;
  GPIO_t gpio[SLOT_NUM_PINS];
} SLOT_t;

//GPIO_t extio[];
extern const GPIO_t extio[];
extern const SLOT_t slot[];
extern const PinDescription zeroPiPinDescription[];
void GPIO_PinMode( EPortType PORTx, uint32_t Pin, uint32_t ulMode );
void GPIO_WriteBit(EPortType PORTx, uint16_t Pin, uint32_t BitVal);
uint8_t GPIO_ReadInputDataBit(EPortType PORTx, uint16_t Pin);


uint32_t analogReadChannel(int channel);
void slotSetup(int slot, int fun);
void stepperSetResolution(int s, int res);
void mosfetInit(void);
void mosfetSet(int index, int value);
void tempInit(void);
void initTemperatureTimer(void);

#endif
