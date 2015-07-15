#ifndef ZEROPI_H
#define ZEROPI_H

#include <inttypes.h>
#include "Arduino.h"

#define NUM_SLOTS 4
#define SLOT_NUM_PINS 6
#define NUM_EXTIO 11

#define SLOT_NULL 0x0
#define SLOT_STEPPER 0x1
#define SLOT_MOTOR 0x2

#define EXT_NULL 0x0
#define EXT_INPUT 0x1
#define EXT_OUTPUT 0x2
#define EXT_SERVO 0x3


// stepper functin define
#define STEP_EN 0x0
#define STEP_MS1 0x1
#define STEP_MS2 0x2
#define STEP_MS3 0x3
#define STEP_STP 0x4
#define STEP_DIR 0x5

typedef struct _slot{
	uint8_t function;
	uint8_t pin[SLOT_NUM_PINS];
}SLOT;

typedef struct _extIO{
	uint8_t function;
	uint8_t pin;
	uint32_t value;
}EXTIO;


// define for ext io silk print
#define MO 23
#define MI 22
#define SCK 24
#define SDA 32 // PA22
#define SCL 33 // PA23

// define Temperature IO
#define TEMP0 A4
#define TEMP1 A5

/*
 * Pins descriptions
 */
const PinDescription j18pinDescrip[]=
{
	// 0..13 - Digital pins
	// ----------------------
	// 0/1 - SERCOM/UART (Serial1)
	{ PORTA, 11, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
	{ PORTA, 10, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]
	
	// 2..12
	// Digital Low
	{ PORTA, 14, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },
	{ PORTA,  9, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9 }, // TCC0/WO[1]
	{ PORTA,  8, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI },  // TCC0/WO[0]
	{ PORTA, 15, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH1, TC3_CH1, EXTERNAL_INT_15 }, // TC3/WO[1]
	{ PORTA, 20, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH6, TCC0_CH6, EXTERNAL_INT_4 }, // TCC0/WO[6]
	{ PORTA, 21, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },
	
	// Digital High
	{ PORTA,  6, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_4 }, // TCC1/WO[0]
	{ PORTA,  7, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_5 }, // TCC1/WO[1]
	{ PORTA, 18, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH0, TC3_CH0, EXTERNAL_INT_2 }, // TC3/WO[0]
	{ PORTA, 16, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_0 }, // TCC2/WO[0]
	{ PORTA, 19, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_3 }, // TCC0/WO[3]
	
	// 13 (LED)
	{ PORTA, 17, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM2_CH1, NOT_ON_TIMER, EXTERNAL_INT_1 }, // TCC2/WO[1]
	
	// 14..19 - Analog pins
	// --------------------
	{ PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[0]
	{ PORTB,  8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel2, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, // ADC/AIN[2]
	{ PORTB,  9, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel3, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // ADC/AIN[3]
	{ PORTA,  4, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // ADC/AIN[4]
	{ PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // ADC/AIN[5]
	{ PORTB,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[10]
	
	// 20..21 I2C pins (SDA/SCL and also EDBG:SDA/SCL)
	// ----------------------
	{ PORTA, 22, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // SDA: SERCOM3/PAD[0]
	{ PORTA, 23, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // SCL: SERCOM3/PAD[1]
	
	// 22..24 - SPI pins (ICSP:MISO,SCK,MOSI)
	// ----------------------
	{ PORTA, 12, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, // MISO: SERCOM4/PAD[0]
	{ PORTB, 10, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // MOSI: SERCOM4/PAD[2]
	{ PORTB, 11, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // SCK: SERCOM4/PAD[3]
	
	// 25..26 - RX/TX LEDS (PB03/PA27)
	// --------------------
	{ PORTB,  3, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only
	{ PORTA, 27, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only
	
	// 27..29 - USB
	// --------------------
	{ PORTA, 28, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable
	{ PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
	{ PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP
	
	// 30..41 - EDBG
	// ----------------------
	// 30/31 - EDBG/UART
	{ PORTB, 22, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // TX: SERCOM5/PAD[2]
	{ PORTB, 23, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // RX: SERCOM5/PAD[3]
	
	// 32/33 I2C (SDA/SCL and also EDBG:SDA/SCL)
	{ PORTA, 22, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SDA: SERCOM3/PAD[0]
	{ PORTA, 23, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCL: SERCOM3/PAD[1]
	
	// 34..37 - EDBG/SPI
	{ PORTA, 19, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MISO: SERCOM1/PAD[3]
	{ PORTA, 16, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MOSI: SERCOM1/PAD[0]
	{ PORTA, 18, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SS: SERCOM1/PAD[2]
	{ PORTA, 17, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCK: SERCOM1/PAD[1]
	
	// 38..41 - EDBG/Digital
	{ PORTA, 13, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM0_CH5, NOT_ON_TIMER, EXTERNAL_INT_13 }, // EIC/EXTINT[13] *TCC2/WO[1] TCC0/WO[7]
	{ PORTA, 21, PIO_PWM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM0_CH7, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 7
	{ PORTA,  6, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH0, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 8
	{ PORTA,  7, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH1, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 9
	
	// 42 (AREF)
	{ PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP
	
	// ----------------------
	// 43 - Alternate use of A0 (DAC output)
	{ PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // DAC/VOUT
	
	
	//44..47
	{ PORTB,  4, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // 
	{ PORTB,  5, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // 
	{ PORTB,  6, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, //
	{ PORTB,  7, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // 
	 //48..51
	{ PORTB,  17, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // 
	{ PORTB,  16, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, // 
	{ PORTB,  1, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, //
	{ PORTB,  0, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // 
	//52..55
	{ PORTB,  12, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, // 
	{ PORTB,  13, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 }, //
	{ PORTB,  14, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, // 
	{ PORTB,  15, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, // 
	//56..57
	{ PORTB,  30, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_14 }, // TCC0/WO[0]
	{ PORTB,  31, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_15 }, // TCC0/WO[1]

} ;

#define READ(pin) (((PORT->Group[j18pinDescrip[pin].ulPort].IN.reg)>>j18pinDescrip[pin].ulPin) & 1ul)
#define WRITE(pin, v) do{if(v) {PORT->Group[j18pinDescrip[pin].ulPort].OUTSET.reg = (1ul << j18pinDescrip[pin].ulPin);} else {PORT->Group[j18pinDescrip[pin].ulPort].OUTCLR.reg = (1ul << j18pinDescrip[pin].ulPin); }}while(0)

#define SET_INPUT(pin) PORT->Group[j18pinDescrip[pin].ulPort].PINCFG[j18pinDescrip[pin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN) ; \
                       PORT->Group[j18pinDescrip[pin].ulPort].DIRCLR.reg = (uint32_t)(1<<j18pinDescrip[pin].ulPin)
#define SET_OUTPUT(pin) PORT->Group[j18pinDescrip[pin].ulPort].PINCFG[j18pinDescrip[pin].ulPin].reg&=~(uint8_t)(PORT_PINCFG_INEN) ; \
                        PORT->Group[j18pinDescrip[pin].ulPort].DIRSET.reg = (uint32_t)(1<<j18pinDescrip[pin].ulPin) ;
#define SET_INPUT_PULLUP(pin) PORT->Group[j18pinDescrip[pin].ulPort].PINCFG[j18pinDescrip[pin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ; \
                              PORT->Group[j18pinDescrip[pin].ulPort].DIRCLR.reg = (uint32_t)(1<<j18pinDescrip[pin].ulPin) ; \
                              PORT->Group[j18pinDescrip[pin].ulPort].OUTSET.reg = (uint32_t)(1<<j18pinDescrip[pin].ulPin) ;
#define TOGGLE(pin) WRITE(pin,!READ(pin))

#define BASEFREQ (48000000/8) // TC_CTRLA_PRESCALER_DIV8_Val
#define SERVO_TIMER             TC4
#define SERVO_TIMER_CHANNEL     0
#define SERVO_TIMER_IRQ         TC4_IRQn
#define SERVO_COMPA_VECTOR      TC4_Handler
#define SERVO_CLOCK_FREQ        1000
#define SERVO2500US             ((BASEFREQ / 1000000) * 2500)
#define SERVO5000US             ((BASEFREQ / 1000000) * 5000)

#define PWM_TIMER               TC5
#define PWM_TIMER_CHANNEL       0
#define PWM_TIMER_IRQ           TC5_IRQn
#define PWM_TIMER_VECTOR        TC5_Handler
#define PWM_CLOCK_FREQ 			3906 //256us


class ZeroPi
{
public:
	static SLOT slots[NUM_SLOTS];
	static EXTIO extios[NUM_EXTIO];
    ZeroPi();
    virtual ~ZeroPi();

	// init slot to some type
	void timerSetup(void);
	void slotSetup(int slot, int fun);
	
	// stepper related functions
	void stepperInit(int slot);
	void stepperSetResolution(int slot, int res);
	void stepperMove(int slot, int dir);
	void stepperEnable(int slot, int en);

	// dc motor related functions
	void motorInit(int slot);
	void motorRun(int slot, int pwm);
	
	// extra io functions
	void extInit(int index, int type);
	int extRead(int index);
	void extWrite(int index, int value);
	void extWriteUs(int index, int us);

	
	// temp sensor specified
	void tempInit(int index);
	uint32_t tempRead(int index);
	
	// mosfet control
	void mosfetInit(void);
	void mosfetSet(int index, int value);

protected:

};


#endif

