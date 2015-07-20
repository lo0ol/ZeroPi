/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

    
    Main author: repetier
 
    Initial port of HAL to Arduino Due: John Silvia 
*/

/**
  This is the main Hardware Abstraction Layer (HAL).
  To make the firmware work with different processors and toolchains,
  all hardware related code should be packed into the hal files.
*/

#ifndef HAL_H
#define HAL_H


#include <inttypes.h>
#include "pins.h"
//#include "Print.h"

// Hack to make 84 MHz Due clock work without changes to pre-existing code
// which would otherwise have problems with int overflow.
// yzj, change for zeropi woking in 48MHz
#define F_CPU       48000000        // should be factor of F_CPU_TRUE
#define F_CPU_TRUE  48000000        // actual CPU clock frequency
#define EEPROM_BYTES 4096  // bytes of eeprom we simulate

// another hack to keep AVR code happy (i.e. SdFat.cpp)
#define SPR0    0
#define SPR1    1

// force SdFat to use HAL (whether or not using SW spi)
#undef  SOFTWARE_SPI

// Some structures assume no padding, need to add this attribute on ARM
#define PACK    __attribute__ ((packed))

// do not use program space memory with Due
#define PROGMEM
#define PGM_P const char *
typedef char prog_char;
#define PSTR(s) s
//#define pgm_read_byte_near(x) (*(int8_t*)x)
//#define pgm_read_byte(x) (*(int8_t*)x)
//#define pgm_read_float(addr) (*(const float *)(addr))
//#define pgm_read_word(addr) (*(const unsigned int *)(addr))
//#define pgm_read_word_near(addr) pgm_read_word(addr)
//#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
//#define pgm_read_dword_near(addr) pgm_read_dword(addr)

#define FSTRINGVALUE(var,value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var


#define EXTRUDER_TIMER          TC4
#define EXTRUDER_TIMER_CHANNEL  0
#define EXTRUDER_TIMER_IRQ      TC4_IRQn
#define EXTRUDER_TIMER_VECTOR   TC4_Handler

#define PWM_TIMER               TC5
#define PWM_TIMER_CHANNEL       0
#define PWM_TIMER_IRQ           TC5_IRQn
#define PWM_TIMER_VECTOR        TC5_Handler

#define TIMER1_TIMER            TC3
#define TIMER1_TIMER_CHANNEL    0
#define TIMER1_TIMER_IRQ        TC3_IRQn
#define TIMER1_COMPA_VECTOR     TC3_Handler

#define SERVO_TIMER             TC2
#define SERVO_TIMER_CHANNEL     0
#define SERVO_TIMER_IRQ         ID_TC6
#define SERVO_COMPA_VECTOR      TC6_Handler

#define BEEPER_TIMER            TC1
#define BEEPER_TIMER_CHANNEL    0
#define BEEPER_TIMER_IRQ        ID_TC3
#define BEEPER_TIMER_VECTOR     TC3_Handler

#define DELAY_TIMER             TC1
#define DELAY_TIMER_CHANNEL     1
#define DELAY_TIMER_IRQ         ID_TC4  // IRQ not really used, needed for pmc id
#define DELAY_TIMER_CLOCK       TC_CMR_TCCLKS_TIMER_CLOCK2
#define DELAY_TIMER_PRESCALE    8

#define EXTRUDER_CLOCK_FREQ     244    // don't know what this should be
#define TIMER0_PRESCALE 128
#define PWM_CLOCK_FREQ          3906
#define TIMER1_CLOCK_FREQ       244
#define TIMER1_PRESCALE         2

#define SERVO_CLOCK_FREQ        1000
#define SERVO_PRESCALE          2      // Using TCLOCK1 therefore 2
#define SERVO2500US             (((F_CPU_TRUE / SERVO_PRESCALE) / 1000000) * 2500)
#define SERVO5000US             (((F_CPU_TRUE / SERVO_PRESCALE) / 1000000) * 5000)

#define AD_PRESCALE_FACTOR      41  // 1 MHz ADC clock 
#define AD_TRACKING_CYCLES      0   // 0 - 15     + 1 adc clock cycles
#define AD_TRANSFER_CYCLES      1   // 0 - 3      * 2 + 3 adc clock cycles

#define ADC_ISR_EOC(channel)    (0x1u << channel) 
#define ENABLED_ADC_CHANNELS    {TEMP_0_PIN, TEMP_1_PIN, TEMP_2_PIN}  

#define PULLUP(IO,v)            {HAL::pinMode(IO, (v!=LOW ? INPUT_PULLUP : INPUT)); }

// INTERVAL / (32Khz/128)  = seconds
#define WATCHDOG_INTERVAL       250  // 1sec  (~16 seconds max)


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#define COMPAT_PRE1
#endif


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
} ;


//#define  READ(pin)  PIO_Get(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin)
#define READ(pin) (((PORT->Group[g_APinDescription[pin].ulPort].IN.reg)>>g_APinDescription[pin].ulPin) & 1ul)
//#define WRITE(pin, v) PIO_SetOutput(g_APinDescription[pin].pPort, g_APinDescription[pin].ulPin, v, 0, PIO_PULLUP)
//#define WRITE(pin, v) do{if(v) {g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;} else {g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin; }}while(0)
#define WRITE(pin, v) do{if(v) {PORT->Group[g_APinDescription[pin].ulPort].OUTSET.reg = (1ul << g_APinDescription[pin].ulPin);} else {PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg = (1ul << g_APinDescription[pin].ulPin); }}while(0)
 
//#define SET_INPUT(pin) pmc_enable_periph_clk(g_APinDescription[pin].ulPeripheralId); \
//    PIO_Configure(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin, 0) 
//#define SET_OUTPUT(pin) PIO_Configure(g_APinDescription[pin].pPort, PIO_OUTPUT_1, \
//    g_APinDescription[pin].ulPin, g_APinDescription[pin].ulPinConfiguration)
#define SET_INPUT(pin) PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN) ; \
                       PORT->Group[g_APinDescription[pin].ulPort].DIRCLR.reg = (uint32_t)(1<<g_APinDescription[pin].ulPin)
#define SET_OUTPUT(pin) PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].reg&=~(uint8_t)(PORT_PINCFG_INEN) ; \
                        PORT->Group[g_APinDescription[pin].ulPort].DIRSET.reg = (uint32_t)(1<<g_APinDescription[pin].ulPin) ;
#define SET_INPUT_PULLUP(pin) PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ; \
                              PORT->Group[g_APinDescription[pin].ulPort].DIRCLR.reg = (uint32_t)(1<<g_APinDescription[pin].ulPin) ; \
                              PORT->Group[g_APinDescription[pin].ulPort].OUTSET.reg = (uint32_t)(1<<g_APinDescription[pin].ulPin) ;


#define TOGGLE(pin) WRITE(pin,!READ(pin))
#define LOW         0
#define HIGH        1

#define BEGIN_INTERRUPT_PROTECTED {uint32_t oldInt = __get_PRIMASK();__disable_irq();
#define END_INTERRUPT_PROTECTED __set_PRIMASK(oldInt);} //__enable_irq();
#define ESCAPE_INTERRUPT_PROTECTED  __set_PRIMASK(oldInt);//__enable_irq();

#define EEPROM_OFFSET               0
#define SECONDS_TO_TICKS(s) (unsigned long)(s*(float)F_CPU)
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

// maximum available RAM
#define MAX_RAM 98303

#define bit_clear(x,y) x&= ~(1<<y) //cbi(x,y)
#define bit_set(x,y)   x|= (1<<y)//sbi(x,y)

/** defines the data direction (reading from I2C device) in i2cStart(),i2cRepStart() */
#define I2C_READ    1
/** defines the data direction (writing to I2C device) in i2cStart(),i2cRepStart() */
#define I2C_WRITE   0

#ifndef DUE_SOFTWARE_SPI
    extern int spiDueDividors[];
#endif

static uint32_t    tone_pin;

/** Set max. frequency to 150000Hz */
#define LIMIT_INTERVAL (F_CPU/150000)


typedef unsigned int speed_t;
typedef unsigned long ticks_t;
typedef unsigned long millis_t;
typedef int flag8_t;

#define RFSERIAL Serial

#define OUT_P_I(p,i) Com::printF(PSTR(p),(int)(i))
#define OUT_P_I_LN(p,i) Com::printFLN(PSTR(p),(int)(i))
#define OUT_P_L(p,i) Com::printF(PSTR(p),(long)(i))
#define OUT_P_L_LN(p,i) Com::printFLN(PSTR(p),(long)(i))
#define OUT_P_F(p,i) Com::printF(PSTR(p),(float)(i))
#define OUT_P_F_LN(p,i) Com::printFLN(PSTR(p),(float)(i))
#define OUT_P_FX(p,i,x) Com::printF(PSTR(p),(float)(i),x)
#define OUT_P_FX_LN(p,i,x) Com::printFLN(PSTR(p),(float)(i),x)
#define OUT_P(p) Com::printF(PSTR(p))
#define OUT_P_LN(p) Com::printFLN(PSTR(p))
#define OUT_ERROR_P(p) Com::printErrorF(PSTR(p))
#define OUT_ERROR_P_LN(p) {Com::printErrorF(PSTR(p));Com::println();}
#define OUT(v) Com::print(v)
#define OUT_LN Com::println()

union eeval_t {
    uint8_t     b[];
    float       f;
    uint32_t    i;
    uint16_t    s;
    long        l;
} PACK;


class HAL
{
public:
    // we use ram instead of eeprom, so reads are faster and safer. Writes store in real eeprom as well
    // as long as hal eeprom functions are used.
    static char virtualEeprom[EEPROM_BYTES];     
    
    HAL();
    virtual ~HAL();

    // do any hardware-specific initialization here
    static inline void hwSetup(void)
    {
        //HAL::i2cInit(TWI_CLOCK_FREQ);
        // make debugging startup easier
        //Serial.begin(115200);
        // yzj, no mcu freq setup
        //TimeTick_Configure(F_CPU_TRUE);

        // yzj, use arduino micro delay instead
        // setup microsecond delay timer
        // pmc_enable_periph_clk(DELAY_TIMER_IRQ);
        //TC_Configure(DELAY_TIMER, DELAY_TIMER_CHANNEL, TC_CMR_WAVSEL_UP | 
        //             TC_CMR_WAVE | DELAY_TIMER_CLOCK);
        //TC_Start(DELAY_TIMER, DELAY_TIMER_CHANNEL);
#if EEPROM_AVAILABLE
        // Copy eeprom to ram for faster access
        int i,n = EEPROM_BYTES;
        for(i=0;i<EEPROM_BYTES;i+=4) {
          eeval_t v = eprGetValue(i, 4);
          *(int*)(&virtualEeprom[i]) = v.i;
        }
#endif
      SET_OUTPUT(X_RES0_PIN);WRITE(X_RES0_PIN,1);
      SET_OUTPUT(X_RES1_PIN);WRITE(X_RES1_PIN,1);
      SET_OUTPUT(X_RES2_PIN);WRITE(X_RES2_PIN,0);
      
      SET_OUTPUT(Y_RES0_PIN);WRITE(Y_RES0_PIN,1);
      SET_OUTPUT(Y_RES1_PIN);WRITE(Y_RES1_PIN,1);
      SET_OUTPUT(Y_RES2_PIN);WRITE(Y_RES2_PIN,0);

      SET_OUTPUT(Z_RES0_PIN);WRITE(Z_RES0_PIN,1);
      SET_OUTPUT(Z_RES1_PIN);WRITE(Z_RES1_PIN,1);
      SET_OUTPUT(Z_RES2_PIN);WRITE(Z_RES2_PIN,0);

      SET_OUTPUT(E0_RES0_PIN);WRITE(E0_RES0_PIN,1);
      SET_OUTPUT(E0_RES1_PIN);WRITE(E0_RES1_PIN,1);
      SET_OUTPUT(E0_RES2_PIN);WRITE(E0_RES2_PIN,0);

    }

    // return val'val
    static inline unsigned long U16SquaredToU32(unsigned int val)
    {
        return (unsigned long) val * (unsigned long) val;
    }
    static inline unsigned int ComputeV(long timer,long accel)
    {
        return static_cast<unsigned int>((static_cast<int64_t>(timer)*static_cast<int64_t>(accel))>>18);
        //return ((timer>>8)*accel)>>10;
    }
// Multiply two 16 bit values and return 32 bit result
    static inline unsigned long mulu16xu16to32(unsigned int a,unsigned int b)
    {
        return (unsigned long) a * (unsigned long) b;
    }
// Multiply two 16 bit values and return 32 bit result
    static inline unsigned int mulu6xu16shift16(unsigned int a,unsigned int b)
    {
        return ((unsigned long)a*(unsigned long)b)>>16;
    }
    static inline unsigned int Div4U2U(unsigned long a,unsigned int b)
    {
        return ((unsigned long)a / (unsigned long)b);
    }
    static inline void digitalWrite(uint8_t pin,uint8_t value)
    {
        WRITE(pin, value);
    }
    static inline uint8_t digitalRead(uint8_t pin)
    {
        return READ(pin);
    }
    static inline void pinMode(uint8_t pin,uint8_t mode)
    {
        if (mode == INPUT) {
          SET_INPUT(pin);
        }else if(mode == OUTPUT){
          SET_OUTPUT(pin);
        }else if(mode == INPUT_PULLUP){
          SET_INPUT_PULLUP(pin);
        }
    }
    static long CPUDivU2(speed_t divisor) {
      return F_CPU/divisor;
    }
    static inline void delayMicroseconds(uint32_t usec)
    {
      if ( usec == 0 )
      {
        return ;
      }
      uint32_t n = usec * (VARIANT_MCK / 1000000) / 3;
      __asm__ __volatile__(
        "1:              \n"
        "   sub %0, #1   \n" // substract 1 from %0 (n)
        "   bne 1b       \n" // if result is not 0 jump to 1
        : "+r" (n)           // '%0' is n variable with RW constraints
        :                    // no input
        :                    // no clobber
      );
      // https://gcc.gnu.org/onlinedocs/gcc/Extended-Asm.html
      // https://gcc.gnu.org/onlinedocs/gcc/Extended-Asm.html#Volatile
    }
    static inline void delayMilliseconds(unsigned int delayMs)
    {
        //Wait(delayMs);
        delay(delayMs);
    }
    static inline void tone(uint8_t pin,int frequency) {
    #if TONE_AVAILABLE
        // set up timer counter 1 channel 0 to generate interrupts for
        // toggling output pin.  
        SET_OUTPUT(pin);
        tone_pin = pin;
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk((uint32_t)BEEPER_TIMER_IRQ);
        // set interrupt to lowest possible priority
        NVIC_SetPriority((IRQn_Type)BEEPER_TIMER_IRQ, NVIC_EncodePriority(4, 6, 3));
        TC_Configure(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | 
                     TC_CMR_TCCLKS_TIMER_CLOCK4);  // TIMER_CLOCK4 -> 128 divisor
        uint32_t rc = VARIANT_MCK / 128 / frequency; 
        TC_SetRA(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, rc/2);                     // 50% duty cycle
        TC_SetRC(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, rc);
        TC_Start(BEEPER_TIMER, BEEPER_TIMER_CHANNEL);
        BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_IER=TC_IER_CPCS;
        BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_IDR=~TC_IER_CPCS;
        NVIC_EnableIRQ((IRQn_Type)BEEPER_TIMER_IRQ);
    #endif
    }
    static inline void noTone(uint8_t pin) {
    #if TONE_AVAILABLE
        TC_Stop(TC1, 0); 
        WRITE(pin, LOW);
    #endif
    }

    static inline void eprSetByte(unsigned int pos,uint8_t value)
    {
        eeval_t v;
        v.b[0] = value;
        eprBurnValue(pos, 1, v);
        *(uint8_t*)&virtualEeprom[pos] = value;
    }
    static inline void eprSetInt16(unsigned int pos,int value)
    {
        eeval_t v;
        v.s = value;
        eprBurnValue(pos, 2, v);
        *(uint16_t*)&virtualEeprom[pos] = value;
    }
    static inline void eprSetInt32(unsigned int pos,int value)
    {
        eeval_t v;
        v.i = value;
        eprBurnValue(pos, 4, v);
        *(int*)&virtualEeprom[pos] = value;
    }
    static inline void eprSetLong(unsigned int pos,long value)
    {
        eeval_t v;
        v.l = value;
        eprBurnValue(pos, sizeof(long), v);
        *(long*)&virtualEeprom[pos] = value;
    }
    static inline void eprSetFloat(unsigned int pos,float value)
    {
        eeval_t v;
        v.f = value;
        eprBurnValue(pos, sizeof(float), v);
        *(float*)&virtualEeprom[pos] = value;
    }
    static inline uint8_t eprGetByte(unsigned int pos)
    {
        return *(uint8_t*)&virtualEeprom[pos];
        //eeval_t v = eprGetValue(pos,1);
        //return v.b[0];
    }
    static inline int16_t eprGetInt16(unsigned int pos)
    {
        return *(int16_t*)&virtualEeprom[pos];
        //eeval_t v;
        //v.i = 0;
        //v = eprGetValue(pos, 2);
        //return v.i;
    }
    static inline int eprGetInt32(unsigned int pos)
    {
        return *(int*)&virtualEeprom[pos];
        //eeval_t v = eprGetValue(pos, 4);
        //return v.i;
    }
    static inline long eprGetLong(unsigned int pos)
    {
        return *(long*)&virtualEeprom[pos];
        //eeval_t v = eprGetValue(pos, sizeof(long));
        //return v.l;
    }
    static inline float eprGetFloat(unsigned int pos) {
        return *(float*)&virtualEeprom[pos];
        //eeval_t v = eprGetValue(pos, sizeof(float));
        //return v.f;
    }

    // Write any data type to EEPROM
    static inline void eprBurnValue(unsigned int pos, int size, union eeval_t newvalue) 
    {
        i2cStartAddr(EEPROM_SERIAL_ADDR << 1 | I2C_WRITE, pos);        
        i2cWriting(newvalue.b[0]);        // write first byte
        for (int i=1;i<size;i++) {
            pos++;
            // writes cannot cross page boundary
            if ((pos % EEPROM_PAGE_SIZE) == 0) {
                // burn current page then address next one
                i2cStop();
                delayMilliseconds(EEPROM_PAGE_WRITE_TIME); 
                i2cStartAddr(EEPROM_SERIAL_ADDR << 1, pos);
            } else {
              i2cTxFinished();      // wait for transmission register to empty
            }
            i2cWriting(newvalue.b[i]);
        }
        i2cStop();          // signal end of transaction
        delayMilliseconds(EEPROM_PAGE_WRITE_TIME);   // wait for page write to complete
    }

    // Read any data type from EEPROM that was previously written by eprBurnValue
    static inline union eeval_t eprGetValue(unsigned int pos, int size)
    {
        int i;
        eeval_t v;

        size--;
        // set read location
        i2cStartAddr(EEPROM_SERIAL_ADDR << 1 | I2C_READ, pos);
        // begin transmission from device
        i2cStartBit();
        for (i=0;i<size;i++) {
            // read an incomming byte 
            v.b[i] = i2cReadAck(); 
        }
        // read last byte 
        v.b[i] = i2cReadNak();
        return v;
    }

    static inline void allowInterrupts()
    {
        __enable_irq();
    }
    static inline void forbidInterrupts()
    {
        __disable_irq();
    }
    static inline unsigned long timeInMilliseconds()
    {
        return millis();
    }
    static inline char readFlashByte(PGM_P ptr)
    {
        return pgm_read_byte(ptr);
    }
    static inline void serialSetBaudrate(long baud)
    {
        RFSERIAL.begin(baud);
    }
    static inline bool serialByteAvailable()
    {
        return RFSERIAL.available();
    }
    static inline uint8_t serialReadByte()
    {
        return RFSERIAL.read();
    }
    static inline void serialWriteByte(char b)
    {
        RFSERIAL.write(b);
    }
    static inline void serialFlush()
    {
        RFSERIAL.flush();
    }
    static void setupTimer();
    static void showStartReason();
    static int getFreeRam();
    static void resetHardware();

    // SPI related functions

#ifdef DUE_SOFTWARE_SPI
    // bitbanging transfer
    // run at ~100KHz (necessary for init)
    static uint8_t spiTransfer(uint8_t b)  // using Mode 0
    {
        for (int bits = 0; bits < 8; bits++) {
            if (b & 0x80) {
                WRITE(MOSI_PIN, HIGH);
            } else {
                WRITE(MOSI_PIN, LOW);
            }
            b <<= 1;

            WRITE(SCK_PIN, HIGH);
            delayMicroseconds(5);

            if(READ(MISO_PIN)) {
                b |= 1;
            }
            WRITE(SCK_PIN, LOW);
            delayMicroseconds(5);
        }
        return b;
    }
    static inline void spiBegin() 
    {
        SET_OUTPUT(SDSS);
        WRITE(SDSS, HIGH);
        SET_OUTPUT(SCK_PIN);
        SET_INPUT(MISO_PIN);
        SET_OUTPUT(MOSI_PIN);
    }

    static inline void spiInit(uint8_t spiClock) 
   {
       WRITE(SDSS, HIGH);
       WRITE(MOSI_PIN, HIGH);
       WRITE(SCK_PIN, LOW);
    }
   static inline uint8_t spiReceive()
   {
       WRITE(SDSS, LOW);
       uint8_t b = spiTransfer(0xff);       
       WRITE(SDSS, HIGH);
       return b;
   }
   static inline void spiReadBlock(uint8_t*buf,uint16_t nbyte) 
   {   
       if (nbyte == 0) return;
       WRITE(SDSS, LOW);  
       for (int i=0; i<nbyte; i++)
        {
            buf[i] = spiTransfer(0xff);  
        }
       WRITE(SDSS, HIGH);

   }
   static inline void spiSend(uint8_t b) {
       WRITE(SDSS, LOW);
       uint8_t response = spiTransfer(b);
       WRITE(SDSS, HIGH);
   }
   
   static inline void spiSend(const uint8_t* buf , size_t n)
   {
        uint8_t response;
        if (n == 0) return;
        WRITE(SDSS, LOW);
        for (uint16_t i = 0; i < n; i++) {
           response = spiTransfer(buf[i]);  
       }
       WRITE(SDSS, HIGH);
   }
   
   inline __attribute__((always_inline))
   static void spiSendBlock(uint8_t token, const uint8_t* buf)
   {
       uint8_t response;

       WRITE(SDSS, LOW);
       response = spiTransfer(token);

       for (uint16_t i = 0; i < 512; i++)
       {
           response = spiTransfer(buf[i]);  
       }
       WRITE(SDSS, HIGH);
   }
   
#else

   // hardware SPI
   static void spiBegin();
   // spiClock is 0 to 6, relecting AVR clock dividers 2,4,8,16,32,64,128
   // Due can only go as slow as AVR divider 32 -- slowest Due clock is 329,412 Hz
    static void spiInit(uint8_t spiClock);
    // Write single byte to SPI
    static void spiSend(byte b);
   static void spiSend(const uint8_t* buf , size_t n);
    // Read single byte from SPI
   static uint8_t spiReceive();
    // Read from SPI into buffer
   static void spiReadBlock(uint8_t*buf,uint16_t nbyte);

    // Write from buffer to SPI

   static void spiSendBlock(uint8_t token, const uint8_t* buf);
#endif  /*DUE_SOFTWARE_SPI*/

    // I2C Support
    static void i2cInit(unsigned long clockSpeedHz);
    static void i2cStartWait(unsigned char address);
    static unsigned char i2cStart(unsigned char address);
    static void i2cStartAddr(unsigned char address, unsigned int pos);
    static void i2cStop(void);
    static void i2cStartBit(void);
    static void i2cCompleted (void);
    static void i2cTxFinished(void);
    static void i2cWriting( uint8_t data );
    static unsigned char i2cWrite( unsigned char data );
    static unsigned char i2cReadAck(void);
    static unsigned char i2cReadNak(void);


    // Watchdog support

    inline static void startWatchdog() {
  #if WATCHDOG_AVAILABLE
      WDT_Enable(WDT, WDT_MR_WDRSTEN | WATCHDOG_INTERVAL );
  #endif
    };
    inline static void stopWatchdog() {}
    inline static void pingWatchdog() {
  #if WATCHDOG_AVAILABLE
      WDT_Restart(WDT);
  #endif
    }

    inline static float maxExtruderTimerFrequency() {return (float)F_CPU/TIMER0_PRESCALE;}
#if FEATURE_SERVO
    static unsigned int servoTimings[4];
    static void servoMicroseconds(uint8_t servo,int ms);
#endif

#if ANALOG_INPUTS>0
    static void analogStart(void);
#endif
    static volatile uint8_t insideTimer1;
        
protected:
};


#endif

