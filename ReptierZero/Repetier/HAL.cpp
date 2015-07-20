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
 
    Initial port of hardware abstraction layer to Arduino Due: John Silvia
*/

#include "Repetier.h"
#include <malloc.h>

//extern "C" void __cxa_pure_virtual() { }
extern "C" char *sbrk(int i);
extern long bresenham_step();

char HAL::virtualEeprom[EEPROM_BYTES];  
volatile uint8_t HAL::insideTimer1=0;
#ifndef DUE_SOFTWARE_SPI
    int spiDueDividors[] = {10,21,42,84,168,255,255};
#endif

HAL::HAL()
{
    //ctor
}

HAL::~HAL()
{
    //dtor
}


// Set up all timer interrupts 
#define BASEFREQ (48000000/8) // TC_CTRLA_PRESCALER_DIV8_Val
void HAL::setupTimer() {
  uint32_t tmp;
  // the init code in arduino already initialized the PM
  // Clock SERCOM for Serial
  //PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 | PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 ;
  // Clock TC/TCC for Pulse and Analog
  //PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 ;
  // Clock ADC/DAC for Analog
  //PM->APBCMASK.reg |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;
 
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 )) ;
  while (GCLK->STATUS.bit.SYNCBUSY);
#if defined(USE_ADVANCE)
  // Timer for extruder control
  tmp = BASEFREQ/TIMER0_PRESCALE/EXTRUDER_CLOCK_FREQ;
  EXTRUDER_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
  EXTRUDER_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
  EXTRUDER_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  EXTRUDER_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
  EXTRUDER_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  NVIC_EnableIRQ( EXTRUDER_TIMER_IRQ ) ;
#endif
  // Regular interrupts for heater control etc
  tmp = BASEFREQ/PWM_CLOCK_FREQ; // todo: add tc_count
  PWM_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
  PWM_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
  PWM_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  PWM_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
  PWM_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  NVIC_EnableIRQ( PWM_TIMER_IRQ ) ;

  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 )) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Timer for stepper motor control
  tmp = BASEFREQ/TIMER1_PRESCALE/TIMER1_CLOCK_FREQ;
  TIMER1_TIMER->COUNT16.CC[0].reg = (tmp & 0xffff);
  TIMER1_TIMER->COUNT8.INTENSET.reg = TC_INTFLAG_MC(1);
  TIMER1_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  TIMER1_TIMER->COUNT16.CTRLA.reg |= (TC_CTRLA_PRESCALER_DIV8_Val << TC_CTRLA_PRESCALER_Pos);
  TIMER1_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  NVIC_EnableIRQ( TIMER1_TIMER_IRQ ) ;

  //todo: servo timer not applied now

  
}


#if ANALOG_INPUTS>0
// Initialize ADC channels
void HAL::analogStart(void)
{
  // init pin peripheral
  for(int i=0; i<ANALOG_INPUTS; i++)
  {
      osAnalogInputCounter[i] = 0;
      osAnalogInputValues[i] = 0;
      int ulPin = osAnalogInputChannels[i];
      if ( g_APinDescription[ulPin].ulPin & 1 ) // is pin odd?
      {
        uint32_t temp ;
        // Get whole current setup for both odd and even pins and remove odd one
        temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXE( 0xF ) ;
        // Set new muxing
        PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp|PORT_PMUX_PMUXO( PIO_ANALOG ) ;
        // Enable port mux
        PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN ;
      }
      else // even pin
      {
        uint32_t temp ;
        temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXO( 0xF ) ;
        PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp|PORT_PMUX_PMUXE( PIO_ANALOG ) ;
        PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN ; // Enable port mux
      }
  }
  ADC->CTRLA.bit.ENABLE = 0x01;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
}

#endif


// Print apparent cause of start/restart
void HAL::showStartReason() {
    int mcu = 0;
    switch (mcu){
    case 0:
        Com::printInfoFLN(Com::tPowerUp);
        break;
    case 1:
        // this is return from backup mode on SAM
        Com::printInfoFLN(Com::tBrownOut);
    case 2:
        Com::printInfoFLN(Com::tWatchdog);
        break;
    case 3:
        Com::printInfoFLN(Com::tSoftwareReset);
        break;
    case 4:
        Com::printInfoFLN(Com::tExternalReset);
    } 
}


// Reset peripherals and cpu
void HAL::resetHardware() {
    NVIC_SystemReset();
}

// Return available memory
int HAL::getFreeRam() {
    struct mallinfo memstruct = mallinfo();
    register char * stack_ptr asm ("sp");

    // avail mem in heap + (bottom of stack addr - end of heap addr)
    return (memstruct.fordblks + (int)stack_ptr -  (int)sbrk(0));
}


/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void HAL::i2cInit(unsigned long clockSpeedHz)
{

}


/*************************************************************************
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char HAL::i2cStart(unsigned char address_and_direction)
{

    return 0;
}


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready

 Input:   address and transfer direction of I2C device
*************************************************************************/
void HAL::i2cStartWait(unsigned char address_and_direction)
{

}

/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 Also specifies internal address of device

 Input:   address and transfer direction of I2C device, internal address
*************************************************************************/
void HAL::i2cStartAddr(unsigned char address_and_direction, unsigned int pos)
{

}

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void HAL::i2cStop(void)
{

}

/*************************************************************************
 Signal start of data transfer
*************************************************************************/
void HAL::i2cStartBit(void)
{
    
}

/*************************************************************************
 Wait for transaction to complete
*************************************************************************/
void HAL::i2cCompleted (void)
{
    
}

/*************************************************************************
 Wait for transmission to complete
*************************************************************************/
void HAL::i2cTxFinished(void)
{
    
}


/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
  Return:   0 write successful
            1 write failed
*************************************************************************/
unsigned char HAL::i2cWrite( uint8_t data )
{    

  return 0;
}

/*************************************************************************
  Send one byte to I2C device
  Transaction can continue with more writes or reads 
************************************************************************/
void HAL::i2cWriting( uint8_t data )
{    
    
}


/*************************************************************************
 Read one byte from the I2C device, request more data from device
 Return:  byte read from I2C device
*************************************************************************/
unsigned char HAL::i2cReadAck(void)
{

    return 0;
}

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition

 Return:  byte read from I2C device
*************************************************************************/
unsigned char HAL::i2cReadNak(void)
{
    return 0;
}

/** \brief Sets the timer 1 compare value to delay ticks.
*/
inline void setTimer(unsigned long delay)
{
    // convert old AVR timer delay value for SAM timers
    uint32_t timer_count = delay;

    if(timer_count < 210) timer_count = 210;
    //TIMER1_TIMER->COUNT16.COUNT.reg = timer_count;
    TIMER1_TIMER->COUNT16.CC[0].reg = timer_count;
}

/** \brief Timer interrupt routine to drive the stepper motors.
*/
void TIMER1_COMPA_VECTOR ()
{
    TIMER1_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
    if(HAL::insideTimer1) return;
    HAL::insideTimer1 = 1;
    if(PrintLine::hasLines())
    {
        setTimer(PrintLine::bresenhamStep());
        HAL::allowInterrupts();
    }
    else
    if(Printer::zBabystepsMissing != 0) {
        Printer::zBabystep();
        setTimer(Printer::interval);
    } else
    {
        if(waitRelax==0)
        {
#ifdef USE_ADVANCE
            if(Printer::advanceStepsSet)
            {
                Printer::extruderStepsNeeded-=Printer::advanceStepsSet;
#ifdef ENABLE_QUADRATIC_ADVANCE
                Printer::advanceExecuted = 0;
#endif
                Printer::advanceStepsSet = 0;
            }
            if((!Printer::extruderStepsNeeded) && (DISABLE_E)) 
                Extruder::disableCurrentExtruderMotor();
#else
            if(DISABLE_E) Extruder::disableCurrentExtruderMotor();
#endif
        }
        else waitRelax--;
        setTimer(10000);
    }
    DEBUG_MEMORY;
    HAL::insideTimer1=0;
}

#if !defined(HEATER_PWM_SPEED)
#define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED<0
#define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED>3
#define HEATER_PWM_SPEED 3
#endif

#if HEATER_PWM_SPEED == 0
#define HEATER_PWM_STEP 1
#define HEATER_PWM_MASK 255
#elif HEATER_PWM_SPEED == 1
#define HEATER_PWM_STEP 2
#define HEATER_PWM_MASK 254
#elif HEATER_PWM_SPEED == 2
#define HEATER_PWM_STEP 4
#define HEATER_PWM_MASK 252
#else
#define HEATER_PWM_STEP 4
#define HEATER_PWM_MASK 252
#endif

/**
This timer is called 3906 times per second. It is used to update
pwm values for heater and some other frequent jobs. 
*/
void PWM_TIMER_VECTOR ()
{
    PWM_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
    static uint8_t pwm_count = 0;
    static uint8_t pwm_count_heater = 0;
    static uint8_t pwm_pos_set[NUM_EXTRUDER+3];
    static uint8_t pwm_cooler_pos_set[NUM_EXTRUDER];

    if(pwm_count==0)
    {
#if EXT0_HEATER_PIN>-1
        if((pwm_pos_set[0] = (pwm_pos[0] & HEATER_PWM_MASK))>0) WRITE(EXT0_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
        if((pwm_pos_set[1] = (pwm_pos[1] & HEATER_PWM_MASK))>0) WRITE(EXT1_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
        if((pwm_pos_set[2] = (pwm_pos[2] & HEATER_PWM_MASK))>0) WRITE(EXT2_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER>3
        if((pwm_pos_set[3] = (pwm_pos[3] & HEATER_PWM_MASK))>0) WRITE(EXT3_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1 && NUM_EXTRUDER>4
        if((pwm_pos_set[4] = (pwm_pos[4] & HEATER_PWM_MASK))>0) WRITE(EXT4_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER>5
        if((pwm_pos_set[5] = (pwm_pos[5] & HEATER_PWM_MASK))>0) WRITE(EXT5_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if HEATED_BED_HEATER_PIN>-1 && HAVE_HEATED_BED
        if((pwm_pos_set[NUM_EXTRUDER] = pwm_pos[NUM_EXTRUDER])>0) WRITE(HEATED_BED_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
    }
    if(pwm_count==0)
    {
#if EXT0_HEATER_PIN>-1 && EXT0_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[0] = extruder[0].coolerPWM)>0) WRITE(EXT0_EXTRUDER_COOLER_PIN,1);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
#if EXT1_EXTRUDER_COOLER_PIN>-1 && EXT1_EXTRUDER_COOLER_PIN!=EXT0_EXTRUDER_COOLER_PIN
        if((pwm_cooler_pos_set[1] = extruder[1].coolerPWM)>0) WRITE(EXT1_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
#if EXT2_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[2] = extruder[2].coolerPWM)>0) WRITE(EXT2_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER>3
#if EXT3_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[3] = extruder[3].coolerPWM)>0) WRITE(EXT3_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1 && NUM_EXTRUDER>4
#if EXT4_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[4] = pwm_pos[4].coolerPWM)>0) WRITE(EXT4_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER>5
#if EXT5_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[5] = extruder[5].coolerPWM)>0) WRITE(EXT5_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if FAN_BOARD_PIN>-1
        if((pwm_pos_set[NUM_EXTRUDER+1] = pwm_pos[NUM_EXTRUDER+1])>0) WRITE(FAN_BOARD_PIN,1);
#endif
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
        if((pwm_pos_set[NUM_EXTRUDER+2] = pwm_pos[NUM_EXTRUDER+2])>0) WRITE(FAN_PIN,1);
#endif
    }
#if EXT0_HEATER_PIN>-1
    if(pwm_pos_set[0] == pwm_count_heater && pwm_pos_set[0]!=HEATER_PWM_MASK) WRITE(EXT0_HEATER_PIN,HEATER_PINS_INVERTED);
#if EXT0_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[0] == pwm_count && pwm_cooler_pos_set[0]!=255) WRITE(EXT0_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
    if(pwm_pos_set[1] == pwm_count_heater && pwm_pos_set[1]!=HEATER_PWM_MASK) WRITE(EXT1_HEATER_PIN,HEATER_PINS_INVERTED);
#if EXT1_EXTRUDER_COOLER_PIN>-1 && EXT1_EXTRUDER_COOLER_PIN!=EXT0_EXTRUDER_COOLER_PIN
    if(pwm_cooler_pos_set[1] == pwm_count && pwm_cooler_pos_set[1]!=255) WRITE(EXT1_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
    if(pwm_pos_set[2] == pwm_count_heater && pwm_pos_set[2]!=HEATER_PWM_MASK) WRITE(EXT2_HEATER_PIN,HEATER_PINS_INVERTED);
#if EXT2_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[2] == pwm_count && pwm_cooler_pos_set[2]!=255) WRITE(EXT2_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER>3
    if(pwm_pos_set[3] == pwm_count_heater && pwm_pos_set[3]!=HEATER_PWM_MASK) WRITE(EXT3_HEATER_PIN,HEATER_PINS_INVERTED);
#if EXT3_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[3] == pwm_count && pwm_cooler_pos_set[3]!=255) WRITE(EXT3_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1 && NUM_EXTRUDER>4
    if(pwm_pos_set[4] == pwm_count_heater && pwm_pos_set[4]!=HEATER_PWM_MASK) WRITE(EXT4_HEATER_PIN,HEATER_PINS_INVERTED);
#if EXT4_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[4] == pwm_count && pwm_cooler_pos_set[4]!=255) WRITE(EXT4_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER>5
    if(pwm_pos_set[5] == pwm_count_heater && pwm_pos_set[5]!=HEATER_PWM_MASK) WRITE(EXT5_HEATER_PIN,HEATER_PINS_INVERTED);
#if EXT5_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[5] == pwm_count && pwm_cooler_pos_set[5]!=255) WRITE(EXT5_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if FAN_BOARD_PIN>-1
    if(pwm_pos_set[NUM_EXTRUDER+1] == pwm_count && pwm_pos_set[NUM_EXTRUDER+1]!=255) WRITE(FAN_BOARD_PIN,0);
#endif
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
    if(pwm_pos_set[NUM_EXTRUDER+2] == pwm_count && pwm_pos_set[NUM_EXTRUDER+2]!=255) WRITE(FAN_PIN,0);
#endif
#if HEATED_BED_HEATER_PIN>-1 && HAVE_HEATED_BED
    if(pwm_pos_set[NUM_EXTRUDER] == pwm_count_heater && pwm_pos_set[NUM_EXTRUDER]!=HEATER_PWM_MASK) WRITE(HEATED_BED_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
    HAL::allowInterrupts();
    counterPeriodical++; // Appxoimate a 100ms timer
    if(counterPeriodical >= 390) //  (int)(F_CPU/40960))
    {
        counterPeriodical=0;
        executePeriodical=1;
    }
// read analog values -- only read one per interrupt
#if ANALOG_INPUTS>0
    // conversion finished? //osAnalogInputChannels[osAnalogInputPos]
    //if(ADC->INTFLAG.bit.RESRDY == 1) 
    {                
      //osAnalogInputChannels
        osAnalogInputBuildup[osAnalogInputPos] += ADC->RESULT.reg;
        
        if(++osAnalogInputCounter[osAnalogInputPos] >= (1 << ANALOG_INPUT_SAMPLE))
        {
#if ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE<12
            osAnalogInputValues[osAnalogInputPos] =
                osAnalogInputBuildup[osAnalogInputPos] <<
                (12-ANALOG_INPUT_BITS-ANALOG_INPUT_SAMPLE);
#endif
#if ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE>12
            osAnalogInputValues[osAnalogInputPos] = 
                osAnalogInputBuildup[osAnalogInputPos] >>
                (ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE-12);
#endif
#if ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE==12
            osAnalogInputValues[osAnalogInputPos] =
                osAnalogInputBuildup[osAnalogInputPos];
#endif
            osAnalogInputBuildup[osAnalogInputPos] = 0;
            osAnalogInputCounter[osAnalogInputPos] = 0;
        }
        // Start next conversion cycle
        if(++osAnalogInputPos>=ANALOG_INPUTS) { 
            osAnalogInputPos = 0;
        }
        // change to next os
        ADC->INTFLAG.bit.RESRDY = 1;
        int ulPin = osAnalogInputChannels[osAnalogInputPos];
        ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ulPin].ulADCChannelNumber;
        ADC->SWTRIG.bit.START = 1;
    }
#endif


    UI_FAST; // Short timed user interface action
    pwm_count++;
    pwm_count_heater += HEATER_PWM_STEP;
}

/** \brief Timer routine for extruder stepper.

Several methods need to move the extruder. To get a optimal 
result, all methods update the printer_state.extruderStepsNeeded 
with the number of additional steps needed. During this 
interrupt, one step is executed. This will keep the extruder 
moving, until the total wanted movement is achieved. This will 
be done with the maximum allowable speed for the extruder. 
*/
#if defined(USE_ADVANCE)
// EXTRUDER_TIMER IRQ handler
void EXTRUDER_TIMER_VECTOR ()
{
    static int8_t extruderLastDirection = 0;
    EXTRUDER_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;

    if(!Printer::isAdvanceActivated()) return; // currently no need
    // get current extruder timer count value
    uint32_t timer = EXTRUDER_TIMER->COUNT16.COUNT.reg;

    if(!Printer::isAdvanceActivated()) return; // currently no need
    if(Printer::extruderStepsNeeded > 0 && extruderLastDirection!=1)
    {
        Extruder::setDirection(true);
        extruderLastDirection = 1;
        timer += 40; // Add some more wait time to prevent blocking
    }

    else if(Printer::extruderStepsNeeded < 0 && extruderLastDirection!=-1)
    {
        Extruder::setDirection(false);
        extruderLastDirection = -1;
        timer += 40; // Add some more wait time to prevent blocking
    }
    else if(Printer::extruderStepsNeeded != 0)
    {
        Extruder::step();
        Printer::extruderStepsNeeded -= extruderLastDirection;
        Printer::insertStepperHighDelay();
        Extruder::unstep();
    }
    EXTRUDER_TIMER->COUNT16.COUNT.reg = timer;

}
#endif



