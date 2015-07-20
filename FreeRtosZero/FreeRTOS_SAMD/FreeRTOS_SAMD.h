/**
 * \file
 * \brief FreeRTOS for Arduino ZERO
 */
#ifndef FreeRTOS_SAMD_h
#define FreeRTOS_SAMD_h

#ifndef ARDUINO_ARCH_SAMD
#error Arduino ZERO required
#else  // __arm__
//------------------------------------------------------------------------------
/** FreeRTOS_ARM version YYYYMMDD */
#define FREE_RTOS_ARM_VERSION 20150710
//------------------------------------------------------------------------------
#include "utility/FreeRTOS.h"
#include "utility/task.h"
#include "utility/queue.h"
#include "utility/semphr.h"
#include "utility/portmacro.h"
#include "basic_io_arm.h"

#endif  // ARDUINO_ARCH_SAMD
#endif  // FreeRTOS_ARM_h