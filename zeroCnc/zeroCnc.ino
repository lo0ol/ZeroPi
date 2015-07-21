#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "FreeRTOSConfig.h"


#include "serialManager.h"
#include "machineManager.h"


void setup() {
	serialInit();
	machineInit();
	/* Start the scheduler. */
	vTaskStartScheduler();
	/* Will only get here if there was not enough heap space to create the idle task. */
}

void loop() {

}
