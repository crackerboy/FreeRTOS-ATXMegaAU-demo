/*
 * FreeRTOS V10.0.0 demo application for ATXMega AU chip family.
 *
 * Copyright (C) 2018 ProdataKey, Inc.
 *
 * Adopted to ATXMega AU chip family from supported demo.
 */
/*
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. If you wish to use our Amazon
 * FreeRTOS name, please do so in a fair use way that does not cause confusion.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 *
 * Main. c also creates a task called "Check".  This only executes every six
 * seconds but has the highest priority so is guaranteed to get processor time.
 * Its main function is to check that all the other tasks are still operational.
 * Each task that does not flash an LED maintains a unique count that is
 * incremented each time the task successfully completes its function.  Should
 * any error occur within such a task the count is permanently halted.  The
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have
 * changed all the tasks are still executing error free, and the check task
 * toggles an LED.  Should any task contain an error at any time the LED toggle
 * will stop.
 *
 * The LED flash and communications test tasks do not maintain a count.
 */

#include <stdlib.h>
#include <string.h>

#ifdef GCC_XMEGAAU_AVR
	#include <avr/io.h>
#endif

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

/* Demo file headers. */
#include "integer.h"
#include "crflash.h"
#include "partest.h"

/* Priority definitions for most of the tasks in the demo application.  Some
tasks just use the idle priority. */
#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 3 )

/* LED that is toggled by the check task.  The check task periodically checks
that all the other tasks are operating without error.  If no errors are found
the LED is toggled.  If an error is found at any time the LED is never toggles
again. */
#define mainCHECK_TASK_LED				( 2 )

/* The period between executions of the check task. */
#define mainCHECK_PERIOD				( ( TickType_t ) 6000 / portTICK_PERIOD_MS )

/* The number of coroutines to create. */
#define mainNUM_FLASH_COROUTINES		( 2 )

/*
 * The task function for the "Check" task.
 */
static void vErrorChecks( void *pvParameters );

/*
 * Checks the unique counts of other tasks to ensure they are still operational.
 * Flashes an LED if everything is okay.
 */
static void prvCheckOtherTasksAreStillRunning( void );

/*
 * The idle hook is used to scheduler co-routines.
 */
void vApplicationIdleHook( void );

/*
 * Set clock source and timing.
 */
static void prvSetupClockSource( void );

/*-----------------------------------------------------------*/

int main( void )
{
	prvSetupClockSource();

	//prvIncrementResetCount();

	/* Setup the LED's for output. */
	vParTestInitialise();

	/* Create the standard demo tasks. */
	vStartIntegerMathTasks( tskIDLE_PRIORITY );

	/* Create the tasks defined within this file. */
	xTaskCreate( vErrorChecks, "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );

	/* Create the co-routines that flash the LED's. */
	vStartFlashCoRoutines( mainNUM_FLASH_COROUTINES );

	/* In this port, to use preemptive scheduler define configUSE_PREEMPTION
	as 1 in portmacro.h.  To use the cooperative scheduler define
	configUSE_PREEMPTION as 0. */
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

static void vErrorChecks( void *pvParameters )
{
static volatile unsigned long ulDummyVariable = 3UL;

	( void ) pvParameters; // The parameters are not used.

	// Cycle for ever, delaying then checking all the other tasks are still
	// operating without error.
	for( ;; )
	{
		vTaskDelay( mainCHECK_PERIOD );

		// Perform a bit of 32bit maths to ensure the registers used by the
		// integer tasks get some exercise. The result here is not important -
		// see the demo application documentation for more info.
		ulDummyVariable *= 3;

		prvCheckOtherTasksAreStillRunning();
	}
}
/*-----------------------------------------------------------*/

static void prvCheckOtherTasksAreStillRunning( void )
{
static portBASE_TYPE xErrorHasOccurred = pdFALSE;

	if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
	{
		xErrorHasOccurred = pdTRUE;
	}

	if( xErrorHasOccurred == pdFALSE )
	{
		// Toggle the LED if everything is okay so we know if an error occurs
		// even if not using console IO.
		vParTestToggleLED( mainCHECK_TASK_LED );
	}
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	vCoRoutineSchedule();
}
/*-----------------------------------------------------------*/

void vCCPWrite(volatile unsigned char* address, unsigned char value);
void vCCPWrite(volatile unsigned char* address, unsigned char value)
{
volatile unsigned char* tmpAddr = address;

	portENTER_CRITICAL();

#ifdef RAMPZ
	RAMPZ = 0;
#endif

	asm volatile(
		"movw r30,  %0"	      "\n\t"
		"ldi  r16,  %2"	      "\n\t"
		"out   %3, r16"	      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" ((unsigned short)(CCP_IOREG_gc)), "i" (&CCP)
		: "r16", "r30", "r31"
	);

	portEXIT_CRITICAL();
}

static void prvSetupClockSource( void )
{
	// Setup 32MHz internal clock
	OSC.CTRL |= OSC_RC32MEN_bm;
	while ( !(OSC.STATUS & OSC_RC32MEN_bm) ) ;

	// Setup 32kHz internal clock
	OSC.CTRL |= OSC_RC32KEN_bm;
	while ( !(OSC.STATUS & OSC_RC32KEN_bm) ) ;

	OSC.DFLLCTRL |= OSC_RC32MCREF_RC32K_gc;
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;

	vCCPWrite( &CLK.PSCTRL, CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc);

	{
		uint8_t clkCtrl = ( CLK.CTRL & ~CLK_SCLKSEL_gm) | CLK_SCLKSEL_RC32M_gc;
		vCCPWrite( &CLK.CTRL, clkCtrl);
	}

}
