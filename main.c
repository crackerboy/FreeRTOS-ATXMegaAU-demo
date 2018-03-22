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
#include "timers.h"
#include "stream_buffer.h"
#include "message_buffer.h"
#include "queue.h"

/* Demo file headers. */
#include "partest.h"

#include "xray_tasks.h"

QueueHandle_t ResponseQueue = NULL;
MessageBufferHandle_t *UsartMessageBuffers = { NULL };

/** \{
 * TODO The following definitions are to migrate into appropriate subsystem files. */

/** Stream buffer with data just read from USART.
 * Basically is an input data FIFO.
 */
StreamBufferHandle_t UsartInputBuffer = NULL;
/** Input command message buffer. */
uint8_t *UsartInputCommand = NULL;

/** REX/DPS notifications buffer. */
StreamBufferHandle_t ABInputBuffer = NULL;

/** Card read input representation. */
MessageBufferHandle_t CardReadInputBuffer = NULL;
/** Card read interim data representation */
struct CardRead_t *CardRead = NULL;
/** CardRead timer. */
TimerHandle_t CardReadTimer = NULL;

/** Message buffer with input voltage value for averaging. */
MessageBufferHandle_t InputVoltageBuffer = NULL;
/** Message buffer with peripheral current value for averaging. */
MessageBufferHandle_t PeripheralCurrentBuffer = NULL;

/** Threshold check timer. */
TimerHandle_t ThresholdChecker = NULL;
/** Low overcurrent check timer. */
TimerHandle_t LowOvercurrent = NULL;
/** High overcurrent check timer. */
TimerHandle_t HighOvercurrent = NULL;

/** Gauge data and charge estimation representation. */
TimerHandle_t GaugeTimer = NULL;

/** \} */

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

enum {
	PRIORITY_IDLE,
	PRIORITY_NORMAL,
	PRIORITY_HIGH,
	PRIORITY_HIGHEST
};

/*
 * Set clock source and timing.
 */
static void prvSetupClockSource( void );

/* CommandProcessor task */
static portTASK_FUNCTION_PROTO( vCommandProcessorTask, pvParams );

/* ABProcessor task */
static portTASK_FUNCTION_PROTO( vABProcessorTask, pvParams );

/* CardReadProcessor task */
static portTASK_FUNCTION_PROTO( vCardReadProcessorTask, pvParams );

/* InputVoltageAverager task */
static portTASK_FUNCTION_PROTO( vInputVoltageAveragerTask, pvParams );

/* PeripheralCurrentAverager task */
static portTASK_FUNCTION_PROTO( vPeripheralCurrentAveragerTask, pvParams );

/* InputVoltageThresholdReporter task */
static portTASK_FUNCTION_PROTO( vInputVoltageThresholdReporterTask, pvParams );

/* HighOvercurrentReporter task */
static portTASK_FUNCTION_PROTO( vHighOvercurrentReporterTask, pvParams );

/* LowOvercurrentReporter task */
static portTASK_FUNCTION_PROTO( vLowOvercurrentReporterTask, pvParams );

/* Create minimum set of timers */
static void vCreateTimers( void );

/* Create all necessary communication channels */
static void vCreateCommChannels( void );

/* Start all XRay tasks */
static void vStartXRayTasks( void );

/*-----------------------------------------------------------*/

int main( void )
{
	prvSetupClockSource();

	/* Setup the LED's for output. */
	vParTestInitialise();

	/* Set initial LED 0 on to see malloc fails */
	vParTestSetLED(0, pdTRUE);

	/* Create communication channels */
	vCreateCommChannels();

	/* Start all tasks */
	vStartXRayTasks();

	/* Create timers */
	vCreateTimers();

	/* Allocate internal representations */
	/* TODO move out of here */
	UsartInputCommand = (uint8_t*) pvPortMalloc( COMMAND_PROCESSOR_IN_MESSAGE_MAX + 1 );
	CardRead = (struct CardRead_t*) pvPortMalloc( sizeof(struct CardRead_t) * N_PORTS );

	/* In this port, to use preemptive scheduler define configUSE_PREEMPTION
	as 1 in portmacro.h.  To use the cooperative scheduler define
	configUSE_PREEMPTION as 0. */
	vTaskStartScheduler();

	return 0;
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

void vApplicationMallocFailedHook( void );
void vApplicationMallocFailedHook( void )
{
	vParTestSetLED(0, pdFALSE);
}

/* CommandProcessor task */
static portTASK_FUNCTION( vCommandProcessorTask, pvParams )
{
	(void)pvParams;
	for (;;)
	{
		vTaskDelay(1);
	}
}

/* ABProcessor task */
static portTASK_FUNCTION( vABProcessorTask, pvParams )
{
	(void)pvParams;
	for (;;)
	{
		vTaskDelay(1);
	}
}

/* CardReadProcessor task */
static portTASK_FUNCTION( vCardReadProcessorTask, pvParams )
{
	(void)pvParams;
	for (;;)
	{
		vTaskDelay(1);
	}
}

/* InputVoltageAverager task */
static portTASK_FUNCTION( vInputVoltageAveragerTask, pvParams )
{
	(void)pvParams;
	for (;;)
	{
		vTaskDelay(1);
	}
}

/* InputVoltageThresholdReporter task */
static portTASK_FUNCTION( vInputVoltageThresholdReporterTask, pvParams )
{
	(void)pvParams;
	for (;;)
	{
		vTaskDelay(1);
	}
}

/* HighOvercurrentReporter task */
static portTASK_FUNCTION( vHighOvercurrentReporterTask, pvParams )
{
	(void)pvParams;
	for (;;)
	{
		vTaskDelay(1);
	}
}

/* LowOvercurrentReporter task */
static portTASK_FUNCTION( vLowOvercurrentReporterTask, pvParams )
{
	(void)pvParams;
	for (;;)
	{
		vTaskDelay(1);
	}
}

/* PeripheralCurrentAverager task */
static portTASK_FUNCTION( vPeripheralCurrentAveragerTask, pvParams )
{
	(void)pvParams;
	for (;;)
	{
		vTaskDelay(1);
	}
}

static void vStartXRayTasks( void )
{
	xTaskCreate( vCommandProcessorTask, "CPr", configMINIMAL_STACK_SIZE, NULL, PRIORITY_NORMAL, ( TaskHandle_t * ) NULL );
	xTaskCreate( vABProcessorTask, "ABP", configMINIMAL_STACK_SIZE, NULL, PRIORITY_HIGHEST, ( TaskHandle_t * ) NULL );
	xTaskCreate( vCardReadProcessorTask, "CRP", configMINIMAL_STACK_SIZE, NULL, PRIORITY_HIGHEST, ( TaskHandle_t * ) NULL );
	xTaskCreate( vInputVoltageAveragerTask, "IVA", configMINIMAL_STACK_SIZE, NULL, PRIORITY_HIGHEST, ( TaskHandle_t * ) NULL );
	xTaskCreate( vPeripheralCurrentAveragerTask, "PCA", configMINIMAL_STACK_SIZE, NULL, PRIORITY_HIGHEST, ( TaskHandle_t * ) NULL );
	xTaskCreate( vInputVoltageThresholdReporterTask, "IVR", configMINIMAL_STACK_SIZE, NULL, PRIORITY_NORMAL, ( TaskHandle_t * ) NULL );
	xTaskCreate( vLowOvercurrentReporterTask, "LOR", configMINIMAL_STACK_SIZE, NULL, PRIORITY_NORMAL, ( TaskHandle_t * ) NULL );
	xTaskCreate( vHighOvercurrentReporterTask, "HOR", configMINIMAL_STACK_SIZE, NULL, PRIORITY_NORMAL, ( TaskHandle_t * ) NULL );
}

static void vCardReadTimerCallback( TimerHandle_t xTimer )
{
	(void) xTimer;
}

static void vThresholdCheckerCallback( TimerHandle_t xTimer )
{
	(void) xTimer;
}

static void vLowOvercurrentCallback( TimerHandle_t xTimer )
{
	(void) xTimer;
}

static void vHighOvercurrentCallback( TimerHandle_t xTimer )
{
	(void) xTimer;
}

static void vGaugeTimerCallback( TimerHandle_t xTimer )
{
	(void) xTimer;
}

static void vCreateTimers( void )
{
	CardReadTimer = xTimerCreate( "CRT", pdMS_TO_TICKS( CARD_READ_TIMEOUT ), pdTRUE, NULL, vCardReadTimerCallback );
	ThresholdChecker = xTimerCreate( "TCT", pdMS_TO_TICKS( THRESHOLD_CHECKER_TIMEOUT ), pdTRUE, NULL, vThresholdCheckerCallback );
	LowOvercurrent = xTimerCreate( "LOT", pdMS_TO_TICKS( LOW_OVERCURRENT_TIMEOUT ), pdFALSE, NULL, vLowOvercurrentCallback );
	HighOvercurrent = xTimerCreate( "HOT", pdMS_TO_TICKS( HIGH_OVERCURRENT_TIMEOUT ), pdFALSE, NULL, vHighOvercurrentCallback );
	GaugeTimer = xTimerCreate( "GT", pdMS_TO_TICKS( GAUGE_TIMER_TIMEOUT ), pdTRUE, NULL, vGaugeTimerCallback );
}

static void vCreateCommChannels( void )
{
	/* ResponseQueue priority queue */
	ResponseQueue = xQueueCreate( USART_BUFFERS_COUNT, 1 );

	/* Create stream buffers */

	/* USART input */
	UsartInputBuffer = xStreamBufferCreate( USART_INPUT_BUFFER_LEN, 1 );
	/* AB input */
	ABInputBuffer = xStreamBufferCreate( AB_BUFFER_LEN, 1 );

	/* Create message buffers */

	/* USART output */
	UsartMessageBuffers = pvPortMalloc( USART_BUFFERS_COUNT * sizeof( MessageBufferHandle_t* ));
	UsartMessageBuffers[AB_PROCESSOR_TASK_ID] = xMessageBufferCreate( AB_MESSAGE_LEN + sizeof(size_t) );
	UsartMessageBuffers[CARD_READ_PROCESSOR_TASK_ID] = xMessageBufferCreate( 2 * ( CARD_READ_MESSAGE_LEN + sizeof(size_t) ) );
	UsartMessageBuffers[COMMAND_PROCESSOR_TASK_ID] = xMessageBufferCreate( COMMAND_PROCESSOR_OUT_MESSAGE_MAX + sizeof(size_t) );
	UsartMessageBuffers[VOLTAGE_THRESHOLD_REPORTER_TASK_ID] = xMessageBufferCreate( VOLTAGE_THRESHOLD_MESSAGE_MAX + sizeof(size_t) );
	UsartMessageBuffers[LOW_OVERCURRENT_REPORTER_TASK_ID] = xMessageBufferCreate( OVERCURRENT_MESSAGE_MAX + sizeof(size_t) );
	UsartMessageBuffers[HIGH_OVERCURRENT_REPORTER_TASK_ID] = xMessageBufferCreate( OVERCURRENT_MESSAGE_MAX + sizeof(size_t) );

	/* Card read input messages */
	CardReadInputBuffer = xMessageBufferCreate( CARD_READ_BUFFER_LEN );
	/* Input voltage averager input data */
	InputVoltageBuffer = xMessageBufferCreate( INPUT_VOLTAGE_AVERAGER_RAW_LEN );
	/* Peripheral current averager input data */
	PeripheralCurrentBuffer = xMessageBufferCreate( PERIPHERAL_CURRENT_AVERAGER_RAW_LEN );
}
