/*
 * FreeRTOS Kernel V10.0.0
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
Changes from V2.0.0

	+ Use scheduler suspends in place of critical sections.

Changes from V2.6.0

	+ Replaced the inb() and outb() functions with direct memory
	  access.  This allows the port to be built with the 20050414 build of
	  WinAVR.
*/

#include "FreeRTOS.h"
#include "task.h"
#include "partest.h"

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

// Maximum LED index
#define partstMAX_OUTPUT_LED			( ( unsigned char ) 4 )

/*-----------------------------------------------------------*/

void vParTestInitialise( void )
{
	/* Set port D direction to outputs.  Start with all output off. */
	PORTD.DIRSET = PIN2_bm | PIN5_bm | PIN3_bm;
	PORTD.OUTCLR = PIN2_bm | PIN5_bm | PIN3_bm;

	PORTC.DIRSET = PIN7_bm;
	PORTC.OUTCLR = PIN7_bm;
}
/*-----------------------------------------------------------*/

void vParTestSetLED( UBaseType_t uxLED, BaseType_t xValue )
{
	if( uxLED <= partstMAX_OUTPUT_LED )
	{
	unsigned char ucBit = ( unsigned char ) ( ( uxLED == 0 ) ? PIN2_bm : PIN5_bm );
	PORT_t *port = (uxLED < partstMAX_OUTPUT_LED) ? &PORTD : &PORTC;

		vTaskSuspendAll();
		{
			if( xValue == pdTRUE )
			{
				port->OUTSET = ucBit;
			}
			else
			{
				port->OUTCLR = ucBit;
			}
		}
		xTaskResumeAll();
	}
}
/*-----------------------------------------------------------*/

void vParTestToggleLED( UBaseType_t uxLED )
{
		vTaskSuspendAll();
		{
			if ( uxLED < 2 )
			{
				PORTD.OUTTGL = ( ( uxLED & 1 ) == 1 ) ? PIN5_bm : PIN2_bm;
			}
			else
			{
				if ( (uxLED & 1) == 1 )
				{
					PORTD.OUTTGL = PIN3_bm;
				}
				else
				{
					PORTC.OUTTGL = PIN7_bm;
				}
			}
		}
		xTaskResumeAll();
}
