/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 * MSP432 SysTick - Blink LED Program
 *
 * Description: This program will use the SysTick module to blink an LED with
 * a one second period. Once setup, the application will go into LPM3
 * mode and only wake up to toggle the GPIO pin.
 *
 * This program runs infinitely until manually halted by the user.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P1.0  |---> P1.0 LED
 *            |                  |
 *      VCC   |                  |
 *      |     |                  |
 *      *---- | P4.1             |
 *      |     |                  |
 *      btn
 *      |
 *      GRD
  *******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/display/Display.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

volatile uint16_t millis_ultrasonic = 0;
volatile uint32_t grams_drank = 0;

/* Ultrasonic: Our ultrasonic only has 1 pin which is used both as trig and echo*/
#define ULTRASONIC_SIG_PORT GPIO_PORT_P4
#define ULTRASONIC_SIG_PIN GPIO_PIN0

static volatile int shared;

//![Simple SysTick Example]
int main(void)
{
    /* Halting the Watchdog */
    MAP_WDT_A_holdTimer();

    /* Configuring GPIO as an output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Configuring SysTick to trigger at 1500000 (MCLK is 1.5MHz so this will 
     * make it toggle every 1s) */
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(1500000); // 1.5M/1000 will make it toggle every microsecond
    //MAP_Interrupt_enableSleepOnIsrExit();
    //MAP_SysTick_enableInterrupt();
    
    /* on board button **/
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    MAP_Interrupt_enableInterrupt(INT_PORT1);

    /* external button */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN1);
    MAP_Interrupt_enableInterrupt(INT_PORT4);

    /* ultrasonic */
    MAP_GPIO_clearInterruptFlag(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);
    MAP_GPIO_enableInterrupt(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);
    MAP_Interrupt_enableInterrupt(ULTRASONIC_SIG_PORT);


    /* Enabling MASTER interrupts */
    MAP_Interrupt_enableMaster();  


    volatile uint8_t x;
    while (1)
    {
        //do something
        //printf("Starting ultrasonic reading..\n");

        // setup
        MAP_GPIO_setAsOutputPin(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);
        MAP_GPIO_disableInterrupt(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);
        MAP_GPIO_setOutputLowOnPin(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);
        Delay(2);

        // pulse
        int startPulse = MAP_SysTick_getValue();
        MAP_GPIO_setOutputHighOnPin(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);
        Delay(5);
        MAP_GPIO_setOutputLowOnPin(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);

        //now using ultrasonic to read
        MAP_GPIO_setAsInputPin(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);
        MAP_GPIO_clearInterruptFlag(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);
        MAP_GPIO_enableInterrupt(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);

        // function returns when sensor detects pulse
        Delay(10000);
        int diff = (shared - startPulse);
        if(diff < 0) diff = -diff;
        float mircos = diff / (float) (1500000/1000/1000); // how many micros
        float cm = diff / 29 / 2;

        if(cm < 45)
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        else
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

        printf("Diff %d\n", cm);
        Delay(100);



    }
}

void SysTick_Handler(void)
{

//    total_millis ++;
//    if(total_millis > 24*60*60*1000) { // one day has passed;
//        total_millis = 0;
//    }
    //MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
}
//![Simple SysTick Example]

/* GPIO ISR */

void PORT1_IRQHandler(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    /* Toggling the output on the LED */
    if(status & GPIO_PIN1)
    {

        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }

}

void PORT4_IRQHandler(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, status);

    /* Toggling the output on the LED */
    if(status & GPIO_PIN1)
    {
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }

    if(status & ULTRASONIC_SIG_PIN)
    {
        //printf("Recieved signal\n ");
        shared = MAP_SysTick_getValue();
    }

}


void Delay(uint32_t micros) {
    volatile uint32_t i;

        for (i = 0 ; i < micros ; i++);

}

uint32_t in_sensor(){

    MAP_GPIO_setAsInputPin(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);
    uint8_t x = MAP_GPIO_getInputPinValue(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN);

    printf("initial: %d\n", x);
    while( x == MAP_GPIO_getInputPinValue(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN)) {

    }
    printf("final: %d\n", MAP_GPIO_getInputPinValue(ULTRASONIC_SIG_PORT, ULTRASONIC_SIG_PIN));
    return x;
}
