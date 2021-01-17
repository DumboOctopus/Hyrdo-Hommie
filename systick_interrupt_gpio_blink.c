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
#include <stddef.h>
#include <stdbool.h>

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
    MAP_SysTick_setPeriod(1500000);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_SysTick_enableInterrupt();
    
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

    /* Enabling MASTER interrupts */
    MAP_Interrupt_enableMaster();  

    /** Setting up UART terminal */
//    Display_Handle    displayHandle;
//    Display_Params    displayParams;
//
//    Display_Params_init(&displayParams);
//    displayHandle = Display_open(Display_Type_UART, NULL);

    volatile uint8_t x;
    while (1)
    {
        //MAP_PCM_gotoLPM0();
        x = MAP_GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN1);
//        Display_printf(displayHandle, 1, 0, "ADC Reading %d", x);
    }
}

void SysTick_Handler(void)
{
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

}

