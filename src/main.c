/*
/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/* Hardware includes. */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"
#include "drivers/opt3001.h"
/*-----------------------------------------------------------*/
/*                         Shared Global Section             */
/* The system clock frequency. */
uint32_t g_ui32SysClock;

// UART Setup
static void prvConfigureUART(void);

// Hardware setup. Sets clock, resets pins and init UART
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/
/*                   Sensor Tasks Global Section             */

// Global Semaphores for I2C0 non-blocking read/write
SemaphoreHandle_t xI2C0OPTSemaphore = NULL;
SemaphoreHandle_t xI2C0BMISemaphore = NULL;
// Global Mutex for I2C0
SemaphoreHandle_t xI2C0Mutex = NULL;

// Functions for initializing tasks
extern void vSensorTaskSetup( void );

/*-----------------------------------------------------------*/
/*                   Motor Tasks Global Section              */


/*-----------------------------------------------------------*/
/*                   GUI Tasks Global Section                */

/*-----------------------------------------------------------*/
int main( void )

{
    /*                   Shared Main Setup                     */
    prvSetupHardware();

    /*                   Sensor Main Setup (Cal)               */
    // Semaphore for I2C non-blocking read/write
    xI2C0OPTSemaphore = xSemaphoreCreateBinary();
    xI2C0BMISemaphore = xSemaphoreCreateBinary();
    // Mutex for I2C
    xI2C0Mutex = xSemaphoreCreateMutex();

    if ( (xI2C0OPTSemaphore != NULL) && (xI2C0BMISemaphore != NULL) && (xI2C0Mutex != NULL) )
    {
        // Create the tasks associated with sensor reading
        vSensorTaskSetup();

    }
    /*-----------------------------------------------------------*/


    /*                   Motor Main Setup (Jim)                  */

    /*-----------------------------------------------------------*/


    /*                   GUI Main Setup (Nikolaj)                */

    /*-----------------------------------------------------------*/

    // Start the scheduler
    vTaskStartScheduler();
    // Should never reach here
    for( ;; );
}

/*-----------------------------------------------------------*/
/*                    Initial Hardware Setup                 */
static void prvConfigureUART(void)
{
    /* Enable GPIO port A which is used for UART0 pins.
     * TODO: change this to whichever GPIO port you are using. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /* Configure the pin muxing for UART0 functions on port A0 and A1.
     * This step is not necessary if your part does not support pin muxing.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    /* Enable UART0 so that we can configure the clock. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Use the internal 16MHz oscillator as the UART clock source. */
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    /* Select the alternate (UART) function for these pins.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Initialize the UART for console I/O. */
    UARTStdioConfig(0, 9600, 16000000);
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    /* Run from the PLL at configCPU_CLOCK_HZ MHz. */
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
            SYSCTL_CFG_VCO_240), configCPU_CLOCK_HZ);

    /* Configure device pins. */
    PinoutSet(false, false);

    /* Configure UART0 to send messages to terminal. */
    prvConfigureUART();
}

/*-----------------------------------------------------------*/
/*                 RTOS Shenanigans                          */
void vApplicationMallocFailedHook( void )
{
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/

void *malloc( size_t xSize )
{
    /* There should not be a heap defined, so trap any attempts to call
    malloc. */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/


