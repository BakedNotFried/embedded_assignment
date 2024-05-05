/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/* Hardware includes. */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"
#include "drivers/opt3001.h"
#include "driverlib/fpu.h"

/* Display includes. */
#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"

/*-----------------------------------------------------------*/
// Binary Semaphore for the Optical Sensor and I2C Read
extern SemaphoreHandle_t xI2C0Semaphore;
extern SemaphoreHandle_t xOpticReadSemaphore;

// Configuration for the OPT3001 sensor
static void prvConfigureOPT3001Sensor( void );

// Task for reading and displaying optical sensor values
static void vPeriodicOpticRead( void *pvParameters );

// Called by main function to setup Sensor Tasks
void vSensorTaskSetup( void );

/*-----------------------------------------------------------*/

void vSensorTaskSetup( void )
{
    // // Create queue
    // xStructQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof( xOpticalMessage ) );

    // Create the task to read the optical sensor
    xTaskCreate( vOpt3001Read,
                 "Opt 3001 Read Task",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );
    
}
/*-----------------------------------------------------------*/
// Periodic Optical Read Task
static void vOpt3001Read( void *pvParameters )
{
    // Configure Optical Sensor
    // Note if using semaphores for read/write, they need to be called within a RTOS task
    prvConfigureOptical();

    // Vars for OPT3001 sensor read
    float convertedLux = 0;
    uint16_t rawData = 0;
    bool success;
    // Array for filtered values
    int filteredLux[10] = {0};
    int index = 0;

    for( ;; )
    {
        // wait on semaphore
        if (xSemaphoreTake(xOpticReadSemaphore, portMAX_DELAY) == pdPASS)
        {
            // // debug print
            // UARTprintf("Optical Read Task\n");

            //Read and convert OPT values
            success = sensorOpt3001Read(&rawData);

            if (success) {
                sensorOpt3001Convert(rawData, &convertedLux);
                int lux_int = (int)convertedLux;
                // UARTprintf("Lux: %5d\n", lux_int);

                // Filtered values
                filteredLux[index] = lux_int;
                index = (index + 1) % 10;
                int sum = 0;
                for (int i = 0; i < 10; i++) {
                    sum += filteredLux[i];
                }
                int average = sum / 10;
                // UARTprintf("Filtered Lux: %5d\n", average);

            }
        }
    }
}

/*-----------------------------------------------------------*/
// Configuration functions
static void prvConfigureOPT3001Sensor( void )
{
        // The I2C0 peripheral must be enabled before use.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

        // Configure the pin muxing for I2C0 functions on port B2 and B3.
        // This step is not necessary if your part does not support pin muxing.
        GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        GPIOPinConfigure(GPIO_PB3_I2C0SDA);

        // Select the I2C function for these pins.  This function will also
        // configure the GPIO pins pins for I2C operation, setting them to
        // open-drain operation with weak pull-ups.  Consult the data sheet
        // to see which functions are allocated per pin.
        GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
        GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

        // Interrupt Configuration for I2C
        IntEnable(INT_I2C0);
        I2CMasterIntEnable(I2C0_BASE);

        // // Enable Master interrupt
        IntMasterEnable();

        // // Test that sensor is set up correctly
        // UARTprintf("\nTesting OPT3001 Sensor:\n");
        // bool worked;
        // worked = sensorOpt3001Test();
        // while (!worked) {
        //     UARTprintf("\nTest Failed, Trying again\n");
        //     worked = sensorOpt3001Test();
        // }

        // UARTprintf("All Tests Passed!\n\n");

        // Initialize opt3001 sensor
        // sensorOpt3001Init();
        sensorOpt3001Enable(true);
}

/*-----------------------------------------------------------*/
// Interrupt Handlers
void xI2C0Handler( void )
{
    /* Clear the interrupt. */
    I2CMasterIntClear(I2C0_BASE);

    // // Get status of the I2C
    // uint32_t ui32Status;
    // ui32Status = I2CMasterIntStatus(I2C0_BASE, true);

    // Give the semaphore to unblock the I2C read
    BaseType_t xI2C0TaskWoken = pdFALSE;
    xI2C0TaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( xI2C0Semaphore, &xI2C0TaskWoken );
    portYIELD_FROM_ISR( xI2C0TaskWoken );
}

// Leave this empty
void vApplicationTickHook( void )
{
}