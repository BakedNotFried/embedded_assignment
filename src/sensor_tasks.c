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

// /* Display includes. */
// #include "grlib/grlib.h"
// #include "grlib/widget.h"
// #include "grlib/canvas.h"
// #include "drivers/Kentec320x240x16_ssd2119_spi.h"
// #include "drivers/touch.h"

/*-----------------------------------------------------------*/
// Binary Semaphore for I2C0 non-blocking read/write
extern SemaphoreHandle_t xI2C0Semaphore;

// Function handle for task notification from Timer interrupt
TaskHandle_t xOpt3001ReadHandle;
TaskHandle_t xIMUReadHandle;

// Queues for Sensor Data Publishing
QueueHandle_t xOPT3001Queue = NULL;

// Structs for Sensor Data Publishing
struct OPT3001Message
{
    uint32_t ulfilteredLux;
} xOPT3001Message;

// Configuration for the OPT3001 sensor
static void prvConfigureOPT3001Sensor( void );

// Configuration for the HW Timer 6A and 7A
static void prvConfigureHWTimer6A( void );
static void prvConfigureHWTimer7A( void );

// Task for reading the opt3001 sensor
static void vOPT3001Read( void *pvParameters );
static void vIMURead( void *pvParameters );

// Called by main function to setup Sensor Tasks
void vSensorTaskSetup( void );

/*-----------------------------------------------------------*/

void vSensorTaskSetup( void )
{
    // Create queue
    xOPT3001Queue = xQueueCreate(1, sizeof(struct OPT3001Message));

    // Create the task to read the optical sensor
    xTaskCreate( vOPT3001Read,
                 "Opt 3001 Read Task",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 &xOpt3001ReadHandle);
    
    // Create the task to read the IMU sensor
    xTaskCreate( vIMURead,
                 "IMU Read Task",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 &xIMUReadHandle);
    
}
/*-----------------------------------------------------------*/
// Periodic Optical Read Task
static void vOPT3001Read( void *pvParameters )
{
    // Configure Optical Sensor
    // Note if using semaphores for read/write, they need to be called within a RTOS task
    prvConfigureOPT3001Sensor();

    // Configure Timer 7A
    prvConfigureHWTimer7A();

    // Vars for OPT3001 sensor read
    float convertedLux = 0;
    uint16_t rawData = 0;
    bool success;
    // Array for filtered values
    int lux_array[10] = {0};
    int index = 0;

    // Queue setup
    struct OPT3001Message xOPT3001Message;

    // Enable the Timer 7A
    TimerEnable(TIMER7_BASE, TIMER_A);
    // TickType_t xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        // Wait for notification from Timer7A interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        {
            // // Calc time passed
            // TickType_t xCurrentTime = xTaskGetTickCount();
            // TickType_t xElapsedTime = (xCurrentTime - xLastWakeTime) * portTICK_PERIOD_MS;
            // UARTprintf("Time Passed: %d ms\n", xElapsedTime);
            // xLastWakeTime = xCurrentTime;

            //Read and convert OPT values
            success = sensorOpt3001Read(&rawData);

            if (success) {
                sensorOpt3001Convert(rawData, &convertedLux);
                int lux_int = (int)convertedLux;

                // Filter values
                lux_array[index] = lux_int;
                index = (index + 1) % 10;
                int sum = 0;
                for (int i = 0; i < 10; i++) {
                    sum += lux_array[i];
                }
                int filtered_lux = sum / 10;
                // UARTprintf("Filtered Lux: %5d\n", filtered_lux);

                // Publish to Queue
                xOPT3001Message.ulfilteredLux = filtered_lux;
                xQueueSend(xOPT3001Queue, ( void * ) &xOPT3001Message, ( TickType_t ) 0 );
            }
        }
    }
}

// Periodic IMU Read Task
static void vIMURead( void *pvParameters )
{
    // Configure Timer 6A
    prvConfigureHWTimer6A();

    // Enable the Timer 6A
    TimerEnable(TIMER6_BASE, TIMER_A);
    // TickType_t xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        // // Calc time passed
        // TickType_t xCurrentTime = xTaskGetTickCount();
        // TickType_t xElapsedTime = (xCurrentTime - xLastWakeTime) * portTICK_PERIOD_MS;
        // UARTprintf("Time Passed: %d ms\n", xElapsedTime);
        // xLastWakeTime = xCurrentTime;

        // Wait for notification from Timer7B interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        {
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

static void prvConfigureHWTimer6A( void )
{
    /* The Timer 6 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER6);

    /* Configure Timer 6 in full-width periodic mode. */
    TimerConfigure(TIMER6_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 6A load value to run at 100 Hz. */
    TimerLoadSet(TIMER6_BASE, TIMER_A, configCPU_CLOCK_HZ / 100);

    /* Configure the Timer 6A interrupt for timeout. */
    TimerIntEnable(TIMER6_BASE, TIMER_TIMA_TIMEOUT);

    /* Enable the Timer 6A interrupt in the NVIC. */
    IntEnable(INT_TIMER6A);

    // /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}

static void prvConfigureHWTimer7A( void )
{
    /* The Timer 7 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER7);

    /* Configure Timer 7 in full-width periodic mode. */
    TimerConfigure(TIMER7_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 7A load value to run at 2 Hz. */
    TimerLoadSet(TIMER7_BASE, TIMER_A, configCPU_CLOCK_HZ / 2);

    /* Configure the Timer 7A interrupt for timeout. */
    TimerIntEnable(TIMER7_BASE, TIMER_TIMA_TIMEOUT);

    /* Enable the Timer 7A interrupt in the NVIC. */
    IntEnable(INT_TIMER7A);

    // /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
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

void xTimer6AHandler( void )
{
    /* Clear the hardware interrupt flag for Timer 7B. */
    TimerIntClear(TIMER6_BASE, TIMER_TIMA_TIMEOUT);

    // Notify the task to read the IMU sensor
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xIMUReadHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // debug print
    // UARTprintf("Timer 6A Interrupt\n");
}

void xTimer7AHandler( void )
{
    /* Clear the hardware interrupt flag for Timer 7A. */
    TimerIntClear(TIMER7_BASE, TIMER_TIMA_TIMEOUT);

    // Notify the task to read the OPT3001 sensor
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xOpt3001ReadHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // debug print
    // UARTprintf("Timer 7A Interrupt\n");
}



// Leave this empty
void vApplicationTickHook( void )
{
}