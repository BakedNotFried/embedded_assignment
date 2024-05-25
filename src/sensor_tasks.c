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

// BMI include
#include "drivers/i2cBMIDriver.h"
#include "drivers/bmi160.h"
#include "drivers/bmi160_defs.h"

// /* Display includes. */
// #include "grlib/grlib.h"
// #include "grlib/widget.h"
// #include "grlib/canvas.h"
// #include "drivers/Kentec320x240x16_ssd2119_spi.h"
// #include "drivers/touch.h"

/*-----------------------------------------------------------*/
// Binary Semaphore for I2C0 non-blocking read/write
extern SemaphoreHandle_t xI2C0OPTSemaphore;
extern SemaphoreHandle_t xI2C0BMISemaphore;

// Semaphores for Sensor Tasks
SemaphoreHandle_t xI2CConfigSemaphore;
SemaphoreHandle_t xOPT3001ReadSemaphore;
SemaphoreHandle_t xBMI160ReadSemaphore;

// Mutex for I2C0
extern SemaphoreHandle_t xI2C0Mutex;

// enum for checking which sensor is using the I2C bus
enum sensorType {NONE, OPT3001, BMI160};
volatile enum sensorType g_I2C_flag = NONE;

// Function handle for task notification from Timer interrupt
TaskHandle_t xOpt3001ReadHandle;
TaskHandle_t xBMI160ReadHandle;

// Queues for Sensor Data Publishing
QueueHandle_t xOPT3001Queue = NULL;
QueueHandle_t xBMI160Queue = NULL;

// Structs for Sensor Data Publishing
struct OPT3001Message
{
    int filteredLux;
} xOPT3001Message;

struct BMI160Message
{
    int32_t ulfilteredAccel;
} xBMI160Message;

// Configuration for the OPT3001 sensor
static void prvConfigureOPT3001Sensor( void );

// Configuration for the HW Timer 6A and 7A
static void prvConfigureHWTimer6A( void );
static void prvConfigureHWTimer7A( void );

// Task for reading the opt3001 sensor
static void vOPT3001Read( void *pvParameters );
static void xBMI160Read( void *pvParameters );

// Called by main function to setup Sensor Tasks
void vSensorTaskSetup( void );

/*-----------------------------------------------------------*/

void vSensorTaskSetup( void )
{
    // Create queues for sensor data pub
    xOPT3001Queue = xQueueCreate(1, sizeof(struct OPT3001Message));
    xBMI160Queue = xQueueCreate(1, sizeof(struct BMI160Message));

    // Create semaphores for sensor tasks
    xI2CConfigSemaphore = xSemaphoreCreateBinary();
    xOPT3001ReadSemaphore = xSemaphoreCreateBinary();
    xBMI160ReadSemaphore = xSemaphoreCreateBinary();

    // Create the task to read the optical sensor
    xTaskCreate( vOPT3001Read,
                 "Opt 3001 Read/Pub Task",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 &xOpt3001ReadHandle);
    
    // Create the task to read the IMU sensor
    xTaskCreate( xBMI160Read,
                 "IMU Read/Pub Task",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 &xBMI160ReadHandle);
}
/*-----------------------------------------------------------*/
// Periodic Optical Read Task
static void vOPT3001Read( void *pvParameters )
{
    // Configure Optical Sensor
    // Note if using semaphores for read/write, they need to be called within a RTOS task
    prvConfigureOPT3001Sensor();

    // Semaphore for I2C Config
    xSemaphoreGive(xI2CConfigSemaphore);

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

    // Timer Enable
    TimerEnable(TIMER7_BASE, TIMER_A);
    for( ;; )
    {
        // Wait for notification from Timer7A interrupt
        if (xSemaphoreTake(xOPT3001ReadSemaphore, portMAX_DELAY) == pdTRUE)
        {
            if ( xSemaphoreTake( xI2C0Mutex, portMAX_DELAY ) == pdTRUE )
            {
                success = sensorOpt3001Read(&rawData);
                xSemaphoreGive( xI2C0Mutex );
            }
            if (success) 
            {
                sensorOpt3001Convert(rawData, &convertedLux);

                // Filter values
                lux_array[index] = (int)convertedLux;
                index = (index + 1) % 10;
                int sum = 0;
                for (int i = 0; i < 10; i++) {
                    sum += lux_array[i];
                }
                int filtered_lux = sum / 10;

                // Publish to Queue
                xOPT3001Message.filteredLux = filtered_lux;
                xQueueSend(xOPT3001Queue, &xOPT3001Message, 0);

                // DEBUG
                UARTprintf(">Lux:%d\n", filtered_lux);
            }
        }
    }
}

// Periodic IMU Read Task
static void xBMI160Read( void *pvParameters )
{
    // Wait for the I2C configuration semaphore
    xSemaphoreTake(xI2CConfigSemaphore, portMAX_DELAY);

    // Configure Timer 6A
    prvConfigureHWTimer6A();

    // Vars for accel x,y,z
    uint8_t x_lsb = 0x00;
    uint8_t x_msb = 0x00;
    uint8_t y_lsb = 0x00;
    uint8_t y_msb = 0x00;
    uint8_t z_lsb = 0x00;
    uint8_t z_msb = 0x00;
    int16_t int_x = 0x0000;
    int16_t int_y = 0x0000;
    int16_t int_z = 0x0000;
    bool success;

    // Array for filtered values
    int filter_len = 5;
    int accel_bias = 17740;
    int32_t accel_array[5] = {0};
    int index = 0;

    // Queue setup
    struct BMI160Message xBMI160Message;

    // BMI160 Config
    uint8_t normal_accl_mode = 0x11;                                // Normal Mode, Accel Only
    if ( xSemaphoreTake( xI2C0Mutex, portMAX_DELAY ) == pdTRUE )     // Take Mutex
    {
        writeI2CBMI(0x69, BMI160_COMMAND_REG_ADDR, &normal_accl_mode);   // Write to Command Register
        vTaskDelay( pdMS_TO_TICKS( 200 ) );                          // Delay for 200ms
        xSemaphoreGive( xI2C0Mutex );                                 // Give Mutex
    }

    // Timer Enable
    TimerEnable(TIMER6_BASE, TIMER_A);
    for( ;; )
    {
        // Wait for notification from Timer7B interrupt
        if (xSemaphoreTake(xBMI160ReadSemaphore, portMAX_DELAY) == pdTRUE)
        {
            if ( xSemaphoreTake( xI2C0Mutex, portMAX_DELAY ) == pdTRUE )     // Take Mutex
            {
                // Get Sensor Data for X,Y,Z
                success = readI2CBMI(0x69, 0x12, &x_lsb);
                success = readI2CBMI(0x69, 0x13, &x_msb);
                int_x = (int16_t)((x_msb << 8) | x_lsb);
                success = readI2CBMI(0x69, 0x14, &y_lsb);
                success = readI2CBMI(0x69, 0x15, &y_msb);
                int_y = (int16_t)((y_msb << 8) | y_lsb);
                success = readI2CBMI(0x69, 0x16, &z_lsb);
                success = readI2CBMI(0x69, 0x17, &z_msb);
                int_z = (int16_t)((z_msb << 8) | z_lsb);
                xSemaphoreGive( xI2C0Mutex );
            }

            if (success)
            {
                // Combine X,Y,Z and get magnitude
                int32_t accel = (int32_t)sqrt((int32_t)int_x*(int32_t)int_x + (int32_t)int_y*(int32_t)int_y + (int32_t)int_z*(int32_t)int_z);
                
                // Filter values
                accel_array[index] = accel;
                index = (index + 1) % filter_len;
                int32_t sum = 0;
                for (int i = 0; i < filter_len; i++) {
                    sum += accel_array[i];
                }
                int32_t filtered_accel = (sum / filter_len) - accel_bias;
                filtered_accel = (filtered_accel < 0) ? -filtered_accel : filtered_accel;

                // // Publish to Queue
                xBMI160Message.ulfilteredAccel = filtered_accel;
                xQueueSend(xBMI160Queue, &xBMI160Message, 0 );

                // DEBUG
                UARTprintf(">Accel:%d\n", filtered_accel);
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
        I2CMasterInitExpClk(I2C0_BASE, 120000000, false);

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
        // 
        if ( xSemaphoreTake( xI2C0Mutex, portMAX_DELAY ) == pdTRUE )     // Take Mutex
        {
            // sensorOpt3001Init();
            sensorOpt3001Enable(true);
            xSemaphoreGive(xI2C0Mutex);
        }
        else
        {
            UARTprintf("Error: Light Sensor not enabled\n");
        }
}

static void prvConfigureHWTimer6A( void )
{
    /* The Timer 6 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER6);

    /* Configure Timer 6 in full-width periodic mode. */
    TimerConfigure(TIMER6_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 6A load value to run at 100 Hz. */
    TimerLoadSet(TIMER6_BASE, TIMER_A, configCPU_CLOCK_HZ / 70);

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

    // // Check which sensor is using the I2C bus
    if (g_I2C_flag == OPT3001)
    {
        // Give the semaphore to unblock the OPT3001 I2C read
        BaseType_t xI2C0TaskWoken = pdFALSE;
        xI2C0TaskWoken = pdFALSE;
        xSemaphoreGiveFromISR( xI2C0OPTSemaphore, &xI2C0TaskWoken );
        portYIELD_FROM_ISR( xI2C0TaskWoken );
    }
    else if (g_I2C_flag == BMI160)
    {
        // Give the semaphore to unblock
        BaseType_t xI2C0TaskWoken = pdFALSE;
        xI2C0TaskWoken = pdFALSE;
        xSemaphoreGiveFromISR( xI2C0BMISemaphore, &xI2C0TaskWoken );
        portYIELD_FROM_ISR( xI2C0TaskWoken );
    }
}

void xTimer6AHandler( void )
{
    /* Clear the hardware interrupt flag for Timer 7B. */
    TimerIntClear(TIMER6_BASE, TIMER_TIMA_TIMEOUT);

    // Notify the task to read the IMU sensor
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xBMI160ReadSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void xTimer7AHandler( void )
{
    /* Clear the hardware interrupt flag for Timer 7A. */
    TimerIntClear(TIMER7_BASE, TIMER_TIMA_TIMEOUT);

    // Notify the task to read the OPT3001 sensor
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xOPT3001ReadSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Leave this empty
void vApplicationTickHook( void )
{
}