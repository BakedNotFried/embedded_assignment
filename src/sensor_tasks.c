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

// Motor config
#include "motor_config.h"

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
// Mutex for I2C0
extern SemaphoreHandle_t xI2C0Mutex;

// enum for checking which sensor is using the I2C bus
enum sensorType {NONE, OPT3001, BMI160};
volatile enum sensorType g_I2C_flag = NONE;

// Function handle for task notification from Timer interrupt
TaskHandle_t xOpt3001ReadHandle;
TaskHandle_t xBMI160ReadHandle;

// Queues for Sensor Data Publishing
extern QueueHandle_t xOPT3001Queue;
extern QueueHandle_t xBMI160Queue;

// Structs for Sensor Data Publishing
extern OPT3001Message xOPT3001Message;
extern BMI160Message xBMI160Message;

// Configuration for the OPT3001 sensor
static void prvConfigureOPT3001Sensor( void );

// Configuration for the I2C0
static void prvConfigureI2C0( void );

// Configuration for the HW Timer 6A and 7A
static void prvConfigureHWTimer6A( void );
static void prvConfigureHWTimer7A( void );

// Task for reading sensors
static void vOPT3001Read( void *pvParameters );
static void xBMI160Read( void *pvParameters );
static void vQueueReadTest( void *pvParameters );

// Called by main function to setup Sensor Tasks
void vSensorTaskSetup( void );

/*-----------------------------------------------------------*/

void vSensorTaskSetup( void )
{
    // Create queues for sensor data pub
    xOPT3001Queue = xQueueCreate(1, sizeof(xOPT3001Message));
    xBMI160Queue = xQueueCreate(1, sizeof(xBMI160Message));

    if (xOPT3001Queue == NULL || xBMI160Queue == NULL)
    {
        UARTprintf("Error creating Sensor Queues\n");
    }

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
                 tskIDLE_PRIORITY + 2,
                 &xBMI160ReadHandle);
    
    // // This is just for testing reads on the queues
    // xTaskCreate( vQueueReadTest,
    //             "Queue Read Test",
    //             configMINIMAL_STACK_SIZE,
    //             NULL,
    //             tskIDLE_PRIORITY + 1,
    //             NULL );

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
    OPT3001Message xOPT3001Message;

    // Timer Enable
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
            xSemaphoreTake(xI2C0Mutex, portMAX_DELAY);
            success = sensorOpt3001Read(&rawData);
            xSemaphoreGive(xI2C0Mutex);
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

                // Publish to Queue
                xOPT3001Message.ulfilteredLux = filtered_lux;
                xQueueSend(xOPT3001Queue, &xOPT3001Message, 0);

                // DEBUG
                // UARTprintf("Filtered Lux: %5d\n", filtered_lux);
                // Get the sensor configuration
                // uint16_t config;
                // readI2C(0x47, 0x7F, (uint8_t *)&config);
                // config = (config << 8) | (config>>8 &0xFF);
                // UARTprintf("OPT3001 Config: 0x%04x\n", config);
            }
        }
    }
}

// Periodic IMU Read Task
static void xBMI160Read( void *pvParameters )
{
    // debug
    int print_idx = 0;

    // // Configure I2C0
    prvConfigureI2C0();

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
    int32_t prev_int_accel = 17367;

    uint8_t error_reg = 0x00;

    // Array for filtered values
    int32_t accel_array[5] = {0};
    int index = 0;

    // Queue setup
    BMI160Message xBMI160Message;

    // BMI160 Config
    uint8_t normal_accl_mode = 0x11;                                // Normal Mode, Accel Only
    xSemaphoreTake(xI2C0Mutex, portMAX_DELAY);
    writeI2CBMI(0x69, BMI160_COMMAND_REG_ADDR, &normal_accl_mode);  // Write to Command Register
    xSemaphoreGive(xI2C0Mutex);
    vTaskDelay(200);                                                // Delay for 200ms to allow for sensor to start

    // Timer Enable
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
            // Get Sensor Data for X,Y,Z
            xSemaphoreTake(xI2C0Mutex, portMAX_DELAY);
            readI2CBMI(0x69, 0x12, &x_lsb);
            readI2CBMI(0x69, 0x13, &x_msb);
            xSemaphoreGive(xI2C0Mutex);
            int_x = (int16_t)((x_msb << 8) | x_lsb);
            // UARTprintf("BMI160 Accel X: %d\n", int_x);
            xSemaphoreTake(xI2C0Mutex, portMAX_DELAY);
            readI2CBMI(0x69, 0x14, &y_lsb);
            readI2CBMI(0x69, 0x15, &y_msb);
            xSemaphoreGive(xI2C0Mutex);
            int_y = (int16_t)((y_msb << 8) | y_lsb);
            // UARTprintf("BMI160 Accel Y: %d\n", int_y);
            xSemaphoreTake(xI2C0Mutex, portMAX_DELAY);
            readI2CBMI(0x69, 0x16, &z_lsb);
            readI2CBMI(0x69, 0x17, &z_msb);
            xSemaphoreGive(xI2C0Mutex);
            int_z = (int16_t)((z_msb << 8) | z_lsb);
            // UARTprintf("BMI160 Accel Z: %d\n", int_z);

            // abs value
            if (int_x < 0)
            {
                int_x = -int_x;
            }
            if (int_y < 0)
            {
                int_y = -int_y;
            }
            if (int_z < 0)
            {
                int_z = -int_z;
            }

            // Combine X,Y,Z into one value
            int32_t int_accel = (int32_t)int_x + (int32_t)int_y + (int32_t)int_z;
            int32_t int_jerk = int_accel - prev_int_accel;
            if (int_jerk < 0)
            {
                int_jerk = -int_jerk;
            }
            prev_int_accel = int_accel;
        
            // Filter values
            accel_array[index] = int_jerk;
            index = (index + 1) % 5;
            int32_t sum = 0;
            for (int i = 0; i < 5; i++) {
                sum += accel_array[i];
            }
            int32_t filtered_accel = sum / 5;
            filtered_accel = (filtered_accel * 1000) / 16384;
            // Publish to Queue
            xBMI160Message.ulfilteredAccel = filtered_accel;
            xQueueSend(xBMI160Queue, &xBMI160Message, 0);

            // DEBUG
            // read from error register
            // xSemaphoreTake(xI2C0Mutex, portMAX_DELAY);
            // readI2CBMI(0x69, 0x02, &error_reg);
            // xSemaphoreGive(xI2C0Mutex);
            // UARTprintf("BMI160 Error Reg: 0x%02x\n", error_reg);
            // if ((print_idx % 50) == 0) {
            //     // Print the values
            //     // UARTprintf("BMI160 Jerk: %d\n", filtered_accel);
            //     UARTprintf("BMI160 Accel: %d\n", int_jerk);
            //     // print error reg
            // }
            // print_idx += 1;
        }
    }
}

/*-----------------------------------------------------------*/
// Queue Read Test Task
void vQueueReadTest( void *pvParameters )
{
    OPT3001Message xOPT3001MessageRecv;
    BMI160Message xBMI160MessageRecv;
    int print_idx = 0;
    for( ;; )
    {
        if ( (xQueueReceive(xOPT3001Queue, &xOPT3001MessageRecv, pdMS_TO_TICKS(100)) == pdPASS) )
        {
            xQueueReceive(xBMI160Queue, &xBMI160MessageRecv, 0);
            // Scale the accelerometer value and cast to int
            UARTprintf("\nAux Sensors:");
            UARTprintf("Lux: %u\n", xOPT3001MessageRecv.ulfilteredLux);
            UARTprintf("Accel: %d\n", xBMI160MessageRecv.ulfilteredAccel);
        }
        // if ( (xQueueReceive(xBMI160Queue, &xBMI160MessageRecv, pdMS_TO_TICKS(100)) == pdPASS) )
        // {
        //     // Scale the accelerometer value and cast to int
        //     if ((print_idx % 50) == 0) 
        //     {
        //         UARTprintf("Accel: %d\n", xBMI160MessageRecv.ulfilteredAccel);
        //     }
        //     print_idx += 1;
        // }
    }
}
/*-----------------------------------------------------------*/
// Configuration functions
static void prvConfigureOPT3001Sensor( void )
{
        // The I2C0 peripheral must be enabled before use.
        // SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
        // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

        // Configure the pin muxing for I2C0 functions on port B2 and B3.
        // This step is not necessary if your part does not support pin muxing.
        // GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        // GPIOPinConfigure(GPIO_PB3_I2C0SDA);
        GPIOPinConfigure(GPIO_PN5_I2C2SCL);
        GPIOPinConfigure(GPIO_PN4_I2C2SDA);

        // Select the I2C function for these pins.  This function will also
        // configure the GPIO pins pins for I2C operation, setting them to
        // open-drain operation with weak pull-ups.  Consult the data sheet
        // to see which functions are allocated per pin.
        // GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
        // GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
        GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
        GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
        // I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
        I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

        // Interrupt Configuration for I2C
        // IntEnable(INT_I2C0);
        // I2CMasterIntEnable(I2C0_BASE);
        IntEnable(INT_I2C2);
        I2CMasterIntEnable(I2C2_BASE);

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
        xSemaphoreTake(xI2C0Mutex, portMAX_DELAY);
        sensorOpt3001Enable(true);
        xSemaphoreGive(xI2C0Mutex);
}

static void prvConfigureI2C0( void )
{
        // The I2C0 peripheral must be enabled before use.
        // SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
        // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        // I2C2 and GPION
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

        // Configure the pin muxing for I2C0 functions on port B2 and B3.
        // This step is not necessary if your part does not support pin muxing.
        // GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        // GPIOPinConfigure(GPIO_PB3_I2C0SDA);
        GPIOPinConfigure(GPIO_PN5_I2C2SCL);
        GPIOPinConfigure(GPIO_PN4_I2C2SDA);

        // Select the I2C function for these pins.  This function will also
        // configure the GPIO pins pins for I2C operation, setting them to
        // open-drain operation with weak pull-ups.  Consult the data sheet
        // to see which functions are allocated per pin.
        // GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
        // GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
        // I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
        GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
        GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
        I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);


        // Interrupt Configuration for I2C
        // IntEnable(INT_I2C0);
        // I2CMasterIntEnable(I2C0_BASE);
        IntEnable(INT_I2C2);
        I2CMasterIntEnable(I2C2_BASE);

        // // Enable Master interrupt
        IntMasterEnable();
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
void xI2C2Handler( void )
{
    /* Clear the interrupt. */
    // I2CMasterIntClear(I2C0_BASE);
    I2CMasterIntClear(I2C2_BASE);

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
    vTaskNotifyGiveFromISR(xBMI160ReadHandle, &xHigherPriorityTaskWoken);
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