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
extern SemaphoreHandle_t xI2C0Semaphore;
extern SemaphoreHandle_t xI2C0BMISemaphore;

// enum for checking which sensor is using the I2C bus
enum sensorType {NONE, OPT3001, BMI160};
volatile enum sensorType g_I2C_flag = NONE;

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

// BMI160 Setup
struct bmi160_dev bmi160_handle;
struct bmi160_sensor_data accel;

// Configuration for the OPT3001 sensor
static void prvConfigureOPT3001Sensor( void );

// Configuration for the I2C0
static void prvConfigureI2C0( void );

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
                 tskIDLE_PRIORITY + 2,
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

                // Get the sensor configuration
                uint16_t config;
                readI2C(0x47, 0x7F, (uint8_t *)&config);
                config = (config << 8) | (config>>8 &0xFF);
                // UARTprintf("OPT3001 Config: 0x%04x\n", config);
            }
        }
    }
}

// Periodic IMU Read Task
static void vIMURead( void *pvParameters )
{
    // // Configure I2C0
    prvConfigureI2C0();

    // Setup BMI
    bmi160_handle.id = 0x69;
    bmi160_handle.intf = BMI160_I2C_INTF;
    bmi160_handle.read = readI2CBMI;
    // Vars for accel x,y,z
    uint8_t x_lsb = 0x00;
    uint8_t x_msb = 0x00;
    uint8_t y_lsb = 0x00;
    uint8_t y_msb = 0x00;
    uint8_t z_lsb = 0x00;
    uint8_t z_msb = 0x00;
    int16_t int_x = 0x0000;
    float x = 0;
    int16_t int_y = 0x0000;
    float y = 0;
    int16_t int_z = 0x0000;
    float z = 0;
    uint8_t normal_accl_mode = 0x11;
    writeI2CBMI(0x69, BMI160_COMMAND_REG_ADDR, &normal_accl_mode);
    vTaskDelay(200);

    // soft reset
    // uint8_t reset = BMI160_SOFT_RESET_CMD;
    // writeI2CBMI(0x69, BMI160_COMMAND_REG_ADDR, &reset);
    // vTaskDelay(200);

    // bmi160_get_regs(BMI160_CHIP_ID_ADDR, &bmi1

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
            // UARTprintf("IMU Read Task\n");
            // readI2CBMI(0x69, BMI160_CHIP_ID_ADDR, &bmi160_handle.chip_id);
            // UARTprintf("BMI160 Config 2: 0x%02x\n", bmi160_handle.chip_id);

            // readI2CBMI(0x69, 0x03, &bmi160_handle.chip_id);
            // UARTprintf("BMI160 Config 2: 0x%02x\n", bmi160_handle.chip_id);

            // readI2CBMI(0x69, 0x1B, &bmi160_handle.chip_id);
            // UARTprintf("BMI160 Config 2: 0x%02x\n", bmi160_handle.chip_id);

            // readI2CBMI(0x69, 0x41, &bmi160_handle.chip_id);
            // UARTprintf("BMI160 Config 2: 0x%02x\n", bmi160_handle.chip_id);

            // Get Sensor Data
            readI2CBMI(0x69, BMI160_ACCEL_DATA_ADDR, &x_lsb);
            readI2CBMI(0x69, 0x13, &x_msb);
            int_x = (int16_t)((x_msb << 8) | x_lsb);
            float x = ((float)int_x) / 16384.0;
            UARTprintf("BMI160 Accel X: %d\n", (int32_t)(x*10000));

            readI2CBMI(0x69, BMI160_ACCEL_DATA_ADDR + 2, &y_lsb);
            readI2CBMI(0x69, BMI160_ACCEL_DATA_ADDR + 3, &y_msb);
            int_y = (int16_t)((y_msb << 8) | y_lsb);
            float y = ((float)int_y) / 16384.0;
            UARTprintf("BMI160 Accel Y: %d\n", (int32_t)(y*10000));

            readI2CBMI(0x69, BMI160_ACCEL_DATA_ADDR + 4, &z_lsb);
            readI2CBMI(0x69, BMI160_ACCEL_DATA_ADDR + 5, &z_msb);
            int_z = (int16_t)((z_msb << 8) | z_lsb);
            float z = ((float)int_z) / 16384.0;
            UARTprintf("BMI160 Accel Z: %d\n", int_z);
            // UARTprintf("BMI160 Accel Z: %d\n", (int32_t)(z*10000));
            
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

static void prvConfigureI2C0( void )
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
}

static void prvConfigureHWTimer6A( void )
{
    /* The Timer 6 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER6);

    /* Configure Timer 6 in full-width periodic mode. */
    TimerConfigure(TIMER6_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 6A load value to run at 100 Hz. */
    TimerLoadSet(TIMER6_BASE, TIMER_A, configCPU_CLOCK_HZ / 2);

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
        xSemaphoreGiveFromISR( xI2C0Semaphore, &xI2C0TaskWoken );
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

    // // Get status of the I2C
    // uint32_t ui32Status;
    // ui32Status = I2CMasterIntStatus(I2C0_BASE, true);

    // Give the semaphore to unblock the I2C read
    // BaseType_t xI2C0TaskWoken = pdFALSE;
    // xI2C0TaskWoken = pdFALSE;
    // xSemaphoreGiveFromISR( xI2C0Semaphore, &xI2C0TaskWoken );
    // portYIELD_FROM_ISR( xI2C0TaskWoken );
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