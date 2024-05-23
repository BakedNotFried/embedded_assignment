/* Standard includes. */
#include "driverlib/pin_map.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//required for motor
#include "utils/ustdlib.h"
#include <stdlib.h>  

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "inc/hw_sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"
#include "drivers/opt3001.h"
#include "driverlib/fpu.h"

// Motor includes
#include "motorlib.h"
#include "motor_config.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"

// BMI include
#include "drivers/i2cBMIDriver.h"
#include "drivers/bmi160.h"
#include "drivers/bmi160_defs.h"

/*-----------------------------------------------------------*/
// Tasks
static void prvMotorTask( void *pvParameters );
static void vCurrentRead( void *pvParameters );
static void vRPMRead( void *pvParameters );
static void vQueueReadTest( void *pvParameters );

// Tasks Creator
void vCreateMotorTask( void );

/*
 * Hardware interrupt handlers
 */
void HallSensorHandler(void);
void ADC1_Sequence1_Handler(void);
// Configuration for the HW Timer 3A
static void prvConfigureHWTimer3A( void );

//helper functions
void ADC1_Read(uint32_t *current_ch0, uint32_t *current_ch4);
void MotorRPMTimerStart(void);
void MotorRPMTimerStop(void);  
void initMotorState(MotorState *motor_state);
void setMotorRPM(uint16_t rpm);
void setMotorEstop(void);

// Function Handle for current read task
TaskHandle_t vCurrentReadHandle;
TaskHandle_t vRPMReadHandle;

//external structs
extern SemaphoreHandle_t xADC1_Semaphore;
extern MotorState motor_state;

// Queues for RPM, Current and Power
extern QueueHandle_t xRPMQueueExternal;
extern QueueHandle_t xRPMQueueInternal;
extern QueueHandle_t xCurrentQueueExternal;
extern QueueHandle_t xPowerQueueExternal;

// Structs for RPM, Current and Power
extern RPMQueueData xRPMvalue;
extern CurrentQueueData xCurrentvalue;
extern PowerQueueData xPowervalue;

// Sensor Stuff
// Queues for Sensor Data Publishing
extern QueueHandle_t xOPT3001Queue;
extern QueueHandle_t xBMI160Queue;
// Structs for Sensor Data Publishing
extern OPT3001Message xOPT3001Message;
extern BMI160Message xBMI160Message;
// Binary Semaphore for I2C0 non-blocking read/write
extern SemaphoreHandle_t xI2C0OPTSemaphore;
extern SemaphoreHandle_t xI2C0BMISemaphore;
// Mutex for I2C0
extern SemaphoreHandle_t xI2C0Mutex;
extern SemaphoreHandle_t xI2C2BusMutex;
// enum for checking which sensor is using the I2C bus
enum sensorType {NONE, OPT3001, BMI160};
volatile enum sensorType g_I2C_flag = NONE;
// Function handle for task notification from Timer interrupt
TaskHandle_t xOpt3001ReadHandle;
TaskHandle_t xBMI160ReadHandle;
TaskHandle_t xTaskToNotify = NULL;

// Sensor Functions
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

//globals
volatile uint32_t pulseCount = 0;
volatile bool timerStarted = false;
volatile uint32_t startTime = 0;
volatile uint32_t endTime = 0;
volatile float integral_error = 0;

/*----MOTOR API FUNCTIONS--------------------------------------------------*/
void motorStart(uint16_t rpm){
    motor_state.Estop_condition =  Disabled;
    //Enable the Hall Interrupts
    GPIOIntEnable(HALL_A_PORT, HALL_A_PIN);
    GPIOIntEnable(HALL_B_PORT, HALL_B_PIN);
    GPIOIntEnable(HALL_C_PORT, HALL_C_PIN);
    
    uint16_t duty_value = 10;
    uint16_t period_value = MOTOR_MAX_DUTY;
    int stateA, stateB, stateC;

    //reset the PID error
    motor_state.controller_ouput = 0;

    setMotorRPM(rpm);
    // stopMotor(1);

    enableMotor();
    initMotorLib(period_value);
    /* Set at >10% to get it to start */
    setDuty(duty_value);

    /* Kick start the motor */
    // Do an initial read of the hall effect sensor GPIO lines
    stateA = GPIOPinRead(HALL_A_PORT, HALL_A_PIN) ? 1 : 0;
    stateB = GPIOPinRead(HALL_B_PORT, HALL_B_PIN) ? 1 : 0;
    stateC = GPIOPinRead(HALL_C_PORT, HALL_C_PIN) ? 1 : 0;
    
    updateMotor(stateA, stateB, stateC);
}

void motorStop(bool brake){
    setMotorRPM(0);
    GPIOIntDisable(HALL_A_PORT, HALL_A_PIN);
    GPIOIntDisable(HALL_B_PORT, HALL_B_PIN);
    GPIOIntDisable(HALL_C_PORT, HALL_C_PIN);
    stopMotor(brake);
}


/*-----------------------------------------------------------*/

void vCreateMotorTask( void )
{
    xRPMQueueInternal = xQueueCreate(1, sizeof( xRPMvalue ) );
    xRPMQueueExternal = xQueueCreate(1, sizeof( xRPMvalue ) );
    xCurrentQueueExternal = xQueueCreate(1, sizeof( xCurrentvalue ) );
    xPowerQueueExternal = xQueueCreate(1, sizeof( xPowervalue ) );
    if( ( xRPMQueueExternal == NULL ) || ( xRPMQueueInternal == NULL ) || ( xCurrentQueueExternal == NULL ) || ( xPowerQueueExternal == NULL ) )
    {
        UARTprintf("RPM Queue or Current Queue or Power Queue creation failed\n");
    }
    // Create queues for sensor data pub
    xOPT3001Queue = xQueueCreate(1, sizeof(xOPT3001Message));
    xBMI160Queue = xQueueCreate(1, sizeof(xBMI160Message));

    if (xOPT3001Queue == NULL || xBMI160Queue == NULL)
    {
        UARTprintf("Error creating Sensor Queues\n");
    }

    xTaskCreate( prvMotorTask,
                 "Motor",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );
    
    xTaskCreate( vCurrentRead,
                 "Current/Power Read",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 &vCurrentReadHandle );

    xTaskCreate( vRPMRead,
                 "RPM Read",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 &vRPMReadHandle );

    // Sensor Tasks
    // Create the task to read the optical sensor
    xTaskCreate( vOPT3001Read,
                 "Opt 3001 Read/Pub Task",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 &xOpt3001ReadHandle);

    // // Create the task to read the IMU sensor
    // xTaskCreate( xBMI160Read,
    //              "IMU Read/Pub Task",
    //              configMINIMAL_STACK_SIZE + 200,
    //              NULL,
    //              tskIDLE_PRIORITY + 1,
    //              &xBMI160ReadHandle);
    
    // This is just for testing reads on the queues
    xTaskCreate( vQueueReadTest,
                 "Queue Read Test",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );

    // Enable Timers
    TimerEnable(TIMER5_BASE, TIMER_A); //RPM Timer
    //TimerEnable(TIMER4_BASE, TIMER_A); //Control Timer
}
/*-----------------------------------------------------------*/
// Motor State Machine
static void prvMotorTask( void *pvParameters )
{
    // was 10. 618 - 1290 - 2080 - 3000 - 4000 - 4615 - 5050 - 5600 - 6000
    motorStart(500);

    vTaskDelay(pdMS_TO_TICKS( 10000 ));
    UARTprintf("Stop");
    motorStop(false);

    vTaskDelay(pdMS_TO_TICKS( 10000 ));
    motorStart(3500);

    vTaskDelay(pdMS_TO_TICKS( 10000 ));
    motor_state.Estop_condition = Enabled;


    for (;;)
    {
        UARTprintf("STATE: %d \n", motor_state.current_state);
        UARTprintf("Current RPM: %d \n", motor_state.current_rpm);
        UARTprintf("set RPM: %u \n", motor_state.set_rpm);
        UARTprintf("Duty Value: %d \n", (uint16_t)motor_state.duty_value);
        UARTprintf("Estop condition: %d \n", motor_state.Estop_condition);
        UARTprintf("Controller Error: %d \n", (int32_t)motor_state.controller_error);
        UARTprintf("Controller Output: %d \n", (int32_t)motor_state.controller_ouput);
        vTaskDelay(pdMS_TO_TICKS(3000));
    if (0){
        //wake on state change (Notify/ Semaphore)
        switch(motor_state.current_state){
            case Idle:
                if ( motor_state.set_rpm > 0){

                    if (motor_state.current_rpm == 0){
                        //motorStart(motor_state.set_rpm);
                    }

                    //motor_state.duty_value = 0.5;
                    //optionally start the timer here so we know we start at timer = 0;
                    motor_state.current_state = Starting;
                }
                break;

            case Starting:
                if (motor_state.Estop_condition == Enabled){
                    //motor_state.duty_value = 1;
                    motor_state.set_rpm = 0;
                    motor_state.current_state = EStop;
                }
                if (motor_state.set_rpm == motor_state.current_rpm){
                    motor_state.current_state = Running;
                }
                break;

            case Running:
                if (motor_state.Estop_condition == Enabled){
                    //motor_state.duty_value = 1;
                    motor_state.set_rpm = 0;
                    motor_state.current_state = EStop;
                }
                if (motor_state.current_rpm == 0){
                    motor_state.current_state = Idle;
                }
                break;
            case EStop:
                if (motor_state.set_rpm == motor_state.current_rpm){
                    motor_state.current_state = Idle;
                }     
                break;       
        }
        }
        /*
        if(duty_value>=period_value){
            stopMotor(1);
            UARTprintf("Motor Stop!\n");
        }else{
            setDuty(duty_value);
            vTaskDelay(pdMS_TO_TICKS( 1000 ));
            duty_value++;
        }
        */
        // RPMQueueData xRecievedRPM;
        // if (xQueueReceive(xRPMQueue, &xRecievedRPM, 100) == pdPASS) {
        //     //motor_state.current_rpm = xRecievedRPM.value; UPDATE IN ISR so that it can react quickly
        //     UARTprintf("RPM: %u\n", xRecievedRPM.value);
        //     // UARTprintf("Ticks: %u\n", xRecievedRPM.ticks);
        // }else{
        //     //did not receive from Queue in 100ms
        //     UARTprintf("RPM: 0\n");
        // }

        // UARTprintf("STATE: %d \n", motor_state.current_state);
        // UARTprintf("Current RPM: %d \n", motor_state.current_rpm);
        // UARTprintf("set RPM: %u \n", motor_state.set_rpm);
        // UARTprintf("Duty Value: %d \n", (uint16_t)motor_state.duty_value);
        // UARTprintf("Estop condition: %d \n", motor_state.Estop_condition);
        // UARTprintf("Controller Error: %d \n", (int32_t)motor_state.controller_error);
        // UARTprintf("Controller Output: %d \n", (int32_t)motor_state.controller_ouput);

        //12 bits ADC
        // UARTprintf("Before Read\n");
        // ADC1_Read(&current_c1);
        // UARTprintf("Current draw: %d \n", 2048 - current_c1);
        // ADC1_Read(current_sensor);
        // char bufferC[100];
        // uint32_t average_0 = (current_sensor[0]+ current_sensor[1]+ current_sensor[2]+ current_sensor[3])/4;
        // usprintf(bufferC, "Channel 0 Value: %u, value: %u, value %u, value: %u average :%u \n", current_sensor[0], current_sensor[1], current_sensor[2], current_sensor[3], average_0);
        // UARTprintf(bufferC);

        // char bufferB[100];
        // uint32_t average_4 = (current_sensor[4]+ current_sensor[5]+ current_sensor[6]+ current_sensor[7])/4;
        // usprintf(bufferB, "Channel 4 Value: %u, value: %u, value %u, value: %u  average: %u \n", current_sensor[4], current_sensor[5], current_sensor[6], current_sensor[7], average_4);
        // UARTprintf(bufferB);

        // int power = (int)calculatePower(current_sensor);
        // UARTprintf("Power draw: %d milliWatts\n", power);

        // vTaskDelay(pdMS_TO_TICKS( 100 ));

        //uint32_t ADC_ref = ADCReferenceGet(ADC1_BASE);
        //UARTprintf("ADC ref: %u \n", ADC_ref);

    }
    // give the read hall effect sensor lines to updateMotor() to move the motor
    // one single phase
    // Recommendation is to use an interrupt on the hall effect sensors GPIO lines 
    // So that the motor continues to be updated every time the GPIO lines change from high to low
    // or low to high
    // Include the updateMotor function call in the ISR to achieve this behaviour.

    /* Motor test - ramp up the duty cycle from 10% to 100%, than stop the motor */
}
/*-----------------------------------------------------------*/
// Current Read Task
static void vCurrentRead( void *pvParameters )
{
    // Configure the HW Timer 3A
    prvConfigureHWTimer3A();

    // Structures to send the current and power values
    CurrentQueueData xCurrentvalueSend;
    PowerQueueData xPowervalueSend;

    // Setup
    uint32_t biased_current_C = 0;
    uint32_t biased_current_B = 0;
    int32_t bias = 2048;
    int arr_len = 25;
    int32_t current_C_array[25] = {0};
    int32_t current_B_array[25] = {0};
    int32_t current_C = 0;
    int32_t current_B = 0;
    int32_t current_A = 0;
    int32_t current = 0;
    uint32_t current_C_filtered = 0;
    uint32_t current_B_filtered = 0;
    uint32_t power = 0;
    int index_C = 0;
    int index_B = 0;
    int32_t sum_C = 0;
    int32_t sum_B = 0;

    int print_idx = 0;

    TimerEnable(TIMER3_BASE, TIMER_A); //Current Timer
    // Loop indefinitely
    for( ;; )
    {
        // Wait for the notification from the HW Timer 3A interrupt
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

        // Read the current sensor
        ADC1_Read(&biased_current_C, &biased_current_B);

        // Unbias and abs
        current_C = bias - biased_current_C;
        current_B = bias - biased_current_B;
        current_C = abs(current_C);
        current_B = abs(current_B);

        // // Filter values
        current_C_array[index_C] = current_C;
        current_B_array[index_B] = current_B;
        index_C = (index_C + 1) % arr_len;
        index_B = (index_B + 1) % arr_len;
        sum_C = 0;
        sum_B = 0;
        for (int i = 0; i < arr_len; i++) {
            sum_C += current_C_array[i];
            sum_B += current_B_array[i];
        }
        current_C_filtered = sum_C / arr_len;
        current_B_filtered = sum_B / arr_len;

        // Estimate the current on the A coil
        current_A = (current_C_filtered + current_B_filtered) / 2;

        // Total current
        current = current_A + current_C_filtered + current_B_filtered;

        // Scale Current to mA
        current = current / 0.07;
        current *= 1000;
        current /= 4096;

        // Calculate power -> P = V * I
        power = current * (24 - 3.3);

        // Send the current and power value to the queue
        xCurrentvalueSend.value = current;
        xPowervalueSend.value = power;
        xQueueSend(xCurrentQueueExternal, &xCurrentvalueSend, 0);
        xQueueSend(xPowerQueueExternal, &xPowervalueSend, 0);
    }
}

/*-----------------------------------------------------------*/
// RPM Read Task
void vRPMRead( void *pvParameters )
{
    // RPM Setup
    uint32_t rpm_array [10] = {0};
    int index = 0;
    uint32_t sum = 0;
    uint32_t rpm_filtered = 0;

    // Structure to recv the RPM value
    RPMQueueData xRPMvalueRecv;
    // Structure to send the RPM value
    RPMQueueData xRPMvalueSend;
    for( ;; )
    {
        // Read the RPM value from the queue
        if (xQueueReceive(xRPMQueueInternal, &xRPMvalueRecv, 0) == pdPASS) {
            // Store the RPM value in the array
            rpm_array[index] = xRPMvalueRecv.value;
            index = (index + 1) % 10;
            sum = 0;
            for (int i = 0; i < 10; i++) {
                sum += rpm_array[i];
            }
            rpm_filtered = sum / 10;
        }
        // Send the filtered RPM value to the queue
        xRPMvalueSend.value = rpm_filtered;
        xRPMvalueSend.ticks = 0;
        xQueueSend(xRPMQueueExternal, &xRPMvalueSend, 0);

        // Delay for 100Hz
        vTaskDelay(pdMS_TO_TICKS( 10 ));
    }
}

void vQueueReadTest( void *pvParameters )
{
    // Read from all sensor queues and print the values
    RPMQueueData xRPMvalueRecv;
    CurrentQueueData xCurrentvalueRecv;
    PowerQueueData xPowervalueRecv;
    OPT3001Message xOPT3001MessageRecv;
    BMI160Message xBMI160MessageRecv;
    int print_idx = 0;
    for( ;; )
    {
        // Test one by one
        // Read the RPM value from the queue
        if (xQueueReceive(xRPMQueueExternal, &xRPMvalueRecv, 0) == pdPASS) 
        {
            // Print the values
            UARTprintf("RPM: %u\n", xRPMvalueRecv.value);
        }

        // // Read the Current value from the queue
        // if (xQueueReceive(xCurrentQueueExternal, &xCurrentvalueRecv, 0) == pdPASS) {
        //     // Print the values
        //     UARTprintf("Current: %d\n", xCurrentvalueRecv.value);
        // }

        // // // Read the Power value from the queue
        if (xQueueReceive(xPowerQueueExternal, &xPowervalueRecv, 0) == pdPASS) {
            // Print the values
            UARTprintf("Power: %u\n", xPowervalueRecv.value);
        }

        // // Read the OPT3001 value from the queue
        // if (xQueueReceive(xOPT3001Queue, &xOPT3001MessageRecv, 0) == pdPASS) {
        //     // Print the values
        //     UARTprintf("Lux: %u\n", xOPT3001MessageRecv.ulfilteredLux);
        // }

        // // Read the BMI160 value from the queue
        // if (xQueueReceive(xBMI160Queue, &xBMI160MessageRecv, 0) == pdPASS) 
        // {
        //     // Print the values
        //     UARTprintf("Accel: %d\n", xBMI160MessageRecv.ulfilteredAccel);
        // }
        vTaskDelay(pdMS_TO_TICKS( 3000 ));
        
    }
}

/*-----------------------------------------------------------*/
/*                  Aux Sensor Functions                     */

// Periodic IMU Read Task
static void xBMI160Read( void *pvParameters )
{
    // debug
    int print_idx = 0;

    // // Configure I2C0
    // prvConfigureI2C0();

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

    // // BMI160 Config
    uint8_t normal_accl_mode = 0x11;                                // Normal Mode, Accel Only
    if (xSemaphoreTake(xI2C0Mutex, 0) == pdTRUE) 
    {
        writeI2CBMI(0x69, BMI160_COMMAND_REG_ADDR, &normal_accl_mode);  // Write to Command Register
        xSemaphoreGive(xI2C0Mutex);
    }
    vTaskDelay(200);                                                // Delay for 200ms to allow for sensor to start

    // Timer Enable
    TimerEnable(TIMER6_BASE, TIMER_A);
    for( ;; )
    {
        // Wait for notification from Timer7B interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        {
            // Read and convert BMI160 values
            if (xSemaphoreTake(xI2C0Mutex, 0) == pdTRUE) 
            {
                readI2CBMI(0x69, 0x12, &x_lsb);
                readI2CBMI(0x69, 0x13, &x_msb);
                int_x = (int16_t)((x_msb << 8) | x_lsb);

                readI2CBMI(0x69, 0x14, &y_lsb);
                readI2CBMI(0x69, 0x15, &y_msb);
                int_y = (int16_t)((y_msb << 8) | y_lsb);

                readI2CBMI(0x69, 0x16, &z_lsb);
                readI2CBMI(0x69, 0x17, &z_msb);
                int_z = (int16_t)((z_msb << 8) | z_lsb);
                xSemaphoreGive(xI2C0Mutex);
            }
            else
            {
                UARTprintf("I2C0 Mutex Timeout\n");
            }
            // take abs value
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
            // int32_t filtered_accel = 100;
            xBMI160Message.ulfilteredAccel = filtered_accel;
            xQueueSend(xBMI160Queue, &xBMI160Message, 0);

            // DEBUG
            // if ((print_idx % 50) == 0) {
            //     UARTprintf("Jerk: %d\n", filtered_accel);
            // }
            // print_idx += 1;
        }
    }
}

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
    for( ;; )
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        {
            //Read and convert OPT values
            if (xSemaphoreTake(xI2C0Mutex, portMAX_DELAY) == pdTRUE)
            {
                success = sensorOpt3001Read(&rawData);
                xSemaphoreGive(xI2C0Mutex);
            }
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
                // uint32_t filtered_lux = 100;
                xOPT3001Message.ulfilteredLux = filtered_lux;
                xQueueSend(xOPT3001Queue, &xOPT3001Message, 0);
            }
        }
    }
}

/* Interrupt handlers */

void Timer4IntHandler(void) {
    // Clear the timer interrupt
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

    if (motor_state.Estop_condition == Disabled) {
        // Calculate error
        int error = motor_state.set_rpm - motor_state.current_rpm;

        // Update integral error
        integral_error += error * TIMER_INTERVAL; // Multiply by dt if available

        // Calculate PID output (scaled appropriately)
        float output = Kp * error + Ki * integral_error;

        // Scale PID output to match the duty cycle range
        // Assuming the maximum motor RPM is 4500 and maximum duty is 45, the scaling factor is 0.01
        float pid_duty = output * 0.01;

        if (pid_duty > MAX_DELTA_DUTY_NORMAL) {
            pid_duty = MAX_DELTA_DUTY_NORMAL;
        }

        if (pid_duty > 0.005){
            pid_duty = 0.005;
        }
        else if (pid_duty < -0.005){
            pid_duty = -0.005;
        }

        motor_state.controller_ouput += pid_duty; 

        // Ensure controller output is within valid range
        if (motor_state.controller_ouput < 1) {
            motor_state.controller_ouput = 1;
        } else if (motor_state.controller_ouput > 45) {
            motor_state.controller_ouput = 45;
        }

        // Apply the output as a new duty cycle
        motor_state.duty_value = (uint16_t)motor_state.controller_ouput;

        setDuty(motor_state.duty_value);
    }


    // Handle Estop condition
    if (motor_state.Estop_condition == Enabled) {

        if (motor_state.duty_value > 1){
            motor_state.duty_value -= 0.1;
            setDuty(motor_state.duty_value);
        }
        else{
            motorStop(true);
        }
        // TODO constant deceleration
        // Reset to a safe duty cycle if needed in an emergency
         // Verify if this should instead be 0 or some other safe value
    }

    // Optionally log error and duty value for debugging
    //UARTprintf("Error: %d, Duty: %d\n", error, motor_state.duty_value);
}

void HallSensorHandler(void)
{
    RPMQueueData xRPMvalue;
    uint32_t HALL_A_Status, HALL_B_Status, HALL_C_Status;
    int stateA, stateB, stateC;
    int num_pole_pairs = 4;
    //BaseType_t xMotorSenseTaskWoken = pdFALSE;

    // Read the interrupt status to find the cause of the interrupt.
    HALL_A_Status = GPIOIntStatus(HALL_A_PORT, true);
    HALL_B_Status = GPIOIntStatus(HALL_B_PORT, true);
    HALL_C_Status = GPIOIntStatus(HALL_C_PORT, true);
    //UARTprintf("Hall Interrupt Read 1: %u, 2: %lu, 3: %lu,\n", HALL_A_Status, HALL_B_Status, HALL_C_Status);

    // Clear interrupt
    GPIOIntClear(HALL_A_PORT, HALL_A_Status);
    GPIOIntClear(HALL_B_PORT, HALL_B_Status);
    GPIOIntClear(HALL_C_PORT, HALL_C_Status);

    //TIMER CALC RPM
    pulseCount++;

    if (pulseCount == 1) {
        // Start the timer on the first pulse
        //MotorRPMTimerStart();
        startTime = xTaskGetTickCount();
    } else if (pulseCount >= 6*num_pole_pairs+1) {
        // Stop the timer on the seventh pulse
        //MotorRPMTimerStop();
        endTime = xTaskGetTickCount();
        pulseCount = 0; // Reset pulse count for the next rotation

        // Calculate the elapsed time and RPM


        //uint32_t elapsedTime = startTime - endTime; // Assuming the timer counts down
        TickType_t elapsedTime = endTime - startTime;
        uint32_t elapsedTimeInMilliseconds = elapsedTime * portTICK_PERIOD_MS;
        //UARTprintf("Elapsed Ticks: %u\n", elapsedTime);
        //float factor = 553846154.0; // should be 720000000, but it was out by a factor of 1.3 ?????
        //calc RPM (60*10^9) nS per minute. elapsedTime in systemticks. each system tick 8.33nS. 60*10^9/8.33333 = 7200000000 (avoid float) 
        //uint32_t rpm = (uint32_t)((factor / ((float)elapsedTime)) * 10.0); //add digits back in 

        // 0.7472 -> motor rpm correction factor
        uint32_t rpm = 0.7472 * (60000 / (elapsedTimeInMilliseconds)); 

        motor_state.current_rpm = rpm; // This updates our global value.

        xRPMvalue.value = rpm;
        xRPMvalue.ticks = elapsedTime;
        if (xQueueSend(xRPMQueueInternal, &xRPMvalue, 0) != pdPASS) {
            // Handle error: Queue is full
        }
        
        // TEST HERE Optionally print or return the RPM
        //UARTprintf("RPM: %u\n", rpm);
        // THEN OUPUT TO QUEUE
    }


    // Read hall effect sensors
    stateA = GPIOPinRead(HALL_A_PORT, HALL_A_PIN) ? 1 : 0;
    stateB = GPIOPinRead(HALL_B_PORT, HALL_B_PIN) ? 1 : 0;
    stateC = GPIOPinRead(HALL_C_PORT, HALL_C_PIN) ? 1 : 0;

    // // Checking current on C coil
    // if (stateC == 1){
    //     ADCProcessorTrigger(ADC1_BASE, ADC_SEQ_1);
    //     // 1 ms delay
    //     // vTaskDelay(pdMS_TO_TICKS( 1 ));
    // }

   // UARTprintf("Hall Read 1: %d, 2: %d, 3: %d,\n", stateA, stateB, stateC);
    // Call updateMotor to change to the next phase
    // try debounce the noise

    // ADCProcessorTrigger(ADC1_BASE, ADC_SEQ_1);
    updateMotor(stateA, stateB, stateC);
    // ADCProcessorTrigger(ADC1_BASE, ADC_SEQ_1);
    //Optionally add code for motor speed sensing or additional control feedback.
    
}

void initMotorState(MotorState *motor_state){
    motor_state->current_state = Idle;
    motor_state->Estop_condition = Disabled;
    motor_state->set_rpm = 0;
    motor_state->current_rpm = 0;
}

void MotorRPMTimerStart() {
    // Configure and start timer
    
    TimerLoadSet(TIMER5_BASE, TIMER_A, (SysCtlClockGet()*2) - 1); // 2-second interval
    TimerEnable(TIMER5_BASE, TIMER_A);
    startTime = TimerValueGet(TIMER5_BASE, TIMER_A); // Capture start time
    timerStarted = true;
}

void MotorRPMTimerStop() {
    endTime = TimerValueGet(TIMER5_BASE, TIMER_A); // Capture end time
    TimerDisable(TIMER5_BASE, TIMER_A);
    timerStarted = false;
}

void setMotorRPM(uint16_t rpm){
    // UARTprintf("RPM set to: %d \n", rpm);
    motor_state.set_rpm = rpm;
    //post sem
}

void setMotorEstop(){
    motor_state.Estop_condition = Enabled;
    motor_state.set_rpm = 0;
    //post Sem
}
/*#################################################################################################################*/

/*-----------------------------------------------------------*/
// Current Sensing Functions and Interrupts

void ADC1_Sequence1_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Clear the ADC sequence interrupt status
    ADCIntClear(ADC1_BASE, ADC_SEQ_1);

    // Give the semaphore from ISR
    xSemaphoreGiveFromISR(xADC1_Semaphore, &xHigherPriorityTaskWoken);

    // Yield if a higher priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ADC1_Read(uint32_t *current_ch0, uint32_t *current_ch4)
{
    ADCProcessorTrigger(ADC1_BASE, ADC_SEQ_1);
    if (xSemaphoreTake(xADC1_Semaphore, portMAX_DELAY) == pdTRUE) {
        uint32_t adc_values[2];
        ADCSequenceDataGet(ADC1_BASE, ADC_SEQ_1, adc_values);
        *current_ch0 = adc_values[0];
        *current_ch4 = adc_values[1];
    }
    ADCIntClear(ADC1_BASE, ADC_SEQ_1);
}

// Timer 3A for triggering current reads
static void prvConfigureHWTimer3A( void )
{
    /* The Timer 3 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    /* Configure Timer 3 in full-width periodic mode. */
    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 3A load value to run at 200 Hz. */
    TimerLoadSet(TIMER3_BASE, TIMER_A, configCPU_CLOCK_HZ / 200);

    /* Configure the Timer 3A interrupt for timeout. */
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    /* Enable the Timer 3A interrupt in the NVIC. */
    IntEnable(INT_TIMER3A);

    // /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}

// Timer 3A interrupt handler
void xTimer3AHandler( void )
{
    /* Clear the hardware interrupt flag for Timer 3A. */
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    // Notify the task to read the Current sensor
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(vCurrentReadHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // // debug print
    // UARTprintf("Timer 3A Interrupt\n");
}

/*-----------------------------------------------------------*/
// BMI160 Sensor Functions and Interrupts
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

void xTimer6AHandler( void )
{
    /* Clear the hardware interrupt flag for Timer 6A. */
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

void xI2C2Handler( void )
{
    I2CMasterIntClear(I2C2_BASE);

    // if (g_I2C_flag == OPT3001)
    // {
    //     // Notify the task that the I2C transaction is complete
    //     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //     vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
    //     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    // }
    // else if (g_I2C_flag == BMI160)
    // {
    //     // Notify the task that the I2C transaction is complete
    //     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //     vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
    //     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    // }
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

// Configuration functions
static void prvConfigureOPT3001Sensor( void )
{
        // The I2C0 peripheral must be enabled before use.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

        // Configure the pin muxing for I2C0 functions on port B2 and B3.
        // This step is not necessary if your part does not support pin muxing.
        GPIOPinConfigure(GPIO_PN5_I2C2SCL);
        GPIOPinConfigure(GPIO_PN4_I2C2SDA);

        // Select the I2C function for these pins.  This function will also
        // configure the GPIO pins pins for I2C operation, setting them to
        // open-drain operation with weak pull-ups.  Consult the data sheet
        // to see which functions are allocated per pin.
        GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
        GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
        I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

        IntEnable(INT_I2C2);
        I2CMasterIntEnable(I2C2_BASE);

        // // Enable Master interrupt
        IntMasterEnable();

        // Initialize opt3001 sensor
        if (xSemaphoreTake(xI2C0Mutex, portMAX_DELAY) == pdTRUE)
        {
            sensorOpt3001Init();
            xSemaphoreGive(xI2C0Mutex);
        }
        
        if (xSemaphoreTake(xI2C0Mutex, portMAX_DELAY) == pdTRUE)
        {
            sensorOpt3001Enable(true);
            xSemaphoreGive(xI2C0Mutex);
        }

}

static void prvConfigureI2C0( void )
{
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

        GPIOPinConfigure(GPIO_PN5_I2C2SCL);
        GPIOPinConfigure(GPIO_PN4_I2C2SDA);

        GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
        GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
        I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

        IntEnable(INT_I2C2);
        I2CMasterIntEnable(I2C2_BASE);

        IntMasterEnable();
}