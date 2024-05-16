/*
 * hello_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/******************************************************************************
 *
 * The Hello task creates a simple task to handle the UART output for the
 * 'Hello World!' message.  A loop is executed five times with a count down
 * before ending with the self-termination of the task that prints the UART
 * message by use of vTaskDelete.  The loop also includes a one second delay
 * that is achieved by using vTaskDelay.
 *
 * This example uses UARTprintf for output of UART messages.  UARTprintf is not
 * a thread-safe API and is only being used for simplicity of the demonstration
 * and in a controlled manner.
 *
 */

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

#include "motorlib.h"
#include "motor_config.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"

/*-----------------------------------------------------------*/
// Tasks
static void prvMotorTask( void *pvParameters );
static void vCurrentRead( void *pvParameters );
static void vRPMRead( void *pvParameters );
static void vRPMReadTest( void *pvParameters );

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

uint32_t calculateCurrent(uint32_t adc_buffer[8], int channel_no);
uint32_t calculatePower(uint32_t adc_buffer[8]);

// Function Handle for current read task
TaskHandle_t vCurrentReadHandle;
TaskHandle_t vRPMReadHandle;

//external structs
extern SemaphoreHandle_t xADC1_Semaphore;
extern MotorState motor_state;
extern QueueHandle_t xRPMQueueExternal;
extern QueueHandle_t xRPMQueueInternal;
extern RPMQueueData xRPMvalue;

//globals
volatile uint32_t pulseCount = 0;
volatile bool timerStarted = false;
volatile uint32_t startTime = 0;
volatile uint32_t endTime = 0;
volatile float integral_error = 0;

/*-----------------------------------------------------------*/

void vCreateMotorTask( void )
{
    xRPMQueueInternal = xQueueCreate(1, sizeof( xRPMvalue ) );
    xRPMQueueExternal = xQueueCreate(1, sizeof( xRPMvalue ) );
    if( ( xRPMQueueExternal == NULL ) || ( xRPMQueueInternal == NULL ) )
    {
        UARTprintf("RPM Queue not created. \n");
    }

    xTaskCreate( prvMotorTask,
                 "Motor",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );
    
    xTaskCreate( vCurrentRead,
                 "Current Read",
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
    
    xTaskCreate( vRPMReadTest,
                 "RPM Read Test",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );

    // Enable Timers
    TimerEnable(TIMER5_BASE, TIMER_A); //RPM Timer
    TimerEnable(TIMER4_BASE, TIMER_A); //Control Timer
}
/*-----------------------------------------------------------*/
// Motor State Machine
static void prvMotorTask( void *pvParameters )
{
    uint16_t duty_value = 10;
    uint16_t period_value = MOTOR_MAX_DUTY;
    int stateA, stateB, stateC;

    // TETSING STATE MACHINE   



    // uint32_t current_sensor[8];
    // uint32_t current_c1 = 0;


    /* Initialise the motors and set the duty cycle (speed) in microseconds */
    initMotorState(&motor_state); // set the struct up

    setMotorRPM(500);
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

    // UARTprintf("Main Read 1: %d, 2: %d, 3: %d,\n", stateA, stateB, stateC);

    // UARTprintf("STATE: %d \n", motor_state.current_state);
    // UARTprintf("Current RPM: %d \n", motor_state.current_rpm);
    // UARTprintf("set RPM: %d \n", motor_state.set_rpm);
    // UARTprintf("Duty Value: %d \n", (uint16_t)motor_state.duty_value);
    // UARTprintf("Estop condition: %d \n", motor_state.Estop_condition);

    for (;;)
    {

        //wake on state change (Notify/ Semaphore)
        switch(motor_state.current_state){
            case Idle:
                if ( motor_state.set_rpm > 0){

                    if (motor_state.current_rpm == 0){
                        stateA = GPIOPinRead(HALL_A_PORT, HALL_A_PIN) ? 1 : 0;
                        stateB = GPIOPinRead(HALL_B_PORT, HALL_B_PIN) ? 1 : 0;
                        stateC = GPIOPinRead(HALL_C_PORT, HALL_C_PIN) ? 1 : 0;
                        
                        updateMotor(stateA, stateB, stateC);
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

        vTaskDelay(pdMS_TO_TICKS( 100 ));

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

        if ((print_idx % 100) == 0) {
            // Print the current value mA
            // UARTprintf("Current mA: %d\n", current);

            // Print power. VI
            power = 24 * current;

            // Print the power value
            // UARTprintf("Power mW: %d\n", power);
        }
        print_idx += 1;
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
            index = (index + 1) % 5;
            sum = 0;
            for (int i = 0; i < 5; i++) {
                sum += rpm_array[i];
            }
            rpm_filtered = sum / 5;
        }
        // Send the filtered RPM value to the queue
        xRPMvalueSend.value = rpm_filtered;
        xRPMvalueSend.ticks = 0;
        xQueueSend(xRPMQueueExternal, &xRPMvalueSend, 0);

        // Delay for 100Hz
        vTaskDelay(pdMS_TO_TICKS( 10 ));
    }
}

void vRPMReadTest( void *pvParameters )
{
    RPMQueueData xRPMvalueRecv;
    int print_idx = 0;
    for( ;; )
    {
        // Read the RPM value from the queue
        if (xQueueReceive(xRPMQueueExternal, &xRPMvalueRecv, 0) == pdPASS) 
        {
            if ((print_idx % 20) == 0) {
                // Print the RPM value
                UARTprintf("RPM: %u\n", xRPMvalueRecv.value);
            }
            print_idx += 1;
        }
    }
}

/*-----------------------------------------------------------*/

/* Interrupt handlers */

void Timer4IntHandler(void) {
    // Clear the timer interrupt
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    

    // Calculate error
    int error = motor_state.set_rpm - motor_state.current_rpm;

    // Update integral error
    integral_error += error * TIMER_INTERVAL; // Multiply by dt if available

    // Calculate PID output (scaled appropriately)
    float output = Kp * error + Ki * integral_error;


    // Scale PID output to match the duty cycle range
    // Assuming the maximum motor RPM is 4500 and maximum duty is 45, the scaling factor is 0.01
    float pid_duty = output * 0.01;

    if (pid_duty > MAX_DELTA_DUTY_NORMAL){
        pid_duty = MAX_DELTA_DUTY_NORMAL;
    }
    
    // Update the controller output with the new calculated duty
    motor_state.controller_ouput += pid_duty;
    
    // Apply the output as a new duty cycle (ensuring it remains within 0 to 45)
    motor_state.duty_value = (int)motor_state.controller_ouput;
    motor_state.duty_value = (motor_state.duty_value < 1) ? 1 : (motor_state.duty_value > 45 ? 45 : motor_state.duty_value);
    
    setDuty(motor_state.duty_value);

    // Handle Estop condition
    if (motor_state.Estop_condition == Enabled) {
        // TODO constant deceleration
        // Reset to a safe duty cycle if needed in an emergency
        setDuty(1); // Verify if this should instead be 0 or some other safe value
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

        uint32_t rpm = 60000 / (elapsedTimeInMilliseconds); 

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
    UARTprintf("RPM set to: %d \n", rpm);
    motor_state.set_rpm = rpm;
    //post sem
}

void setMotorEstop(){
    motor_state.Estop_condition = Enabled;
    motor_state.set_rpm = 0;
    //post Sem
}

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