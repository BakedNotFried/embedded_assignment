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
/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvMotorTask( void *pvParameters );
static void prvMotorTestTask( void *pvParameters );

/*
 * Called by main() to create the Hello print task.
 */
void vCreateMotorTask( void );

/*
 * Hardware interrupt handlers
 */
void HallSensorHandler(void);
void ADC1_Sequence1_Handler(void);

/* Hardware Setup*/
extern void ADC1_Sequence1_Handler(void);
extern void prvConfigureHallInts( void );
extern void prvADC1_Init(void) ;
extern void prvConfigureMotorRPMTimer(void);
extern void prvConfigureMotorControlTimer(void);

//helper functions
extern void ADC1_Read(uint32_t current_sensor[8]);
extern void MotorRPMTimerStart(void);
extern void MotorRPMTimerStop(void);  
extern void initMotorState(MotorState *motor_state);
extern void setMotorRPM(uint16_t rpm);
extern void setMotorEstop(void);
extern void unsetMotorEStop(void);
extern void startKickMotor(void);

extern uint32_t calculateCurrent(uint32_t adc_buffer[8], int channel_no);
extern uint32_t calculatePower(uint32_t adc_buffer[8]);


//external structs
extern SemaphoreHandle_t xADC1_Semaphore;
extern SemaphoreHandle_t xMotorState_Semaphore;
extern MotorState motor_state;
extern QueueHandle_t xRPMQueue;
extern RPMQueueData xRPMvalue;

/*-----------------------------------------------------------*/

void vCreateMotorTask( void )
{
    //Hardware setups
    prvConfigureHallInts();

    prvADC1_Init();

    prvConfigureMotorRPMTimer();

    prvConfigureMotorControlTimer();

    xRPMQueue = xQueueCreate(
                        RPM_QUEUE_LENGTH,
                        sizeof( xRPMvalue ) );
    if( ( xRPMQueue == NULL ) )
    {
        //error here
    }
    
    xMotorState_Semaphore = xSemaphoreCreateBinary();
    xADC1_Semaphore = xSemaphoreCreateBinary();

    xTaskCreate( prvMotorTask,
                 "Motor",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );

    xTaskCreate( prvMotorTestTask,
            "Motor",
            configMINIMAL_STACK_SIZE,
            NULL,
            tskIDLE_PRIORITY + 2,
            NULL );            

    TimerEnable(TIMER5_BASE, TIMER_A); //RPM Timer
    TimerEnable(TIMER4_BASE, TIMER_A); //Control Timer
}
/*-----------------------------------------------------------*/

static void prvMotorTask( void *pvParameters )
{
    uint16_t duty_value = 10; // start at 10 duty to kick start the motor
    uint16_t period_value = MOTOR_MAX_DUTY;
    uint32_t current_sensor[8];

    /* Initialise the motors and set the duty cycle (speed) in microseconds */
    initMotorState(&motor_state); // set the struct up

    // TETSING STATE MACHINE & RPM moved to other task.
    //setMotorRPM(1200); //uncomment for single speed
    enableMotor();
    initMotorLib(period_value);
    //motorKickStart(duty_value); //uncomment for single speed 

    
    UARTprintf("STATE: %d \n", motor_state.current_state);
    UARTprintf("Current RPM: %d \n", motor_state.current_rpm);
    UARTprintf("set RPM: %d \n", motor_state.set_rpm);
    UARTprintf("Duty Value: %d \n", (uint16_t)motor_state.duty_value);
    UARTprintf("Estop condition: %d \n", motor_state.Estop_condition);

    // ----------------Start State Machine ------------------------------------------
    for (;;)
    {
        if (xSemaphoreTake(xMotorState_Semaphore, portMAX_DELAY) == pdTRUE) {
            //wake on state change (Notify/Semaphore) and check state change
            switch(motor_state.current_state){
                case Idle:
                    if ( motor_state.set_rpm > 0){
                        if (motor_state.current_rpm == 0){
                            uint16_t duty_value = 10;
                            motorKickStart(duty_value);
                        }
                        motor_state.current_state = Starting;
                    }
                break;

                case Starting:
                    if (motor_state.Estop_condition == Enabled){
                        motor_state.current_state = EStop;
                    }
                    if (motor_state.set_rpm == motor_state.current_rpm){
                        motor_state.current_state = Running;
                    }
                break;

                case Running:
                    if (motor_state.Estop_condition == Enabled){
                        motor_state.current_state = EStop;
                    }
                    if (motor_state.current_rpm == 0){
                        motor_state.current_state = Idle;
                    }
                break;

                case EStop:
                    if (motor_state.current_rpm == 0){
                        motor_state.current_state = Idle;
                        unsetMotorEstop();
                    }     
                break;    
            }   
        }

        // ---------------------- DEBUG PRINTS BELOW -----------------------------

        // Check the RPM Queue from Hall Interrupts, if no data we have 0 RPM
        RPMQueueData xRecievedRPM;
        if (xQueueReceive(xRPMQueue, &xRecievedRPM, 100) == pdPASS) {
            //motor_state.current_rpm = xRecievedRPM.value; UPDATE IN ISR so that it can react quickly
            UARTprintf("RPM: %u\n", xRecievedRPM.value);
            UARTprintf("Ticks: %u\n", xRecievedRPM.ticks);
        }else{
            //did not receive from Queue in 100ms
            UARTprintf("RPM: 0\n");
        }

        UARTprintf("STATE: %d \n", motor_state.current_state);
        UARTprintf("Current RPM: %d \n", motor_state.current_rpm);
        UARTprintf("set RPM: %u \n", motor_state.set_rpm);
        UARTprintf("Duty Value: %d \n", (uint16_t)motor_state.duty_value);
        UARTprintf("Estop condition: %d \n", motor_state.Estop_condition);
        UARTprintf("Controller Error: %d \n", (int32_t)motor_state.controller_error);
        UARTprintf("Controller Output: %d \n", (int32_t)motor_state.controller_ouput);

        ADC1_Read(current_sensor);
        char bufferC[100];
        uint32_t average_0 = (current_sensor[0]+ current_sensor[1]+ current_sensor[2]+ current_sensor[3])/4;
        usprintf(bufferC, "Channel 0 Value: %u, value: %u, value %u, value: %u average :%u \n", current_sensor[0], current_sensor[1], current_sensor[2], current_sensor[3], average_0);
        UARTprintf(bufferC);

        char bufferB[100];
        uint32_t average_4 = (current_sensor[4]+ current_sensor[5]+ current_sensor[6]+ current_sensor[7])/4;
        usprintf(bufferB, "Channel 4 Value: %u, value: %u, value %u, value: %u  average: %u \n", current_sensor[4], current_sensor[5], current_sensor[6], current_sensor[7], average_4);
        UARTprintf(bufferB);

        int power = (int)calculatePower(current_sensor);
        UARTprintf("Power draw: %d milliWatts\n", power);

        //vTaskDelay(pdMS_TO_TICKS( 3000 ));
    }
}

/*----------------------------    Motor Testing Sequence -------------------------------*/

static void prvMotorTestTask( void *pvParameters )
{
    // implement testing here after 3secs, try change state - enable estop etc.
    //- IDLE STATE - 
    // setMotorRPM(0);
    // setMotorRPM(1000);
    // - STARTING STATE - 
    // vTaskDelay(pdMS_TO_TICKS( 3000 ));
    // - RUNNING STATE - 
    // setMotorRPM(4500);
    // vTaskDelay(pdMS_TO_TICKS( 3000 ));
    // - RUNNING STATE -
    // setMotorEstop(void);
    // - ESTOP STATE -
    //  vTaskDelay(pdMS_TO_TICKS( 3000 ));
    // setMotorRPM(1000);

    // Test reading conditions
    // Read from ADC Queue
    // Read from RPM Queue
}
/*----------------------------          END         -------------------------------*/

