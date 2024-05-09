
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
#include "driverlib/interrupt.h"

#include "motorlib.h"
#include "motor_config.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"

//############################## GLOBAL MOTOR VARIABLES ###################################//
//############################## GLOBAL MOTOR VARIABLES ###################################//
volatile uint32_t pulseCount = 0;
volatile bool timerStarted = false;
volatile uint32_t startTime = 0;
volatile uint32_t endTime = 0;
volatile float integral_error = 0;

//############################### HARDWARE SETUP ###########################################
//############################### HARDWARE SETUP ###########################################
void prvConfigureHallInts( void )
{
    // Enable the peripheral for Hall Ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    // Wait for the GPIO module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH)){}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)){}
    
    // Set the pin type for HALL pins as input
    GPIOPinTypeGPIOInput(HALL_A_PORT, HALL_A_PIN);
    GPIOPinTypeGPIOInput(HALL_B_PORT, HALL_B_PIN);
    GPIOPinTypeGPIOInput(HALL_C_PORT, HALL_C_PIN);

    // Set the interrupt type for both falling and rising edge
    GPIOIntTypeSet(HALL_A_PORT, HALL_A_PIN, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(HALL_B_PORT, HALL_B_PIN, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(HALL_C_PORT, HALL_C_PIN, GPIO_BOTH_EDGES);

    // Enable the interrupt for GPIOP pin 2
    GPIOIntEnable(HALL_A_PORT, HALL_A_PIN);
    GPIOIntEnable(HALL_B_PORT, HALL_B_PIN);
    GPIOIntEnable(HALL_C_PORT, HALL_C_PIN);

    // Enable the GPIOP interrupt in the NVIC
    IntEnable(INT_GPIOM);
    IntEnable(INT_GPION);
    IntEnable(INT_GPIOH);

    IntMasterEnable(); //
}

void prvADC1_Init(void) //ADC1 on PE3 
{
    SysCtlPeripheralEnable( SYSCTL_PERIPH_ADC1 );

    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );

    //TODO: CALIBRATE? 

    //Makes GPIO an INPUT and sets them to be ANALOG
    GPIOPinTypeADC( GPIO_PORTE_BASE, GPIO_PIN_3 );
    GPIOPinTypeADC( GPIO_PORTD_BASE, GPIO_PIN_7 );

    ADCSequenceConfigure( ADC1_BASE, ADC_SEQ_1, ADC_TRIGGER_PROCESSOR, 0);
    
    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQ_1, ADC_STEP, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQ_1, ADC_STEP+1, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQ_1, ADC_STEP+2, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQ_1, ADC_STEP+3, ADC_CTL_CH0);

    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQ_1, ADC_STEP+4, ADC_CTL_CH4);
    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQ_1, ADC_STEP+5, ADC_CTL_CH4);
    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQ_1, ADC_STEP+6, ADC_CTL_CH4);
    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQ_1, ADC_STEP+7, ADC_CTL_IE | ADC_CTL_CH4 | ADC_CTL_END );
    //uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Step, uint32_t ui32Config
    
    ADCSequenceEnable( ADC1_BASE, ADC_SEQ_1);
    ADCIntClear( ADC1_BASE, ADC_SEQ_1);

    // Register the ADC1 sequence 1 interrupt handler
    ADCIntRegister(ADC1_BASE, ADC_SEQ_1, ADC1_Sequence1_Handler);

    // Enable the ADC1 sequence 1 interrupt
    ADCIntEnable(ADC1_BASE, ADC_SEQ_1);
}


void prvConfigureMotorRPMTimer(void) {
    // Enable the peripheral clock for the timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);

    // Wait for the timer module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER5)) {}

    // Configure Timer5 as a 32-bit timer in periodic mode
    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);

    //120Mhz sys clock, load set for 2 seconds as low RPM eg 30 RPM = 1 revolution every 2 seconds
    uint32_t ui32Period = SysCtlClockGet()*2; // 
    TimerLoadSet(TIMER5_BASE, TIMER_A, ui32Period - 1);
}

void prvConfigureMotorControlTimer(void) {
    uint32_t ui32Period;

    // Enable the peripheral clock for the timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);

    // Wait for the timer module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4)) {}

    // Configure Timer4 as a 32-bit timer in periodic mode
    TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);

    // Calculate the load value for a 10 ms interval
    ui32Period = (SysCtlClockGet() / 100) - 1;
    TimerLoadSet(TIMER4_BASE, TIMER_A, ui32Period);

    // Enable the timer to start running
    TimerEnable(TIMER4_BASE, TIMER_A);

    // set up an interrupt for the timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER4A);
}

//######################## HARDWARE SETUP END ################################//////////////////////////
//######################## HARDWARE SETUP END ################################//////////////////////////

//######################## INTERRUPT HANDLERS ################################//////////////////////////
//######################## INTERRUPT HANDLERS ################################//////////////////////////

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
    if (motor_state.Estop_condition == Disabled){
        if (motor_state.duty_value > 1.01 && motor_state.current_rpm > 0){
            // we are accelerating
            motor_state.duty_value = (int)motor_state.controller_ouput;
            motor_state.duty_value = (motor_state.duty_value < 1) ? 1 : (motor_state.duty_value > 45 ? 45 : motor_state.duty_value);
            setDuty(motor_state.duty_value);
        }else{
            // we have stopped. hit the soft brake.
            stopMotor(false);
        }
    }else{
        // TEST THIS ESTOP -----------------------
        if (motor_state.duty_value > 1.02){
            // we are decelerating
            motor_state.duty_value -= MAX_DELTA_DUTY_ESTOP;
            setDuty(motor_state.duty_value);
        }else{
            // we have stopped hit the hard brake.
            stopMotor(true);
        }
    }
}

void HallSensorHandler(void)
{
    RPMQueueData xRPMvalue;
    uint32_t HALL_A_Status, HALL_B_Status, HALL_C_Status;
    int stateA, stateB, stateC;
    int num_pole_pairs = 4;

    // Read the interrupt status to find the cause of the interrupt.
    HALL_A_Status = GPIOIntStatus(HALL_A_PORT, true);
    HALL_B_Status = GPIOIntStatus(HALL_B_PORT, true);
    HALL_C_Status = GPIOIntStatus(HALL_C_PORT, true);

    // Clear interrupt
    GPIOIntClear(HALL_A_PORT, HALL_A_Status);
    GPIOIntClear(HALL_B_PORT, HALL_B_Status);
    GPIOIntClear(HALL_C_PORT, HALL_C_Status);

    //TIMER CALC RPM
    pulseCount++;

    if (pulseCount == 1) {

        //MotorRPMTimerStart(); // uncomment for Method 1
        startTime = xTaskGetTickCount(); // uncomment for Method 2
    } else if (pulseCount >= 6*num_pole_pairs+1) {

        // Stop the timer on the seventh pulse
        //MotorRPMTimerStop(); //uncomment for Method 1
        endTime = xTaskGetTickCount(); //uncomment for Method 2
        pulseCount = 0; // Reset pulse count for the next rotation

        // --------------RPM Method 2 - Using systemTick. --------------------
        TickType_t elapsedTime = endTime - startTime;
        uint32_t elapsedTimeInMilliseconds = elapsedTime * portTICK_PERIOD_MS;
        uint32_t rpm = 60000 / (elapsedTimeInMilliseconds); 

        // --------------RPM Method 1 Using Periodic Timer----------------------
        //uint32_t elapsedTime = startTime - endTime; // Assuming the timer counts down
        //float factor = 553846154.0; // should be 720000000, but it was out by a factor of 1.3 ?????
        //uint32_t rpm = (uint32_t)((factor / ((float)elapsedTime)) * 10.0); //add digits back in 
        //calc RPM elapsedTime in systemticks. each system tick 8.33nS. 60*10^9/8.33333 = 7200000000 (avoid float) 

        motor_state.current_rpm = rpm; // This updates our global value.

        xRPMvalue.value = rpm;
        xRPMvalue.ticks = elapsedTime;
        if (xQueueSend(xRPMQueue, &xRPMvalue, 0) != pdPASS) {
            // Handle error: Queue is full
        }

    }

    // Read hall effect sensors
    stateA = GPIOPinRead(HALL_A_PORT, HALL_A_PIN) ? 1 : 0;
    stateB = GPIOPinRead(HALL_B_PORT, HALL_B_PIN) ? 1 : 0;
    stateC = GPIOPinRead(HALL_C_PORT, HALL_C_PIN) ? 1 : 0;

    //UARTprintf("Hall Read 1: %d, 2: %d, 3: %d,\n", stateA, stateB, stateC);
    //Note be cautious calling UARTPrint in Interrupt. motor will shudder.

    // Call updateMotor to change to the next phase
    updateMotor(stateA, stateB, stateC);

    //Optionally add code for motor speed sensing or additional control feedback.
}
void ADC1_Sequence1_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Clear the ADC sequence interrupt status
    ADCIntClear(ADC1_BASE, ADC_SEQ_1);

    // Give the semaphore from ISR
    xSemaphoreGiveFromISR(xADC1_Semaphore, &xHigherPriorityTaskWoken);

    // Yield if a higher priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//######################## INTERRUPT HANDLERS END ################################//////////////////////////
//######################## INTERRUPT HANDLERS END ################################//////////////////////////

//######################## RPM TIMER HELPERS  ####################################//////////////////////////
//########################  RPM TIMER HELPERS ####################################//////////////////////////

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

//######################## RPM TIMER HELPERS END  ####################################//////////////////////////
//########################  RPM TIMER HELPERS END ####################################//////////////////////////

//########################  HELPER FUNCTIONS ####################################//////////////////////////
//######################## HELPER FUNCTIONS ####################################//////////////////////////

void initMotorState(MotorState *motor_state){
    motor_state->current_state = Idle;
    motor_state->Estop_condition = Disabled;
    motor_state->set_rpm = 0;
    motor_state->current_rpm = 0;
}

void ADC1_Read(uint32_t current_sensor[8]) {
    // Trigger the ADC conversion.
    ADCProcessorTrigger(ADC1_BASE, ADC_SEQ_1);

    // Wait for the semaphore to be given by the interrupt handler
    if (xSemaphoreTake(xADC1_Semaphore, portMAX_DELAY) == pdTRUE) {
        // Read ADC FIFO buffer from sample sequence
        ADCSequenceDataGet(ADC1_BASE, ADC_SEQ_1, current_sensor);
    }

    // Clear any potential pending ADC interrupts (safety)
    ADCIntClear(ADC1_BASE, ADC_SEQ_1);
}

uint32_t calculateCurrent(uint32_t adc_buffer[8], int channel_no) {
    int32_t v_shunt_mV;
    // Convert ADC value to voltage across the shunt in millivolts (mV)
    if (channel_no == 0){
        uint32_t average_0 = (adc_buffer[0]+ adc_buffer[1]+ adc_buffer[2]+ adc_buffer[3])/4;
        v_shunt_mV = ((int32_t)average_0) * 3300 / 4096; // Subtract midpoint, scale to mV
    }else if (channel_no == 4){
        uint32_t average_4 = (adc_buffer[4]+ adc_buffer[5]+ adc_buffer[6]+ adc_buffer[7])/4;
        v_shunt_mV = ((int32_t)average_4) * 3300 / 4096; // Subtract midpoint, scale to mV
    }else{
        v_shunt_mV = 0;
    }    

    // Current calculation in milliamperes (mA), using shunt resistor value in milliohms (mOhms)
    // Shunt value: 7 mOhms (0.007 Ohms)
    uint32_t current_mA = (v_shunt_mV / 7) / 10; // Calculate current through the shunt resistor, with gain of 10

    return current_mA; // Return the current in milliamperes
}

uint32_t calculatePower(uint32_t adc_buffer[8]) {
    // Calculate currents for two phases in milliamperes (mA)
    uint32_t current1_mA = calculateCurrent(adc_buffer, 0);
    uint32_t current2_mA = calculateCurrent(adc_buffer, 4);
    //UARTprintf("Current draw: %u milliAmps\n", current1_mA);
    //UARTprintf("Current draw: %u milliAmps\n", current2_mA);

    // Calculate the average of two measured currents
    uint32_t i_avg_mA = (current1_mA + current2_mA) / 2;

    // Estimate the total current as three times the average current
    uint32_t i_total_mA = current1_mA + current2_mA + i_avg_mA; // Total current for three phases

    // Calculate power in milliwatts (mW), then convert to watts (W)
    // Note: Motor voltage (VVM) = 24000 mV (24 V)
    uint32_t power_mW = ((24000 - 3300) * (i_total_mA)) /1000; // Power in mWatts, (mV * Amps)

    return power_mW ; // Convert milliwatts 
}

//######################## HELPER FUNCTIONS END####################################//////////////////////////
//######################## HELPER FUNCTIONS END####################################//////////////////////////

//########################  THREAD INTERFACE FUNCTIONS  ####################################//////////////////////////
//########################  THREAD INTERFACE FUNCTIONS ####################################//////////////////////////

void setMotorRPM(uint16_t rpm){
    UARTprintf("RPM set to: %d \n", rpm);
    motor_state.set_rpm = rpm;
    xSemaphoreGive(xMotorState_Semaphore);
    //yield?
}

void setMotorEstop(){
    motor_state.Estop_condition = Enabled;
    motor_state.set_rpm = 0;
    xSemaphoreGive(xMotorState_Semaphore);
    //yield?
}

void unsetMotorEstop(){
    motor_state.Estop_condition = Disabled;
    xSemaphoreGive(xMotorState_Semaphore);
    //yield?
}

void motorKickStart(uint16_t duty_start){
    //start motor from stop
    setDuty(duty_start); // give a duty cycle as initial condition for control system
    int stateA, stateB, stateC;
    stateA = GPIOPinRead(HALL_A_PORT, HALL_A_PIN) ? 1 : 0;
    stateB = GPIOPinRead(HALL_B_PORT, HALL_B_PIN) ? 1 : 0;
    stateC = GPIOPinRead(HALL_C_PORT, HALL_C_PIN) ? 1 : 0;

    UARTprintf("Motor Kick Start Hall 1: %d, 2: %d, 3: %d,\n", stateA, stateB, stateC);
    updateMotor(stateA, stateB, stateC);
}