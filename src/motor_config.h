
#define HALL_A_PIN GPIO_PIN_3
#define HALL_A_PORT GPIO_PORTM_BASE
#define HALL_B_PIN GPIO_PIN_2
#define HALL_B_PORT GPIO_PORTH_BASE
#define HALL_C_PIN GPIO_PIN_2
#define HALL_C_PORT GPIO_PORTN_BASE

#define CURRENT_A_PIN GPIO_PIN_6
#define CURRENT_A_PORT GPIO_PORTA_BASE
#define CURRENT_B_PIN GPIO_PIN_7            //Analogue input 4
#define CURRENT_B_PORT GPIO_PORTD_BASE
#define CURRENT_C_PIN GPIO_PIN_3            //Analogue input 0
#define CURRENT_C_PORT GPIO_PORTE_BASE

#define ADC_SEQ_1 0
#define ADC_SEQ_2 2
#define ADC_STEP 0

#define RPM_QUEUE_LENGTH 1
#define MOTOR_MAX_DUTY 45

#define Kp 0.0003  // Proportional gain
#define Ki 0.000012  // Integral gain
#define TIMER_INTERVAL 0.01  // Timer interval in seconds (assuming 100ms here)
#define MAX_DELTA_DUTY_NORMAL 0.01  // Max change in duty cycle under normal conditions per interval
#define MAX_DELTA_DUTY_ESTOP 0.02   // Max change in duty cycle under Estop per interval
#define INTEGRAL_MAX 1000

// Global Structs
SemaphoreHandle_t xADC1_Semaphore;
QueueHandle_t xRPMQueueInternal;
QueueHandle_t xRPMQueueExternal;
QueueHandle_t xCurrentQueueInternal;
QueueHandle_t xCurrentQueueExternal;
QueueHandle_t xPowerQueueInternal;
QueueHandle_t xPowerQueueExternal;

typedef struct 
{
    uint32_t value;
    uint32_t ticks; //for debug
} RPMQueueData;

typedef struct 
{
    int32_t value;  // Signed
} CurrentQueueData;

typedef struct 
{
    uint32_t value; // Unsigned
} PowerQueueData;

#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

typedef enum {
    Idle = 0,
    Starting,
    Running,
    EStop,
} MotorStateMachine_e;

typedef enum {
    Disabled,
    Enabled
} EStop_e;

typedef struct{
    volatile MotorStateMachine_e current_state;
    volatile EStop_e Estop_condition;
    volatile uint32_t set_rpm;
    volatile uint32_t current_rpm;
    volatile float duty_value;
    volatile float controller_ouput;
    volatile float controller_error;
} MotorState;

MotorState motor_state;
//RPMQueueData xRPMvalue;
#endif // MOTOR_STATE_H