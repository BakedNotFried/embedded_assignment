/*
/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/* Hardware includes. */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"
#include "drivers/opt3001.h"
#include "driverlib/pwm.h"

// Motor lib
#include <motorlib.h>
#include "driverlib/adc.h"
#include "motor_config.h"
#include "driverlib/timer.h"

/*-----------------------------------------------------------*/
/*                         Shared Global Section             */
/* The system clock frequency. */
uint32_t g_ui32SysClock;

// Hardware and UART setup
static void prvSetupHardware( void );
static void prvConfigureUART(void);

/*-----------------------------------------------------------*/
/*                   Sensor Tasks Global Section             */

// Global Semaphores for I2C0 non-blocking read/write
SemaphoreHandle_t xI2C0OPTSemaphore = NULL;
SemaphoreHandle_t xI2C0BMISemaphore = NULL;
// Global Mutex for I2C0
SemaphoreHandle_t xI2C0Mutex = NULL;
SemaphoreHandle_t xI2C2BusMutex = NULL;

// Functions for initializing tasks
extern void vSensorTaskSetup( void );

/*-----------------------------------------------------------*/
/*                   Motor Tasks Global Section              */

// Motor Tings
extern void vCreateMotorTask( void );
extern void prvMotorPrint( void );
extern void ADC1_Sequence1_Handler(void);
static void prvConfigureHallInts( void );
static void prvADC1_Init(void) ;
static void prvConfigureMotorRPMTimer(void);
static void prvConfigureMotorControlTimer(void);

/*-----------------------------------------------------------*/
/*                   GUI Tasks Global Section                */

/*-----------------------------------------------------------*/
int main( void )

{
    /*                   Shared Main Setup                     */
    prvSetupHardware();
    prvConfigureUART();
    UARTprintf("Input Configuration");  // This is needed to make UART work. Don't Delete (idk wtf is going on here)

    /*-----------------------------------------------------------*/
    /*                   Motor Main Setup (Jim)                  */
    // Do Motor Stuff
    // Semaphore for I2C non-blocking read/write
    xI2C0OPTSemaphore = xSemaphoreCreateBinary();
    xI2C0BMISemaphore = xSemaphoreCreateBinary();
    // Mutex for I2C
    xI2C0Mutex = xSemaphoreCreateMutex();
    xI2C2BusMutex = xSemaphoreCreateMutex();

    if ( (xI2C0OPTSemaphore != NULL) && (xI2C0BMISemaphore != NULL) && (xI2C0Mutex != NULL) && (xI2C2BusMutex != NULL) )
    {
        vCreateMotorTask();
        
    }

    UARTprintf("Input Configuration");  // This is needed to make UART work. Don't Delete (idk wtf is going on here)

    /*                   Sensor Main Setup (Cal)               */
    // // Semaphore for I2C non-blocking read/write
    // xI2C0OPTSemaphore = xSemaphoreCreateBinary();
    // xI2C0BMISemaphore = xSemaphoreCreateBinary();
    // // Mutex for I2C
    // xI2C0Mutex = xSemaphoreCreateMutex();
    // xI2C2BusMutex = xSemaphoreCreateMutex();

    // if ( (xI2C0OPTSemaphore != NULL) && (xI2C0BMISemaphore != NULL) && (xI2C0Mutex != NULL) && (xI2C2BusMutex != NULL) )
    // {
    //     // Create the tasks associated with sensor reading
    //     vSensorTaskSetup();
    // }


    /*-----------------------------------------------------------*/
    /*                   GUI Main Setup (Nikolaj)                */

    /*-----------------------------------------------------------*/
    // Start the scheduler
    vTaskStartScheduler();
    // Should never reach here
    for( ;; );
}

/*-----------------------------------------------------------*/
/*                    Initial Hardware Setup                 */
static void prvConfigureUART(void)
{
    /* Enable GPIO port A which is used for UART0 pins.
     * TODO: change this to whichever GPIO port you are using. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /* Configure the pin muxing for UART0 functions on port A0 and A1.
     * This step is not necessary if your part does not support pin muxing.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    /* Enable UART0 so that we can configure the clock. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Use the internal 16MHz oscillator as the UART clock source. */
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    /* Select the alternate (UART) function for these pins.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Initialize the UART for console I/O. */
    UARTStdioConfig(0, 9600, 16000000);
}
/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    /* Run from the PLL at configCPU_CLOCK_HZ MHz. */
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
            SYSCTL_CFG_VCO_240), configCPU_CLOCK_HZ);

    /* Configure device pins. */
    PinoutSet(false, false);

    /* Set-up interrupts for hall sensors */
    prvConfigureHallInts();

    prvADC1_Init();

    prvConfigureMotorRPMTimer();

    prvConfigureMotorControlTimer();
}

/*-----------------------------------------------------------*/
/*                 Motor Specific Setup                      */
static void prvConfigureHallInts( void )
{
    /* Configure GPIO ports to trigger an interrupt on rising/falling or both edges. */
    /* Enable the interrupt for LaunchPad GPIO Port in the GPIO peripheral. */
    /* Enable the Ports interrupt in the NVIC. */
    /* Enable global interrupts in the NVIC. */

    // Enable the peripheral for Hall Ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    // Wait for the GPIO module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }
    
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

static void prvADC1_Init(void) //ADC1 on PE3
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //Makes GPIO an INPUT and sets them to be ANALOG
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);

    ADCSequenceConfigure(ADC1_BASE, ADC_SEQ_1, ADC_TRIGGER_PROCESSOR, 0);

    // Configure step 0 for channel 0 (PE3)
    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQ_1, 0, ADC_CTL_CH0);

    // Configure step 1 for channel 4 (PD7) and enable interrupt and end sequence
    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQ_1, 1, ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC1_BASE, ADC_SEQ_1);
    ADCIntClear(ADC1_BASE, ADC_SEQ_1);

    // Create the semaphore
    xADC1_Semaphore = xSemaphoreCreateBinary();

    // Register the ADC1 sequence 1 interrupt handler
    ADCIntRegister(ADC1_BASE, ADC_SEQ_1, ADC1_Sequence1_Handler);

    // Enable the ADC1 sequence 1 interrupt
    ADCIntEnable(ADC1_BASE, ADC_SEQ_1);
}

static void prvConfigureMotorRPMTimer(void) {
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

static void prvConfigureMotorControlTimer(void) {
    uint32_t ui32Period;

    // Enable the peripheral clock for the timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);

    // Wait for the timer module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4)) {}

    // Configure Timer4 as a 32-bit timer in periodic mode
    TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);

    // Calculate the load value for a 100 ms interval
    // System clock in Hz = SysCtlClockGet(), e.g., 120 MHz = 120000000 Hz
    // Timer interval in seconds = 10 ms = 0.01 seconds
    // Load value = (System Clock * Timer Interval) - 1
    ui32Period = (SysCtlClockGet() / 100) - 1;
    TimerLoadSet(TIMER4_BASE, TIMER_A, ui32Period);

    // Enable the timer to start running
    TimerEnable(TIMER4_BASE, TIMER_A);

    // set up an interrupt for the timer
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER4A);
}

/*-----------------------------------------------------------*/
/*                 RTOS Shenanigans                          */
void vApplicationMallocFailedHook( void )
{
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/

void *malloc( size_t xSize )
{
    /* There should not be a heap defined, so trap any attempts to call
    malloc. */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/


