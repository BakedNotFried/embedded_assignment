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
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"


//From opt
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/ustdlib.h"
#include "drivers/pinout.h"
#include "drivers/opt3001.h"
#include <driverlib/timer.h>
#include "semphr.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

//Screen 
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/flash.h"
#include "driverlib/systick.h"
#include "driverlib/udma.h"
#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/checkbox.h"
#include "grlib/container.h"
#include "grlib/radiobutton.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"
#include "images.h"
#include "grlib/canvas.h"
#include "grlib/pushbutton.h"
#include "grlib/slider.h"

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

// BMI include
#include "drivers/i2cBMIDriver.h"
#include "drivers/bmi160.h"
#include "drivers/bmi160_defs.h"

//required for motor
#include <stdlib.h>  

#define DIGITS_AFTER_DP 10
/*-----------------------------------------------------------*/

/*
 * Define the task IDs.
 */
#define TASK1_ID 0
#define TASK2_ID 1
#define TASK3_ID 2
#define TASK4_ID 3

/*
 * Priorities at which the tasks are created.
 */
#define mainQUEUE_RECEIVE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_SEND_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1 )

/*
 * The number of items the queue can hold.  This is 4 as the receive task
 * will remove items as they are added, meaning the send task should always find
 * the queue empty.
 */
#define mainQUEUE_LENGTH                    ( 4 )
#define maxSamples 10
#define mainDataQUEUE_LENGTH                ( maxSamples )


#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

/*
 * The queue used by both tasks.
 */
struct AMessage
{
    uint32_t ulMessageID;
    uint32_t ulTimeStamp;
} xMessage;

struct AData
{
    int ulSamples;
} xData;

//uint32_t g_ui32SysClock;


/*
 * Queue used to send and receive complete struct AMessage structures.
 */
QueueHandle_t xStructQueue = NULL;

/*
 * Queue used to send and receive pointers to struct AMessage structures.
 */
QueueHandle_t xPointerQueue = NULL;

QueueHandle_t xOptDataQueue = NULL;

QueueHandle_t xOptAverageQueue = NULL;

SemaphoreHandle_t xOptSemaphore = NULL;


/* The system clock frequency. */
uint32_t g_ui32SysClock;





//*****************************************************************************
//
// The DMA control structure table.
//
//*****************************************************************************
#ifdef ewarm
#pragma data_alignment=1024
tDMAControlTable psDMAControlTable[64];
#elif defined(ccs)
#pragma DATA_ALIGN(psDMAControlTable, 1024)
tDMAControlTable psDMAControlTable[64];
#else
tDMAControlTable psDMAControlTable[64] __attribute__ ((aligned(1024)));
#endif



//Event 
EventGroupHandle_t task_Event;
const uint32_t raw_task_id = (1<<0);
const uint32_t avg_task_id = (1<<1);

const uint32_t button_task_id = (1<<2);

//For screen 
uint32_t g_ui32Panel = 0;
bool MotorData = false;
bool MotorPlot = false;
bool ligthData = false;
bool SensorData = false;
bool SensorPlot = false;
bool dayChange = 1;
bool nightChange = 1;
int countspeed = 0;
int countpower = 0;
int countlux = 0;
int countacc = 0;

extern tCanvasWidget g_psPanels[];
tContext sContext;
tRectangle sRect;

//uint32_t task_all_bits = (raw_task_id|avg_task_id);
/*
 * The tasks as described in the comments at the top of this file.
 */
// static void prvQueueReceiveTask1( void *pvParameters );
// static void prvQueueReceiveTask2( void *pvParameters );
// static void prvQueueSendTask1( void *pvParameters );
// static void prvQueueSendTask2( void *pvParameters );
// static void prvQueueSendTask3( void *pvParameters );
// static void prvQueueSendTask4( void *pvParameters );
// Tasks
static void prvOpt( void *pvParameters );
static void prvDisplay( void *pvParameters );
static void prvMotorTask( void *pvParameters );
static void vCurrentRead( void *pvParameters );
static void vRPMRead( void *pvParameters );
//static void vQueueReadTest( void *pvParameters );

void movingAverageFilter(float *newReading, float *average);



//*****************************************************************************
//
// Forward declarations for the globals required to define the widgets at
// compile-time.
//
//*****************************************************************************
void OnPrevious(tWidget *psWidget);
void OnNext(tWidget *psWidget);
void OnIntroPaint(void);
void MotorOnOff( void );
void MotorSpeed(void);
void MotorLimitLow(void);
void MotorLimitHigh(void);
void MotorLimitAcc(void);
void OnButtonPress(tWidget *psWidget);
void OnSliderSpeed(tWidget *psWidget, int32_t i32Value);
void OnSliderLimitLow(tWidget *psWidget, int32_t i32Value);
void OnSliderLimitHigh(tWidget *psWidget, int32_t i32Value);
void OnSliderLimitAcc(tWidget *psWidget, int32_t i32Value);



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
void motorStart(uint16_t rpm);
void motorStop(bool brake);

//extern tCanvasWidget g_psPanels[];

// Canvas(g_sIntroduction, g_psPanels, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 24,
//        320, 166, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnIntroPaint);

//*****************************************************************************
//
// The buttons and text across the bottom of the screen.
//
//*****************************************************************************
RectangularButton(g_sPrevious, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 190,
                  50, 50, PB_STYLE_FILL, ClrBlack, ClrBlack, 0, ClrSilver,
                  &g_sFontCm20, "-", g_pui8Blue50x50, g_pui8Blue50x50Press, 0, 0,
                  OnPrevious);

Canvas(g_sTitle, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 50, 190, 220, 50,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE, 0, 0, ClrSilver,
       &g_sFontCm20, 0, 0, 0);

RectangularButton(g_sNext, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 270, 190,
                  50, 50, PB_STYLE_IMG | PB_STYLE_TEXT, ClrBlack, ClrBlack, 0,
                  ClrSilver, &g_sFontCm20, "+", g_pui8Blue50x50,
                  g_pui8Blue50x50Press, 0, 0, OnNext);



    CircularButton(g_sMotorOn, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 80, 120,
                  60,PB_STYLE_IMG | PB_STYLE_TEXT,
                  ClrLimeGreen, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm20b, "ON", 0, 0, 0, 0, OnButtonPress);

    CircularButton(g_sMotorOff, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 80, 120,
                  60,PB_STYLE_IMG | PB_STYLE_TEXT,
                  ClrRed, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm20b, "OFF", 0, 0, 0, 0, OnButtonPress);

    Slider(g_sSliderSpeed, 0, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 180, 50, 110, 30, 0, 4530, 500,
                  (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "500", 0, 0, OnSliderSpeed);

    Slider(g_sSliderLimitsLow, 0, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 180, 100, 110, 30, 0, 15, 0,
                  (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "0", 0, 0, OnSliderLimitLow);
                
    Slider(g_sSliderLimitsHigh, 0, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 180, 150, 110, 30, 0, 15, 15,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "15", 0, 0, OnSliderLimitHigh);

    Slider(g_sSliderAcc, 0, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 180, 100, 110, 30, 0, 100, 100,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "100", 0, 0, OnSliderLimitAcc);


//*****************************************************************************
//
// The number of panels.
//
//*****************************************************************************
#define NUM_PANELS              5//(sizeof(g_psPanels) / sizeof(g_psPanels[0]))


//*****************************************************************************
//
// The names for each of the panels, which is displayed at the bottom of the
// screen.
//
//*****************************************************************************
char *g_pcPanei32Names[] =
{
    "     Introduction     ",
    "     Motor control     ",
    "     Motor data     ",
    "     Sensor control     ",
    "     Sensor data     "
};




//*****************************************************************************
//
// Handles press notifications for the push button widgets.
//
//*****************************************************************************
bool motorState = 0;
int32_t speed = 500;

//Motor on off button
void
OnButtonPress(tWidget *psWidget)
{
    //WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sMotorOn);
    //WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sMotorOff);
    motorState = !motorState;
    if(motorState)
    {
        EXTERNAL_SET_RPM(speed);
        LEDWrite(LED_D1 , true);
        //WidgetRemove((tWidget *)(&g_sMotorOn));
        WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sMotorOff);
        PushButtonImageOff(&g_sMotorOn);
        PushButtonFillOff(&g_sMotorOn);
        PushButtonTextOff(&g_sMotorOn);
        PushButtonTextOff(&g_sMotorOn);
        PushButtonFillOff(&g_sMotorOn);

        PushButtonImageOn(&g_sMotorOff);
        PushButtonFillOn(&g_sMotorOff);
        PushButtonTextOn(&g_sMotorOff);
        PushButtonTextOn(&g_sMotorOff);
        PushButtonFillOn(&g_sMotorOff);
        WidgetPaint((tWidget *)&g_sMotorOff);
    }else{
        LEDWrite(LED_D1 , false);
        EXTERNAL_SET_RPM(0);
        //EXTERNAL_SET_ESTOP();
        //WidgetRemove((tWidget *)(&g_sMotorOff));
        WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sMotorOn);
        PushButtonImageOff(&g_sMotorOff);
        PushButtonFillOff(&g_sMotorOff);
        PushButtonTextOff(&g_sMotorOff);
        PushButtonTextOff(&g_sMotorOff);
        PushButtonFillOff(&g_sMotorOff);

        PushButtonImageOn(&g_sMotorOn);
        PushButtonFillOn(&g_sMotorOn);
        PushButtonTextOn(&g_sMotorOn);
        PushButtonTextOn(&g_sMotorOn);
        PushButtonFillOn(&g_sMotorOn);
        WidgetPaint((tWidget *)&g_sMotorOn);
    }

}

//Slider for speed
void
OnSliderSpeed(tWidget *psWidget, int32_t i32Value)
{
    //static char pcCanvasText[5];
    static char pcSliderText[5];

    speed = i32Value;

        // usprintf(pcCanvasText, "%3d%%", i32Value);
        // CanvasTextSet(&g_sSliderValueCanvas, pcCanvasText);
        // WidgetPaint((tWidget *)&g_sSliderValueCanvas);
    if(motorState)
    {
        EXTERNAL_SET_RPM(i32Value);
    }

        usprintf(pcSliderText, "%3d", i32Value);
        SliderTextSet(&g_sSliderSpeed, pcSliderText);
        WidgetPaint((tWidget *)&g_sSliderSpeed);

}

//Slider for low limit
int32_t limitLow = 0;
int32_t limitHigh = 15;
void
OnSliderLimitLow(tWidget *psWidget, int32_t i32Value)
{
    //static char pcCanvasText[5];
    static char pcSliderText[5];
    


        // usprintf(pcCanvasText, "%3d%%", i32Value);
        // CanvasTextSet(&g_sSliderValueCanvas, pcCanvasText);
        // WidgetPaint((tWidget *)&g_sSliderValueCanvas);

    if(i32Value <= limitHigh - 1)
    {
        limitLow = i32Value;
        usprintf(pcSliderText, "%3d", i32Value);
        SliderTextSet(&g_sSliderLimitsLow, pcSliderText);
        WidgetPaint((tWidget *)&g_sSliderLimitsLow);
    }

}

//Slider for hihg limit 
void
OnSliderLimitHigh(tWidget *psWidget, int32_t i32Value)
{
    //static char pcCanvasText[5];
    static char pcSliderText[5];
    


        // usprintf(pcCanvasText, "%3d%%", i32Value);
        // CanvasTextSet(&g_sSliderValueCanvas, pcCanvasText);
        // WidgetPaint((tWidget *)&g_sSliderValueCanvas);


    if(limitLow + 1 <= i32Value)
    {
        limitHigh = i32Value;
        usprintf(pcSliderText, "%3d", i32Value);
        SliderTextSet(&g_sSliderLimitsHigh, pcSliderText);
        WidgetPaint((tWidget *)&g_sSliderLimitsHigh);
    }
}

int32_t limitAcc = 75;
//Slider for body acc limit 
void
OnSliderLimitAcc(tWidget *psWidget, int32_t i32Value)
{
    //static char pcCanvasText[5];
    static char pcSliderText[5];
    


        // usprintf(pcCanvasText, "%3d%%", i32Value);
        // CanvasTextSet(&g_sSliderValueCanvas, pcCanvasText);
        // WidgetPaint((tWidget *)&g_sSliderValueCanvas);

        limitAcc = i32Value;
        usprintf(pcSliderText, "%3d", i32Value);
        SliderTextSet(&g_sSliderAcc, pcSliderText);
        WidgetPaint((tWidget *)&g_sSliderAcc);

}

//Back button
void
OnPrevious(tWidget *psWidget)
{
    //
    // There is nothing to be done if the first panel is already being
    // displayed.
    //
    if(g_ui32Panel == 0)
    {
        return;
    }

    sRect.i16XMin = 0;
    sRect.i16YMin = 24;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 190;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);
    //
    // Remove the current panel.
    //
    //WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Decrement the panel index.
    //
    g_ui32Panel--;

    //
    // Add and draw the new panel.
    //
    //WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    //WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));


    switch (g_ui32Panel) {
        case 0:
            WidgetRemove((tWidget *)(&g_sMotorOn));
            WidgetRemove((tWidget *)(&g_sMotorOff));
            WidgetRemove((tWidget *)(&g_sSliderSpeed));
            WidgetRemove((tWidget *)(&g_sSliderLimitsLow));
            WidgetRemove((tWidget *)(&g_sSliderLimitsHigh));
            OnIntroPaint();
            break;
        case 1:
            MotorOnOff();
            MotorSpeed();
            MotorLimitLow();
            MotorLimitHigh();
            MotorPlot = false;
            //WidgetRemove((tWidget *)(&g_sSliderAcc));
            break;
        case 2:
            MotorData = true;
            MotorPlot = true;
            countspeed = 0;
            countpower = 0;
            //WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sSliderSpeed);
            WidgetRemove((tWidget *)(&g_sSliderAcc));
            ligthData = false;
            break;
        case 3:
            MotorLimitAcc();
            dayChange = 1;
            nightChange = 1;
            ligthData = true;
            SensorPlot = false;
            //WidgetRemove((tWidget *)(&g_sSliderAcc));
            break;
        case 4:
            SensorData = true;
            SensorPlot = true; 
            countlux = 0;
            countacc = 0;
            break;
        case 5:

            break;
    }
    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[g_ui32Panel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // See if this is the first panel.
    //
    if(g_ui32Panel == 0)
    {
        //
        // Clear the previous button from the display since the first panel is
        // being displayed.
        //
        PushButtonImageOff(&g_sPrevious);
        PushButtonTextOff(&g_sPrevious);
        PushButtonFillOn(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }

    //
    // See if the previous panel was the last panel.
    //
    if(g_ui32Panel == (NUM_PANELS - 2))
    {
        //
        // Display the next button.
        //
        PushButtonImageOn(&g_sNext);
        PushButtonTextOn(&g_sNext);
        PushButtonFillOff(&g_sNext);
        WidgetPaint((tWidget *)&g_sNext);
    }

}

//*****************************************************************************
//
// Handles presses of the next panel button.
//
//*****************************************************************************
//Forward button
void
OnNext(tWidget *psWidget)
{

    //UARTprintf("num: %5d\n", g_ui32Panel);

    //
    // There is nothing to be done if the last panel is already being
    // displayed.
    //
    if(g_ui32Panel == (NUM_PANELS - 1))
    {
        return;
    }
    sRect.i16XMin = 0;
    sRect.i16YMin = 24;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 190;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);
    //
    // Remove the current panel.
    //
    //WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Increment the panel index.
    //
    g_ui32Panel++;
    //UARTprintf("num2: %5d\n", g_ui32Panel);

    // Add and draw the new panel.
    //
    //WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    //WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[g_ui32Panel]);
    WidgetPaint((tWidget *)&g_sTitle);


    switch (g_ui32Panel) {
        case 0:
            //WidgetRemove((tWidget *)(&g_sMotorOn));
            //WidgetRemove((tWidget *)(&g_sMotorOff));
            OnIntroPaint();
            break;
        case 1:
            MotorOnOff();
            MotorSpeed();
            MotorLimitLow();
            MotorLimitHigh();
            break;
        case 2:
            //WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sSliderSpeed);
            WidgetRemove((tWidget *)(&g_sMotorOn));
            WidgetRemove((tWidget *)(&g_sMotorOff));
            WidgetRemove((tWidget *)(&g_sSliderSpeed));
            WidgetRemove((tWidget *)(&g_sSliderLimitsLow));
            WidgetRemove((tWidget *)(&g_sSliderLimitsHigh));
            //WidgetRemove((tWidget *)(&g_sSliderAcc));
            MotorData = true;
            MotorPlot = true;
            countspeed = 0;
            countpower = 0;
            break;
        case 3:
            MotorPlot = false;
            MotorLimitAcc();
            ligthData = true;
            dayChange = 1;
            nightChange = 1;
            break;
        case 4:
            ligthData = false;
            WidgetRemove((tWidget *)(&g_sSliderAcc));
            SensorData = true;
            SensorPlot = true;
            countlux = 0;
            countacc = 0;
            break;
        case 5:
            
            break;
    }
    //
    // See if the previous panel was the first panel.
    //
    if(g_ui32Panel == 1)
    {
        //
        // Display the previous button.
        //
        PushButtonImageOn(&g_sPrevious);
        PushButtonTextOn(&g_sPrevious);
        PushButtonFillOff(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }

    //
    // See if this is the last panel.
    //
    if(g_ui32Panel == (NUM_PANELS - 1))
    {
        //
        // Clear the next button from the display since the last panel is being
        // displayed.
        //
        PushButtonImageOff(&g_sNext);
        PushButtonTextOff(&g_sNext);
        PushButtonFillOn(&g_sNext);
        WidgetPaint((tWidget *)&g_sNext);
    }

}

bool switchButtonPressed = true;

// For the intro
void OnIntroPaint(void)
{
    //UARTprintf("INTRO\n");
        GrContextFontSet(&sContext, &g_sFontCm20);
    GrContextForegroundSet(&sContext, ClrSilver);
    GrStringDraw(&sContext, "Welcome this is for ass2", -1,
                 0, 32, 0);
    GrStringDraw(&sContext, "There are 4 pages.", -1, 0, 50, 0);
    GrStringDraw(&sContext, "Press + or - to navigate pages.", -1, 0,
                 74, 0);
    GrStringDraw(&sContext, "Page 1: Motor start or stop.", -1, 0,
                 92, 0);
    GrStringDraw(&sContext, "Page 2: Motor control....", -1, 0,
                 110, 0);
    GrStringDraw(&sContext, "Page 3: Choosen sensor", -1, 0,
                 128, 0);
    GrStringDraw(&sContext, "Page 4: Choosen sensor data.", -1, 0,
                 146, 0);
    GrStringDraw(&sContext, "Emergy stop of motor is ......", -1, 0,
                 164, 0);
}


void MotorOnOff(void)
{
    //UARTprintf("Motor\n");
    if(motorState)
    {        
        //WidgetRemove((tWidget *)(&g_sMotorOn));
        WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sMotorOff);
        PushButtonImageOn(&g_sMotorOff);
        PushButtonFillOn(&g_sMotorOff);
        PushButtonTextOn(&g_sMotorOff);
        PushButtonTextOn(&g_sMotorOff);
        PushButtonFillOn(&g_sMotorOff);
        WidgetPaint((tWidget *)&g_sMotorOff);
    }else{
        //WidgetRemove((tWidget *)(&g_sMotorOff));
        WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sMotorOn);
        PushButtonImageOn(&g_sMotorOn);
        PushButtonFillOn(&g_sMotorOn);
        PushButtonTextOn(&g_sMotorOn);
        PushButtonTextOn(&g_sMotorOn);
        PushButtonFillOn(&g_sMotorOn);
        WidgetPaint((tWidget *)&g_sMotorOn);
    }
}

void MotorSpeed(void)
{

        //WidgetRemove((tWidget *)(&g_sMotorOff));
        WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sSliderSpeed);
        // PushButtonImageOn(&g_sSliderSpeed);
        // PushButtonFillOn(&g_sSliderSpeed);
        // PushButtonTextOn(&g_sSliderSpeed);
        // PushButtonTextOn(&g_sSliderSpeed);
        // PushButtonFillOn(&g_sSliderSpeed);
        WidgetPaint((tWidget *)&g_sSliderSpeed);
        GrContextFontSet(&sContext, &g_sFontCm20);
        GrContextForegroundSet(&sContext, ClrSilver);
        GrStringDraw(&sContext, "S:", -1, 160, 55, 0);
}

void MotorLimitLow(void)
{

        //WidgetRemove((tWidget *)(&g_sMotorOff));
        WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sSliderLimitsLow);
        // PushButtonImageOn(&g_sSliderLimitsLow);
        // PushButtonFillOn(&g_sSliderLimitsLow);
        // PushButtonTextOn(&g_sSliderLimitsLow);
        // PushButtonTextOn(&g_sSliderLimitsLow);
        // PushButtonFillOn(&g_sSliderLimitsLow);
        WidgetPaint((tWidget *)&g_sSliderLimitsLow);
        GrContextFontSet(&sContext, &g_sFontCm20);
        GrContextForegroundSet(&sContext, ClrSilver);
        GrStringDraw(&sContext, "L:", -1, 160, 105, 0);
    
}

void MotorLimitHigh(void)
{

        // //WidgetRemove((tWidget *)(&g_sMotorOff));
        WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sSliderLimitsHigh);
        // PushButtonImageOn(&g_sSliderLimitsHigh);
        // PushButtonFillOn(&g_sSliderLimitsHigh);
        // PushButtonTextOn(&g_sSliderLimitsHigh);
        // PushButtonTextOn(&g_sSliderLimitsHigh);
        // PushButtonFillOn(&g_sSliderLimitsHigh);
        WidgetPaint((tWidget *)&g_sSliderLimitsHigh);
        GrContextFontSet(&sContext, &g_sFontCm20);
        GrContextForegroundSet(&sContext, ClrSilver);
        GrStringDraw(&sContext, "H:", -1, 160, 155, 0);
    
}


void MotorLimitAcc(void)
{
        //WidgetRemove((tWidget *)(&g_sMotorOff));
        WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sSliderAcc);
        // PushButtonImageOn(g_sSliderAcc);
        // PushButtonFillOn(g_sSliderAcc);
        // PushButtonTextOn(g_sSliderAcc);
        // PushButtonTextOn(g_sSliderAcc);
        // PushButtonFillOn(g_sSliderAcc);
        WidgetPaint((tWidget *)&g_sSliderAcc);
        GrContextFontSet(&sContext, &g_sFontCm20);
        GrContextForegroundSet(&sContext, ClrSilver);
        GrStringDraw(&sContext, "Acc Limit:", -1, 180, 80, 0);
    
}




// Function Handle for current read task
TaskHandle_t vCurrentReadHandle;
TaskHandle_t vRPMReadHandle;

//external structs
extern SemaphoreHandle_t xADC1_Semaphore;
extern MotorState motor_state;
extern MotorStateExternal motor_state_external;

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

// Sensor Functions
// Configuration for the OPT3001 sensor
static void prvConfigureOPT3001Sensor( void );
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
    
    //reset both our Estops
    motor_state.Estop_condition = Disabled;
    motor_state_external.Estop_condition = Disabled;

    //Enable the Hall Interrupts
    GPIOIntEnable(HALL_A_PORT, HALL_A_PIN);
    GPIOIntEnable(HALL_B_PORT, HALL_B_PIN);
    GPIOIntEnable(HALL_C_PORT, HALL_C_PIN);
    
    uint16_t duty_value = 10;
    uint16_t period_value = MOTOR_MAX_DUTY;
    int stateA, stateB, stateC;

    //reset the PID error
    motor_state.controller_ouput = 0;
    //reset the integral error
    motor_state.controller_error = 0;

    setMotorRPM(rpm);

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

/*
 * Called by main() to create the various queue tasks.
 */
void vQueueTask( void );
/*-----------------------------------------------------------*/


void vQueueTask( void )
{

    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
        SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
        SYSCTL_CFG_VCO_240), configCPU_CLOCK_HZ);

        /* Attempt to create the event group. */
    task_Event = xEventGroupCreate();

    /* Was the event group created successfully? */
    if(task_Event == NULL )
    {
        /* The event group was not created because there was insufficient
        FreeRTOS heap available. */
    }
    else
    {
        /* The event group was created. */
    }
    FPUEnable();
    FPULazyStackingEnable();

        //
    // Configure and enable uDMA
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlDelay(10);
    uDMAControlBaseSet(&psDMAControlTable[0]);
    uDMAEnable();


    TouchScreenInit(g_ui32SysClock);
    TouchScreenCallbackSet(WidgetPointerMessage);

    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sPrevious);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sTitle);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sNext);
    
    // WidgetRemove((tWidget *)(&g_sMotorOn));
    // WidgetRemove((tWidget *)(&g_sMotorOff));
    // WidgetRemove((tWidget *)(&g_sSliderSpeed));
    // WidgetRemove((tWidget *)(&g_sSliderLimitsLow));
    // WidgetRemove((tWidget *)(&g_sSliderLimitsHigh));
    // WidgetRemove((tWidget *)(&g_sSliderAcc));

    //g_ui32Panel = 0;
    //WidgetAdd(WIDGET_ROOT, (tWidget *)g_psPanels);
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[0]);


    WidgetPaint(WIDGET_ROOT);
//     //
//     // Initialize the graphics context.
//     //
     GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

        // Enable the GPIO port that is used for the on-board LEDs.
    //Timer periodic timer interrupt.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Enable the GPIO pins for the LEDs (PN0 & PN1).
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);


    //
    // Enable the peripherals used by this example.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Configure the two 32-bit periodic timers.
    //
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock/100);

    //
    // Setup the interrupts for the timer timeouts.
    //
    MAP_IntEnable(INT_TIMER1A);
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    //
    // Enable the timers.
    //
    MAP_TimerEnable(TIMER1_BASE, TIMER_A);




    xOptSemaphore = xSemaphoreCreateBinary();
    /* Create the queue used to send complete struct AMessage structures.  This can
    also be created after the schedule starts, but care must be task to ensure
    nothing uses the queue until after it has been created. */
    xStructQueue = xQueueCreate(
                          /* The number of items the queue can hold. */
                          mainQUEUE_LENGTH,
                          /* Size of each item is big enough to hold the
                          whole structure. */
                          sizeof( xMessage ) );

    /* Create the queue used to send pointers to struct AMessage structures. */
    xPointerQueue = xQueueCreate(
                          /* The number of items the queue can hold. */
                          mainQUEUE_LENGTH,
                          /* Size of each item is big enough to hold only a
                          pointer. */
                          sizeof( &xMessage ) );

    xOptDataQueue = xQueueCreate(
                          /* The number of items the queue can hold. */
                          mainDataQUEUE_LENGTH,
                          /* Size of each item is big enough to hold the
                          whole structure. */
                          sizeof( xData ) );

    xOptAverageQueue = xQueueCreate(
                          /* The number of items the queue can hold. */
                          mainDataQUEUE_LENGTH,
                          /* Size of each item is big enough to hold the
                          whole structure. */
                          sizeof( xData ) );


    if( ( xStructQueue == NULL ) || ( xPointerQueue == NULL ) || (xOptDataQueue == NULL) || (xOptAverageQueue == NULL) )
    {

    }

/*-----------------------------------------------------------------------------------*/
//MOTOR STUFF
    xRPMQueueInternal = xQueueCreate(1, sizeof( xRPMvalue ) );
    xRPMQueueExternal = xQueueCreate(1, sizeof( xRPMvalue ) );
    xCurrentQueueExternal = xQueueCreate(1, sizeof( xCurrentvalue ) );
    xPowerQueueExternal = xQueueCreate(1, sizeof( xPowervalue ) );
    if( ( xRPMQueueExternal == NULL ) || ( xRPMQueueInternal == NULL ) || ( xCurrentQueueExternal == NULL ) || ( xPowerQueueExternal == NULL ) )
    {
        UARTprintf("RPM Queue or Current Queue or Power Queue creation failed\n");
    }

    UARTprintf("Creating tasks\n");

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

    // Create queues for sensor data pub
    xOPT3001Queue = xQueueCreate(1, sizeof(struct OPT3001Message));
    xBMI160Queue = xQueueCreate(1, sizeof(struct BMI160Message));

    if (xOPT3001Queue == NULL || xBMI160Queue == NULL)
    {
        UARTprintf("Error creating Sensor Queues\n");
        return 0;
    }

    // Create semaphores for sensor tasks
    xI2CConfigSemaphore = xSemaphoreCreateBinary();
    xOPT3001ReadSemaphore = xSemaphoreCreateBinary();
    xBMI160ReadSemaphore = xSemaphoreCreateBinary();

    if (xI2CConfigSemaphore == NULL || xOPT3001ReadSemaphore == NULL || xBMI160ReadSemaphore == NULL)
    {
        UARTprintf("Error creating Sensor Semaphores\n");
        return 0;
    }

    // Sensor Tasks
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
    
    // This is just for testing reads on the queues
    // xTaskCreate( vQueueReadTest,
    //              "Queue Read Test",
    //              configMINIMAL_STACK_SIZE,
    //              NULL,
    //              tskIDLE_PRIORITY + 1,
    //              NULL );
//MOTOR STUFF END
/*-----------------------------------------------------------------------------------*/
   
   

   
//    xTaskCreate(prvOpt,
//                 "Opt",
//                 configMINIMAL_STACK_SIZE,
//                 NULL,
//                 mainQUEUE_RECEIVE_TASK_PRIORITY,
//                 NULL );

    xTaskCreate(prvDisplay,
                "Display",
                configMINIMAL_STACK_SIZE*3,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL );

 TimerEnable(TIMER5_BASE, TIMER_A);
}
/*-----------------------------------------------------------*/

uint32_t hour = 2;
uint32_t hour2 = 1;
uint32_t minut = 5;
uint32_t minut2 = 1;
uint32_t second = 0;
uint32_t second2 = 0;
uint32_t milis = 0;
void xTimer1IntHandler(void) {
    BaseType_t xOptTaskWoken;
    xOptTaskWoken = pdFALSE;
    milis++;
    // Clear the timer interrupt
    MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
 //uxQueueSpacesAvailable(xOptDataQueue) == 0  &&    
    xSemaphoreGiveFromISR( xOptSemaphore, &xOptTaskWoken );

}



void plotMotorSpeed(int speed){


        if(MotorData)
        {
            GrContextFontSet(&sContext, &g_sFontCm20);
            GrContextForegroundSet(&sContext, ClrWhite);
            GrStringDraw(&sContext, "Speed (rpm):", -1, 35, 25, 0);
            //MotorData = false;
            //Plot for speed
            sRect.i16XMin = 10;
            sRect.i16YMin = 50;
            sRect.i16XMax = 11;
            sRect.i16YMax = 180;
            GrContextForegroundSet(&sContext, ClrGreen);
            GrRectFill(&sContext, &sRect);
            GrContextForegroundSet(&sContext, ClrWhite);
            GrRectDraw(&sContext, &sRect);
            
            sRect.i16XMin = 10;
            sRect.i16YMin = 180;
            sRect.i16XMax = 150;
            sRect.i16YMax = 181;
            GrContextForegroundSet(&sContext, ClrGreen);
            GrRectFill(&sContext, &sRect);    
            GrContextForegroundSet(&sContext, ClrWhite);
            GrRectDraw(&sContext, &sRect);
        }

        //output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
        speed = 0 + (((speed - 0)*(200 - 0))/(4530-0));
        //speed = 0 + ((200 - 0)/(4530-0)) * (speed - 0);
        //power = 0 + ((200 - 0)/(15-0)) * (power - 0);
        //UARTprintf("speed %d power %d\n", speed, power);
        //UARTprintf("Data: %5d\n", data);
        if(speed <= 200 && speed >= 0)
        {
            
            GrCircleDraw(&sContext, 15+countspeed,178-speed,1);
            //UARTprintf("count speed %d\n", countspeed);
            countspeed = countspeed + 2;
            if((countspeed+15) >= 150)
            {
                sRect.i16XMin = 12;
                sRect.i16YMin = 50;
                sRect.i16XMax = 150;
                sRect.i16YMax = 180;
                GrContextForegroundSet(&sContext, ClrBlack);
                GrRectFill(&sContext, &sRect);  
                countspeed = 0;
                //UARTprintf("Speed forfra %d\n", countspeed);
            }
        }
}



void plotSensorLux(int lux){

        if(SensorData)
        {
            
            GrContextFontSet(&sContext, &g_sFontCm20);
            GrContextForegroundSet(&sContext, ClrWhite);
            GrStringDraw(&sContext, "Light (lux):", -1, 35, 25, 0);

            //SensorData = false;
            //Plot for speed
            sRect.i16XMin = 10;
            sRect.i16YMin = 50;
            sRect.i16XMax = 11;
            sRect.i16YMax = 180;
            GrContextForegroundSet(&sContext, ClrGreen);
            GrRectFill(&sContext, &sRect);
            GrContextForegroundSet(&sContext, ClrWhite);
            GrRectDraw(&sContext, &sRect);
            
            sRect.i16XMin = 10;
            sRect.i16YMin = 180;
            sRect.i16XMax = 150;
            sRect.i16YMax = 181;
            GrContextForegroundSet(&sContext, ClrGreen);
            GrRectFill(&sContext, &sRect);    
            GrContextForegroundSet(&sContext, ClrWhite);
            GrRectDraw(&sContext, &sRect);

        }

        lux = 0 + (((lux - 0)*(200 - 0))/(2000-0));
        //lux = 0 + ((200 - 0)/(2000-0)) * (speed - 0);
        //acc = 0 + ((200 - 0)/(2000-200)) * (power - 0);
        //UARTprintf("Data: %5d\n", data);
        if(lux <= 200 && lux >= 0)
        {
            
            GrCircleDraw(&sContext, 15 + countlux,178-lux,1);
            countlux = countlux + 2;
            if((countlux+15) >= 150)
            {
                countlux = 0;
                sRect.i16XMin = 12;
                sRect.i16YMin = 50;
                sRect.i16XMax = 150;
                sRect.i16YMax = 180;
                GrContextForegroundSet(&sContext, ClrBlack);
                GrRectFill(&sContext, &sRect);  
            }
        }
}


void plotMotorPower(int power){


        if(MotorData)
        {
            GrContextFontSet(&sContext, &g_sFontCm20);
            GrContextForegroundSet(&sContext, ClrWhite);
            GrStringDraw(&sContext, "Power (watts):", -1, 180, 25, 0);
            MotorData = false;

            //PLot for power
            sRect.i16XMin = 160;
            sRect.i16YMin = 50;
            sRect.i16XMax = 161;
            sRect.i16YMax = 180;
            GrContextForegroundSet(&sContext, ClrGreen);
            GrRectFill(&sContext, &sRect);
            GrContextForegroundSet(&sContext, ClrWhite);
            GrRectDraw(&sContext, &sRect);
            
            sRect.i16XMin = 160;
            sRect.i16YMin = 180;
            sRect.i16XMax = 299;
            sRect.i16YMax = 181;
            GrContextForegroundSet(&sContext, ClrGreen);
            GrRectFill(&sContext, &sRect);    
            GrContextForegroundSet(&sContext, ClrWhite);
            GrRectDraw(&sContext, &sRect);
        }

        //output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
        power = 0 + (((power - 0)*(200 - 0))/(15-0));
        if(power <= 200 && power >= 0)
        {
            
            GrCircleDraw(&sContext, 165+countpower,178-power,1);
            countpower = countpower + 2;
            if((countpower+15) >= 150)
            {
                sRect.i16XMin = 163;
                sRect.i16YMin = 50;
                sRect.i16XMax = 300;
                sRect.i16YMax = 180;
                GrContextForegroundSet(&sContext, ClrBlack);
                GrRectFill(&sContext, &sRect);
                countpower = 0;  
                //UARTprintf("power forfra %d\n", countpower);
            }
        }
}



void plotSensorAcc(int acc){

        if(SensorData)
        {
            
            GrContextFontSet(&sContext, &g_sFontCm20);
            GrContextForegroundSet(&sContext, ClrWhite);
            GrStringDraw(&sContext, "Acc (whatever):", -1, 180, 25, 0);
            SensorData = false;

            //PLot for Acc
            sRect.i16XMin = 160;
            sRect.i16YMin = 50;
            sRect.i16XMax = 161;
            sRect.i16YMax = 180;
            GrContextForegroundSet(&sContext, ClrGreen);
            GrRectFill(&sContext, &sRect);
            GrContextForegroundSet(&sContext, ClrWhite);
            GrRectDraw(&sContext, &sRect);
            
            sRect.i16XMin = 160;
            sRect.i16YMin = 180;
            sRect.i16XMax = 299;
            sRect.i16YMax = 181;
            GrContextForegroundSet(&sContext, ClrGreen);
            GrRectFill(&sContext, &sRect);    
            GrContextForegroundSet(&sContext, ClrWhite);
            GrRectDraw(&sContext, &sRect);
        }

        acc = 0 + (((acc - 0)*(200 - 0))/(2000-250));

        //lux = 0 + ((200 - 0)/(2000-0)) * (speed - 0);
        //acc = 0 + ((200 - 0)/(2000-200)) * (power - 0);
        //UARTprintf("Data: %5d\n", data);

        if(acc <= 200 && acc >= 0)
        {
            GrCircleDraw(&sContext, 165 + countacc,178-acc,1);
            countacc = countacc + 2;
            if((countacc+15) >= 150)
            {
                countacc = 0;
                sRect.i16XMin = 163;
                sRect.i16YMin = 50;
                sRect.i16XMax = 300;
                sRect.i16YMax = 180;
                GrContextForegroundSet(&sContext, ClrBlack);
                GrRectFill(&sContext, &sRect);  

            }
        }
}


void plotNight(void){
    sRect.i16XMin = 0;
    sRect.i16YMin = 24;
    sRect.i16XMax = 160;
    sRect.i16YMax = 190;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);


    GrContextForegroundSet(&sContext, ClrWhite);
    GrCircleFill(&sContext, 80,107,60);

    GrContextForegroundSet(&sContext, ClrBlack);
    GrCircleFill(&sContext, 100,107,60);


}

void plotDay(void){
    sRect.i16XMin = 0;
    sRect.i16YMin = 24;
    sRect.i16XMax = 160;
    sRect.i16YMax = 190;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);

    GrContextForegroundSet(&sContext, ClrYellow);
    GrCircleFill(&sContext, 80,107,60);




    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 80, 30 );

    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 80, 184 );

    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 157, 107 );

    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 3, 107 );

    //Set 1
    //K
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 100, 181 );
    //L
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 119, 173 );
    //M
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 160, 119 );
    //N
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 153, 133 );
    //O
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 144, 149 );
    //P
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 134, 162 );

    //Set 2
    //K
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 65, 32 );
    //L
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 46, 38 );
    //M
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 4, 93 );
    //N
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 9, 77 );
    //O
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 18, 61 );
    //P
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 30, 48 );

    //Set 3
    //K
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 4, 121 );
    //L
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 10, 139 );
    //M
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 63, 182 );
    //N
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 46, 176 );
    //O
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 31, 166 );
    //P
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 19, 154 );

    //Set 4
    //K
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 155, 91 );
    //L
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 151, 76 );
    //M
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 96, 32 );
    //N
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 116, 39 );
    //O
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 132, 50 );
    //P
    GrContextForegroundSet(&sContext, ClrYellow);
    GrLineDraw(&sContext, 80, 107, 143, 62 );
}


static void prvDisplay( void *pvParameters )
{       


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }

    //
    // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // Like pinMode
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);


    RPMQueueData xRPMvalueRecv;
    PowerQueueData xPowervalueRecv;
    struct BMI160Message xBMI160Message;
    struct OPT3001Message xOPT3001Message;

    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 23;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);
    char str[35];
    usprintf(str, "Clock:         ");
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, str, -1,
                        GrContextDpyWidthGet(&sContext) / 2, 9, 0);

    GrContextForegroundSet(&sContext, ClrWhite);
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDraw(&sContext, "30/05/24", -1, 240, 2, 0);
    OnIntroPaint();
    bool clockSec = 1;
    struct AData xReadSpeed;
    struct AData xReadPower;
    for (;;)
    {   
        WidgetMessageQueueProcess();
             if(milis >= 100){
            milis = 0;
            second = second + 1;
            clockSec = 1;
            if(second >= 10)
            {
                second2 = second2 + 1;
                second = 0;
            }
            
            if(second2 >= 6)
            {      
                second2 = 0;
                minut = minut + 1;
                if(minut >=  10)
                {
                    minut2 = minut2 + 1;
                    minut = 0;
                }
                if(minut2 >= 6)
                {
                    minut2 = 0;
                    hour = hour + 1;
                    if(hour >= 10)
                    {
                        hour = 0;
                        hour2 = hour + 1;
                    }
                    if(hour2 > 2)
                    {
                        hour2 = 0;
                    }                
                }
            }
        }
        
        if(clockSec)
        {
        clockSec = 0;
        sRect.i16XMin = 160;
        sRect.i16YMin = 2;
        sRect.i16XMax = 230;
        sRect.i16YMax = 20;
        GrContextForegroundSet(&sContext, ClrDarkBlue);
        GrRectFill(&sContext, &sRect);
        char str[35];
        usprintf(str,"%d%d:%d%d:%d%d", hour2, hour, minut2, minut,second2, second);
        GrContextForegroundSet(&sContext, ClrWhite);
        GrContextFontSet(&sContext, &g_sFontCm20);
            GrStringDraw(&sContext, str, -1, 150,
                 2, 0);
    
        }

        //RPM and Power plotting 
        if((xQueueReceive(xPowerQueueExternal, &(xPowervalueRecv), ( TickType_t ) 0) == pdPASS))       
        {
            //UARTprintf("power: %u\n", xPowervalueRecv.value/1000);
            //UARTprintf("RPM: %u\n", xRPMvalueRecv.value);
            // UARTprintf("power: %u\n", xPowervalueRecv.value);
            if(((xPowervalueRecv.value/1000 <= limitLow) || (xPowervalueRecv.value/1000 >= limitHigh)) && motorState)
            {
                //UARTprintf("power: %u\n", xPowervalueRecv.value/1000);
                 EXTERNAL_SET_ESTOP();
            } 

            if(MotorPlot && motorState)
            {
                //UARTprintf("power: %u\n", xPowervalueRecv.value/1000);
                //UARTprintf("RPM: %u\n", xRPMvalueRecv.value);
                plotMotor(xPowervalueRecv.value/1000);

            }
        }
        if((xQueueReceive(xRPMQueueExternal, &(xRPMvalueRecv), 0) == pdPASS))       
        {
            if(MotorPlot && motorState)
            {
                //UARTprintf("power: %u\n", xPowervalueRecv.value/1000);
                //UARTprintf("RPM: %u\n", xRPMvalueRecv.value);
                plotMotorSpeed(xRPMvalueRecv.value);

            }
        }


        //Checking opt and acc
        if((xQueueReceive( xOPT3001Queue, &( xOPT3001Message ), ( TickType_t ) 10 ) == pdPASS) || (xQueueReceive( xBMI160Queue, &( xBMI160Message), ( TickType_t ) 10 ) == pdPASS))
        {
            //UARTprintf(">Lux:%d\n", xOPT3001Message.filteredLux);
            //UARTprintf(">Acc:%d\n", xBMI160Message.ulfilteredAccel);

            if(xOPT3001Message.filteredLux <= 5)
            {
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
            }else{
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x0);
            }


            if(SensorPlot)
            {
                plotSensorLux(xOPT3001Message.filteredLux);
            }
            if(ligthData)
            {
                if(xOPT3001Message.filteredLux <=  5 && dayChange)
                {
                    dayChange = 0;
                    nightChange = 1;
                    plotNight();
                }
                if(xOPT3001Message.filteredLux >  5 && nightChange)
                {
                    nightChange = 0;
                    dayChange = 1;
                    plotDay();
                }

            }
        }
        if((xQueueReceive( xBMI160Queue, &( xBMI160Message), ( TickType_t ) 10 ) == pdPASS))
        {
            //UARTprintf(">Lux:%d\n", xOPT3001Message.filteredLux);
            //UARTprintf(">Acc:%d\n", xBMI160Message.ulfilteredAccel);

            if(SensorPlot)
            {
                plotSensorAcc(xBMI160Message.ulfilteredAccel);
            }
        }
    }
}


/*-----------------------------------------------------------*/
// Motor State Machine
static void prvMotorTask( void *pvParameters )
{
    // Testing states
    //####################### FROM HERE TO ----- ##############################
    // motorStart(500);

    // vTaskDelay(pdMS_TO_TICKS( 10000 ));
    // UARTprintf("Stop\n");
    // motorStop(false);

    // vTaskDelay(pdMS_TO_TICKS( 10000 ));
    // motorStart(3500);

    // vTaskDelay(pdMS_TO_TICKS( 10000 ));
    // motor_state.Estop_condition = Enabled;


    //######################## ---- HERE IS TESTED  ######################################## 
    //####################### BELOW STATE MACHINE IS NOT TESTED ############################

    //
    // TODO: Test State Machine
    // Uncommented tested zone above and set if() to true.
    // USE EXTERNAL_SET_RPM and EXTERNAL_SET_ESTOP only to test state machine. all state should be handled internally.

    for (;;)
    {
        //UARTprintf("STATE: %d \n", motor_state.current_state);

        vTaskDelay(pdMS_TO_TICKS(500));
        if (1){
            //switch on the internal state. 
            // check conditions on external API state.
            switch(motor_state.current_state){
                case Idle:
                    if ( motor_state_external.set_rpm > 0){
                        //set the internal state to our external setpoint 
                        motorStart(motor_state_external.set_rpm);
                        // go to starting state
                        motor_state.current_state = Starting;
                    }
                    if (motor_state_external.set_rpm == 0){
                        //soft brake to 0 rpm
                        motorStop(false);
                    }
                    break;

                case Starting:
                    // if we hit Estop during start go to EStop
                    if (motor_state_external.Estop_condition == Enabled){
                        setMotorEstop();
                        motor_state.current_state = EStop;
                    }
                    // if our external API set point is within 10 percent of the internal rpm go to running
                    if (motor_state_external.set_rpm > motor_state.current_rpm *0.9 ||  motor_state_external.set_rpm < motor_state.current_rpm * 1.1  ){
                        motor_state.current_state = Running;
                    }
                    break;

                case Running:
                    // we are already at set RPM but Estop is true
                    if (motor_state_external.Estop_condition == Enabled){
                        setMotorEstop();
                        motor_state.current_state = EStop;
                    }
                    // we have set current RPM to 0 go IDLE
                    if (motor_state.current_rpm == 0){
                        motorStop(false);
                        motor_state.current_state = Idle;
                    }else{
                        //we are free to set the RPM with external API
                        motor_state.set_rpm = motor_state_external.set_rpm;
                    }

                    break;
                case EStop:
                    //wait until we have safely stoppped. then go to IDLE
                    if (motor_state.current_rpm == 0){
                        motor_state.current_state = Idle;
                    }     
                    break;       
                }
        }
    }
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
        //UARTprintf("power2: %d \n", power/1000);
        // Send the current and power value to the queue
        xCurrentvalueSend.value = current;
        xPowervalueSend.value = power;
        xQueueSend(xCurrentQueueExternal, &xCurrentvalueSend, 0);
        if(power/1000 <= 15)
        {
            xQueueSend(xPowerQueueExternal, ( void * ) &xPowervalueSend, ( TickType_t ) 0);
            //UARTprintf(">Power%d\n", power);
        }
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
        }else{
            // we didnt get any data in the queue, must be stopped.
            //
            // TODO: TEST THIS
            //
            rpm_array[index] = 0;
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
        //UARTprintf(">RPM%d\n", rpm_filtered);
        // Delay for 100Hz
        vTaskDelay(pdMS_TO_TICKS( 10 ));
    }
}

// void vQueueReadTest( void *pvParameters )
// {
//     // Read from all sensor queues and print the values
//     RPMQueueData xRPMvalueRecv;
//     CurrentQueueData xCurrentvalueRecv;
//     PowerQueueData xPowervalueRecv;
//     OPT3001Message xOPT3001MessageRecv;
//     BMI160Message xBMI160MessageRecv;
//     int print_idx = 0;
//     for( ;; )
//     {
//         // Test one by one
//         // // Read the RPM value from the queue
//         // if (xQueueReceive(xRPMQueueExternal, &xRPMvalueRecv, 0) == pdPASS) 
//         // {
//         //     // Print the values
//         //     UARTprintf("RPM: %u\n", xRPMvalueRecv.value);
//         // }

//         // // // Read the Current value from the queue
//         // // if (xQueueReceive(xCurrentQueueExternal, &xCurrentvalueRecv, 0) == pdPASS) {
//         // //     // Print the values
//         // //     UARTprintf("Current: %d\n", xCurrentvalueRecv.value);
           
//         // // }

//         // // // // Read the Power value from the queue
//         if (xQueueReceive(xPowerQueueExternal, &xPowervalueRecv, 0) == pdPASS) {
//             // Print the values
//             UARTprintf("Power: %u\n", xPowervalueRecv.value);
//         }

//         // // Read the OPT3001 value from the queue
//         // if (xQueueReceive(xOPT3001Queue, &xOPT3001MessageRecv, 0) == pdPASS) {
//         //     // Print the values
//         //     UARTprintf("Lux: %u\n", xOPT3001MessageRecv.ulfilteredLux);
//         // }

//         // // Read the BMI160 value from the queue
//         // if (xQueueReceive(xBMI160Queue, &xBMI160MessageRecv, 0) == pdPASS) 
//         // {
//         //     // Print the values
//         //     UARTprintf("Accel: %d\n", xBMI160MessageRecv.ulfilteredAccel);
//         // }
//         //vTaskDelay(pdMS_TO_TICKS( 3000 ));
        
//     }
// }

/*-----------------------------------------------------------*/
/*                  Aux Sensor Functions                     */

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
        writeI2CBMI(0x69, 0x7E, &normal_accl_mode);                 // Write to Command Register
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

            if(success)
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
                //UARTprintf(">Accel:%d\n", filtered_accel);
            }
        }
    }
}

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
                //UARTprintf(">Lux:%d\n", filtered_lux);
            }
        }
    }
}

/* Interrupt handlers */

void Timer4IntHandler(void) {
    //
    // TODO: Test the Acceleration and Deceleration
    //

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

        //set our accelertion limits
        // every 10ms the maximum adjust is 0.05 duty cycle. we have 100 rpm per duty value. therefore 1s/10ms = 100.
        // then (0.05 duty change) * 100 (timer ticks in 1 second) = 5duty change per second
        // we have 100 rpm per duty spo 5 duty change per second * 100 = [[[ 500 rpm/s ]]] 
        if (pid_duty > 0.05){
            pid_duty = 0.05;
        }
        else if (pid_duty < -0.05){
            pid_duty = -0.05;
        }

        //add our adjusted duty to the controller 
        motor_state.controller_ouput += pid_duty; 

        // Ensure controller output is within valid range
        if (motor_state.controller_ouput < 1) {
            motor_state.controller_ouput = 1;
        } else if (motor_state.controller_ouput > 45) {
            motor_state.controller_ouput = 45;
        }

        // Apply the output as a new duty cycle
        //convert the float to uint16_t and write to the motor
        motor_state.duty_value = (uint16_t)motor_state.controller_ouput;

        setDuty(motor_state.duty_value);
    }

    // Handle Estop condition
    // similar calculation as above except we want 1000rpm/s so double the max duty change.
    // this will be constant acceleration.
    if (motor_state.Estop_condition == Enabled) {

        if (motor_state.duty_value > 1){
            motor_state.duty_value -= 0.1;
            setDuty(motor_state.duty_value);
        }
        else{
            motorStop(true);
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
        // Start the timer on the first pulse
        //MotorRPMTimerStart();
        startTime = xTaskGetTickCount();
    } else if (pulseCount >= 6*num_pole_pairs+1) {
        // Stop the timer on the seventh pulse
        //MotorRPMTimerStop();
        endTime = xTaskGetTickCount();
        pulseCount = 0; // Reset pulse count for the next rotation

        //uint32_t elapsedTime = startTime - endTime; // Assuming the timer counts down
        TickType_t elapsedTime = endTime - startTime;
        uint32_t elapsedTimeInMilliseconds = elapsedTime * portTICK_PERIOD_MS;

        // 0.7472 -> motor rpm correction factor
        uint32_t rpm = 0.7472 * (60000 / (elapsedTimeInMilliseconds)); 

        motor_state.current_rpm = rpm; // This updates our global value.

        xRPMvalue.value = rpm;
        xRPMvalue.ticks = elapsedTime;
        if (xQueueSend(xRPMQueueInternal, &xRPMvalue, 0) != pdPASS) {
            // Handle error: Queue is full
        }

    }

    // Read hall effect sensors
    stateA = GPIOPinRead(HALL_A_PORT, HALL_A_PIN) ? 1 : 0;
    stateB = GPIOPinRead(HALL_B_PORT, HALL_B_PIN) ? 1 : 0;
    stateC = GPIOPinRead(HALL_C_PORT, HALL_C_PIN) ? 1 : 0;

    updateMotor(stateA, stateB, stateC);
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
    motor_state.set_rpm = rpm;
}

void setMotorEstop(){
    motor_state.Estop_condition = Enabled;
    motor_state.set_rpm = 10;
}

/*########################EXTERNAL MOTOR CONTROL API##############################################################*/

void EXTERNAL_SET_RPM(uint16_t set_rpm){
    //
    //TODO: ADD MUTEX
    // 
    if (set_rpm > 4500){
        set_rpm = 4500;
    }
    if (set_rpm < 0){
        set_rpm = 0;
    }
    motor_state_external.set_rpm = set_rpm;
}

void EXTERNAL_SET_ESTOP(){
    //
    //TODO: ADD MUTEX
    // 
    motor_state_external.Estop_condition = Enabled;
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

void xTimer6AHandler( void )
{
    /* Clear the hardware interrupt flag for Timer 6A. */
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

void xI2C2Handler( void )
{
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
        I2CMasterInitExpClk(I2C2_BASE, 120000000, false);

        IntEnable(INT_I2C2);
        I2CMasterIntEnable(I2C2_BASE);

        // // Enable Master interrupt
        IntMasterEnable();

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

void vApplicationTickHook( void )
{
}

