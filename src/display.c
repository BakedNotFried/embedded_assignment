/*
 * queue_task
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
 * This example demonstrates passing a complex structure by value as well as
 * by reference using FreeRTOS's queue between multiple tasks.  Two queues are
 * set up with one for passing messages by value and another for passing by
 * reference.  A total of six tasks are created with four of them sending
 * messages through the two queues while the other two tasks receiving the
 * message from the two queues either by value or by reference.  The four tasks
 * transmit their messages at different times.  Their time stamp is sent as
 * the data as part of the message.  Once the data is received, the receiving
 * tasks print the task ID and its corresponding time stamp on the terminal
 * window.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include"event_groups.h"

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
int count = 0;
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
static void prvOpt( void *pvParameters );
static void prvDisplay( void *pvParameters );


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


    // CircularButton(g_psPanels + 1, g_psPushButtons + 1, 0,
    //                      &g_sKentec320x240x16_SSD2119, 160, 120, 50,
    //                      PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
    //                      ClrGreen, ClrBlack, ClrGray, ClrSilver,
    //                      &g_sFontCm22, "ON", 0, 0, 0, 0, OnButtonPress);
    // CircularButton(g_psPanels + 1, g_psPushButtons + 2, 0,
    //                     &g_sKentec320x240x16_SSD2119, 160, 120, 50,
    //                     PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
    //                     ClrRed, ClrBlack, ClrGray, ClrSilver,
    //                     &g_sFontCm22, "OFF", 0, 0, 0, 0, OnButtonPress);

    CircularButton(g_sMotorOn, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 80, 120,
                  60,PB_STYLE_IMG | PB_STYLE_TEXT,
                  ClrLimeGreen, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm20b, "ON", 0, 0, 0, 0, OnButtonPress);

    CircularButton(g_sMotorOff, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 80, 120,
                  60,PB_STYLE_IMG | PB_STYLE_TEXT,
                  ClrRed, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm20b, "OFF", 0, 0, 0, 0, OnButtonPress);

    Slider(g_sSliderSpeed, 0, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 180, 50, 110, 30, 0, 100, 25,
                  (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "25", 0, 0, OnSliderSpeed);

    Slider(g_sSliderLimitsLow, 0, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 180, 100, 110, 30, 0, 100, 25,
                  (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "25", 0, 0, OnSliderLimitLow);
                
    Slider(g_sSliderLimitsHigh, 0, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 180, 150, 110, 30, 0, 100, 75,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "75", 0, 0, OnSliderLimitHigh);

    Slider(g_sSliderAcc, 0, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 180, 100, 110, 30, 0, 100, 100,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "100", 0, 0, OnSliderLimitAcc);

    // Slider(g_sSliderAcc, 0, 0, 0,
    //              &g_sKentec320x240x16_SSD2119, 180, 100, 110, 30, 0, 100, 100,
    //               (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
    //               SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
    //              ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
    //              &g_sFontCm20, "100", 0, 0, OnSliderLimitAcc);


    // CircularButton(g_sSliderAcc, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 80, 120,
    //               60,PB_STYLE_IMG | PB_STYLE_TEXT,
    //               ClrRed, ClrBlack, ClrGray, ClrBlack,
    //               &g_sFontCm20b, "OFF", 0, 0, 0, 0, OnSliderLimitAcc);
    // SliderStruct(g_psPanels + 7, g_psSliders + 1, 0,
    //              &g_sKentec320x240x16_SSD2119, 5, 115, 220, 30, 0, 100, 25,
    //              (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
    //               SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
    //              ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
    //              &g_sFontCm20, "25", 0, 0, OnSliderChange),
// tPushButtonWidget g_psPushButtons[] =
// {
//         CircularButton(g_sMotorOn, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 160, 120,
//                   60,PB_STYLE_IMG | PB_STYLE_TEXT,
//                   ClrLimeGreen, ClrBlack, ClrGray, ClrBlack,
//                   &g_sFontCm20b, "ON", 0, 0, 0, 0, OnButtonPress),
//     CircularButton(g_sMotorOff, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 160, 120,
//                   60,PB_STYLE_IMG | PB_STYLE_TEXT,
//                   ClrRed, ClrBlack, ClrGray, ClrBlack,
//                   &g_sFontCm20b, "OFF", 0, 0, 0, 0, OnButtonPress),
// };

// tCanvasWidget g_psPanels[] =
//     {
//         CanvasStruct(0, 0, &g_sIntroduction, &g_sKentec320x240x16_SSD2119, 0, 24,
//                      320, 166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
//         CanvasStruct(0, 0, g_psPushButtons, &g_sKentec320x240x16_SSD2119, 0, 24,
//                     320, 166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
// };


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



void
OnButtonPress(tWidget *psWidget)
{
    //WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sMotorOn);
    //WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sMotorOff);
    motorState = !motorState;
    if(motorState)
    {
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


void
OnSliderSpeed(tWidget *psWidget, int32_t i32Value)
{
    //static char pcCanvasText[5];
    static char pcSliderText[5];



        // usprintf(pcCanvasText, "%3d%%", i32Value);
        // CanvasTextSet(&g_sSliderValueCanvas, pcCanvasText);
        // WidgetPaint((tWidget *)&g_sSliderValueCanvas);


        usprintf(pcSliderText, "%3d", i32Value);
        SliderTextSet(&g_sSliderSpeed, pcSliderText);
        WidgetPaint((tWidget *)&g_sSliderSpeed);

}
int32_t limitLow = 25;
int32_t limitHigh = 75;
void
OnSliderLimitLow(tWidget *psWidget, int32_t i32Value)
{
    //static char pcCanvasText[5];
    static char pcSliderText[5];
    


        // usprintf(pcCanvasText, "%3d%%", i32Value);
        // CanvasTextSet(&g_sSliderValueCanvas, pcCanvasText);
        // WidgetPaint((tWidget *)&g_sSliderValueCanvas);

    if(i32Value <= limitHigh - 10)
    {
        limitLow = i32Value;
        usprintf(pcSliderText, "%3d", i32Value);
        SliderTextSet(&g_sSliderLimitsLow, pcSliderText);
        WidgetPaint((tWidget *)&g_sSliderLimitsLow);
    }

}

void
OnSliderLimitHigh(tWidget *psWidget, int32_t i32Value)
{
    //static char pcCanvasText[5];
    static char pcSliderText[5];
    


        // usprintf(pcCanvasText, "%3d%%", i32Value);
        // CanvasTextSet(&g_sSliderValueCanvas, pcCanvasText);
        // WidgetPaint((tWidget *)&g_sSliderValueCanvas);


    if(limitLow + 10 <= i32Value)
    {
        limitHigh = i32Value;
        usprintf(pcSliderText, "%3d", i32Value);
        SliderTextSet(&g_sSliderLimitsHigh, pcSliderText);
        WidgetPaint((tWidget *)&g_sSliderLimitsHigh);
    }
}



void
OnSliderLimitAcc(tWidget *psWidget, int32_t i32Value)
{
    //static char pcCanvasText[5];
    static char pcSliderText[5];
    


        // usprintf(pcCanvasText, "%3d%%", i32Value);
        // CanvasTextSet(&g_sSliderValueCanvas, pcCanvasText);
        // WidgetPaint((tWidget *)&g_sSliderValueCanvas);


        usprintf(pcSliderText, "%3d", i32Value);
        SliderTextSet(&g_sSliderAcc, pcSliderText);
        WidgetPaint((tWidget *)&g_sSliderAcc);

}


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
            count = 0;
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
            count = 0;
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
            count = 0;
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
            count = 0;
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



    xTaskCreate(prvOpt,
                "Opt",
                configMINIMAL_STACK_SIZE,
                NULL,
                mainQUEUE_RECEIVE_TASK_PRIORITY,
                NULL );

    xTaskCreate(prvDisplay,
                "Display",
                configMINIMAL_STACK_SIZE*3,
                NULL,
                mainQUEUE_RECEIVE_TASK_PRIORITY+1,
                NULL );

}
/*-----------------------------------------------------------*/

uint32_t hour = 6;
uint32_t hour2 = 1;
uint32_t minut = 9;
uint32_t minut2 = 5;
uint32_t second = 0;
uint32_t second2 = 5;
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



static void prvOpt( void *pvParameters )
{
    bool      success;
    uint16_t  rawData = 0;
    float     convertedLux = 0;
    struct AData xDataPoints;
    float     AverageLux = 0;
    struct AData xAveragePoints;

    for (;;)
    {
         
        if(xSemaphoreTake(xOptSemaphore, portMAX_DELAY) == pdPASS)
        {
            /* Create a 1 second delay */
            //vTaskDelay( pdMS_TO_TICKS( 100 ));

            //SysCtlDelay(g_ui32SysClock/100);

            //Read and convert OPT values
            success = sensorOpt3001Read(&rawData);

            if (success) {
                sensorOpt3001Convert(rawData, &convertedLux);
                movingAverageFilter(&convertedLux, &AverageLux);
                int Averagelux_int = (int)AverageLux;
                xAveragePoints.ulSamples = Averagelux_int;
                xQueueSend(xOptAverageQueue, ( void * ) &xAveragePoints, ( TickType_t ) 0 );
                xEventGroupSetBits(task_Event, avg_task_id);
                // Construct Text
                // sprintf(tempStr, "Lux: %5.2f\n", convertedLux);
                int lux_int = (int)convertedLux;
                //UARTprintf("Lux: %5d\n", lux_int);
                xDataPoints.ulSamples = lux_int;
                xQueueSend(xOptDataQueue, ( void * ) &xDataPoints, ( TickType_t ) 0 );
                xEventGroupSetBits(task_Event, raw_task_id);
            }    
        }
    }
}
float sensorArray[maxSamples] = {0};

void movingAverageFilter(float *newReading, float *average)
{
    static uint8_t newIndex = 0;

    sensorArray[newIndex] = *newReading;
    uint8_t invalidCount = 0;
    float sum = 0;
    for (int i = 0; i < maxSamples; i++)
    {
        if (sensorArray[i] == 0)
        {
            invalidCount++;
        }
        else
        {
            sum += sensorArray[i];
        }
    }
    *average = sum / (maxSamples - invalidCount);
    newIndex++;
    newIndex = newIndex % maxSamples;
}




void plotMotor(int speed, int power){


        if(MotorData)
        {
            GrContextFontSet(&sContext, &g_sFontCm20);
            GrContextForegroundSet(&sContext, ClrWhite);
            GrStringDraw(&sContext, "Speed (rpm):", -1, 35, 25, 0);

            GrContextForegroundSet(&sContext, ClrWhite);
            GrStringDraw(&sContext, "Power (watts):", -1, 180, 25, 0);
            MotorData = false;
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

        speed = 0 + (((speed - 0)*(200 - 0))/(2000-0));
        power = 0 + (((power - 0)*(200 - 0))/(2000-0));
        //UARTprintf("Data: %5d\n", data);
        if(speed <= 200)
        {
            
            GrCircleDraw(&sContext, 15 + count,180-speed,1);
            GrCircleDraw(&sContext, 165 + count,180-power,1);
            count = count + 2;
            if((count+15) >= 150)
            {
                count = 0;
                sRect.i16XMin = 12;
                sRect.i16YMin = 50;
                sRect.i16XMax = 150;
                sRect.i16YMax = 180;
                GrContextForegroundSet(&sContext, ClrBlack);
                GrRectFill(&sContext, &sRect);  

                sRect.i16XMin = 163;
                sRect.i16YMin = 50;
                sRect.i16XMax = 300;
                sRect.i16YMax = 180;
                GrContextForegroundSet(&sContext, ClrBlack);
                GrRectFill(&sContext, &sRect);  

            }
        }
}



void plotSensor(int lux, int acc){

        if(SensorData)
        {
            
            GrContextFontSet(&sContext, &g_sFontCm20);
            GrContextForegroundSet(&sContext, ClrWhite);
            GrStringDraw(&sContext, "Light (lux):", -1, 35, 25, 0);

            GrContextForegroundSet(&sContext, ClrWhite);
            GrStringDraw(&sContext, "Acc (whatever):", -1, 180, 25, 0);
            SensorData = false;
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

        lux = 0 + (((lux - 0)*(200 - 0))/(2000-0));
        acc = 0 + (((acc - 0)*(200 - 0))/(2000-0));
        //UARTprintf("Data: %5d\n", data);
        if(lux <= 200)
        {
            
            GrCircleDraw(&sContext, 15 + count,180-lux,1);
            GrCircleDraw(&sContext, 165 + count,180-acc,1);
            count = count + 2;
            if((count+15) >= 150)
            {
                count = 0;
                sRect.i16XMin = 12;
                sRect.i16YMin = 50;
                sRect.i16XMax = 150;
                sRect.i16YMax = 180;
                GrContextForegroundSet(&sContext, ClrBlack);
                GrRectFill(&sContext, &sRect);  

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
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 23;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);
    char str[35];
    usprintf(str, "Clock:     ");
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, str, -1,
                        GrContextDpyWidthGet(&sContext) / 2, 8, 0);
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
        sRect.i16XMax = 240;
        sRect.i16YMax = 20;
        GrContextForegroundSet(&sContext, ClrDarkBlue);
        GrRectFill(&sContext, &sRect);
        char str[35];
        usprintf(str,"%d%d:%d%d:%d%d", hour2, hour, minut2, minut,second2, second);
        GrContextForegroundSet(&sContext, ClrWhite);
        GrContextFontSet(&sContext, &g_sFontCm20);
            GrStringDraw(&sContext, str, -1, 170,
                 2, 0);
    
        }

        if(MotorPlot)
        {
            if((xQueueReceive( xOptAverageQueue, &( xReadSpeed ), ( TickType_t ) 10 ) == pdPASS) || (xQueueReceive( xOptDataQueue, &( xReadPower ), ( TickType_t ) 10 ) == pdPASS))
            {
                plotMotor(xReadSpeed.ulSamples, xReadPower.ulSamples);
            }
        }

        if(SensorPlot)
        {
            if((xQueueReceive( xOptAverageQueue, &( xReadSpeed ), ( TickType_t ) 10 ) == pdPASS) || (xQueueReceive( xOptDataQueue, &( xReadPower ), ( TickType_t ) 10 ) == pdPASS))
            {
                plotSensor(xReadSpeed.ulSamples, xReadPower.ulSamples);
            }
        }

        if(ligthData)
        {
            //int oldVal = 0;
            if((xQueueReceive( xOptAverageQueue, &( xReadSpeed ), ( TickType_t ) 10 ) == pdPASS))
            {

                if(xReadSpeed.ulSamples <=  5 && dayChange)
                {
                    dayChange = 0;
                    nightChange = 1;
                    plotNight();
                }
                if(xReadSpeed.ulSamples >  5 && nightChange)
                {
                    nightChange = 0;
                    dayChange = 1;
                    plotDay();
                }

            }
        }
      
    }
}



void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */

    /* Only the full demo uses the tick hook so there is no code is
        executed here. */
}


