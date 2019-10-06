#include "api_hal_gpio.h"
#include "stdint.h"
#include "stdbool.h"
#include "api_debug.h"
#include "api_os.h"
#include "api_hal_pm.h"
#include "api_os.h"
#include "api_event.h"
#include "math.h"
//GPS
#include <api_gps.h>
#include "gps_parse.h"
#include "gps.h"
//GPRS
//MQTT


#define MAIN_TASK_STACK_SIZE    (1024 * 2)
#define MAIN_TASK_PRIORITY      0 
#define MAIN_TASK_NAME         "MAIN Test Task"

#define LED28_BLINK_TASK_STACK_SIZE    (1024 * 2)
#define LED28_BLINK_TASK_PRIORITY      1
#define LED28_BLINK_TASK_NAME         "LED28 Blink Task"

#define GPS_TASK_STACK_SIZE     (1024 * 2)
#define GPS_TASK_PRIORITY       2
#define GPS_TASK_NAME          "GPS Task"

#define TRACE_PREFIX           "smartmotor0.15::"

static HANDLE mainTaskHandle = NULL;
static HANDLE led28BlinkTaskHandle = NULL;
static HANDLE gpsTaskHandle  = NULL;

///////////////////////
// Helper definition //
///////////////////////
void highPowerLed() {
    GPIO_config_t gpioPin = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin          = GPIO_PIN27,
        .defaultLevel = GPIO_LEVEL_HIGH
    };
    GPIO_Init(gpioPin);
}

/////////////////////
// Task definition //
/////////////////////

// Nothing else just blink the led28
void LED28_BlinkTask()
{
    static GPIO_LEVEL ledBlueLevel = GPIO_LEVEL_LOW;

    GPIO_config_t gpioLedBlue = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin          = GPIO_PIN28,
        .defaultLevel = GPIO_LEVEL_LOW
    };    
    GPIO_Init(gpioLedBlue); 
    
    while(1)
    {
        ledBlueLevel = !ledBlueLevel;
        Trace(1, TRACE_PREFIX "LED 28 - %d", ledBlueLevel);
        GPIO_SetLevel(gpioLedBlue,ledBlueLevel);        //Set level
        OS_Sleep(1000);                                 //Sleep 500 ms
    }
}

void GPS_TASK() {
    GPS_Info_t* gpsInfo = Gps_GetInfo();
    uint8_t buffer[300];

    //open GPS hardware(UART2 open either)
    GPS_Init();
    GPS_Open(NULL);

    //wait for gps start up, or gps will not response command
    while(gpsInfo->rmc.latitude.value == 0) {
        Trace(2, TRACE_PREFIX "GPS - waiting startup");
        OS_Sleep(1000);
    }

    // set gps nmea output interval
    for(uint8_t i = 0;i<5;++i)
    {
        bool ret = GPS_SetOutputInterval(10000);
        Trace(2, TRACE_PREFIX "GPS - Set ret: %d", ret);
        if(ret) break;
        OS_Sleep(1000);
    }

    if(!GPS_GetVersion(buffer,150)){
        Trace(2, TRACE_PREFIX "GPS - Get gps firmware version fail");
    } else {
        Trace(2,  TRACE_PREFIX "GPS - Firmware version: %s", buffer);
    }

    if(!GPS_SetOutputInterval(1000))
        Trace(2, TRACE_PREFIX "GPS - Set nmea output interval fail");
    
    Trace(2, TRACE_PREFIX "GPS - Init OK");

    while(1) {
        Trace(2, TRACE_PREFIX "GPS - Latitude %lf", gpsInfo->rmc.latitude.value);
        OS_Sleep(1000);
    }
}


/////////////////////
////// Startup //////
/////////////////////

void setup()
{
    // I don't know what is this :)
    PM_PowerEnable(POWER_TYPE_MAX, true);

    // Turn on the power led (io27)
    highPowerLed();
}

void EventDispatch(API_Event_t* pEvent)
{
    switch(pEvent->id)
    {
        default:
            break;
    }
}


void MainTask(void *pData)
{
    API_Event_t* event = NULL;
    
    //open UART1 to print NMEA infomation
    UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_115200,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity   = UART_PARITY_NONE,
        .rxCallback = NULL,
        .useEvent   = true
    };
    UART_Init(UART1,config);

    led28BlinkTaskHandle = OS_CreateTask(LED28_BlinkTask, NULL, NULL, LED28_BLINK_TASK_STACK_SIZE, LED28_BLINK_TASK_PRIORITY, 0, 0, LED28_BLINK_TASK_NAME);
    gpsTaskHandle = OS_CreateTask(GPS_TASK, NULL, NULL, GPS_TASK_STACK_SIZE, GPS_TASK_PRIORITY, 0, 0, GPS_TASK_NAME);
    while(1) {
        if(OS_WaitEvent(mainTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}


void smartmotor_Main() {
    setup();
    mainTaskHandle = OS_CreateTask(MainTask, NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&mainTaskHandle);
}
