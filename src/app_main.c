#include <string.h>
#include <stdio.h>
#include "math.h"
#include "buffer.h"
#include "time.h"
#include "stdbool.h"
#include "stdint.h"
#include <ctype.h>
//Main
#include "api_hal_gpio.h"
#include "stdint.h"
#include "stdbool.h"
#include "api_debug.h"
#include "api_os.h"
#include "api_hal_pm.h"
#include "api_event.h"
#include "assert.h"
//GPS
#include <api_gps.h>
#include "gps_parse.h"
#include "gps.h"
#include <api_hal_uart.h>
#include "api_info.h"

//GPRS
#include "api_socket.h"
#include "api_network.h"
#include "api_lbs.h"

//MQTT
#include "api_mqtt.h"

#define DEBUG 0

#define MAIN_TASK_STACK_SIZE (1024 * 2)
#define MAIN_TASK_PRIORITY 0
#define MAIN_TASK_NAME "MAIN Task"

#define GPS_TASK_STACK_SIZE (2048 * 2)
#define GPS_TASK_PRIORITY 0
#define GPS_TASK_NAME "GPS Task"

#define MQTT_TASK_STACK_SIZE (2048 * 2)
#define MQTT_TASK_PRIORITY 0
#define MQTT_TASK_NAME "MQTT Task"

#define ALARM_TASK_STACK_SIZE (1024 * 2)
#define ALARM_TASK_PRIORITY 1
#define ALARM_TASK_NAME "Alarm Task"

#define LOCK_TASK_STACK_SIZE (1024 * 2)
#define LOCK_TASK_PRIORITY 1
#define LOCK_TASK_NAME "Lock Task"

#define LED27_BLINK_TASK_STACK_SIZE (1024 * 2)
#define LED27_BLINK_TASK_PRIORITY 2
#define LED27_BLINK_TASK_NAME "LED27 Blink Task"

#define LED28_BLINK_TASK_STACK_SIZE (1024 * 2)
#define LED28_BLINK_TASK_PRIORITY 2
#define LED28_BLINK_TASK_NAME "LED28 Blink Task"

#define TRACE_PREFIX "LOG:: "

#define BROKER_IP "206.189.90.18"
#define BROKER_PORT 1883
#define CLIENT_ID "D01"
#define CLIENT_USER NULL
#define CLIENT_PASS NULL

#define SUBSCRIBE_TOPIC "smartmotor/control/"

#define PUBLISH_TOPIC "smartmotor/track/"

#define ALARM_SIGNAL_PIN 25
// #define LOCK_PIN 30
// #define SIGNAL_PIN 26

// #define GPS_NMEA_LOG_FILE_PATH "/t/gps_nmea.log"

static HANDLE mainTaskHandle = NULL;
static HANDLE gpsTaskHandle = NULL;
static HANDLE mqttTaskHandle = NULL;

static HANDLE alarmHandle = NULL;
static HANDLE lockHandle = NULL;
static HANDLE signalHandle = NULL;

static MQTT_Client_t *client = NULL;

bool isGpsOn = true;
bool networkFlag = false;

bool isAlarm = false;
bool isLock  = false;
bool isSignal= false;

int mqtt_Status = 0;
int gps_Status = 0;

char subscribe_topic_id [100] = SUBSCRIBE_TOPIC;
char publish_topic_id [100] = PUBLISH_TOPIC;


uint8_t deviceId[300];

static HANDLE semMqttStart = NULL;
static HANDLE semAlarm = NULL;
static HANDLE semLock = NULL;
// static HANDLE semNetworkStart = NULL;
HANDLE semGetCellInfo = NULL;

float latitudeLbs = 0.0;
float longitudeLbs = 0.0;

typedef enum
{
    MQTT_EVENT_CONNECTED = 0,
    MQTT_EVENT_DISCONNECTED,
    MQTT_EVENT_MAX
} MQTT_Event_ID_t;

typedef struct
{
    MQTT_Event_ID_t id;
    MQTT_Client_t *client;
} MQTT_Event_t;

typedef enum
{
    MQTT_STATUS_DISCONNECTED = 0,
    MQTT_STATUS_CONNECTED,
    MQTT_STATUS_MAX
} MQTT_Status_t;

MQTT_Status_t mqttStatus = MQTT_STATUS_DISCONNECTED;
GPIO_config_t gpioPin27, gpioPin28;

///////////////////////
// Helper definition //
///////////////////////
void highPowerLed()
{
    GPIO_config_t gpioPin27 = {
        .mode = GPIO_MODE_OUTPUT,
        .pin = GPIO_PIN27,
        .defaultLevel = GPIO_LEVEL_HIGH};
    GPIO_config_t gpioPin28 = {
        .mode = GPIO_MODE_OUTPUT,
        .pin = GPIO_PIN28,
        .defaultLevel = GPIO_LEVEL_HIGH};
    GPIO_Init(gpioPin27);
    GPIO_Init(gpioPin28);
    int counter = 0;
    for (counter = 0; counter < 3; counter++) {
        GPIO_SetLevel(gpioPin27, GPIO_LEVEL_HIGH);
        GPIO_SetLevel(gpioPin28, GPIO_LEVEL_HIGH);
        OS_Sleep(200);
        GPIO_SetLevel(gpioPin27, GPIO_LEVEL_LOW);
        GPIO_SetLevel(gpioPin28, GPIO_LEVEL_LOW);
        OS_Sleep(200);
    }
}

void blink(GPIO_config_t gpioPin) {
    GPIO_SetLevel(gpioPin, GPIO_LEVEL_LOW);
    OS_Sleep(200);
    GPIO_SetLevel(gpioPin, GPIO_LEVEL_HIGH);
    OS_Sleep(200);
    GPIO_SetLevel(gpioPin, GPIO_LEVEL_LOW);
    OS_Sleep(200);
}

void doubleBlink(GPIO_config_t gpioPin) {
    GPIO_SetLevel(gpioPin, GPIO_LEVEL_LOW);
    OS_Sleep(100);
    GPIO_SetLevel(gpioPin, GPIO_LEVEL_HIGH);
    OS_Sleep(100);
    GPIO_SetLevel(gpioPin, GPIO_LEVEL_LOW);
    OS_Sleep(100);
    GPIO_SetLevel(gpioPin, GPIO_LEVEL_HIGH);
    OS_Sleep(100);
    GPIO_SetLevel(gpioPin, GPIO_LEVEL_LOW);
    OS_Sleep(100);
}

void OnPublish(void* arg, MQTT_Error_t err)
{
    GPIO_config_t gpioPin27 = {
        .mode = GPIO_MODE_OUTPUT,
        .pin = GPIO_PIN27,
        .defaultLevel = GPIO_LEVEL_HIGH};
    
    GPIO_Init(gpioPin27);
    if(err == MQTT_ERROR_NONE) {
        Trace(1, TRACE_PREFIX "MQTT publish success");
        blink(gpioPin27);
    } else {
        Trace(1, TRACE_PREFIX "MQTT publish error, error code:%d",err);
        doubleBlink(gpioPin27);
    }
}
void MqttPublish(char* payload, int isRetain) {
    MQTT_Error_t err;
    if(mqttStatus != MQTT_STATUS_CONNECTED || client == NULL)
    {
        mqtt_Status = 3;
        Trace(1, TRACE_PREFIX "MQTT not connected to broker! can not publish");
        return;
    } 
    Trace(1,TRACE_PREFIX "MQTT publishing");
    mqtt_Status = 2;
    
    if (isRetain){ 
        err = MQTT_Publish(client,publish_topic_id,payload,strlen(payload),1,2,1,OnPublish,NULL);
    } else {
        err = MQTT_Publish(client,publish_topic_id,payload,strlen(payload),1,2,0,OnPublish,NULL);
    }
}

void GPS_TASK()
{
    //wait for gprs register complete
    //The process of GPRS registration network may cause the power supply voltage of GPS to drop,
    //which resulting in GPS restart.

    // Waiting for Network
    gps_Status = 0;
    while (!networkFlag)
    {
        Trace(1, TRACE_PREFIX "Wait for gprs regiter complete");
        OS_Sleep(2000);
    }


    // Starting up...
    gps_Status = 1;

    GPS_Info_t *gpsInfo = Gps_GetInfo();
    uint8_t buffer[300], buffer2[400];

    //open GPS hardware(UART2 open either)
    GPS_Init();
    // GPS_SaveLog(true,GPS_NMEA_LOG_FILE_PATH);
    GPS_Open(NULL);

    //wait for gps start up, or gps will not response command
    while (gpsInfo->rmc.latitude.value == 0)
    {
        Trace(2, TRACE_PREFIX "GPS - waiting startup");
        OS_Sleep(1000);
    }

    // set gps nmea output interval
    for (uint8_t i = 0; i < 5; ++i)
    {
        bool ret = GPS_SetOutputInterval(10000);
        Trace(2, TRACE_PREFIX "GPS - Set ret: %d", ret);
        if (ret)
            break;
        OS_Sleep(1000);
    }

    if (!GPS_GetVersion(buffer, 150))
    {
        Trace(2, TRACE_PREFIX "GPS - Get gps firmware version fail");
    }
    else
    {
        Trace(2, TRACE_PREFIX "GPS - Firmware version: %s", buffer);
    }

    //get location through LBS
    semGetCellInfo = OS_CreateSemaphore(0);
    if (!Network_GetCellInfoRequst())
    {
        Trace(1, TRACE_PREFIX "Network get cell info fail");
    }
    OS_WaitForSemaphore(semGetCellInfo, OS_TIME_OUT_WAIT_FOREVER);
    OS_DeleteSemaphore(semGetCellInfo);
    semGetCellInfo = NULL;

    //send location to GPS and update brdc GPD file
    Trace(1, TRACE_PREFIX "Do AGPS now");

    if (!GPS_AGPS(latitudeLbs, longitudeLbs, 0, true))
    {
        Trace(1, TRACE_PREFIX "Agps fail");
    }
    else
    {
        Trace(1, TRACE_PREFIX "Do AGPS success");
    }

    //set nmea output interval as 1s
    if (!GPS_SetOutputInterval(1000))
        Trace(2, TRACE_PREFIX "GPS - Set nmea output interval fail");

    Trace(2, TRACE_PREFIX "GPS - Init OK");
    
    gps_Status = 2;

    GPIO_config_t gpioPin28 = {
        .mode = GPIO_MODE_OUTPUT,
        .pin = GPIO_PIN28,
        .defaultLevel = GPIO_LEVEL_LOW};
    GPIO_Init(gpioPin28);

    while (1)
    {
        if (isGpsOn)
        {
            //show fix info
            uint8_t isFixed = gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type
                                  ? gpsInfo->gsa[0].fix_type
                                  : gpsInfo->gsa[1].fix_type;

            char *isFixedStr = NULL;

            if (isFixed == 2)
            {
                isFixedStr = "2D";
            }
            else if (isFixed == 3)
            {
                if (gpsInfo->gga.fix_quality == 1)
                    isFixedStr = "3D";
                else if (gpsInfo->gga.fix_quality == 2)
                    isFixedStr = "3D/DGPS";
            }
            else
            {
                isFixedStr = "no";
            }

            // Convert unit ddmm.mmmm to degree(°)
            int temp = (int)(gpsInfo->rmc.latitude.value / gpsInfo->rmc.latitude.scale / 100);
            double latitude = temp + (double)(gpsInfo->rmc.latitude.value - temp * gpsInfo->rmc.latitude.scale * 100) / gpsInfo->rmc.latitude.scale / 60.0;
            temp = (int)(gpsInfo->rmc.longitude.value / gpsInfo->rmc.longitude.scale / 100);
            double longitude = temp + (double)(gpsInfo->rmc.longitude.value - temp * gpsInfo->rmc.longitude.scale * 100) / gpsInfo->rmc.longitude.scale / 60.0;

            // Send to UART1
            UART_Write(UART1, buffer, strlen(buffer));
            UART_Write(UART1, "\r\n\r\n", 4);

            // Send to server
            char *requestPath = buffer2;
            uint8_t percent;
            uint16_t v = PM_Voltage(&percent);
            // Trace(1, TRACE_PREFIX "Power:%d %d",v,percent);
            // memset(buffer, 0, sizeof(buffer));
            // if (!INFO_GetIMEI(buffer))
            //     Assert(false, TRACE_PREFIX "NO IMEI");
            // Trace(1, TRACE_PREFIX "Device name:%s",buffer);
            snprintf(requestPath, sizeof(buffer2),
                     "{\"deviceNumber\":%s,"
                     "\"gpsFixMode\":%d,"
                     "\"bdsFixMode\":%d,"
                     "\"fixQuality\":%d,"
                     "\"satelitesTracked\":%d,"
                     "\"gpsSatesTotal\":%d,"
                     "\"isFixed\":\"%s\","
                     "\"lat\":%f,"
                     "\"lon\":%f,"
                     "\"batt\":%.1f}",
                     deviceId,
                     gpsInfo->gsa[0].fix_type,
                     gpsInfo->gsa[1].fix_type,
                     gpsInfo->gga.fix_quality,
                     gpsInfo->gga.satellites_tracked,
                     gpsInfo->gsv[0].total_sats,
                     isFixedStr,
                     latitude,
                     longitude,
                     percent * 1.0);
            Trace(1, TRACE_PREFIX "JSON: %s", requestPath);
            if (gpsInfo->gga.fix_quality) {
                MqttPublish(requestPath, 1);
                blink(gpioPin28);
            } else {
                MqttPublish(requestPath, 0);
                doubleBlink(gpioPin28);
            }
        }
        OS_Sleep(5000);
    }
}

/////////////////////
////// Startup //////
/////////////////////

void setup()
{
    PM_PowerEnable(POWER_TYPE_MAX, true);
    PM_PowerEnable(POWER_TYPE_VPAD, true);
    highPowerLed();
}

void EventDispatch(API_Event_t *pEvent)
{
    static uint8_t lbsCount = 0;
    switch (pEvent->id)
    {
    case API_EVENT_ID_SYSTEM_READY:
        Trace(1, TRACE_PREFIX "System initialize complete");
        break;
    case API_EVENT_ID_NO_SIMCARD:
        Trace(10, TRACE_PREFIX "!!NO SIM CARD%d!!!!", pEvent->param1);
        networkFlag = false;
        break;
    case API_EVENT_ID_NETWORK_REGISTER_SEARCHING:
        Trace(2, TRACE_PREFIX "Network register searching");
        networkFlag = false;
        break;
    case API_EVENT_ID_NETWORK_REGISTER_DENIED:
        Trace(2, TRACE_PREFIX "Network register denied");
    case API_EVENT_ID_NETWORK_REGISTER_NO:
        Trace(2, TRACE_PREFIX "Network register no");
        break;
    case API_EVENT_ID_GPS_UART_RECEIVED:
        // Trace(1,"received GPS data,length:%d, data:%s,flag:%d",pEvent->param1,pEvent->pParam1,flag);
        GPS_Update(pEvent->pParam1, pEvent->param1);
        break;
    case API_EVENT_ID_NETWORK_REGISTERED_HOME:
    case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
    {
        uint8_t status;
        Trace(2, TRACE_PREFIX "Network register success");
        bool ret = Network_GetAttachStatus(&status);
        if (!ret)
            Trace(1, TRACE_PREFIX "Get attach status fail");
        Trace(1, TRACE_PREFIX "Attach status:%d", status);
        if (status == 0)
        {
            ret = Network_StartAttach();
            if (!ret)
            {
                Trace(1, TRACE_PREFIX "Network attach fail");
            }
        }
        else
        {
            Network_PDP_Context_t context = {
                .apn = "v-internet",
                .userName = "",
                .userPasswd = ""};
            Network_StartActive(context);
        }
        break;
    }
    case API_EVENT_ID_NETWORK_ATTACHED:
        Trace(2, TRACE_PREFIX "Network attach success");
        Network_PDP_Context_t context = {
            .apn = "v-internet",
            .userName = "",
            .userPasswd = ""};
        Network_StartActive(context);
        break;

    case API_EVENT_ID_NETWORK_ACTIVATED:
        Trace(2, TRACE_PREFIX "Network activate success");
        networkFlag = true;
        OS_ReleaseSemaphore(semMqttStart);
        break;

    case API_EVENT_ID_UART_RECEIVED:
        if (pEvent->param1 == UART1)
        {
            uint8_t data[pEvent->param2 + 1];
            data[pEvent->param2] = 0;
            memcpy(data, pEvent->pParam1, pEvent->param2);
            Trace(1, TRACE_PREFIX "Uart received data,length:%d,data:%s", pEvent->param2, data);
            if (strcmp(data, "close") == 0)
            {
                Trace(1, TRACE_PREFIX "Close gps");
                GPS_Close();
                isGpsOn = false;
            }
            else if (strcmp(data, "open") == 0)
            {
                Trace(1, TRACE_PREFIX "Open gps");
                GPS_Open(NULL);
                isGpsOn = true;
            }
        }
        break;
    case API_EVENT_ID_NETWORK_CELL_INFO:
    {
        uint8_t number = pEvent->param1;
        Network_Location_t *location = (Network_Location_t *)pEvent->pParam1;
        Trace(2, TRACE_PREFIX "Network cell infomation,serving cell number:1, neighbor cell number:%d", number - 1);

        for (int i = 0; i < number; ++i)
        {
            Trace(2, TRACE_PREFIX "Cell %d info:%d%d%d,%d%d%d,%d,%d,%d,%d,%d,%d",
                  i,
                  location[i].sMcc[0],
                  location[i].sMcc[1],
                  location[i].sMcc[2],
                  location[i].sMnc[0],
                  location[i].sMnc[1],
                  location[i].sMnc[2],
                  location[i].sLac,
                  location[i].sCellID,
                  location[i].iBsic,
                  location[i].iRxLev,
                  location[i].iRxLevSub,
                  location[i].nArfcn);
        }
        // LBS == Location Based Services
        if (!LBS_GetLocation(location, number, 15, &longitudeLbs, &latitudeLbs))
            Trace(1, TRACE_PREFIX "===LBS get location fail===");
        else
            Trace(1, TRACE_PREFIX "===LBS get location success, latitude:%f,longitude:%f===", latitudeLbs, longitudeLbs);
        if ((latitudeLbs == 0) && (longitudeLbs == 0)) //not get location from server, try again
        {
            if (++lbsCount > 6)
            {
                lbsCount = 0;
                Trace(1, TRACE_PREFIX "Try 6 times to get location from lbs but fail!!");
                OS_ReleaseSemaphore(semGetCellInfo);
                break;
            }
            if (!Network_GetCellInfoRequst())
            {
                Trace(1, TRACE_PREFIX "Network get cell info fail");
                OS_ReleaseSemaphore(semGetCellInfo);
            }
            break;
        }
        OS_ReleaseSemaphore(semGetCellInfo);
        lbsCount = 0;
        break;
    }
    case API_EVENT_ID_SOCKET_CONNECTED:
        Trace(1, TRACE_PREFIX "socket connected");
        break;

    case API_EVENT_ID_SOCKET_CLOSED:
        Trace(1, TRACE_PREFIX "socket closed");
        break;

    case API_EVENT_ID_SIGNAL_QUALITY:
        Trace(1, TRACE_PREFIX "CSQ:%d", pEvent->param1);
        break;

    default:
        break;
    }
}

// MQTT HANDLE
void OnMqttReceived(void *arg, const char *topic, uint32_t payloadLen)
{
    Trace(2, TRACE_PREFIX "MQTT received publish data request, topic:%s, payload length:%d", topic, payloadLen);
}

void OnMqttReceiedData(void *arg, const uint8_t *data, uint16_t len, MQTT_Flags_t flags)
{
    Trace(2, TRACE_PREFIX "MQTT recieved publish data,  length:%d,data:%s", len, data);

    if (memcmp(data, "0", 1) == 0) {
        OS_ReleaseSemaphore(semLock);
        isLock = !isLock;
    } else {
        OS_ReleaseSemaphore(semAlarm);
        // isAlarm = 1;
    }

    if (flags == MQTT_FLAG_DATA_LAST)
        Trace(2, TRACE_PREFIX "MQTT data is last frame");
}
void OnMqttSubscribed(void *arg, MQTT_Error_t err)
{
    if (err != MQTT_ERROR_NONE)
        Trace(2, TRACE_PREFIX "MQTT subscribe fail,error code:%d", err);
    else
        Trace(2, TRACE_PREFIX "MQTT subscribe success,topic:%s", (const char *)arg);
}

void OnMqttConnection(MQTT_Client_t *in_client, void *arg, MQTT_Connection_Status_t status)
{
    Trace(2, TRACE_PREFIX "MQTT connection status:%d", status);
    MQTT_Event_t *event = (MQTT_Event_t *)OS_Malloc(sizeof(MQTT_Event_t));
    if (!event)
    {
        Trace(2, TRACE_PREFIX "MQTT no memory");
        return;
    }
    if (status == MQTT_CONNECTION_ACCEPTED)
    {
        // mqtt_Status = 2;
        Trace(2, TRACE_PREFIX "MQTT succeed connect to broker");
        //!!! DO NOT suscribe here(interrupt function), do MQTT suscribe in task, or it will not excute
        event->id = MQTT_EVENT_CONNECTED;
        event->client = in_client;
        OS_SendEvent(mqttTaskHandle, event, OS_TIME_OUT_WAIT_FOREVER, OS_EVENT_PRI_NORMAL);
    }
    else
    {
        // mqtt_Status = 3;
        Trace(2, TRACE_PREFIX "MQTT connect to broker fail,error code:%d", status);
        event->id = MQTT_EVENT_DISCONNECTED;
        event->client = in_client;
        OS_SendEvent(mqttTaskHandle, event, OS_TIME_OUT_WAIT_FOREVER, OS_EVENT_PRI_NORMAL);
    }
    Trace(2, TRACE_PREFIX "MQTT OnMqttConnection() end");
}

void MqttTaskEventDispatch(MQTT_Event_t *pEvent)
{
    switch (pEvent->id)
    {
        case MQTT_EVENT_CONNECTED:
            mqttStatus = MQTT_STATUS_CONNECTED;
            Trace(2, TRACE_PREFIX "MqttTaskEventDispatch MQTT connected, now subscribe: %s", subscribe_topic_id);
            MQTT_Error_t err;
            MQTT_SetInPubCallback(pEvent->client, OnMqttReceived, OnMqttReceiedData, NULL);
            err = MQTT_Subscribe(pEvent->client,subscribe_topic_id,2,OnMqttSubscribed,(void*)subscribe_topic_id);
            if (err != MQTT_ERROR_NONE)
                Trace(2, TRACE_PREFIX "MqttTaskEventDispatch MQTT sub err, code:%d", err);
            break;
        case MQTT_EVENT_DISCONNECTED:
            Trace(2, TRACE_PREFIX "MqttTaskEventDispatch MQTT STATUS DISCONNECTED");
            mqttStatus = MQTT_STATUS_DISCONNECTED;
            break;
        default:
            break;
    }
}

void MqttTask(void *pData)
{
    MQTT_Event_t *event = NULL;
    semMqttStart = OS_CreateSemaphore(0);
    // Trace(1, TRACE_PREFIX "Waiting for Semaphore MQTT");
    OS_WaitForSemaphore(semMqttStart, OS_WAIT_FOREVER);
    // Trace(1, TRACE_PREFIX "Semaphore MQTT signed");
    OS_DeleteSemaphore(semMqttStart);
    Trace(2, TRACE_PREFIX "Start MQTT test with host: %s, port %d and user: %s, pass: %s", BROKER_IP, BROKER_PORT, CLIENT_USER, CLIENT_PASS);
    
    client = MQTT_ClientNew();
    MQTT_Connect_Info_t ci;
    MQTT_Error_t err;
    memset(&ci, 0, sizeof(MQTT_Connect_Info_t));
    ci.client_id = CLIENT_ID;
    ci.client_user = CLIENT_USER;
    ci.client_pass = CLIENT_PASS;
    ci.keep_alive = 60;
    ci.clean_session = 1;
    ci.use_ssl = false;
    
    err = MQTT_Connect(client, BROKER_IP, BROKER_PORT, OnMqttConnection, NULL, &ci);
    if (err != MQTT_ERROR_NONE)
        Trace(2, TRACE_PREFIX "MQTT connect fail,error code:%d", err);

    while (1)
    {
        if (OS_WaitEvent(mqttTaskHandle, (void **)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            MqttTaskEventDispatch(event);
            OS_Free(event);
        }
    }
}

void AlarmTask(void* pData) {
    GPIO_config_t gpioAlarmSignal = {
        .mode = GPIO_MODE_OUTPUT,
        .pin = GPIO_PIN30,
        .defaultLevel = GPIO_LEVEL_LOW};

    GPIO_Init(gpioAlarmSignal);
    while (1)
    {
        OS_WaitForSemaphore(semAlarm, OS_WAIT_FOREVER);
        Trace(2, TRACE_PREFIX "BUZZING!");
        //---------------------------------
        GPIO_SetLevel(gpioAlarmSignal, GPIO_LEVEL_HIGH);
        OS_Sleep(200);
        GPIO_SetLevel(gpioAlarmSignal, GPIO_LEVEL_LOW);
        OS_Sleep(200);
        GPIO_SetLevel(gpioAlarmSignal, GPIO_LEVEL_HIGH);
        OS_Sleep(200);
        GPIO_SetLevel(gpioAlarmSignal, GPIO_LEVEL_LOW);
        OS_Sleep(200);
        GPIO_SetLevel(gpioAlarmSignal, GPIO_LEVEL_HIGH);
        OS_Sleep(200);
        GPIO_SetLevel(gpioAlarmSignal, GPIO_LEVEL_LOW);
        OS_Sleep(200);
        //---------------------------------
        Trace(2, TRACE_PREFIX "STOP BUZZING!");
        // GPIO_SetLevel(gpioAlarmSignal, GPIO_LEVEL_LOW);
    }
}

void LockTask(void* pData) {
    GPIO_config_t gpioLockSignal = {
        .mode = GPIO_MODE_OUTPUT,
        .pin = GPIO_PIN26,
        .defaultLevel = GPIO_LEVEL_LOW};

    GPIO_Init(gpioLockSignal);

    while (1)
    {
        OS_WaitForSemaphore(semLock, OS_WAIT_FOREVER);
        if (isLock) {
            GPIO_SetLevel(gpioLockSignal, GPIO_LEVEL_HIGH); //Set level
            Trace(2, TRACE_PREFIX "LOCK!");
        } else {
            GPIO_SetLevel(gpioLockSignal, GPIO_LEVEL_LOW);
            Trace(2, TRACE_PREFIX "UNLOCK!");
        }
    }
}

void MainTask(void *pData)
{
    TIME_SetIsAutoUpdateRtcTime(true);

    //open UART1 to print NMEA infomation
    UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_115200,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity = UART_PARITY_NONE,
        .rxCallback = NULL,
        .useEvent = true};
    UART_Init(UART1, config);

    semAlarm = OS_CreateSemaphore(0);
    semLock = OS_CreateSemaphore(0);

    memset(deviceId, 0, sizeof(deviceId));
    if (!INFO_GetIMEI(deviceId))
        Assert(false, TRACE_PREFIX "NO IMEI");

    strcat(subscribe_topic_id, (char*) deviceId);
    strcat(publish_topic_id, (char*) deviceId);

    Trace(3, TRACE_PREFIX "Sub topic: %s", subscribe_topic_id);
    Trace(3, TRACE_PREFIX "Pub topic: %s", publish_topic_id);

    gpsTaskHandle = OS_CreateTask(GPS_TASK, NULL, NULL, GPS_TASK_STACK_SIZE, GPS_TASK_PRIORITY, 0, 0, GPS_TASK_NAME);
    mqttTaskHandle = OS_CreateTask(MqttTask, NULL, NULL, MQTT_TASK_STACK_SIZE, MQTT_TASK_PRIORITY, 0, 0, MQTT_TASK_NAME);
    alarmHandle = OS_CreateTask(AlarmTask, NULL, NULL, ALARM_TASK_STACK_SIZE, ALARM_TASK_PRIORITY, 0, 0, ALARM_TASK_NAME);
    lockHandle = OS_CreateTask(LockTask, NULL, NULL, LOCK_TASK_STACK_SIZE, LOCK_TASK_PRIORITY, 0, 0, LOCK_TASK_NAME);
    
    // if(DEBUG) {
    //     led27BlinkTaskHandle = OS_CreateTask(LED27_BlinkTask, NULL, NULL, LED27_BLINK_TASK_STACK_SIZE, LED27_BLINK_TASK_PRIORITY, 0, 0, LED27_BLINK_TASK_NAME);
    //     led28BlinkTaskHandle = OS_CreateTask(LED28_BlinkTask, NULL, NULL, LED28_BLINK_TASK_STACK_SIZE, LED28_BLINK_TASK_PRIORITY, 0, 0, LED28_BLINK_TASK_NAME);
    // }
    
    API_Event_t *event = NULL;
    while (1)
    {
        if (OS_WaitEvent(mainTaskHandle, (void **)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}

void SmartMotor_Device_Main()
{
    setup();
    mainTaskHandle = OS_CreateTask(MainTask, NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&mainTaskHandle);
}
