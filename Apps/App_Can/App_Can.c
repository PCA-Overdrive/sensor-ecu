/**********************************************************************************************************************
 * \file App_Can.c
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "App_Can.h"

#include "Apps/App_HallSensor/App_HallSensor.h"
#include "Apps/App_Ultrasonic/App_Ultrasonic.h"
#include "Drivers/Can/McmcanFd.h"

#include "FreeRTOS.h"
#include "task.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define CAN_APP_SEND_INTERVAL_MS                (100U)
#define ULTRASONIC_DISTANCE_INDEX_SENSOR0       (0U)
#define ULTRASONIC_DISTANCE_INDEX_SENSOR5       (1U)
#define ULTRASONIC_DISTANCE_INDEX_SENSOR1       (2U)
#define ULTRASONIC_DISTANCE_INDEX_SENSOR6       (3U)
#define ULTRASONIC_DISTANCE_INDEX_SENSOR2       (4U)
#define ULTRASONIC_DISTANCE_INDEX_SENSOR7       (5U)
#define ULTRASONIC_DISTANCE_INDEX_SENSOR3       (6U)
#define ULTRASONIC_DISTANCE_INDEX_SENSOR8       (7U)
#define ULTRASONIC_DISTANCE_INDEX_SENSOR4       (8U)
#define ULTRASONIC_DISTANCE_INDEX_SENSOR9       (9U)

/*********************************************************************************************************************/
/*-------------------------------------------------Global Variables--------------------------------------------------*/
/*********************************************************************************************************************/
static boolean g_isInitialized = FALSE;
static sint16 g_imuYaw = 0;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
static void sendUltrasonicDistanceMessage(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
static void sendUltrasonicDistanceMessage(void)
{
    UltrasonicDistanceCmd_t message;

    message.frontDist = g_distances[ULTRASONIC_DISTANCE_INDEX_SENSOR0];
    message.frontRightDist = g_distances[ULTRASONIC_DISTANCE_INDEX_SENSOR1];
    message.rightFrontDist = g_distances[ULTRASONIC_DISTANCE_INDEX_SENSOR2];
    message.rightBehindDist = g_distances[ULTRASONIC_DISTANCE_INDEX_SENSOR3];
    message.behindRightDist = g_distances[ULTRASONIC_DISTANCE_INDEX_SENSOR4];
    message.behindDist = g_distances[ULTRASONIC_DISTANCE_INDEX_SENSOR5];
    message.behindLeftDist = g_distances[ULTRASONIC_DISTANCE_INDEX_SENSOR6];
    message.leftBehindDist = g_distances[ULTRASONIC_DISTANCE_INDEX_SENSOR7];
    message.leftFrontDist = g_distances[ULTRASONIC_DISTANCE_INDEX_SENSOR8];
    message.frontLeftDist = g_distances[ULTRASONIC_DISTANCE_INDEX_SENSOR9];
    message.imuYaw = g_imuYaw;
    message.vehicleSpeed = g_hallVehicleSpeed;

    McmcanFd_SendUltrasonic(&message);
}

void CanApp_Init(void)
{
    if (g_isInitialized == TRUE)
    {
        return;
    }

    McmcanFd_Init();

    g_isInitialized = TRUE;
}

void CanApp_Run(void *arg)
{
    (void)arg;

    CanApp_Init();

    while (1)
    {
        sendUltrasonicDistanceMessage();
        vTaskDelay(pdMS_TO_TICKS(CAN_APP_SEND_INTERVAL_MS));
    }
}

void CanApp_SetImuYaw(sint16 imuYaw)
{
    g_imuYaw = imuYaw;
}

