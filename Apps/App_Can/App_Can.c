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
    UltrasonicDistanceCmd_t message = {0};

    message.frontDist = g_distancesMm[ULTRASONIC_FC];
    message.frontRightDist = g_distancesMm[ULTRASONIC_FR];
    message.rightFrontDist = g_distancesMm[ULTRASONIC_RF];
    message.rightBehindDist = g_distancesMm[ULTRASONIC_RM];
    message.behindRightDist = g_distancesMm[ULTRASONIC_RR];
    message.behindDist = g_distancesMm[ULTRASONIC_BC];
    message.behindLeftDist = g_distancesMm[ULTRASONIC_RL];
    message.leftBehindDist = g_distancesMm[ULTRASONIC_LM];
    message.leftFrontDist = g_distancesMm[ULTRASONIC_LF];
    message.frontLeftDist = g_distancesMm[ULTRASONIC_FL];
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
