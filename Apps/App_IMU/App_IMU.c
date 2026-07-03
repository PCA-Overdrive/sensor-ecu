#include "App_IMU.h"

#include "tc375_icm20948_port.h"
#include "App_CalibStorage.h"
#include "App_Led2.h"
#include "App_Can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

volatile uint32_t g_imuYawDebugStep = 0u;

static TC375_ICM20948_t g_imu;
static ImuYawApp_Output_t g_out;

static volatile bool g_calibrationRequested = false;
static bool g_yawOffsetSet = false;
static bool g_readyTimingStarted = false;
static ImuCalState_t g_calState = IMU_CAL_STATE_IDLE;
static bool g_accelAcc3Notified = false;
static bool g_gyroAcc3Notified = false;

static uint32_t g_readyStartMs = 0u;
static uint32_t g_lastPrintMs = 0u;
static uint32_t g_lastAccuracyReadMs = 0u;

static double g_yawSinSum = 0.0;
static double g_yawCosSum = 0.0;
static int g_yawAvgCount = 0;

static double g_yawOffset = 0.0;
static double g_filteredRelYaw = 0.0;
static bool g_filteredInit = false;

static void ImuYawApp_delayMs(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        tc375_delay_ms(ms);
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

static double wrap360(double angle)
{
    while (angle < 0.0) angle += 360.0;
    while (angle >= 360.0) angle -= 360.0;
    return angle;
}

static double wrap180(double angle)
{
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

static int normalizeAccuracy(uint16_t raw)
{
    if (raw <= 3u)
    {
        return (int)raw;
    }

    if (((raw >> 8) <= 3u) && ((raw & 0x00FFu) == 0u))
    {
        return (int)(raw >> 8);
    }

    return (int)(raw & 0x0003u);
}

static void updateAccuracyDirect(void)
{
    uint16_t rawAccelAcc = 0u;
    uint16_t rawGyroAcc = 0u;

    if (TC375_ICM20948_readDMP16(&g_imu, ACCEL_ACCURACY, &rawAccelAcc) == ICM_20948_Stat_Ok)
    {
        g_out.accelAcc = normalizeAccuracy(rawAccelAcc);
    }

    if (TC375_ICM20948_readDMP16(&g_imu, GYRO_ACCURACY, &rawGyroAcc) == ICM_20948_Stat_Ok)
    {
        g_out.gyroAcc = normalizeAccuracy(rawGyroAcc);
    }
}

static void updateAccuracyDirectForCurrentState(void)
{
    uint32_t now;

    /*
     * Before READY or during calibration, read accuracy every IMU loop so the
     * calibration state changes immediately. After READY, reduce I2C load by
     * reading DMP accuracy only every READY_ACCURACY_READ_PERIOD_MS.
     */
    if ((g_out.yawReady == false) || (g_calState != IMU_CAL_STATE_IDLE))
    {
        updateAccuracyDirect();
        return;
    }

    now = tc375_millis();

    if ((uint32_t)(now - g_lastAccuracyReadMs) < READY_ACCURACY_READ_PERIOD_MS)
    {
        return;
    }

    g_lastAccuracyReadMs = now;
    updateAccuracyDirect();
}

static void resetYawCalibrationAccumulator(void)
{
    g_readyTimingStarted = false;
    g_yawSinSum = 0.0;
    g_yawCosSum = 0.0;
    g_yawAvgCount = 0;
    g_filteredInit = false;
}

static void startYawCalibration(void)
{
    g_yawOffsetSet = false;
    g_out.yawReady = false;
    g_out.calibrating = false;
    g_calState = IMU_CAL_STATE_WAIT_ACCURACY;
    g_accelAcc3Notified = false;
    g_gyroAcc3Notified = false;

    resetYawCalibrationAccumulator();

    IMU_YAW_PRINTF("\r\n[CAL] BUTTON1 calibration start. Keep sensor flat and still.\r\n");
    IMU_YAW_PRINTF("[CAL] Waiting for AccelAcc=3 and GyroAcc=3...\r\n");
    /* LED: no blink on calibration start by user request. */
}

static void updateYawCalibration(double yaw360)
{
    bool accReady;
    bool gyroReady;
    double yawRad;
    double avgYawRad;

    if (g_calibrationRequested)
    {
        g_calibrationRequested = false;
        startYawCalibration();
    }

    if (g_calState == IMU_CAL_STATE_IDLE)
    {
        return;
    }

    accReady = (g_out.accelAcc == 3);
    gyroReady = (g_out.gyroAcc == 3);

    if ((accReady == true) && (g_accelAcc3Notified == false))
    {
        g_accelAcc3Notified = true;
        IMU_YAW_PRINTF("[CAL] AccelAcc reached 3.\r\n");
        AppLed2_blink(2u, 100u);
    }

    if ((gyroReady == true) && (g_gyroAcc3Notified == false))
    {
        g_gyroAcc3Notified = true;
        IMU_YAW_PRINTF("[CAL] GyroAcc reached 3.\r\n");
        AppLed2_blink(2u, 100u);
    }

    if ((accReady == true) && (gyroReady == true))
    {
        if (!g_readyTimingStarted)
        {
            g_readyTimingStarted = true;
            g_readyStartMs = tc375_millis();

            g_yawSinSum = 0.0;
            g_yawCosSum = 0.0;
            g_yawAvgCount = 0;
            g_calState = IMU_CAL_STATE_AVERAGING;
            g_out.calibrating = true;

            IMU_YAW_PRINTF("\r\n[CAL] AccelAcc=3, GyroAcc=3 detected. Averaging yaw offset...\r\n");
            /* LED: no blink when both accuracy values are 3 by user request. */
        }

        yawRad = yaw360 * M_PI / 180.0;
        g_yawSinSum += sin(yawRad);
        g_yawCosSum += cos(yawRad);
        g_yawAvgCount++;

        if (((uint32_t)(tc375_millis() - g_readyStartMs) >= REQUIRED_READY_MS) && (g_yawAvgCount > 0))
        {
            avgYawRad = atan2(g_yawSinSum / g_yawAvgCount, g_yawCosSum / g_yawAvgCount);

            g_yawOffset = wrap360(avgYawRad * 180.0 / M_PI);
            g_yawOffsetSet = true;
            g_filteredInit = false;

            g_out.yawReady = true;
            g_out.calibrating = false;
            g_calState = IMU_CAL_STATE_IDLE;
            g_readyTimingStarted = false;

            if (AppCalibStorage_saveYawOffset(g_yawOffset) == true)
            {
                IMU_YAW_PRINTF("\r\n[READY] yawOffset = %.2f deg saved to DFLASH. RelYaw starts from 0 deg.\r\n", g_yawOffset);
                AppLed2_blink(3u, 100u);
            }
            else
            {
                IMU_YAW_PRINTF("\r\n[READY] yawOffset = %.2f deg, but DFLASH save failed.\r\n", g_yawOffset);
                AppLed2_blink(10u, 100u);
            }
        }
    }
    else
    {
        if (g_readyTimingStarted)
        {
            IMU_YAW_PRINTF("\r\n[CAL] Accuracy dropped. Restarting wait.\r\n");
            AppLed2_blink(5u, 100u);
        }

        resetYawCalibrationAccumulator();
        g_out.calibrating = false;
        g_calState = IMU_CAL_STATE_WAIT_ACCURACY;
    }
}


static void updateEulerFromQuat6(const icm_20948_DMP_data_t *data)
{
    const double q1 = ((double)data->Quat6.Data.Q1) / 1073741824.0;
    const double q2 = ((double)data->Quat6.Data.Q2) / 1073741824.0;
    const double q3 = ((double)data->Quat6.Data.Q3) / 1073741824.0;

    double q0sq = 1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
    if (q0sq < 0.0)
    {
        q0sq = 0.0;
    }

    const double q0 = sqrt(q0sq);

    /*
     * Same axis conversion as SparkFun Example7.
     */
    const double qw = q0;
    const double qx = q2;
    const double qy = q1;
    const double qz = -q3;

    const double t0 = 2.0 * (qw * qx + qy * qz);
    const double t1 = 1.0 - 2.0 * (qx * qx + qy * qy);
    g_out.rollDeg = atan2(t0, t1) * 180.0 / M_PI;

    double t2 = 2.0 * (qw * qy - qz * qx);
    if (t2 > 1.0) t2 = 1.0;
    if (t2 < -1.0) t2 = -1.0;
    g_out.pitchDeg = asin(t2) * 180.0 / M_PI;

    const double t3 = 2.0 * (qw * qz + qx * qy);
    const double t4 = 1.0 - 2.0 * (qy * qy + qz * qz);
    g_out.yawDeg360 = wrap360(atan2(t3, t4) * 180.0 / M_PI);

    updateYawCalibration(g_out.yawDeg360);

    if (g_yawOffsetSet)
    {
        g_out.relYawDeg = wrap180(g_out.yawDeg360 - g_yawOffset);

        if (!g_filteredInit)
        {
            g_filteredRelYaw = g_out.relYawDeg;
            g_filteredInit = true;
        }
        else
        {
            g_filteredRelYaw = (0.85 * g_filteredRelYaw) + (0.15 * g_out.relYawDeg);
        }

        g_out.filteredRelYawDeg = g_filteredRelYaw;
        CanApp_SetImuYaw((sint16)g_out.filteredRelYawDeg);
    }
}

static void printOutputPeriodically(void)
{
    const uint32_t now = tc375_millis();
    const uint32_t periodMs = (g_out.yawReady == true) ? READY_PRINT_PERIOD_MS : PRINT_PERIOD_MS;

    if ((uint32_t)(now - g_lastPrintMs) < periodMs)
    {
        return;
    }

    g_lastPrintMs = now;

    if (g_out.yawReady)
    {
        IMU_YAW_PRINTF("AccelAcc:%d GyroAcc:%d Roll:%.1f Pitch:%.1f Yaw:%.2f RelYaw:%.2f FilteredRelYaw:%.2f READY\r\n",
                       g_out.accelAcc,
                       g_out.gyroAcc,
                       g_out.rollDeg,
                       g_out.pitchDeg,
                       g_out.yawDeg360,
                       g_out.relYawDeg,
                       g_out.filteredRelYawDeg);
    }
    else if (g_readyTimingStarted)
    {
        IMU_YAW_PRINTF("AccelAcc:%d GyroAcc:%d Roll:%.1f Pitch:%.1f Yaw:%.2f RelYaw:CAL %.1fs\r\n",
                       g_out.accelAcc,
                       g_out.gyroAcc,
                       g_out.rollDeg,
                       g_out.pitchDeg,
                       g_out.yawDeg360,
                       ((double)(now - g_readyStartMs)) / 1000.0);
    }
    else
    {
        IMU_YAW_PRINTF("AccelAcc:%d GyroAcc:%d Roll:%.1f Pitch:%.1f Yaw:%.2f RelYaw:WAIT\r\n",
                       g_out.accelAcc,
                       g_out.gyroAcc,
                       g_out.rollDeg,
                       g_out.pitchDeg,
                       g_out.yawDeg360);
    }
}

void ImuYawApp_init(void)
{
    double savedYawOffset = 0.0;
    ICM_20948_Status_e st;

    memset(&g_out, 0, sizeof(g_out));
    g_lastAccuracyReadMs = 0u;
    g_lastPrintMs = 0u;
    g_out.accelAcc = -1;
    g_out.gyroAcc = -1;
    g_out.status = ICM_20948_Stat_Unknown;

    IMU_YAW_PRINTF("[IMU] task init start\r\n");

    g_imuYawDebugStep = 10u;
    IMU_YAW_PRINTF("[IMU] I2C init start\r\n");
    tc375_i2c_hw_init(IMU_I2C_BAUDRATE_HZ);
    IMU_YAW_PRINTF("[IMU] I2C init done\r\n");

    do
    {
        g_imuYawDebugStep = 20u;
        IMU_YAW_PRINTF("[IMU] beginI2C start\r\n");
        st = TC375_ICM20948_beginI2C(&g_imu, ICM_AD0_HIGH);
        IMU_YAW_PRINTF("[IMU] beginI2C returned\r\n");

        g_out.status = st;
        IMU_YAW_PRINTF("ICM begin: %s\r\n", TC375_ICM20948_statusString(st));

        if (st != ICM_20948_Stat_Ok)
        {
            ImuYawApp_delayMs(500u);
        }
    } while (st != ICM_20948_Stat_Ok);

    g_imuYawDebugStep = 30u;
    st = TC375_ICM20948_initializeDMP(&g_imu);

    g_out.status = st;
    IMU_YAW_PRINTF("DMP init: %s\r\n", TC375_ICM20948_statusString(st));

    if (st != ICM_20948_Stat_Ok)
    {
        while (1)
        {
            ImuYawApp_delayMs(1000u);
        }
    }

    g_imuYawDebugStep = 40u;
    st = TC375_ICM20948_applyVehicleYawDLPF(&g_imu, USE_50HZ_DLPF != 0);

    g_out.status = st;
    IMU_YAW_PRINTF("DLPF apply: %s\r\n", TC375_ICM20948_statusString(st));

    if (AppCalibStorage_loadYawOffset(&savedYawOffset) == true)
    {
        g_yawOffset = wrap360(savedYawOffset);
        g_yawOffsetSet = true;
        g_out.yawReady = true;
        g_out.calibrating = false;
        g_calState = IMU_CAL_STATE_IDLE;
        g_readyTimingStarted = false;
        g_filteredInit = false;
        g_accelAcc3Notified = false;
        g_gyroAcc3Notified = false;

        IMU_YAW_PRINTF("[IMU] Saved yawOffset loaded from DFLASH: %.2f deg\r\n", g_yawOffset);
        IMU_YAW_PRINTF("[IMU] READY. Press BUTTON1 to recalibrate.\r\n");
        AppLed2_blink(1u, 50u);
    }
    else
    {
        IMU_YAW_PRINTF("[IMU] No saved yawOffset in DFLASH. First calibration starts automatically.\r\n");
        IMU_YAW_PRINTF("[IMU] Keep sensor flat and still until AccelAcc=3 and GyroAcc=3.\r\n");
        AppLed2_blink(1u, 1000u);
        g_calibrationRequested = true;
    }

    g_imuYawDebugStep = 50u;
}

void ImuYawApp_task(void)
{
    icm_20948_DMP_data_t data;
    ICM_20948_Status_e st;

    g_imuYawDebugStep = 100u;
    st = inv_icm20948_read_dmp_data(&g_imu.dev, &data);

    g_imu.status = st;
    g_out.status = st;

    g_imuYawDebugStep = 110u;
    updateAccuracyDirectForCurrentState();

    if ((st == ICM_20948_Stat_Ok) || (st == ICM_20948_Stat_FIFOMoreDataAvail))
    {
        if ((data.header & DMP_header_bitmap_Quat6) > 0u)
        {
            g_imuYawDebugStep = 120u;
            updateEulerFromQuat6(&data);
            printOutputPeriodically();
        }
    }

    if (st != ICM_20948_Stat_FIFOMoreDataAvail)
    {
        ImuYawApp_delayMs(5u);
    }
}


void ImuYawApp_requestCalibration(void)
{
    g_calibrationRequested = true;
}

bool ImuYawApp_isCalibrationInProgress(void)
{
    return (g_calState == IMU_CAL_STATE_WAIT_ACCURACY) ||
           (g_calState == IMU_CAL_STATE_AVERAGING);
}

bool ImuYawApp_isReady(void)
{
    return g_yawOffsetSet;
}


void task_app_imu(void *arg)
{
    (void)arg;

    ImuYawApp_init();

    for (;;)
    {
        ImuYawApp_task();
    }
}

const ImuYawApp_Output_t *ImuYawApp_getOutput(void)
{
    return &g_out;
}
