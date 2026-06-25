/**********************************************************************************************************************
 * \file App_HallSensor.c
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "App_HallSensor.h"

#include "Apps/App_Debug/App_Debug.h"

#include "Port/Std/IfxPort.h"

#include "FreeRTOS.h"
#include "task.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define HALL_APP_UPDATE_PERIOD_MS           (1U)

/*
 * DM2246 D0 -> TC375 P40.9
 */
#define HALL_PORT                           (&MODULE_P40)
#define HALL_PIN_INDEX                      (9U)

/*
 * Wheel has 2 magnets.
 * 1 wheel revolution = 2 pulses.
 */
#define HALL_MAGNETS_PER_REV                (2U)

/*
 * Wheel circumference [mm]
 * 73cm = 730mm
 */
#define HALL_WHEEL_CIRCUMFERENCE_MM         (730U)

/*
 * Ignore too-short pulse intervals as noise/glitches.
 */
#define HALL_MIN_PULSE_INTERVAL_MS          (10U)

/*
 * If no new pulse arrives for this time, treat the vehicle as stopped.
 */
#define HALL_NO_PULSE_TIMEOUT_MS            (800U)

/*
 * Estimate decreasing speed while waiting for the next pulse.
 */
#define HALL_DECAY_ENABLE                   (1U)

/*
 * Initial estimate before the second pulse provides a real interval.
 *
 * Unit: km/h x100
 */
#define HALL_STARTUP_ESTIMATE_ENABLE        (1U)
#define HALL_STARTUP_SPEED_X100             (100U)

#define HALL_DECAY_START_PERCENT            (100U)
#define HALL_DECAY_GAIN_PERCENT             (120U)
#define HALL_CAN_SPEED_SCALE_DIVIDER        (10U)
#define HALL_CAN_SPEED_MAX                  (0xFFU)

#define HALL_DEBUG_ENABLE                   (0U)
#define HALL_DEBUG_PRINT_PERIOD_MS          (100U)

/*********************************************************************************************************************/
/*-------------------------------------------------Global Variables--------------------------------------------------*/
/*********************************************************************************************************************/
volatile uint8 g_hallVehicleSpeed = 0U;

/*********************************************************************************************************************/
/*------------------------------------------------Static Variables---------------------------------------------------*/
/*********************************************************************************************************************/
static volatile uint8  s_detected = 0U;
static volatile uint32 s_pulseCount = 0U;
static volatile uint16 s_vehicleSpeedX100 = 0U;

static boolean s_isInitialized = FALSE;
static uint8   s_prevDetected = 0U;
static uint8   s_hasPulseBase = 0U;
static uint8   s_hasValidInterval = 0U;

/*
 * Software time advanced by HallSensor_updateMs().
 */
static uint32 s_timeMs = 0U;

/*
 * Pulse interval based speed calculation state.
 */
static uint32 s_lastPulseTimeMs = 0U;
static uint32 s_lastPulseIntervalMs = 0U;
static uint32 s_lastDebugPrintTimeMs = 0U;

/*********************************************************************************************************************/
/*------------------------------------------------Debug Variables----------------------------------------------------*/
/*********************************************************************************************************************/
volatile uint8  debugHallRawLevel = 0U;
volatile uint8  debugHallDetected = 0U;
volatile uint32 debugHallTimeMs = 0U;
volatile uint32 debugHallPulseCount = 0U;
volatile uint32 debugHallLastPulseTimeMs = 0U;
volatile uint32 debugHallLastPulseIntervalMs = 0U;
volatile uint32 debugHallPulseAgeMs = 0U;
volatile uint16 debugHallVehicleSpeedX100 = 0U;
volatile uint32 debugHallIgnoredPulseCount = 0U;
volatile uint8  debugHallHasSpeed = 0U;

volatile uint16 debugHallAgedVehicleSpeedX100 = 0U;
volatile uint32 debugHallDecayCount = 0U;
volatile uint8  debugHallTimeoutZero = 0U;
volatile uint8  debugHallStartupEstimated = 0U;

/*
 * Legacy watch variables. Kept for existing debugger watch setups.
 */
volatile uint16 debugHallRawVehicleSpeed = 0U;
volatile uint16 debugHallFilteredVehicleSpeed = 0U;
volatile uint8  debugHallHasValidInterval = 0U;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
static uint8  HallSensor_readRawLevel(void);
static uint8  HallSensor_convertRawToDetected(uint8 rawLevel);
static void   HallSensor_updateDebug(uint8 rawLevel, uint8 detected);
static uint16 HallSensor_calcSpeedX100(uint32 intervalMs);
static uint8  HallSensor_convertSpeedX100ToCan(uint16 speedX100);
static void   HallSensor_updatePulseAgeAndTimeout(void);
static void   HallSensor_storeSpeed(uint16 speedX100);
static void   HallSensor_reset(void);
static void   HallSensor_updateMs(uint32 periodMs);
#if (HALL_DEBUG_ENABLE != 0U)
static void   HallSensor_printPeriodicDebug(uint8 rawLevel, uint8 detected);
#endif

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
static uint8 HallSensor_readRawLevel(void)
{
    boolean isHigh;

    isHigh = IfxPort_getPinState(HALL_PORT, HALL_PIN_INDEX);

    if (isHigh != FALSE)
    {
        return 1U;
    }

    return 0U;
}

static uint8 HallSensor_convertRawToDetected(uint8 rawLevel)
{
    /*
     * Active low:
     * raw 0 -> magnet detected
     * raw 1 -> magnet not detected
     */
    return (rawLevel == 0U) ? 1U : 0U;
}

static void HallSensor_updateDebug(uint8 rawLevel, uint8 detected)
{
    debugHallRawLevel = rawLevel;
    debugHallDetected = detected;
}

#if (HALL_DEBUG_ENABLE != 0U)
static void HallSensor_printPeriodicDebug(uint8 rawLevel, uint8 detected)
{
    if ((uint32)(s_timeMs - s_lastDebugPrintTimeMs) < HALL_DEBUG_PRINT_PERIOD_MS)
    {
        return;
    }

    s_lastDebugPrintTimeMs = s_timeMs;

    DebugLog_printf("[HALL] t:%u raw:%u det:%u prev:%u cnt:%u int:%u age:%u speedX100:%u can:%u ignored:%u\r\n",
                    (unsigned int)s_timeMs,
                    (unsigned int)rawLevel,
                    (unsigned int)detected,
                    (unsigned int)s_prevDetected,
                    (unsigned int)s_pulseCount,
                    (unsigned int)s_lastPulseIntervalMs,
                    (unsigned int)debugHallPulseAgeMs,
                    (unsigned int)s_vehicleSpeedX100,
                    (unsigned int)g_hallVehicleSpeed,
                    (unsigned int)debugHallIgnoredPulseCount);
}
#endif

static uint16 HallSensor_calcSpeedX100(uint32 intervalMs)
{
    uint32 denominator;
    uint32 speedX100;

    denominator = HALL_MAGNETS_PER_REV * intervalMs;

    if (denominator == 0U)
    {
        return 0U;
    }

    /*
     * speed[km/h] = circumference[mm] / interval[ms] / magnets * 3.6
     *
     * speedX100 = speed[km/h] * 100
     *           = circumference[mm] * 360 / (magnets * intervalMs)
     */
    speedX100 = (HALL_WHEEL_CIRCUMFERENCE_MM * 360U) / denominator;

    if (speedX100 > 0xFFFFU)
    {
        speedX100 = 0xFFFFU;
    }

    return (uint16)speedX100;
}

static uint8 HallSensor_convertSpeedX100ToCan(uint16 speedX100)
{
    uint32 speedX10;

    speedX10 = ((uint32)speedX100 + (HALL_CAN_SPEED_SCALE_DIVIDER / 2U)) / HALL_CAN_SPEED_SCALE_DIVIDER;

    if (speedX10 > HALL_CAN_SPEED_MAX)
    {
        speedX10 = HALL_CAN_SPEED_MAX;
    }

    return (uint8)speedX10;
}

static void HallSensor_updatePulseAgeAndTimeout(void)
{
    uint32 pulseAgeMs;

    if (s_hasPulseBase == 0U)
    {
        debugHallPulseAgeMs = 0U;
        return;
    }

    pulseAgeMs = s_timeMs - s_lastPulseTimeMs;
    debugHallPulseAgeMs = pulseAgeMs;

#if (HALL_DECAY_ENABLE != 0U)
    if ((s_hasValidInterval != 0U) && (s_lastPulseIntervalMs > 0U))
    {
        uint32 decayStartMs;

        decayStartMs = (s_lastPulseIntervalMs * HALL_DECAY_START_PERCENT) / 100U;

        if (decayStartMs < HALL_MIN_PULSE_INTERVAL_MS)
        {
            decayStartMs = HALL_MIN_PULSE_INTERVAL_MS;
        }

        if (pulseAgeMs > decayStartMs)
        {
            uint32 extraAgeMs;
            uint32 effectiveAgeMs;
            uint16 agedSpeedX100;

            extraAgeMs = pulseAgeMs - decayStartMs;
            effectiveAgeMs = s_lastPulseIntervalMs + ((extraAgeMs * HALL_DECAY_GAIN_PERCENT) / 100U);

            if (effectiveAgeMs < s_lastPulseIntervalMs)
            {
                effectiveAgeMs = s_lastPulseIntervalMs;
            }

            agedSpeedX100 = HallSensor_calcSpeedX100(effectiveAgeMs);
            debugHallAgedVehicleSpeedX100 = agedSpeedX100;

            if (agedSpeedX100 < s_vehicleSpeedX100)
            {
                HallSensor_storeSpeed(agedSpeedX100);
                debugHallDecayCount++;
            }
        }
    }
#endif

    if (pulseAgeMs > HALL_NO_PULSE_TIMEOUT_MS)
    {
        HallSensor_storeSpeed(0U);

        /*
         * After timeout, the next pulse becomes a new baseline.
         * This prevents a restart from using the long stopped interval.
         */
        s_hasPulseBase = 0U;
        s_hasValidInterval = 0U;
        s_lastPulseIntervalMs = 0U;

        debugHallHasSpeed = 0U;
        debugHallHasValidInterval = 0U;
        debugHallTimeoutZero = 1U;
        debugHallStartupEstimated = 0U;

#if (HALL_DEBUG_ENABLE != 0U)
        DebugLog_printf("[HALL] timeout t:%u age:%u can:0\r\n",
                        (unsigned int)s_timeMs,
                        (unsigned int)pulseAgeMs);
#endif
    }
}

static void HallSensor_storeSpeed(uint16 speedX100)
{
    s_vehicleSpeedX100 = speedX100;
    g_hallVehicleSpeed = HallSensor_convertSpeedX100ToCan(speedX100);

    debugHallVehicleSpeedX100 = speedX100;

    /*
     * Legacy aliases.
     */
    debugHallRawVehicleSpeed = speedX100;
    debugHallFilteredVehicleSpeed = speedX100;
}

static void HallSensor_reset(void)
{
    s_detected = 0U;
    s_pulseCount = 0U;
    s_vehicleSpeedX100 = 0U;

    s_prevDetected = 0U;
    s_hasPulseBase = 0U;
    s_hasValidInterval = 0U;

    s_timeMs = 0U;
    s_lastPulseTimeMs = 0U;
    s_lastPulseIntervalMs = 0U;
    s_lastDebugPrintTimeMs = 0U;

    g_hallVehicleSpeed = 0U;

    debugHallRawLevel = 0U;
    debugHallDetected = 0U;
    debugHallTimeMs = 0U;
    debugHallPulseCount = 0U;
    debugHallLastPulseTimeMs = 0U;
    debugHallLastPulseIntervalMs = 0U;
    debugHallPulseAgeMs = 0U;
    debugHallVehicleSpeedX100 = 0U;
    debugHallIgnoredPulseCount = 0U;
    debugHallHasSpeed = 0U;

    debugHallAgedVehicleSpeedX100 = 0U;
    debugHallDecayCount = 0U;
    debugHallTimeoutZero = 0U;
    debugHallStartupEstimated = 0U;

    debugHallRawVehicleSpeed = 0U;
    debugHallFilteredVehicleSpeed = 0U;
    debugHallHasValidInterval = 0U;
}

static void HallSensor_updateMs(uint32 periodMs)
{
    uint8 rawLevel;
    uint8 nowDetected;

    if (periodMs == 0U)
    {
        periodMs = 1U;
    }

    s_timeMs += periodMs;
    debugHallTimeMs = s_timeMs;

    rawLevel = HallSensor_readRawLevel();
    nowDetected = HallSensor_convertRawToDetected(rawLevel);

    HallSensor_updateDebug(rawLevel, nowDetected);

    s_detected = nowDetected;

    /*
     * Count only the transition from not-detected to detected.
     */
    if ((s_prevDetected == 0U) && (nowDetected == 1U))
    {
        uint32 nowMs;
        uint32 intervalMs;

        nowMs = s_timeMs;

        if (s_hasPulseBase == 0U)
        {
            s_pulseCount++;
            s_hasPulseBase = 1U;
            s_hasValidInterval = 0U;

            s_lastPulseTimeMs = nowMs;
            s_lastPulseIntervalMs = 0U;

#if (HALL_STARTUP_ESTIMATE_ENABLE != 0U)
            HallSensor_storeSpeed(HALL_STARTUP_SPEED_X100);
            debugHallHasSpeed = 1U;
            debugHallStartupEstimated = 1U;
#endif

            debugHallPulseCount = s_pulseCount;
            debugHallLastPulseTimeMs = s_lastPulseTimeMs;
            debugHallLastPulseIntervalMs = s_lastPulseIntervalMs;
            debugHallPulseAgeMs = 0U;
            debugHallTimeoutZero = 0U;

#if (HALL_DEBUG_ENABLE != 0U)
            DebugLog_printf("[HALL] first pulse t:%u raw:%u det:%u cnt:%u speedX100:%u can:%u\r\n",
                            (unsigned int)s_timeMs,
                            (unsigned int)rawLevel,
                            (unsigned int)nowDetected,
                            (unsigned int)s_pulseCount,
                            (unsigned int)s_vehicleSpeedX100,
                            (unsigned int)g_hallVehicleSpeed);
#endif
        }
        else
        {
            intervalMs = nowMs - s_lastPulseTimeMs;

            if (intervalMs >= HALL_MIN_PULSE_INTERVAL_MS)
            {
                uint16 speedX100;

                s_pulseCount++;
                s_lastPulseIntervalMs = intervalMs;
                s_lastPulseTimeMs = nowMs;
                s_hasValidInterval = 1U;

                speedX100 = HallSensor_calcSpeedX100(intervalMs);
                HallSensor_storeSpeed(speedX100);

                debugHallPulseCount = s_pulseCount;
                debugHallLastPulseIntervalMs = s_lastPulseIntervalMs;
                debugHallLastPulseTimeMs = s_lastPulseTimeMs;
                debugHallPulseAgeMs = 0U;
                debugHallHasSpeed = 1U;
                debugHallHasValidInterval = s_hasValidInterval;
                debugHallTimeoutZero = 0U;
                debugHallStartupEstimated = 0U;

#if (HALL_DEBUG_ENABLE != 0U)
                DebugLog_printf("[HALL] pulse t:%u raw:%u det:%u cnt:%u int:%u speedX100:%u can:%u\r\n",
                                (unsigned int)s_timeMs,
                                (unsigned int)rawLevel,
                                (unsigned int)nowDetected,
                                (unsigned int)s_pulseCount,
                                (unsigned int)s_lastPulseIntervalMs,
                                (unsigned int)s_vehicleSpeedX100,
                                (unsigned int)g_hallVehicleSpeed);
#endif
            }
            else
            {
                debugHallIgnoredPulseCount++;

#if (HALL_DEBUG_ENABLE != 0U)
                DebugLog_printf("[HALL] ignored pulse t:%u raw:%u det:%u int:%u ignored:%u\r\n",
                                (unsigned int)s_timeMs,
                                (unsigned int)rawLevel,
                                (unsigned int)nowDetected,
                                (unsigned int)intervalMs,
                                (unsigned int)debugHallIgnoredPulseCount);
#endif
            }
        }
    }

    s_prevDetected = nowDetected;

    HallSensor_updatePulseAgeAndTimeout();

#if (HALL_DEBUG_ENABLE != 0U)
    HallSensor_printPeriodicDebug(rawLevel, nowDetected);
#endif
}

void HallSensorApp_Init(void)
{
    if (s_isInitialized == TRUE)
    {
        return;
    }

    /*
     * DM2246 D0 input.
     * Existing hardware uses active-low output for magnet detection.
     */
    IfxPort_setPinModeInput(HALL_PORT,
                            HALL_PIN_INDEX,
                            IfxPort_InputMode_pullUp);

    HallSensor_reset();

#if (HALL_DEBUG_ENABLE != 0U)
    DebugLog_printf("[HALL] init port:P40.9 activeLow pullUp period:%u ms\r\n",
                    (unsigned int)HALL_APP_UPDATE_PERIOD_MS);
#endif

    s_isInitialized = TRUE;
}

void HallSensorApp_Run(void *arg)
{
    (void)arg;

    HallSensorApp_Init();

    while (1)
    {
        HallSensor_updateMs(HALL_APP_UPDATE_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(HALL_APP_UPDATE_PERIOD_MS));
    }
}
