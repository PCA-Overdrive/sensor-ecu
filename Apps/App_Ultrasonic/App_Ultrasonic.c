/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "App_Ultrasonic.h"

#include "Gtm/Std/IfxGtm.h"
#include "Gtm/Std/IfxGtm_Cmu.h"
#include "Gtm/Std/IfxGtm_Tim.h"
#include "Port/Io/IfxPort_Io.h"
#include "Stm/Std/IfxStm.h"

#include "FreeRTOS.h"
#include "task.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define ULTRASONIC_SOUND_SPEED_MM_PER_MS        (343U)

#define ULTRASONIC_TRIGGER_SETTLE_US            (2U)
#define ULTRASONIC_TRIGGER_PULSE_US             (10U)
#define ULTRASONIC_ECHO_WAIT_MS                 (20U)
#define ULTRASONIC_GUARD_MS                     (60U)
#define ULTRASONIC_SLOT_MS                      (ULTRASONIC_ECHO_WAIT_MS + ULTRASONIC_GUARD_MS)

#define ULTRASONIC_STALE_THRESHOLD_MS           (2000U)
#define ULTRASONIC_NEAR_SUSPECT_LOWER_US        (100U)
#define ULTRASONIC_NEAR_SUSPECT_UPPER_US        (300U)
#define ULTRASONIC_EMERGENCY_DISTANCE_MM        (350U)
#define ULTRASONIC_NEAR_ZONE_MM                 (800U)
#define ULTRASONIC_FAR_JUMP_GATE_MM             (500U)
#define ULTRASONIC_CONFIRM_DELTA_MM             (250U)

#define ULTRASONIC_OUT_OF_RANGE_CONFIRM_COUNT   (3U)
#define ULTRASONIC_NEAR_CONFIRM_COUNT           (2U)
#define ULTRASONIC_BAD_PUBLISH_COUNT            (2U)
#define ULTRASONIC_BAD_ERROR_COUNT              (8U)

#define ULTRASONIC_ALPHA_APPROACH_EMERGENCY     (100U)
#define ULTRASONIC_ALPHA_APPROACH_NEAR          (85U)
#define ULTRASONIC_ALPHA_APPROACH_FAR           (80U)
#define ULTRASONIC_ALPHA_RECEDING               (25U)

/*********************************************************************************************************************/
/*----------------------------------------------------Data Types-----------------------------------------------------*/
/*********************************************************************************************************************/
typedef enum
{
    ULTRASONIC_CAP_IDLE = 0,
    ULTRASONIC_CAP_ARMED,
    ULTRASONIC_CAP_DONE,
    ULTRASONIC_CAP_BAD_EDGE,
    ULTRASONIC_CAP_LATE_EDGE
} UltrasonicCaptureState;

typedef enum
{
    ULTRASONIC_SAMPLE_VALID_IN_RANGE = 0,
    ULTRASONIC_SAMPLE_VALID_OUT_OF_RANGE,
    ULTRASONIC_SAMPLE_NO_ECHO,
    ULTRASONIC_SAMPLE_NEAR_SUSPECT,
    ULTRASONIC_SAMPLE_BAD_MEASUREMENT,
    ULTRASONIC_SAMPLE_HW_ERROR
} UltrasonicSampleKind;

typedef struct
{
    Ifx_P              *port;
    uint8               pin;
    IfxGtm_Tim          tim;
    IfxGtm_Tim_Ch       timChannel;
    uint8               timInputSelect;

    Ifx_GTM_TIM_CH     *timChannelHandle;
    float32             captureClockFrequency;
    uint32              durationUs;
    UltrasonicCaptureState captureState;
} UltrasonicSensor;

typedef struct
{
    UltrasonicSampleKind kind;
    uint16               distanceMm;
    uint32               echoUs;
} UltrasonicSample;

typedef struct
{
    uint16     filteredDistance;
    uint16     lastRawDistance;
    uint16     lastPublishedValue;

    uint8      outOfRangeCount;
    uint8      badMeasurementCount;
    uint8      nearSuspectCount;
    uint8      farJumpCount;

    TickType_t lastPublishTime;
    boolean    lateEchoFlag;
    boolean    prevSlotHadLateEcho;

    boolean    initialized;
    boolean    fault;
} UltrasonicFilterState;

/*********************************************************************************************************************/
/*-------------------------------------------------Global Variables--------------------------------------------------*/
/*********************************************************************************************************************/
static UltrasonicSensor g_sensors[ULTRASONIC_SENSOR_COUNT] =
{
    /* ID order: FC, FR, RF, RM, RR, BC, RL, LM, LF, FL */
    {&MODULE_P15, 3U, IfxGtm_Tim_3, IfxGtm_Tim_Ch_6, 4U, NULL_PTR, 0.0f, 0U, ULTRASONIC_CAP_IDLE},
    {&MODULE_P15, 2U, IfxGtm_Tim_3, IfxGtm_Tim_Ch_5, 4U, NULL_PTR, 0.0f, 0U, ULTRASONIC_CAP_IDLE},
    {&MODULE_P02, 0U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_0, 2U, NULL_PTR, 0.0f, 0U, ULTRASONIC_CAP_IDLE},
    {&MODULE_P02, 1U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_1, 2U, NULL_PTR, 0.0f, 0U, ULTRASONIC_CAP_IDLE},
    {&MODULE_P10, 4U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_6, 2U, NULL_PTR, 0.0f, 0U, ULTRASONIC_CAP_IDLE},
    {&MODULE_P02, 3U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_3, 2U, NULL_PTR, 0.0f, 0U, ULTRASONIC_CAP_IDLE},
    {&MODULE_P02, 5U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_5, 1U, NULL_PTR, 0.0f, 0U, ULTRASONIC_CAP_IDLE},
    {&MODULE_P02, 4U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_4, 1U, NULL_PTR, 0.0f, 0U, ULTRASONIC_CAP_IDLE},
    {&MODULE_P02, 6U, IfxGtm_Tim_1, IfxGtm_Tim_Ch_6, 1U, NULL_PTR, 0.0f, 0U, ULTRASONIC_CAP_IDLE},
    {&MODULE_P02, 7U, IfxGtm_Tim_1, IfxGtm_Tim_Ch_7, 1U, NULL_PTR, 0.0f, 0U, ULTRASONIC_CAP_IDLE}
};

static const uint8 g_fireOrder[ULTRASONIC_SENSOR_COUNT] =
{
    (uint8)ULTRASONIC_FC,
    (uint8)ULTRASONIC_BC,
    (uint8)ULTRASONIC_FR,
    (uint8)ULTRASONIC_RL,
    (uint8)ULTRASONIC_RF,
    (uint8)ULTRASONIC_LM,
    (uint8)ULTRASONIC_RM,
    (uint8)ULTRASONIC_LF,
    (uint8)ULTRASONIC_RR,
    (uint8)ULTRASONIC_FL
};

static UltrasonicFilterState g_filterState[ULTRASONIC_SENSOR_COUNT];
static boolean g_isInitialized = FALSE;
static uint32  g_ticksPerUs = 1U;
static uint8   g_fireOrderIndex = 0U;
static boolean g_previousSlotHadLateEcho = FALSE;

volatile uint16 g_distancesMm[ULTRASONIC_SENSOR_COUNT] =
{
    ULTRASONIC_NOT_UPDATED,
    ULTRASONIC_NOT_UPDATED,
    ULTRASONIC_NOT_UPDATED,
    ULTRASONIC_NOT_UPDATED,
    ULTRASONIC_NOT_UPDATED,
    ULTRASONIC_NOT_UPDATED,
    ULTRASONIC_NOT_UPDATED,
    ULTRASONIC_NOT_UPDATED,
    ULTRASONIC_NOT_UPDATED,
    ULTRASONIC_NOT_UPDATED
};

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
static void initGtmTim(void);
static void initTimChannel(UltrasonicSensor *sensor);
static void disableTimChannel(UltrasonicSensor *sensor);
static void clearTimStatus(UltrasonicSensor *sensor);

static void setPinOutput(UltrasonicSensor *sensor);
static void setPinInput(UltrasonicSensor *sensor);
static void writePinHigh(UltrasonicSensor *sensor);
static void writePinLow(UltrasonicSensor *sensor);

static void delayUs(uint32 us);
static uint32 usToTicks(uint32 us);
static uint32 timTicksToUs(UltrasonicSensor *sensor, uint32 ticks);
static uint16 echoUsToDistanceMm(uint32 echoUs);
static boolean isPublishedDistance(uint16 value);
static uint16 sanitizePublishedValue(uint16 value);

static void publishValue(uint8 sensorId, uint16 value);
static void holdLastPublishedValue(uint8 sensorId);
static void markSensorFault(uint8 sensorId);
static void markAllSensorsFault(void);
static uint8 incrementCounter(uint8 value);

static void resetFilterState(uint8 sensorId, TickType_t now);
static uint16 moveFilteredDistance(uint16 current, uint16 raw, uint8 alphaPercent);
static void processAcceptedDistance(uint8 sensorId, uint16 rawDistanceMm);
static void processOutOfRange(uint8 sensorId);
static void processNearSuspect(uint8 sensorId, uint16 rawDistanceMm);
static void processBadMeasurement(uint8 sensorId);
static void processSample(uint8 sensorId, const UltrasonicSample *sample);

static boolean startMeasurement(uint8 sensorId);
static UltrasonicSample readMeasurementResult(UltrasonicSensor *sensor);
static void startGuardObservation(UltrasonicSensor *sensor);
static void closeGuardObservation(uint8 sensorId);
static void checkStaleSensors(void);
static boolean isDistanceSample(const UltrasonicSample *sample);
static boolean isOutOfRangeSample(const UltrasonicSample *sample);
static boolean shouldRetrySample(uint8 sensorId, const UltrasonicSample *sample);
static UltrasonicSample chooseSample(uint8 sensorId, const UltrasonicSample *first, const UltrasonicSample *second);
static boolean runSingleMeasurement(uint8 sensorId, UltrasonicSample *sample);
static void runSensorSlot(uint8 sensorId);
static void Ultrasonic_InternalInit(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
static void initGtmTim(void)
{
    IfxGtm_enable(&MODULE_GTM);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);
}

static void initTimChannel(UltrasonicSensor *sensor)
{
    IfxGtm_Tim_ChannelControl control = {0};

    sensor->timChannelHandle = IfxGtm_Tim_getChannel(&MODULE_GTM.TIM[sensor->tim], sensor->timChannel);

    IfxGtm_Tim_Ch_resetChannel(&MODULE_GTM.TIM[sensor->tim], sensor->timChannel);
    IfxGtm_Tim_Ch_setTimTin(sensor->tim, sensor->timChannel, sensor->timInputSelect);

    control.enable = TRUE;
    control.mode = IfxGtm_Tim_Mode_pwmMeasurement;
    control.channelInputControl = IfxGtm_Tim_Input_currentChannel;
    control.gpr0Sel = IfxGtm_Tim_GprSel_cnts;
    control.gpr1Sel = IfxGtm_Tim_GprSel_cnts;
    control.cntsSel = IfxGtm_Tim_CntsSel_cntReg;
    control.signalLevelControl = TRUE;
    control.clkSel = IfxGtm_Cmu_Clk_0;
    control.timeoutControl = IfxGtm_Tim_Timeout_disabled;

    IfxGtm_Tim_Ch_setControl(sensor->timChannelHandle, control);
    IfxGtm_Tim_Ch_setChannelNotification(sensor->timChannelHandle, FALSE, FALSE, FALSE, FALSE);

    sensor->captureClockFrequency = IfxGtm_Tim_Ch_getCaptureClockFrequency(&MODULE_GTM, sensor->timChannelHandle);
    sensor->captureState = ULTRASONIC_CAP_ARMED;
    clearTimStatus(sensor);
}

static void disableTimChannel(UltrasonicSensor *sensor)
{
    if (sensor->timChannelHandle != NULL_PTR)
    {
        sensor->timChannelHandle->CTRL.B.TIM_EN = 0U;
        clearTimStatus(sensor);
    }

    sensor->captureState = ULTRASONIC_CAP_IDLE;
}

static void clearTimStatus(UltrasonicSensor *sensor)
{
    if (sensor->timChannelHandle == NULL_PTR)
    {
        return;
    }

    IfxGtm_Tim_Ch_clearNewValueEvent(sensor->timChannelHandle);
    IfxGtm_Tim_Ch_clearCntOverflowEvent(sensor->timChannelHandle);
    IfxGtm_Tim_Ch_clearEcntOverflowEvent(sensor->timChannelHandle);
    IfxGtm_Tim_Ch_clearDataLostEvent(sensor->timChannelHandle);
    IfxGtm_Tim_Ch_clearGlitchEvent(sensor->timChannelHandle);
}

static void setPinOutput(UltrasonicSensor *sensor)
{
    IfxPort_setPinMode(sensor->port, sensor->pin, IfxPort_Mode_outputPushPullGeneral);
}

static void setPinInput(UltrasonicSensor *sensor)
{
    IfxPort_setPinModeInput(sensor->port, sensor->pin, IfxPort_InputMode_noPullDevice);
}

static void writePinHigh(UltrasonicSensor *sensor)
{
    IfxPort_setPinState(sensor->port, sensor->pin, IfxPort_State_high);
}

static void writePinLow(UltrasonicSensor *sensor)
{
    IfxPort_setPinState(sensor->port, sensor->pin, IfxPort_State_low);
}

static void delayUs(uint32 us)
{
    IfxStm_waitTicks(&MODULE_STM0, usToTicks(us));
}

static uint32 usToTicks(uint32 us)
{
    return us * g_ticksPerUs;
}

static uint32 timTicksToUs(UltrasonicSensor *sensor, uint32 ticks)
{
    float32 durationUs;

    if (sensor->captureClockFrequency <= 0.0f)
    {
        return 0U;
    }

    durationUs = ((float32)ticks * 1000000.0f) / sensor->captureClockFrequency;
    return (uint32)(durationUs + 0.5f);
}

static uint16 echoUsToDistanceMm(uint32 echoUs)
{
    uint32 distanceMm = ((echoUs * ULTRASONIC_SOUND_SPEED_MM_PER_MS) + 1000U) / 2000U;

    if (distanceMm > 0xFFFFU)
    {
        distanceMm = 0xFFFFU;
    }

    return (uint16)distanceMm;
}

static boolean isPublishedDistance(uint16 value)
{
    return (value <= ULTRASONIC_MAX_DISTANCE_MM) ? TRUE : FALSE;
}

static uint16 sanitizePublishedValue(uint16 value)
{
    if ((value > ULTRASONIC_MAX_DISTANCE_MM) && (value < ULTRASONIC_OUT_OF_RANGE))
    {
        return ULTRASONIC_OUT_OF_RANGE;
    }

    return value;
}

static void publishValue(uint8 sensorId, uint16 value)
{
    UltrasonicFilterState *state = &g_filterState[sensorId];
    uint16 sanitizedValue = sanitizePublishedValue(value);

    g_distancesMm[sensorId] = sanitizedValue;
    state->lastPublishedValue = sanitizedValue;
    state->lastPublishTime = xTaskGetTickCount();
}

static void holdLastPublishedValue(uint8 sensorId)
{
    UltrasonicFilterState *state = &g_filterState[sensorId];

    if (state->lastPublishedValue != ULTRASONIC_NOT_UPDATED)
    {
        publishValue(sensorId, state->lastPublishedValue);
    }
}

static void markSensorFault(uint8 sensorId)
{
    UltrasonicFilterState *state = &g_filterState[sensorId];

    state->fault = TRUE;
    state->initialized = FALSE;
    publishValue(sensorId, ULTRASONIC_ERROR);
}

static void markAllSensorsFault(void)
{
    for (uint8 i = 0U; i < ULTRASONIC_SENSOR_COUNT; i++)
    {
        markSensorFault(i);
    }
}

static uint8 incrementCounter(uint8 value)
{
    return (value < 255U) ? (uint8)(value + 1U) : value;
}

static void resetFilterState(uint8 sensorId, TickType_t now)
{
    UltrasonicFilterState *state = &g_filterState[sensorId];

    state->filteredDistance = 0U;
    state->lastRawDistance = 0U;
    state->lastPublishedValue = ULTRASONIC_NOT_UPDATED;
    state->outOfRangeCount = 0U;
    state->badMeasurementCount = 0U;
    state->nearSuspectCount = 0U;
    state->farJumpCount = 0U;
    state->lastPublishTime = now;
    state->lateEchoFlag = FALSE;
    state->prevSlotHadLateEcho = FALSE;
    state->initialized = FALSE;
    state->fault = FALSE;
}

static uint16 moveFilteredDistance(uint16 current, uint16 raw, uint8 alphaPercent)
{
    sint32 delta;
    sint32 next;
    sint32 correction;

    if (alphaPercent >= 100U)
    {
        return raw;
    }

    delta = (sint32)raw - (sint32)current;
    correction = (delta * (sint32)alphaPercent) / 100;
    next = (sint32)current + correction;

    if (next < 0)
    {
        next = 0;
    }
    else if (next > (sint32)ULTRASONIC_MAX_DISTANCE_MM)
    {
        next = (sint32)ULTRASONIC_MAX_DISTANCE_MM;
    }

    return (uint16)next;
}

static void processAcceptedDistance(uint8 sensorId, uint16 rawDistanceMm)
{
    UltrasonicFilterState *state = &g_filterState[sensorId];
    uint16 filtered;

    state->lastRawDistance = rawDistanceMm;
    state->outOfRangeCount = 0U;
    state->badMeasurementCount = 0U;
    state->nearSuspectCount = 0U;

    if (state->initialized == FALSE)
    {
        state->filteredDistance = rawDistanceMm;
        state->lastRawDistance = rawDistanceMm;
        state->farJumpCount = 0U;
        state->initialized = TRUE;
        publishValue(sensorId, rawDistanceMm);
        return;
    }

    filtered = state->filteredDistance;

    if (rawDistanceMm < filtered)
    {
        uint8 alpha;

        if (rawDistanceMm <= ULTRASONIC_EMERGENCY_DISTANCE_MM)
        {
            alpha = ULTRASONIC_ALPHA_APPROACH_EMERGENCY;
        }
        else if (rawDistanceMm <= ULTRASONIC_NEAR_ZONE_MM)
        {
            alpha = ULTRASONIC_ALPHA_APPROACH_NEAR;
        }
        else
        {
            alpha = ULTRASONIC_ALPHA_APPROACH_FAR;
        }

        state->farJumpCount = 0U;
        state->filteredDistance = moveFilteredDistance(filtered, rawDistanceMm, alpha);
        publishValue(sensorId, state->filteredDistance);
    }
    else if (rawDistanceMm > filtered)
    {
        uint16 delta = rawDistanceMm - filtered;

        if (delta > ULTRASONIC_FAR_JUMP_GATE_MM)
        {
            state->farJumpCount = incrementCounter(state->farJumpCount);

            if (state->farJumpCount < 2U)
            {
                holdLastPublishedValue(sensorId);
                return;
            }
        }
        else
        {
            state->farJumpCount = 0U;
        }

        state->filteredDistance = moveFilteredDistance(filtered, rawDistanceMm, ULTRASONIC_ALPHA_RECEDING);
        publishValue(sensorId, state->filteredDistance);
    }
    else
    {
        state->farJumpCount = 0U;
        publishValue(sensorId, filtered);
    }
}

static void processOutOfRange(uint8 sensorId)
{
    UltrasonicFilterState *state = &g_filterState[sensorId];

    state->outOfRangeCount = incrementCounter(state->outOfRangeCount);
    state->badMeasurementCount = 0U;
    state->nearSuspectCount = 0U;
    state->farJumpCount = 0U;

    if (state->lastPublishedValue == ULTRASONIC_OUT_OF_RANGE)
    {
        state->initialized = FALSE;
        publishValue(sensorId, ULTRASONIC_OUT_OF_RANGE);
        return;
    }

    if ((isPublishedDistance(state->lastPublishedValue) != FALSE) &&
        (state->outOfRangeCount < ULTRASONIC_OUT_OF_RANGE_CONFIRM_COUNT))
    {
        holdLastPublishedValue(sensorId);
        return;
    }

    state->initialized = FALSE;
    publishValue(sensorId, ULTRASONIC_OUT_OF_RANGE);
}

static void processNearSuspect(uint8 sensorId, uint16 rawDistanceMm)
{
    UltrasonicFilterState *state = &g_filterState[sensorId];

    state->lastRawDistance = rawDistanceMm;
    state->outOfRangeCount = 0U;
    state->badMeasurementCount = 0U;
    state->farJumpCount = 0U;
    state->nearSuspectCount = incrementCounter(state->nearSuspectCount);

    if ((state->prevSlotHadLateEcho != FALSE) &&
        (state->nearSuspectCount < ULTRASONIC_NEAR_CONFIRM_COUNT))
    {
        holdLastPublishedValue(sensorId);
        return;
    }

    if (state->nearSuspectCount >= ULTRASONIC_NEAR_CONFIRM_COUNT)
    {
        processAcceptedDistance(sensorId, rawDistanceMm);
    }
    else
    {
        holdLastPublishedValue(sensorId);
    }
}

static void processBadMeasurement(uint8 sensorId)
{
    UltrasonicFilterState *state = &g_filterState[sensorId];

    state->badMeasurementCount = incrementCounter(state->badMeasurementCount);
    state->outOfRangeCount = 0U;
    state->nearSuspectCount = 0U;
    state->farJumpCount = 0U;

    if (state->badMeasurementCount >= ULTRASONIC_BAD_ERROR_COUNT)
    {
        markSensorFault(sensorId);
    }
    else if (state->badMeasurementCount >= ULTRASONIC_BAD_PUBLISH_COUNT)
    {
        publishValue(sensorId, ULTRASONIC_BAD_MEASUREMENT);
    }
    else
    {
        holdLastPublishedValue(sensorId);
    }
}

static void processSample(uint8 sensorId, const UltrasonicSample *sample)
{
    UltrasonicFilterState *state = &g_filterState[sensorId];

    if (state->fault != FALSE)
    {
        publishValue(sensorId, ULTRASONIC_ERROR);
        return;
    }

    switch (sample->kind)
    {
    case ULTRASONIC_SAMPLE_VALID_IN_RANGE:
        processAcceptedDistance(sensorId, sample->distanceMm);
        break;

    case ULTRASONIC_SAMPLE_VALID_OUT_OF_RANGE:
    case ULTRASONIC_SAMPLE_NO_ECHO:
        processOutOfRange(sensorId);
        break;

    case ULTRASONIC_SAMPLE_NEAR_SUSPECT:
        processNearSuspect(sensorId, sample->distanceMm);
        break;

    case ULTRASONIC_SAMPLE_BAD_MEASUREMENT:
        processBadMeasurement(sensorId);
        break;

    case ULTRASONIC_SAMPLE_HW_ERROR:
    default:
        markSensorFault(sensorId);
        break;
    }
}

static boolean startMeasurement(uint8 sensorId)
{
    UltrasonicSensor *sensor = &g_sensors[sensorId];
    UltrasonicFilterState *state = &g_filterState[sensorId];

    if (sensor->port == NULL_PTR)
    {
        return FALSE;
    }

    sensor->durationUs = 0U;
    sensor->captureState = ULTRASONIC_CAP_IDLE;
    state->lateEchoFlag = FALSE;
    state->prevSlotHadLateEcho = g_previousSlotHadLateEcho;

    setPinOutput(sensor);
    writePinLow(sensor);
    delayUs(ULTRASONIC_TRIGGER_SETTLE_US);
    writePinHigh(sensor);
    delayUs(ULTRASONIC_TRIGGER_PULSE_US);
    writePinLow(sensor);
    setPinInput(sensor);

    initTimChannel(sensor);

    if ((sensor->timChannelHandle == NULL_PTR) || (sensor->captureClockFrequency <= 0.0f))
    {
        sensor->captureState = ULTRASONIC_CAP_BAD_EDGE;
        return FALSE;
    }

    return TRUE;
}

static UltrasonicSample readMeasurementResult(UltrasonicSensor *sensor)
{
    UltrasonicSample sample;

    sample.kind = ULTRASONIC_SAMPLE_NO_ECHO;
    sample.distanceMm = 0U;
    sample.echoUs = 0U;

    if ((sensor->timChannelHandle == NULL_PTR) || (sensor->captureClockFrequency <= 0.0f))
    {
        sample.kind = ULTRASONIC_SAMPLE_HW_ERROR;
        return sample;
    }

    if ((IfxGtm_Tim_Ch_isDataLostEvent(sensor->timChannelHandle) != FALSE) ||
        (IfxGtm_Tim_Ch_isCntOverflowEvent(sensor->timChannelHandle) != FALSE) ||
        (IfxGtm_Tim_Ch_isEcntOverflowEvent(sensor->timChannelHandle) != FALSE) ||
        (IfxGtm_Tim_Ch_isGlitchEvent(sensor->timChannelHandle) != FALSE))
    {
        sensor->captureState = ULTRASONIC_CAP_BAD_EDGE;
        sample.kind = ULTRASONIC_SAMPLE_BAD_MEASUREMENT;
        clearTimStatus(sensor);
        return sample;
    }

    if (IfxGtm_Tim_Ch_isNewValueEvent(sensor->timChannelHandle) != FALSE)
    {
        Ifx_GTM_TIM_CH_GPR0 gpr0;
        Ifx_GTM_TIM_CH_GPR1 gpr1;
        uint32 pulseLengthTicks;

        gpr0.U = sensor->timChannelHandle->GPR0.U;
        gpr1.U = sensor->timChannelHandle->GPR1.U;
        pulseLengthTicks = gpr0.B.GPR0;

        if ((pulseLengthTicks == 0U) || (gpr0.B.ECNT != gpr1.B.ECNT))
        {
            sensor->captureState = ULTRASONIC_CAP_BAD_EDGE;
            sample.kind = ULTRASONIC_SAMPLE_BAD_MEASUREMENT;
            clearTimStatus(sensor);
            return sample;
        }

        sensor->durationUs = timTicksToUs(sensor, pulseLengthTicks);
        sample.echoUs = sensor->durationUs;
        sample.distanceMm = echoUsToDistanceMm(sample.echoUs);

        if (sample.echoUs < ULTRASONIC_NEAR_SUSPECT_LOWER_US)
        {
            sample.kind = ULTRASONIC_SAMPLE_BAD_MEASUREMENT;
            sensor->captureState = ULTRASONIC_CAP_BAD_EDGE;
        }
        else if (sample.echoUs <= ULTRASONIC_NEAR_SUSPECT_UPPER_US)
        {
            sample.kind = ULTRASONIC_SAMPLE_NEAR_SUSPECT;
            sensor->captureState = ULTRASONIC_CAP_DONE;
        }
        else if (sample.distanceMm > ULTRASONIC_MAX_DISTANCE_MM)
        {
            sample.kind = ULTRASONIC_SAMPLE_VALID_OUT_OF_RANGE;
            sensor->captureState = ULTRASONIC_CAP_DONE;
        }
        else
        {
            sample.kind = ULTRASONIC_SAMPLE_VALID_IN_RANGE;
            sensor->captureState = ULTRASONIC_CAP_DONE;
        }

        clearTimStatus(sensor);
    }
    else
    {
        sensor->captureState = ULTRASONIC_CAP_IDLE;
    }

    return sample;
}

static void startGuardObservation(UltrasonicSensor *sensor)
{
    clearTimStatus(sensor);
    sensor->captureState = ULTRASONIC_CAP_ARMED;
}

static void closeGuardObservation(uint8 sensorId)
{
    UltrasonicSensor *sensor = &g_sensors[sensorId];
    UltrasonicFilterState *state = &g_filterState[sensorId];
    boolean lateEcho = FALSE;

    if (sensor->timChannelHandle != NULL_PTR)
    {
        if ((IfxGtm_Tim_Ch_isNewValueEvent(sensor->timChannelHandle) != FALSE) ||
            (IfxGtm_Tim_Ch_isDataLostEvent(sensor->timChannelHandle) != FALSE) ||
            (IfxGtm_Tim_Ch_isCntOverflowEvent(sensor->timChannelHandle) != FALSE) ||
            (IfxGtm_Tim_Ch_isEcntOverflowEvent(sensor->timChannelHandle) != FALSE) ||
            (IfxGtm_Tim_Ch_isGlitchEvent(sensor->timChannelHandle) != FALSE))
        {
            lateEcho = TRUE;
            sensor->captureState = ULTRASONIC_CAP_LATE_EDGE;
        }
    }

    state->lateEchoFlag = lateEcho;
    g_previousSlotHadLateEcho = lateEcho;
    disableTimChannel(sensor);
}

static void checkStaleSensors(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t threshold = pdMS_TO_TICKS(ULTRASONIC_STALE_THRESHOLD_MS);

    for (uint8 i = 0U; i < ULTRASONIC_SENSOR_COUNT; i++)
    {
        UltrasonicFilterState *state = &g_filterState[i];

        if ((state->fault != FALSE) || (state->lastPublishedValue == ULTRASONIC_ERROR))
        {
            continue;
        }

        if ((TickType_t)(now - state->lastPublishTime) > threshold)
        {
            state->initialized = FALSE;
            state->outOfRangeCount = 0U;
            state->badMeasurementCount = 0U;
            state->nearSuspectCount = 0U;
            state->farJumpCount = 0U;
            publishValue(i, ULTRASONIC_STALE);
        }
    }
}

static boolean isDistanceSample(const UltrasonicSample *sample)
{
    return ((sample->kind == ULTRASONIC_SAMPLE_VALID_IN_RANGE) ||
            (sample->kind == ULTRASONIC_SAMPLE_NEAR_SUSPECT)) ? TRUE : FALSE;
}

static boolean isOutOfRangeSample(const UltrasonicSample *sample)
{
    return ((sample->kind == ULTRASONIC_SAMPLE_VALID_OUT_OF_RANGE) ||
            (sample->kind == ULTRASONIC_SAMPLE_NO_ECHO)) ? TRUE : FALSE;
}

static uint16 distanceDelta(uint16 a, uint16 b)
{
    return (a >= b) ? (uint16)(a - b) : (uint16)(b - a);
}

static boolean shouldRetrySample(uint8 sensorId, const UltrasonicSample *sample)
{
    UltrasonicFilterState *state = &g_filterState[sensorId];

    if (sample->kind == ULTRASONIC_SAMPLE_HW_ERROR)
    {
        return FALSE;
    }

    if (sample->kind == ULTRASONIC_SAMPLE_BAD_MEASUREMENT)
    {
        return TRUE;
    }

    if (isDistanceSample(sample) != FALSE)
    {
        if (isPublishedDistance(state->lastPublishedValue) == FALSE)
        {
            return TRUE;
        }

        if ((state->prevSlotHadLateEcho != FALSE) && (sample->distanceMm > ULTRASONIC_EMERGENCY_DISTANCE_MM))
        {
            return TRUE;
        }

        if ((state->initialized != FALSE) &&
            (state->filteredDistance > sample->distanceMm) &&
            ((state->filteredDistance - sample->distanceMm) > ULTRASONIC_FAR_JUMP_GATE_MM) &&
            (sample->distanceMm > ULTRASONIC_EMERGENCY_DISTANCE_MM))
        {
            return TRUE;
        }
    }
    else if ((isOutOfRangeSample(sample) != FALSE) &&
             (isPublishedDistance(state->lastPublishedValue) != FALSE))
    {
        return TRUE;
    }

    return FALSE;
}

static UltrasonicSample chooseSample(uint8 sensorId, const UltrasonicSample *first, const UltrasonicSample *second)
{
    UltrasonicSample selected = *first;
    UltrasonicFilterState *state = &g_filterState[sensorId];

    if ((isDistanceSample(first) != FALSE) && (isDistanceSample(second) != FALSE))
    {
        if (distanceDelta(first->distanceMm, second->distanceMm) <= ULTRASONIC_CONFIRM_DELTA_MM)
        {
            selected.kind = ULTRASONIC_SAMPLE_VALID_IN_RANGE;
            selected.distanceMm = (uint16)(((uint32)first->distanceMm + (uint32)second->distanceMm + 1U) / 2U);
            selected.echoUs = ((first->echoUs + second->echoUs + 1U) / 2U);
            return selected;
        }

        selected.kind = ULTRASONIC_SAMPLE_BAD_MEASUREMENT;
        selected.distanceMm = 0U;
        selected.echoUs = 0U;
        return selected;
    }

    if ((isDistanceSample(first) != FALSE) && (isDistanceSample(second) == FALSE))
    {
        return *first;
    }

    if ((isDistanceSample(first) == FALSE) && (isDistanceSample(second) != FALSE))
    {
        if (first->kind == ULTRASONIC_SAMPLE_BAD_MEASUREMENT)
        {
            return *second;
        }

        if ((isOutOfRangeSample(first) != FALSE) &&
            (second->distanceMm <= ULTRASONIC_EMERGENCY_DISTANCE_MM) &&
            (isPublishedDistance(state->lastPublishedValue) != FALSE))
        {
            return *second;
        }

        return *first;
    }

    if ((isOutOfRangeSample(first) != FALSE) && (second->kind == ULTRASONIC_SAMPLE_BAD_MEASUREMENT))
    {
        return *first;
    }

    return *second;
}

static boolean runSingleMeasurement(uint8 sensorId, UltrasonicSample *sample)
{
    if (startMeasurement(sensorId) == FALSE)
    {
        return FALSE;
    }

    vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_ECHO_WAIT_MS));

    *sample = readMeasurementResult(&g_sensors[sensorId]);
    startGuardObservation(&g_sensors[sensorId]);
    vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_GUARD_MS));
    closeGuardObservation(sensorId);

    return TRUE;
}

static void runSensorSlot(uint8 sensorId)
{
    UltrasonicSample first;
    UltrasonicSample second;
    UltrasonicSample selected;

    if (g_filterState[sensorId].fault != FALSE)
    {
        publishValue(sensorId, ULTRASONIC_ERROR);
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SLOT_MS));
        return;
    }

    if (runSingleMeasurement(sensorId, &first) == FALSE)
    {
        markSensorFault(sensorId);
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SLOT_MS));
        return;
    }

    selected = first;

    if (shouldRetrySample(sensorId, &first) != FALSE)
    {
        if (runSingleMeasurement(sensorId, &second) == FALSE)
        {
            markSensorFault(sensorId);
            vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SLOT_MS));
            return;
        }

        selected = chooseSample(sensorId, &first, &second);
    }

    processSample(sensorId, &selected);
}

static void Ultrasonic_InternalInit(void)
{
    sint32 ticksPerUs;
    TickType_t now;

    if (g_isInitialized == TRUE)
    {
        return;
    }

    ticksPerUs = IfxStm_getTicksFromMicroseconds(&MODULE_STM0, 1U);
    now = xTaskGetTickCount();

    for (uint8 i = 0U; i < ULTRASONIC_SENSOR_COUNT; i++)
    {
        resetFilterState(i, now);
        g_distancesMm[i] = ULTRASONIC_NOT_UPDATED;
    }

    if (ticksPerUs <= 0)
    {
        markAllSensorsFault();
        g_isInitialized = TRUE;
        return;
    }

    g_ticksPerUs = (uint32)ticksPerUs;

    initGtmTim();

    for (uint8 i = 0U; i < ULTRASONIC_SENSOR_COUNT; i++)
    {
        UltrasonicSensor *sensor = &g_sensors[i];

        sensor->durationUs = 0U;
        sensor->captureState = ULTRASONIC_CAP_IDLE;

        setPinOutput(sensor);
        writePinLow(sensor);
        setPinInput(sensor);
        initTimChannel(sensor);
        disableTimChannel(sensor);

        if ((sensor->timChannelHandle == NULL_PTR) || (sensor->captureClockFrequency <= 0.0f))
        {
            markSensorFault(i);
        }
    }

    g_fireOrderIndex = 0U;
    g_previousSlotHadLateEcho = FALSE;
    g_isInitialized = TRUE;
}

void UltrasonicApp_Init(void)
{
    Ultrasonic_InternalInit();
}

void Ultrasonic_Run(void *arg)
{
    (void)arg;

    Ultrasonic_InternalInit();

    while (1)
    {
        uint8 sensorId = g_fireOrder[g_fireOrderIndex];

        runSensorSlot(sensorId);
        checkStaleSensors();

        g_fireOrderIndex++;

        if (g_fireOrderIndex >= ULTRASONIC_SENSOR_COUNT)
        {
            g_fireOrderIndex = 0U;
        }
    }
}

void UltrasonicApp_Run(void *arg)
{
    Ultrasonic_Run(arg);
}
