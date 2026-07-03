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
#define ULTRASONIC_ECHO_POLL_MS                 (1U)
#define ULTRASONIC_GUARD_MS                     (40U)
#define ULTRASONIC_SLOT_MS                      (ULTRASONIC_ECHO_WAIT_MS + ULTRASONIC_GUARD_MS)

#define ULTRASONIC_NEAR_SUSPECT_LOWER_US        (100U)
#define ULTRASONIC_NEAR_SUSPECT_UPPER_US        (300U)
#define ULTRASONIC_STALE_THRESHOLD_MS           (2000U)

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
    uint16     lastPublishedValue;

    TickType_t lastPublishTime;
    boolean    fault;
} UltrasonicSensorState;

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

static UltrasonicSensorState g_sensorState[ULTRASONIC_SENSOR_COUNT];
static boolean g_isInitialized = FALSE;
static uint32  g_ticksPerUs = 1U;
static uint8   g_fireOrderIndex = 0U;

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
static uint16 sanitizePublishedValue(uint16 value);

static void publishValue(uint8 sensorId, uint16 value);
static void markSensorFault(uint8 sensorId);
static void markAllSensorsFault(void);

static void resetSensorState(uint8 sensorId, TickType_t now);
static void processSample(uint8 sensorId, const UltrasonicSample *sample);

static boolean startMeasurement(uint8 sensorId);
static UltrasonicSample readMeasurementResult(UltrasonicSensor *sensor);
static void startGuardObservation(UltrasonicSensor *sensor);
static void closeGuardObservation(uint8 sensorId);
static void checkStaleSensors(void);
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
    UltrasonicSensorState *state = &g_sensorState[sensorId];
    uint16 sanitizedValue = sanitizePublishedValue(value);

    g_distancesMm[sensorId] = sanitizedValue;
    state->lastPublishedValue = sanitizedValue;
    state->lastPublishTime = xTaskGetTickCount();
}

static void markSensorFault(uint8 sensorId)
{
    UltrasonicSensorState *state = &g_sensorState[sensorId];

    state->fault = TRUE;
    publishValue(sensorId, ULTRASONIC_ERROR);
}

static void markAllSensorsFault(void)
{
    for (uint8 i = 0U; i < ULTRASONIC_SENSOR_COUNT; i++)
    {
        markSensorFault(i);
    }
}

static void resetSensorState(uint8 sensorId, TickType_t now)
{
    UltrasonicSensorState *state = &g_sensorState[sensorId];

    state->lastPublishedValue = ULTRASONIC_NOT_UPDATED;
    state->lastPublishTime = now;
    state->fault = FALSE;
}

static void processSample(uint8 sensorId, const UltrasonicSample *sample)
{
    UltrasonicSensorState *state = &g_sensorState[sensorId];

    if (state->fault != FALSE)
    {
        publishValue(sensorId, ULTRASONIC_ERROR);
        return;
    }

    switch (sample->kind)
    {
    case ULTRASONIC_SAMPLE_VALID_IN_RANGE:
    case ULTRASONIC_SAMPLE_NEAR_SUSPECT:
        publishValue(sensorId, sample->distanceMm);
        break;

    case ULTRASONIC_SAMPLE_VALID_OUT_OF_RANGE:
    case ULTRASONIC_SAMPLE_NO_ECHO:
        publishValue(sensorId, ULTRASONIC_OUT_OF_RANGE);
        break;

    case ULTRASONIC_SAMPLE_BAD_MEASUREMENT:
        publishValue(sensorId, ULTRASONIC_BAD_MEASUREMENT);
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

    if (sensor->port == NULL_PTR)
    {
        return FALSE;
    }

    sensor->durationUs = 0U;
    sensor->captureState = ULTRASONIC_CAP_IDLE;

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

    disableTimChannel(sensor);
}

static void checkStaleSensors(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t threshold = pdMS_TO_TICKS(ULTRASONIC_STALE_THRESHOLD_MS);

    for (uint8 i = 0U; i < ULTRASONIC_SENSOR_COUNT; i++)
    {
        UltrasonicSensorState *state = &g_sensorState[i];

        if ((state->fault != FALSE) || (state->lastPublishedValue == ULTRASONIC_ERROR))
        {
            continue;
        }

        if ((TickType_t)(now - state->lastPublishTime) > threshold)
        {
            publishValue(i, ULTRASONIC_STALE);
        }
    }
}

static boolean runSingleMeasurement(uint8 sensorId, UltrasonicSample *sample)
{
    uint32 elapsedMs = 0U;

    if (startMeasurement(sensorId) == FALSE)
    {
        return FALSE;
    }

    do
    {
        uint32 waitMs = ULTRASONIC_ECHO_POLL_MS;

        if ((elapsedMs + waitMs) > ULTRASONIC_ECHO_WAIT_MS)
        {
            waitMs = ULTRASONIC_ECHO_WAIT_MS - elapsedMs;
        }

        vTaskDelay(pdMS_TO_TICKS(waitMs));
        elapsedMs += waitMs;

        *sample = readMeasurementResult(&g_sensors[sensorId]);

        if (sample->kind != ULTRASONIC_SAMPLE_NO_ECHO)
        {
            break;
        }
    } while (elapsedMs < ULTRASONIC_ECHO_WAIT_MS);

    startGuardObservation(&g_sensors[sensorId]);

    return TRUE;
}

static void waitGuardObservation(uint8 sensorId)
{
    vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_GUARD_MS));
    closeGuardObservation(sensorId);
}

static void runSensorSlot(uint8 sensorId)
{
    UltrasonicSample sample;

    if (g_sensorState[sensorId].fault != FALSE)
    {
        publishValue(sensorId, ULTRASONIC_ERROR);
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SLOT_MS));
        return;
    }

    if (runSingleMeasurement(sensorId, &sample) == FALSE)
    {
        markSensorFault(sensorId);
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SLOT_MS));
        return;
    }

    processSample(sensorId, &sample);
    waitGuardObservation(sensorId);
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
        resetSensorState(i, now);
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
