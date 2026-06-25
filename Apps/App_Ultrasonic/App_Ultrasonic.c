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
#define ULTRASONIC_SOUND_SPEED_MM_PER_MS    (343U)
#define ULTRASONIC_MIN_DISTANCE_MM          (10U)
#define ULTRASONIC_MAX_DISTANCE_MM          (5000U)

#define ULTRASONIC_TRIGGER_SETTLE_US        (2U)
#define ULTRASONIC_TRIGGER_PULSE_US         (12U)
#define ULTRASONIC_ECHO_TIMEOUT_US          (((ULTRASONIC_MAX_DISTANCE_MM * 2000U) / \
                                              ULTRASONIC_SOUND_SPEED_MM_PER_MS) + 1000U)
#define ULTRASONIC_SENSOR_GUARD_US          (10000U)
#define ULTRASONIC_SENSOR_SLOT_US           (ULTRASONIC_ECHO_TIMEOUT_US + ULTRASONIC_SENSOR_GUARD_US)
#define ULTRASONIC_NO_ECHO_CONFIRM_COUNT    (2U)

/*********************************************************************************************************************/
/*----------------------------------------------------Data Types-----------------------------------------------------*/
/*********************************************************************************************************************/
typedef enum
{
    ULTRASONIC_MEASURE_IDLE = 0,
    ULTRASONIC_MEASURE_WAIT_ECHO,
    ULTRASONIC_MEASURE_INTER_SENSOR_DELAY
} UltrasonicMeasureState;

typedef struct
{
    Ifx_P        *port;
    uint8         pin;
    IfxGtm_Tim    tim;
    IfxGtm_Tim_Ch timChannel;
    uint8         timInputSelect;

    Ifx_GTM_TIM_CH  *timChannelHandle;
    float32          captureClockFrequency;
    uint32           durationUs;
    uint16           distanceMm;
} UltrasonicSensor;

/*********************************************************************************************************************/
/*-------------------------------------------------Global Variables--------------------------------------------------*/
/*********************************************************************************************************************/
static UltrasonicSensor g_sensors[ULTRASONIC_SENSOR_COUNT] =
{
    /* Measurement order: 0, 5, 1, 6, 2, 7, 3, 8, 4, 9 */
    /* Sensor 0 */ {&MODULE_P15, 3U, IfxGtm_Tim_3, IfxGtm_Tim_Ch_6, 4U, NULL_PTR, 0.0f, 0U, (uint16)ULTRASONIC_MAX_DISTANCE_MM},
    /* Sensor 5 */ {&MODULE_P02, 3U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_3, 2U, NULL_PTR, 0.0f, 0U, (uint16)ULTRASONIC_MAX_DISTANCE_MM},
    /* Sensor 1 */ {&MODULE_P15, 2U, IfxGtm_Tim_3, IfxGtm_Tim_Ch_5, 4U, NULL_PTR, 0.0f, 0U, (uint16)ULTRASONIC_MAX_DISTANCE_MM},
    /* Sensor 6 */ {&MODULE_P02, 5U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_5, 1U, NULL_PTR, 0.0f, 0U, (uint16)ULTRASONIC_MAX_DISTANCE_MM},
    /* Sensor 2 */ {&MODULE_P02, 0U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_0, 2U, NULL_PTR, 0.0f, 0U, (uint16)ULTRASONIC_MAX_DISTANCE_MM},
    /* Sensor 7 */ {&MODULE_P02, 4U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_4, 1U, NULL_PTR, 0.0f, 0U, (uint16)ULTRASONIC_MAX_DISTANCE_MM},
    /* Sensor 3 */ {&MODULE_P02, 1U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_1, 2U, NULL_PTR, 0.0f, 0U, (uint16)ULTRASONIC_MAX_DISTANCE_MM},
    /* Sensor 8 */ {&MODULE_P02, 6U, IfxGtm_Tim_1, IfxGtm_Tim_Ch_6, 1U, NULL_PTR, 0.0f, 0U, (uint16)ULTRASONIC_MAX_DISTANCE_MM},
    /* Sensor 4 */ {&MODULE_P10, 4U, IfxGtm_Tim_0, IfxGtm_Tim_Ch_6, 2U, NULL_PTR, 0.0f, 0U, (uint16)ULTRASONIC_MAX_DISTANCE_MM},
    /* Sensor 9 */ {&MODULE_P02, 7U, IfxGtm_Tim_1, IfxGtm_Tim_Ch_7, 1U, NULL_PTR, 0.0f, 0U, (uint16)ULTRASONIC_MAX_DISTANCE_MM}
};

static boolean g_isInitialized = FALSE;
static uint32  g_ticksPerUs = 1U;
static uint8   g_activeSensorIndex = 0U;
static uint32  g_stateStartTicks = 0U;
uint16 g_distances[ULTRASONIC_SENSOR_COUNT] =
{
    (uint16)ULTRASONIC_MAX_DISTANCE_MM,
    (uint16)ULTRASONIC_MAX_DISTANCE_MM,
    (uint16)ULTRASONIC_MAX_DISTANCE_MM,
    (uint16)ULTRASONIC_MAX_DISTANCE_MM,
    (uint16)ULTRASONIC_MAX_DISTANCE_MM,
    (uint16)ULTRASONIC_MAX_DISTANCE_MM,
    (uint16)ULTRASONIC_MAX_DISTANCE_MM,
    (uint16)ULTRASONIC_MAX_DISTANCE_MM,
    (uint16)ULTRASONIC_MAX_DISTANCE_MM,
    (uint16)ULTRASONIC_MAX_DISTANCE_MM
};
static uint8 g_noEchoCount[ULTRASONIC_SENSOR_COUNT] =
{
    ULTRASONIC_NO_ECHO_CONFIRM_COUNT,
    ULTRASONIC_NO_ECHO_CONFIRM_COUNT,
    ULTRASONIC_NO_ECHO_CONFIRM_COUNT,
    ULTRASONIC_NO_ECHO_CONFIRM_COUNT,
    ULTRASONIC_NO_ECHO_CONFIRM_COUNT,
    ULTRASONIC_NO_ECHO_CONFIRM_COUNT,
    ULTRASONIC_NO_ECHO_CONFIRM_COUNT,
    ULTRASONIC_NO_ECHO_CONFIRM_COUNT,
    ULTRASONIC_NO_ECHO_CONFIRM_COUNT,
    ULTRASONIC_NO_ECHO_CONFIRM_COUNT
};
static UltrasonicMeasureState g_measureState = ULTRASONIC_MEASURE_IDLE;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
static void initGtmTim(void);
static void initTimChannel(UltrasonicSensor *sensor);
static void clearTimStatus(UltrasonicSensor *sensor);

static void setPinOutput(UltrasonicSensor *sensor);
static void setPinInput(UltrasonicSensor *sensor);
static void writePinHigh(UltrasonicSensor *sensor);
static void writePinLow(UltrasonicSensor *sensor);

static void delayUs(uint32 us);
static uint32 usToTicks(uint32 us);
static uint32 timTicksToUs(UltrasonicSensor *sensor, uint32 ticks);
static boolean isTimeElapsedUs(uint32 startTicks, uint32 timeoutUs);
static boolean isDistanceUpdateValid(uint16 distanceMm);

static uint16 echoUsToDistanceMm(uint32 echoUs);
static void acceptDistance(uint8 index, uint16 distanceMm);
static void handleNoEcho(uint8 index);
static void startMeasurement(UltrasonicSensor *sensor);
static void finishMeasurement(void);
static void updateMeasurement(void);

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
    clearTimStatus(sensor);
}

static void clearTimStatus(UltrasonicSensor *sensor)
{
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

static boolean isTimeElapsedUs(uint32 startTicks, uint32 timeoutUs)
{
    uint32 timeoutTicks = usToTicks(timeoutUs);

    return ((IfxStm_getLower(&MODULE_STM0) - startTicks) >= timeoutTicks) ? TRUE : FALSE;
}

static boolean isDistanceUpdateValid(uint16 distanceMm)
{
    return ((distanceMm > 0U) && (distanceMm != ULTRASONIC_INVALID_DISTANCE_MM)) ? TRUE : FALSE;
}

static uint16 echoUsToDistanceMm(uint32 echoUs)
{
    uint32 distanceMm = ((echoUs * ULTRASONIC_SOUND_SPEED_MM_PER_MS) + 1000U) / 2000U;

    if ((distanceMm < ULTRASONIC_MIN_DISTANCE_MM) || (distanceMm > ULTRASONIC_MAX_DISTANCE_MM))
    {
        return ULTRASONIC_INVALID_DISTANCE_MM;
    }

    return (uint16)distanceMm;
}

static void acceptDistance(uint8 index, uint16 distanceMm)
{
    g_noEchoCount[index] = 0U;
    g_distances[index] = distanceMm;
    g_sensors[index].distanceMm = distanceMm;
}

static void handleNoEcho(uint8 index)
{
    if (g_noEchoCount[index] < 255U)
    {
        g_noEchoCount[index]++;
    }

    if (g_noEchoCount[index] >= ULTRASONIC_NO_ECHO_CONFIRM_COUNT)
    {
        g_distances[index] = (uint16)ULTRASONIC_MAX_DISTANCE_MM;
        g_sensors[index].distanceMm = (uint16)ULTRASONIC_MAX_DISTANCE_MM;
    }
}

static void startMeasurement(UltrasonicSensor *sensor)
{
    sensor->durationUs = 0U;
    g_stateStartTicks = IfxStm_getLower(&MODULE_STM0);

    setPinOutput(sensor);
    writePinLow(sensor);
    delayUs(ULTRASONIC_TRIGGER_SETTLE_US);
    writePinHigh(sensor);
    delayUs(ULTRASONIC_TRIGGER_PULSE_US);
    writePinLow(sensor);
    setPinInput(sensor);

    /* Arm TIM after the driven trigger pulse so only the echo pulse is measured. */
    initTimChannel(sensor);

    g_measureState = ULTRASONIC_MEASURE_WAIT_ECHO;
}

static void finishMeasurement(void)
{
    g_measureState = ULTRASONIC_MEASURE_INTER_SENSOR_DELAY;
}

static void updateMeasurement(void)
{
    UltrasonicSensor *sensor = &g_sensors[g_activeSensorIndex];
    uint32 pulseLengthTicks;
    uint16 distanceMm;

    switch (g_measureState)
    {
    case ULTRASONIC_MEASURE_IDLE:
        startMeasurement(sensor);
        break;

    case ULTRASONIC_MEASURE_WAIT_ECHO:
        if (IfxGtm_Tim_Ch_isNewValueEvent(sensor->timChannelHandle) != FALSE)
        {
            pulseLengthTicks = sensor->timChannelHandle->GPR0.B.GPR0;
            sensor->durationUs = timTicksToUs(sensor, pulseLengthTicks);
            clearTimStatus(sensor);

            distanceMm = echoUsToDistanceMm(sensor->durationUs);

            if (isDistanceUpdateValid(distanceMm) != FALSE)
            {
                acceptDistance(g_activeSensorIndex, distanceMm);
                finishMeasurement();
            }
            else
            {
                /*
                * invalid pulse는 no echo가 아님.
                * 잔향/글리치/잘못된 캡처일 수 있으니 거리값 유지.
                * 아직 timeout 전이면 실제 echo를 더 기다린다.
                */
                if (isTimeElapsedUs(g_stateStartTicks, ULTRASONIC_ECHO_TIMEOUT_US) != FALSE)
                {
                    handleNoEcho(g_activeSensorIndex);
                    finishMeasurement();
                }
            }
        }
        else if ((IfxGtm_Tim_Ch_isDataLostEvent(sensor->timChannelHandle) != FALSE) ||
         (IfxGtm_Tim_Ch_isCntOverflowEvent(sensor->timChannelHandle) != FALSE))
        {
            clearTimStatus(sensor);

            /*
            * dataLost/overflow는 하드웨어 캡처 이상이지,
            * 장애물이 없다는 뜻이 아님.
            * 바로 max로 보내면 튐이 커진다.
            */
            if (isTimeElapsedUs(g_stateStartTicks, ULTRASONIC_ECHO_TIMEOUT_US) != FALSE)
            {
                handleNoEcho(g_activeSensorIndex);
                finishMeasurement();
            }
        }
        else if (isTimeElapsedUs(g_stateStartTicks, ULTRASONIC_ECHO_TIMEOUT_US) != FALSE)
        {
            clearTimStatus(sensor);
            sensor->durationUs = 0U;
            handleNoEcho(g_activeSensorIndex);
            finishMeasurement();
        }
        break;

    case ULTRASONIC_MEASURE_INTER_SENSOR_DELAY:
        if (isTimeElapsedUs(g_stateStartTicks, ULTRASONIC_SENSOR_SLOT_US) != FALSE)
        {
            g_activeSensorIndex++;

            if (g_activeSensorIndex >= ULTRASONIC_SENSOR_COUNT)
            {
                g_activeSensorIndex = 0U;
            }

            g_measureState = ULTRASONIC_MEASURE_IDLE;
        }
        break;

    default:
        g_measureState = ULTRASONIC_MEASURE_IDLE;
        break;
    }
}

void UltrasonicApp_Init(void)
{
    sint32 ticksPerUs = IfxStm_getTicksFromMicroseconds(&MODULE_STM0, 1U);

    if (g_isInitialized == TRUE)
    {
        return;
    }

    if (ticksPerUs > 0)
    {
        g_ticksPerUs = (uint32)ticksPerUs;
    }

    initGtmTim();

    for (uint8 i = 0U; i < ULTRASONIC_SENSOR_COUNT; i++)
    {
        g_sensors[i].durationUs = 0U;
        g_sensors[i].distanceMm = (uint16)ULTRASONIC_MAX_DISTANCE_MM;
        g_distances[i] = (uint16)ULTRASONIC_MAX_DISTANCE_MM;
        g_noEchoCount[i] = ULTRASONIC_NO_ECHO_CONFIRM_COUNT;

        setPinOutput(&g_sensors[i]);
        writePinLow(&g_sensors[i]);
        setPinInput(&g_sensors[i]);
        initTimChannel(&g_sensors[i]);
    }

    g_activeSensorIndex = 0U;
    g_stateStartTicks = IfxStm_getLower(&MODULE_STM0);
    g_measureState = ULTRASONIC_MEASURE_IDLE;
    g_isInitialized = TRUE;
}

void UltrasonicApp_Run(void *arg)
{
    (void)arg;

    UltrasonicApp_Init();

    while (1)
    {
        updateMeasurement();
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}
