#ifndef APP_IMU_H_
#define APP_IMU_H_

#include <stdbool.h>
#include <stdint.h>
#include "ICM_20948_C.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TASK_PRIO_IMU       (1u)
#define TASK_STACK_IMU      (configMINIMAL_STACK_SIZE)

/* Set to 0 if UART/printf causes traps or is not needed. */
#ifndef IMU_YAW_ENABLE_PRINTF
#define IMU_YAW_ENABLE_PRINTF   1
#endif

#if IMU_YAW_ENABLE_PRINTF
#include "App_Debug.h"
#define IMU_YAW_PRINTF(...)     DebugLog_printf(__VA_ARGS__)
#else
#define IMU_YAW_PRINTF(...)     do { } while (0)
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define IMU_I2C_BAUDRATE_HZ     400000u
#define ICM_AD0_HIGH            true     /* SparkFun Qwiic default: AD0=1 -> 0x69 */
#define PRINT_PERIOD_MS         100u
#define READY_PRINT_PERIOD_MS   500u
#define READY_ACCURACY_READ_PERIOD_MS 50u
#define REQUIRED_READY_MS       2000u
#define USE_50HZ_DLPF           1

/*
 * Application layer for:
 * - ICM-20948 DMP Game Rotation Vector / Quat6
 * - yaw offset calibration after AccelAcc=3 and GyroAcc=3
 * - relative yaw filtering
 *
 * Cpu0_Main.c should stay clean. Put sensor behavior here.
 */

typedef struct
{
    int accelAcc;
    int gyroAcc;

    double rollDeg;
    double pitchDeg;
    double yawDeg360;
    double relYawDeg;
    double filteredRelYawDeg;

    bool yawReady;
    bool calibrating;

    ICM_20948_Status_e status;
} ImuYawApp_Output_t;

typedef enum
{
    IMU_CAL_STATE_IDLE = 0,
    IMU_CAL_STATE_WAIT_ACCURACY,
    IMU_CAL_STATE_AVERAGING
} ImuCalState_t;

void ImuYawApp_init(void);
void ImuYawApp_task(void);
void task_app_imu(void *arg);

void ImuYawApp_requestCalibration(void);
bool ImuYawApp_isCalibrationInProgress(void);
bool ImuYawApp_isReady(void);

const ImuYawApp_Output_t *ImuYawApp_getOutput(void);

/* Watch this in debugger to locate where init/task fails. */
extern volatile uint32_t g_imuYawDebugStep;

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_H_ */
