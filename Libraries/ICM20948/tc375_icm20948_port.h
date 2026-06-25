#ifndef TC375_ICM20948_PORT_H_
#define TC375_ICM20948_PORT_H_

#include <stdint.h>
#include <stdbool.h>
#include "ICM_20948_C.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TC375 Lite Kit V2 + SparkFun ICM-20948 C-driver port layer.
 *
 * Edit tc375_i2c_hw_init() in tc375_icm20948_port.c to match the SDA/SCL pins
 * you physically wired on the TC375 Lite Kit V2.
 */

typedef struct
{
    ICM_20948_Device_t dev;
    ICM_20948_Serif_t serif;
    uint8_t i2cAddr7;      /* 0x68 if AD0=0, 0x69 if AD0=1 */
    ICM_20948_Status_e status;
} TC375_ICM20948_t;

void tc375_delay_ms(uint32_t ms);
uint32_t tc375_millis(void);

void tc375_i2c_hw_init(uint32_t baudrateHz);

ICM_20948_Status_e TC375_ICM20948_beginI2C(TC375_ICM20948_t *imu, bool ad0High);
ICM_20948_Status_e TC375_ICM20948_initializeDMP(TC375_ICM20948_t *imu);

ICM_20948_Status_e TC375_ICM20948_applyVehicleYawDLPF(TC375_ICM20948_t *imu, bool use50Hz);
ICM_20948_Status_e TC375_ICM20948_readDMP16(TC375_ICM20948_t *imu, uint16_t reg, uint16_t *value);
const char *TC375_ICM20948_statusString(ICM_20948_Status_e s);

#ifdef __cplusplus
}
#endif

#endif
