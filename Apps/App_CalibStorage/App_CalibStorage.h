#ifndef APP_CALIBSTORAGE_H_
#define APP_CALIBSTORAGE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TC3xx DFLASH0 EEPROM area base is commonly mapped at 0xAF000000.
 * Use the first 4 KB logical sector by default.
 * Change this address if your project already uses DFLASH0 for something else.
 */
#define FLASH_MODULE            0
#define CALIB_DFLASH_ADDR       (0xAF000000u)
#define CALIB_DFLASH_SECTORS    (1u)
#define CALIB_MAGIC             (0x494D5531u)  /* 'IMU1' */
#define CALIB_VERSION           (1u)
#define CALIB_ERASED_WORD       (0x00000000u)

/* DFLASH programming page is 8 bytes. Keep structure multiple of 8 bytes. */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    double yawOffsetDeg;
    uint32_t checksum;
    uint32_t reserved;
} CalibRecord_t;

bool AppCalibStorage_loadYawOffset(double *yawOffsetDeg);
bool AppCalibStorage_saveYawOffset(double yawOffsetDeg);
bool AppCalibStorage_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CALIBSTORAGE_H_ */
