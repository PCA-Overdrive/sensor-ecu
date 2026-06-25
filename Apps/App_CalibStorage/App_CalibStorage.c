#include "App_CalibStorage.h"

#include <string.h>
#include "Ifx_Types.h"
#include "IfxFlash.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"

static uint32_t calcChecksum(const CalibRecord_t *rec)
{
    const uint8_t *p;
    uint32_t sum;
    uint32_t i;

    p = (const uint8_t *)rec;
    sum = 0xA5A55A5Au;

    for (i = 0u; i < (uint32_t)offsetof(CalibRecord_t, checksum); i++)
    {
        sum = (sum << 5) ^ (sum >> 2) ^ (uint32_t)p[i];
    }

    return sum;
}

static bool recordIsValid(const CalibRecord_t *rec)
{
    if (rec == NULL_PTR)
    {
        return false;
    }

    if ((rec->magic == CALIB_ERASED_WORD) || (rec->magic != CALIB_MAGIC))
    {
        return false;
    }

    if (rec->version != CALIB_VERSION)
    {
        return false;
    }

    if (rec->checksum != calcChecksum(rec))
    {
        return false;
    }

    if ((rec->yawOffsetDeg < -360.0) || (rec->yawOffsetDeg >= 720.0))
    {
        return false;
    }

    return true;
}

static void waitDFlashReady(void)
{
    IfxFlash_waitUnbusy(FLASH_MODULE, IfxFlash_FlashType_D0);
}

static void eraseCalibSector(void)
{
    uint16 endInitSafetyPassword = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(endInitSafetyPassword);
    IfxFlash_eraseMultipleSectors(CALIB_DFLASH_ADDR, CALIB_DFLASH_SECTORS);
    IfxScuWdt_setSafetyEndinit(endInitSafetyPassword);

    waitDFlashReady();
}

static void programCalibRecord(const CalibRecord_t *rec)
{
    const uint32_t *words;
    uint32_t pageAddr;
    uint32_t page;
    uint16 endInitSafetyPassword = IfxScuWdt_getSafetyWatchdogPassword();

    words = (const uint32_t *)rec;
    pageAddr = CALIB_DFLASH_ADDR;

    for (page = 0u; page < (uint32_t)(sizeof(CalibRecord_t) / 8u); page++)
    {
        IfxFlash_enterPageMode(pageAddr);
        waitDFlashReady();

        IfxFlash_loadPage2X32(pageAddr,
                              words[(page * 2u)],
                              words[(page * 2u) + 1u]);

        IfxScuWdt_clearSafetyEndinit(endInitSafetyPassword);
        IfxFlash_writePage(pageAddr);
        IfxScuWdt_setSafetyEndinit(endInitSafetyPassword);

        waitDFlashReady();

        pageAddr += 8u;
    }    
}

bool AppCalibStorage_loadYawOffset(double *yawOffsetDeg)
{
    const CalibRecord_t *rec;

    if (yawOffsetDeg == NULL_PTR)
    {
        return false;
    }

    rec = (const CalibRecord_t *)CALIB_DFLASH_ADDR;

    if (recordIsValid(rec) == false)
    {
        return false;
    }

    *yawOffsetDeg = rec->yawOffsetDeg;
    return true;
}

bool AppCalibStorage_saveYawOffset(double yawOffsetDeg)
{
    CalibRecord_t rec;
    const CalibRecord_t *verifyRec;

    memset(&rec, 0xFF, sizeof(rec));

    rec.magic = CALIB_MAGIC;
    rec.version = CALIB_VERSION;
    rec.yawOffsetDeg = yawOffsetDeg;
    rec.reserved = 0u;
    rec.checksum = calcChecksum(&rec);

    eraseCalibSector();
    programCalibRecord(&rec);

    verifyRec = (const CalibRecord_t *)CALIB_DFLASH_ADDR;

    if (recordIsValid(verifyRec) == false)
    {
        return false;
    }

    if (verifyRec->yawOffsetDeg != yawOffsetDeg)
    {
        return false;
    }

    return true;
}

bool AppCalibStorage_clear(void)
{
    eraseCalibSector();
    return true;
}