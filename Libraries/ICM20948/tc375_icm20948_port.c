#include "tc375_icm20948_port.h"
#include <string.h>

/* AURIX iLLD includes. Matched to the user's MT_24AA02E48 I2C style. */
#include "Bsp.h"
#include "Ifx_Types.h"
#include "IfxI2c_I2c.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "IfxStm_reg.h"

#ifndef TC375_I2C_MAX_RETRY
#define TC375_I2C_MAX_RETRY 1000u
#endif

static IfxI2c_I2c g_i2c;
static IfxI2c_I2c_Device g_i2cDev;

/*
 * Same timing style as the user's existing MT_24AA02E48 driver.
 * No external 1 ms tick interrupt is required.
 */
uint32_t tc375_millis(void)
{
    uint64 stmTicks = IfxStm_get(&MODULE_STM0);
    uint32 stmFreq = IfxStm_getFrequency(&MODULE_STM0);

    if (stmFreq == 0u)
    {
        return 0u;
    }

    return (uint32_t)((stmTicks * 1000ull) / (uint64)stmFreq);
}

void tc375_delay_ms(uint32_t ms)
{
    wait(IfxStm_getTicksFromMilliseconds(&MODULE_STM0, ms));
}

/* I2C hardware init: aligned with your MT_24AA02E48.c driver. */
void tc375_i2c_hw_init(uint32_t baudrateHz)
{
    IfxI2c_I2c_Config i2cConfig;
    IfxI2c_I2c_initConfig(&i2cConfig, &MODULE_I2C0);

    /*
     * SCL = P02.5, SDA = P02.4, TTL pad driver.
     */
    static const IfxI2c_Pins i2cPins = {
        // .scl       = &IfxI2c0_SCL_P02_5_INOUT,
        // .sda       = &IfxI2c0_SDA_P02_4_INOUT,
        .scl       = &IfxI2c0_SCL_P15_4_INOUT,
        .sda       = &IfxI2c0_SDA_P15_5_INOUT,        
        .padDriver = IfxPort_PadDriver_ttlSpeed1
    };

    i2cConfig.pins = &i2cPins;
    i2cConfig.baudrate = (float)baudrateHz;

    /*
     * ICM-20948 register read sequence is:
     *   START - SLA+W - REG - REPEATED START - SLA+R - DATA - STOP
     *
     * So the first one-byte register write must end in MASTER RESTART,
     * not STOP. Keep stopOnPacketEnd FALSE and explicitly release the bus
     * after the whole write or read transaction is complete.
     */

    IfxI2c_I2c_initModule(&g_i2c, &i2cConfig);

    IfxI2c_I2c_deviceConfig devCfg;
    IfxI2c_I2c_initDeviceConfig(&devCfg, &g_i2c);

    /* Default. TC375_ICM20948_beginI2C updates this after AD0 is selected.
     * Your EEPROM code uses 8-bit shifted address, so this port does the same.
     */
    devCfg.deviceAddress = ICM_20948_I2C_ADDR_AD1 << 1;

    IfxI2c_I2c_initDevice(&g_i2cDev, &devCfg);
}

static void tc375_i2c_release_bus(void)
{
    /* Generate STOP after the complete I2C transaction is finished. */
    IfxI2c_releaseBus(&MODULE_I2C0);
    IfxI2c_waitBusFree(&MODULE_I2C0);
}

static ICM_20948_Status_e tc375_i2c_write(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    (void)user;

    if (len > INV_MAX_SERIAL_WRITE)
    {
        return ICM_20948_Stat_ParamErr;
    }

    uint8_t tx[INV_MAX_SERIAL_WRITE + 1u];
    tx[0] = reg;

    if (len > 0u)
    {
        memcpy(&tx[1], data, len);
    }

    /*
     * ICM-20948 register write sequence:
     *   START - SLA+W - REG - DATA... - STOP
     */
    IfxI2c_I2c_write2(&g_i2cDev, tx, (Ifx_SizeT)(len + 1u));
    tc375_i2c_release_bus();

    return ICM_20948_Stat_Ok;
}

static ICM_20948_Status_e tc375_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    (void)user;

    if (len > INV_MAX_SERIAL_READ)
    {
        return ICM_20948_Stat_ParamErr;
    }

    /*
     * ICM-20948 register read sequence:
     *   START - SLA+W - REG - REPEATED START - SLA+R - DATA... - STOP
     *
     * This depends on stopOnPacketEnd = FALSE. The register-address write
     * ends in MASTER RESTART, then read2 performs the read phase. STOP is
     * generated explicitly after the read phase.
     */
    uint8_t addrBuf = reg;
    IfxI2c_I2c_write2(&g_i2cDev, &addrBuf, 1u);
    IfxI2c_I2c_read2(&g_i2cDev, data, (Ifx_SizeT)len);
    tc375_i2c_release_bus();

    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e TC375_ICM20948_beginI2C(TC375_ICM20948_t *imu, bool ad0High)
{
    if (imu == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }

    memset(imu, 0, sizeof(*imu));
    imu->i2cAddr7 = ad0High ? ICM_20948_I2C_ADDR_AD1 : ICM_20948_I2C_ADDR_AD0;

    /* Update the iLLD device address after AD0 selection.
     * Matched to your existing code: deviceAddress = 7-bit address << 1.
     */
    g_i2cDev.deviceAddress = imu->i2cAddr7 << 1;

    ICM_20948_init_struct(&imu->dev);
    imu->serif.write = tc375_i2c_write;
    imu->serif.read = tc375_i2c_read;
    imu->serif.user = imu;
    ICM_20948_link_serif(&imu->dev, &imu->serif);

#if defined(ICM_20948_USE_DMP)
    imu->dev._dmp_firmware_available = true;
#else
    imu->dev._dmp_firmware_available = false;
#endif
    imu->dev._firmware_loaded = false;
    imu->dev._last_bank = 255;
    imu->dev._last_mems_bank = 255;

    imu->status = ICM_20948_check_id(&imu->dev);
    if (imu->status != ICM_20948_Stat_Ok) return imu->status;

    imu->status = ICM_20948_sw_reset(&imu->dev);
    if (imu->status != ICM_20948_Stat_Ok) return imu->status;
    tc375_delay_ms(50);

    imu->status = ICM_20948_sleep(&imu->dev, false);
    if (imu->status != ICM_20948_Stat_Ok) return imu->status;

    imu->status = ICM_20948_low_power(&imu->dev, false);
    if (imu->status != ICM_20948_Stat_Ok) return imu->status;

    /* 6-axis Game Rotation Vector does not require AK09916 compass output. */
    return imu->status;
}

ICM_20948_Status_e TC375_ICM20948_applyVehicleYawDLPF(TC375_ICM20948_t *imu, bool use50Hz)
{
    ICM_20948_dlpcfg_t cfg;
    if (use50Hz)
    {
        cfg.g = gyr_d51bw2_n73bw3;
        cfg.a = acc_d50bw4_n68bw8;
    }
    else
    {
        cfg.g = gyr_d23bw9_n35bw9;
        cfg.a = acc_d23bw9_n34bw4;
    }

    ICM_20948_Status_e worst = ICM_20948_Stat_Ok;
    ICM_20948_Status_e r;
    r = ICM_20948_set_dlpf_cfg(&imu->dev, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), cfg); if (r > worst) worst = r;
    r = ICM_20948_enable_dlpf(&imu->dev, ICM_20948_Internal_Acc, true); if (r > worst) worst = r;
    r = ICM_20948_enable_dlpf(&imu->dev, ICM_20948_Internal_Gyr, true); if (r > worst) worst = r;
    imu->status = worst;
    return worst;
}

ICM_20948_Status_e TC375_ICM20948_readDMP16(TC375_ICM20948_t *imu, uint16_t reg, uint16_t *value)
{
    uint8_t data[2] = {0};
    ICM_20948_Status_e st = inv_icm20948_read_mems(&imu->dev, reg, 2, data);
    if (st == ICM_20948_Stat_Ok)
    {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }
    imu->status = st;
    return st;
}

const char *TC375_ICM20948_statusString(ICM_20948_Status_e s)
{
    switch (s)
    {
        case ICM_20948_Stat_Ok: return "OK";
        case ICM_20948_Stat_Err: return "General Error";
        case ICM_20948_Stat_NotImpl: return "Not Implemented";
        case ICM_20948_Stat_ParamErr: return "Parameter Error";
        case ICM_20948_Stat_WrongID: return "Wrong ID";
        case ICM_20948_Stat_InvalSensor: return "Invalid Sensor";
        case ICM_20948_Stat_NoData: return "No Data";
        case ICM_20948_Stat_SensorNotSupported: return "Sensor Not Supported";
        case ICM_20948_Stat_DMPNotSupported: return "DMP Not Supported";
        case ICM_20948_Stat_DMPVerifyFail: return "DMP Verify Fail";
        case ICM_20948_Stat_FIFONoDataAvail: return "FIFO No Data";
        case ICM_20948_Stat_FIFOIncompleteData: return "FIFO Incomplete";
        case ICM_20948_Stat_FIFOMoreDataAvail: return "FIFO More Data";
        case ICM_20948_Stat_UnrecognisedDMPHeader: return "Unrecognised DMP Header";
        case ICM_20948_Stat_UnrecognisedDMPHeader2: return "Unrecognised DMP Header2";
        case ICM_20948_Stat_InvalDMPRegister: return "Invalid DMP Register";
        default: return "Unknown";
    }
}

ICM_20948_Status_e TC375_ICM20948_initializeDMP(TC375_ICM20948_t *imu)
{
    if ((imu == NULL) || (imu->dev._dmp_firmware_available != true))
    {
        return ICM_20948_Stat_DMPNotSupported;
    }

#if defined(ICM_20948_USE_DMP)
    ICM_20948_Status_e worst = ICM_20948_Stat_Ok;
    ICM_20948_Status_e r;

    /* This is the essential SparkFun initializeDMP sequence, converted to C API calls. */
    r = ICM_20948_i2c_master_enable(&imu->dev, true); if (r > worst) worst = r;
    r = ICM_20948_set_clock_source(&imu->dev, ICM_20948_Clock_Auto); if (r > worst) worst = r;

    r = ICM_20948_set_bank(&imu->dev, 0); if (r > worst) worst = r;
    uint8_t pwrMgmt2 = 0x40;
    r = ICM_20948_execute_w(&imu->dev, AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (r > worst) worst = r;

    r = ICM_20948_set_sample_mode(&imu->dev, ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (r > worst) worst = r;
    r = ICM_20948_enable_FIFO(&imu->dev, false); if (r > worst) worst = r;
    r = ICM_20948_enable_DMP(&imu->dev, false); if (r > worst) worst = r;

    ICM_20948_fss_t fss;
    fss.a = gpm4;
    fss.g = dps2000;
    r = ICM_20948_set_full_scale(&imu->dev, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss); if (r > worst) worst = r;
    r = ICM_20948_enable_dlpf(&imu->dev, ICM_20948_Internal_Gyr, true); if (r > worst) worst = r;

    uint8_t zero = 0;
    r = ICM_20948_set_bank(&imu->dev, 0); if (r > worst) worst = r;
    r = ICM_20948_execute_w(&imu->dev, AGB0_REG_FIFO_EN_1, &zero, 1); if (r > worst) worst = r;
    r = ICM_20948_execute_w(&imu->dev, AGB0_REG_FIFO_EN_2, &zero, 1); if (r > worst) worst = r;

    ICM_20948_INT_enable_t inten;
    memset(&inten, 0, sizeof(inten));
    inten.RAW_DATA_0_RDY_EN = 0;
    r = ICM_20948_int_enable(&imu->dev, &inten, NULL); if (r > worst) worst = r;

    r = ICM_20948_reset_FIFO(&imu->dev); if (r > worst) worst = r;

    ICM_20948_smplrt_t smplrt;
    smplrt.g = 19;
    smplrt.a = 19;
    r = ICM_20948_set_sample_rate(&imu->dev, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), smplrt); if (r > worst) worst = r;

    r = ICM_20948_set_dmp_start_address(&imu->dev, DMP_START_ADDRESS); if (r > worst) worst = r;
    r = ICM_20948_firmware_load(&imu->dev); if (r > worst) worst = r;
    r = ICM_20948_set_dmp_start_address(&imu->dev, DMP_START_ADDRESS); if (r > worst) worst = r;

    r = ICM_20948_set_bank(&imu->dev, 0); if (r > worst) worst = r;
    uint8_t fix = 0x48;
    r = ICM_20948_execute_w(&imu->dev, AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (r > worst) worst = r;
    uint8_t fifoPrio = 0xE4;
    r = ICM_20948_execute_w(&imu->dev, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (r > worst) worst = r;

    const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
    r = inv_icm20948_write_mems(&imu->dev, ACC_SCALE, 4, accScale); if (r > worst) worst = r;
    const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
    r = inv_icm20948_write_mems(&imu->dev, ACC_SCALE2, 4, accScale2); if (r > worst) worst = r;

    /* Gyro scale factor: SparkFun initializeDMP uses divider 19 and 2000 dps. */
    r = inv_icm20948_set_gyro_sf(&imu->dev, 19, 3); if (r > worst) worst = r;

    /* Enable the same DMP sensor as Arduino example: Game Rotation Vector = Quat6. */
    r = inv_icm20948_enable_dmp_sensor(&imu->dev, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, true); if (r > worst) worst = r;
    r = inv_icm20948_set_dmp_sensor_period(&imu->dev, DMP_ODR_Reg_Quat6, 0); if (r > worst) worst = r;

    r = ICM_20948_enable_FIFO(&imu->dev, true); if (r > worst) worst = r;
    r = ICM_20948_enable_DMP(&imu->dev, true); if (r > worst) worst = r;
    r = ICM_20948_reset_DMP(&imu->dev); if (r > worst) worst = r;
    r = ICM_20948_reset_FIFO(&imu->dev); if (r > worst) worst = r;

    imu->status = worst;
    return worst;
#else
    return ICM_20948_Stat_DMPNotSupported;
#endif
}
