/*
 * adis.h
 *
 * Copyright (C) Hefei Sunwin Intelligent Co., Ltd. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 *  Created on: 2016Äê4ÔÂ15ÈÕ
 *      Author: Qiuyang Wang <wangqiuyang@siwill.com>
 */
#ifndef SW_FC_HW04_CPU2_NEW_DRIVERS_ADIS_H_
#define SW_FC_HW04_CPU2_NEW_DRIVERS_ADIS_H_

#include "../../library/math/AP_Math.h"

typedef struct{
    Uint32 timestamp;
    Uint16 resv;
    Uint16 status;
    int16 gyro[3];
    int16 accel[3];
    int16 temp;
    int16 time_stmp;
    Uint16 checksum;
}ADISDataRaw;

typedef struct{
    Uint32 timestamp;
    float32 gyro[3];
    float32 accel[3];
    Vector3f mag;
    float32 baro;
    float32 altitude;
    float32 temp;
}ADISDataEng;

typedef enum{
    ADIS_FLASH_CNT      = 0x0000,  //Flash memory write count
    ADIS_DIAG_STAT      = 0x0200,  //Diagnostic and operational status
    ADIS_X_GYRO_LOW     = 0x0400,  //X-axis gyroscope output, lower word
    ADIS_X_GYRO_OUT     = 0x0600,  //X-axis gyroscope output, upper word
    ADIS_Y_GYRO_LOW     = 0x0800,  //Y-axis gyroscope output, lower word
    ADIS_Y_GYRO_OUT     = 0x0A00,  //Y-axis gyroscope output, upper word
    ADIS_Z_GYRO_LOW     = 0x0C00,  //Z-axis gyroscope output, lower word
    ADIS_Z_GYRO_OUT     = 0x0E00,  //Z-axis gyroscope output, upper word
    ADIS_X_ACCL_LOW     = 0x1000,  //X-axis accelerometer output, lower word
    ADIS_X_ACCL_OUT     = 0x1200,  //X-axis accelerometer output, upper word
    ADIS_Y_ACCL_LOW     = 0x1400,  //Y-axis accelerometer output, lower word
    ADIS_Y_ACCL_OUT     = 0x1600,  //Y-axis accelerometer output, upper word
    ADIS_Z_ACCL_LOW     = 0x1800,  //Z-axis accelerometer output, lower word
    ADIS_Z_ACCL_OUT     = 0x1A00,  //Z-axis accelerometer output, upper word
    ADIS_TEMP_OUT       = 0x1C00,  //Temperature output (internal, not calibrated)
    ADIS_TIME_STAMP     = 0x1E00,  //PPS mode time stamp
    ADIS_DATA_CNTR      = 0x2200,  //New data counter
    ADIS_X_DELTANG_LOW  = 0x2400,  //X-axis delta angle output, lower word
    ADIS_X_DELTANG_OUT  = 0x2600,  //X-axis delta angle output, upper word
    ADIS_Y_DELTANG_LOW  = 0x2800,  //Y-axis delta angle output, lower word
    ADIS_Y_DELTANG_OUT  = 0x2A00,  //Y-axis delta angle output, upper word
    ADIS_Z_DELTANG_LOW  = 0x2C00,  //Z-axis delta angle output, lower word
    ADIS_Z_DELTANG_OUT  = 0x2E00,  //Z-axis delta angle output, upper word
    ADIS_X_DELTVEL_LOW  = 0x3000,  //X-axis delta velocity output, lower word
    ADIS_X_DELTVEL_OUT  = 0x3200,  //X-axis delta velocity output, upper word
    ADIS_Y_DELTVEL_LOW  = 0x3400,  //Y-axis delta velocity output, lower word
    ADIS_Y_DELTVEL_OUT  = 0x3600,  //Y-axis delta velocity output, upper word
    ADIS_Z_DELTVEL_LOW  = 0x3800,  //Z-axis delta velocity output, lower word
    ADIS_Z_DELTVEL_OUT  = 0x3A00,  //Z-axis delta velocity output, upper word
    ADIS_XG_BIAS_LOW    = 0x4000,  //X-axis gyroscope bias offset correction, lower word
    ADIS_XG_BIAS_HIGH   = 0x4200,  //X-axis gyroscope bias offset correction, upper word
    ADIS_YG_BIAS_LOW    = 0x4400,  //Y-axis gyroscope bias offset correction, lower word
    ADIS_YG_BIAS_HIGH   = 0x4600,  //Y-axis gyroscope bias offset correction, upper word
    ADIS_ZG_BIAS_LOW    = 0x4800,  //Z-axis gyroscope bias offset correction, lower word
    ADIS_ZG_BIAS_HIGH   = 0x4A00,  //Z-axis gyroscope bias offset correction, upper word
    ADIS_XA_BIAS_LOW    = 0x4C00,  //X-axis accelerometer bias offset correction, lower word
    ADIS_XA_BIAS_HIGH   = 0x4E00,  //X-axis accelerometer bias offset correction, upper word
    ADIS_YA_BIAS_LOW    = 0x5000,  //Y-axis accelerometer bias offset correction, lower word
    ADIS_YA_BIAS_HIGH   = 0x5200,  //Y-axis accelerometer bias offset correction, upper word
    ADIS_ZA_BIAS_LOW    = 0x5400,  //Z-axis accelerometer bias offset correction, lower word
    ADIS_ZA_BIAS_HIGH   = 0x5600,  //Z-axis accelerometer bias offset correction, upper word
    ADIS_FILT_CTRL      = 0x5C00,  //Filter control
    ADIS_MSC_CTRL       = 0x6000,  //Miscellaneous control
    ADIS_UP_SCALE       = 0x6200,  //Clock scale factor, PPS mode
    ADIS_DEC_RATE       = 0x6400,  //Decimation rate control (output data rate)
    ADIS_NULL_CFG       = 0x6600,  //Auto-null configuration control
    ADIS_GLOB_CMD       = 0x6800,  //Global commands
    ADIS_FIRM_REV       = 0x6C00,  //Firmware revision
    ADIS_FIRM_DM        = 0x6E00,  //Firmware revision date, month and day
    ADIS_FIRM_Y         = 0x7000,  //Firmware revision date, year
    ADIS_PROD_ID        = 0x7200,  //Product identification
    ADIS_SERIAL_NUM     = 0x7400,  //Serial number (relative to assembly lot)
    ADIS_USER_SCR1      = 0x7600,  //User scratch register 1
    ADIS_USER_SCR2      = 0x7800,  //User scratch register 2
    ADIS_USER_SCR3      = 0x7A00,  //User scratch register 3
    ADIS_FLSHCNT_LOW    = 0x7C00,  //Flash update count, lower word
    ADIS_FLSHCNT_HIGH   = 0x7E00   //Flash update count, upper word
}AdisRegister;

extern ADISDataRaw adis_data_raw;
extern ADISDataEng adis_data_eng;
extern ADISDataEng adis_data_eng_t;

Uint16 adis_reset();
Uint16 adis_hardware_reset();
Uint16 adis_hardware_reset_noRTOS();
Uint16 adis_selfTest();
Uint16 adis_regRead(Uint16 addr);
Uint16 adis_regWrite(Uint16 addr, Uint16 data);
Uint16 adis_burstRead();
bool adis_burstRead(Uint16 *buf);
void adis_convert();
void adis_regDefault(void);
Uint16 adis_checksum(Uint16 * burstArray);


#endif /* SW_FC_HW04_CPU2_NEW_DRIVERS_ADIS_H_ */
