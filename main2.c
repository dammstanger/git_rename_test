/*
 * task.c
 *
 *  Created on: 2018Äê9ÔÂ5ÈÕ
 *      Author: xubenteng
 */

#include "../copter/task.h"

#include "gps/gps_module.h"
#include "rtk/UB482.h"

#include <barometer/baro.h>
#include <clock/clock.h>
#include <dev/adc.h>
#include <dev/config.h>
#include <dev/fc_cpu2.h>
#include <imu/adis.h>
#include <ipc/ipc.h>
#include "../library/ahrs/ahrs.h"
#include "../library/logger/data_exchange.h"
#include "../library/logger/log_write.h"
#include "../drivers/barometer/dps310.h"
#include "../drivers/magnetometer/mmc5883ma.h"
#include "../drivers/magnetometer/compass_ist8310.h"
#include "../drivers/magnetometer/Compass.h"
#include "../copter/copter.h"
#include <imu/inertial_sensor.h>

#include "../library/compasscalib/Compass_Calibration.h"
#include "../library/InertialSensor/AP_InertialSensor.h"

extern AP_InertialSensor ins;

void four_hundred_hz_task()
{
    adis_burstRead();
//    ins.poll_data();
    adis_convert();
    calc_vibration_and_clipping(0,adis_data_eng.accel,MAIN_LOOP_SEC);
    imu_calibrate();
    SW_NavEKF();
    ahrs_data_refresh();
    epwm_400 = 0;
}

void one_hundred_hz_task()
{
    static Uint8 task_cnt = 0;
    static bool mag_calib_flag_last = false;
    switch(task_cnt){
        case 0:{
//            RUN_MMC();
//            adis_data_eng.mag.x = MMC_Result.Xout * 1000.0f;
//            adis_data_eng.mag.y = -MMC_Result.Yout * 1000.0f;
//            adis_data_eng.mag.z = -MMC_Result.Zout * 1000.0f;
            task_cnt++;
            break;
        }
        case 1:{
            ist8310Read();
            adis_data_eng_t.mag.x = ist8310_data.magx * 1.0f;
            adis_data_eng_t.mag.y = ist8310_data.magy * 1.0f;
            adis_data_eng_t.mag.z = ist8310_data.magz * 1.0f;

            Vector3f mag = adis_data_eng_t.mag;
            //correct field
            compass_correct_field_without_trim(mag, ist8310_id);

            //must be called between
            compass_cal_new_sample(ist8310_id, &mag, &adis_data_eng_t.mag);

            //correct x&y trim
            compass_correct_offset_xytrim(&mag, ist8310_id);
            adis_data_eng.mag = mag;

            //compass calibrate
            compass_cal_loop(ipc_cpu1tocpu2_data.armed);

            //
            if(!mag_calib_flag_last && compass_config.mag_calib_flag && !ipc_cpu1tocpu2_data.armed){
                start_calibration_all(compass_config.calib_mode);
            }else if(mag_calib_flag_last && !compass_config.mag_calib_flag){
                cancel_calibration_all();
            }
            mag_calib_flag_last = compass_config.mag_calib_flag;
            task_cnt++;
            break;
        }
        case 2:{
//            RUN_DPS();
//            baro.altitude = DPS_Result.altitude;
            calc_baro_alt_ref();
            task_cnt++;
            break;
        }
        case 3:{
            baro_run();
            task_cnt = 0;
            break;
        }
        default:{
            task_cnt = 0;
        }

    }

    epwm_100 = 0;
}

void fifty_hz_task()
{
    epwm_50 = 0;
}

void ten_hz_task()
{
    calc_gps_alt_ref();
    epwm_10 = 0;
}

void one_hz_task()
{
    ipc_run();
    if(ipc_cpu1tocpu2_data.armed) {
//        log_start_write_flag = 1;
//        log_page0_write_flag++;
    }
    epwm_1 = 0;
}
