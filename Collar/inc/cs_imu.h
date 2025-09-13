/*
 * cs_imu.h
 *
 *  Created on: May 3, 2025
 *      Author: sukoonad
 */

#ifndef CS_IMU_H_
#define CS_IMU_H_


#include "sl_imu.h"
#include "stdlib.h"
#include "sl_status.h"


sl_status_t sensor_imu_enable(bool enable, float sample_rate);

sl_status_t sensor_imu_get_avec(int16_t avec[3]);

sl_status_t sensor_imu_get(int16_t ovec[3], int16_t avec[3]);

sl_status_t sensor_imu_calibrate(void);


#endif /* CS_IMU_H_ */
