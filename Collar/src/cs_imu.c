/*
 * cs_imu.c
 *
 *  Created on: May 3, 2025
 *      Author: sukoonad
 */

#include "cs_imu.h"

static bool initialized = false;


sl_status_t sensor_imu_enable(bool enable, float sample_rate)
{
  sl_status_t sc = SL_STATUS_OK;
  uint8_t state = sl_imu_get_state();
  if (enable && (IMU_STATE_DISABLED == state)) {
    sc = sl_imu_init();
    if (SL_STATUS_OK == sc) {
      sl_imu_configure(sample_rate);
      initialized = true;
    } else {
      initialized = false;
    }
  } else if (!enable && (IMU_STATE_READY == state)) {
    sl_imu_deinit();
  }
  return sc;
}


sl_status_t sensor_imu_get_avec(int16_t avec[3])
{
  sl_status_t sc = SL_STATUS_NOT_READY;
  if (initialized) {
    if(sl_imu_is_data_ready()) {
      sl_imu_update();
      sl_imu_get_acceleration(avec);
      sc = SL_STATUS_OK;
    }
  } else {
    sc = SL_STATUS_NOT_INITIALIZED;
  }
  return sc;
}


sl_status_t sensor_imu_get(int16_t ovec[3], int16_t avec[3])
{
  sl_status_t sc = SL_STATUS_NOT_READY;
  if (initialized) {
    if(sl_imu_is_data_ready()) {
      sl_imu_update();
      sl_imu_get_orientation(ovec);
      sl_imu_get_acceleration(avec);
      sc = SL_STATUS_OK;
    }
  } else {
    sc = SL_STATUS_NOT_INITIALIZED;
  }
  return sc;
}



sl_status_t sensor_imu_calibrate(void)
{
  if (initialized) {
    return sl_imu_calibrate_gyro();
  } else {
    return SL_STATUS_NOT_INITIALIZED;
  }
}



