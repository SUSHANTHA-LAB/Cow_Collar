/*
 * cs_temp.h
 *
 *  Created on: May 3, 2025
 *      Author: sukoonad
 */

#ifndef CS_TEMP_H_
#define CS_TEMP_H_


sl_status_t sl_sensor_rht_init(void);

void sl_sensor_rht_deinit(void);

sl_status_t sl_sensor_rht_get(uint32_t *rh, int32_t *t);



#endif /* CS_TEMP_H_ */
