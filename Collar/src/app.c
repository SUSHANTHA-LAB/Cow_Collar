/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "sl_common.h"
#include "sl_bt_api.h"
#include "app_assert.h"
#include "app.h"

#include "gatt_db.h"

//#include "app_log.h"

#include "sl_sleeptimer.h"


#include "cs_imu.h"
#include "cs_temp.h"
#include "cs_adc.h"

#include "em_rmu.h"
#include "em_wdog.h"
#include "em_cmu.h"


#define SAMPLE_IMU  1
#define SAMPLE_TEMP 2
#define CLOSE_CONNECTION  3


// Legacy adv interval milliseconds*1.6
#define LEGACY_ADV_INT  1000 * 1.6

// Extended adv interval milliseconds*16
#define EXTENDED_ADV_INT 1000 * 1.6

// periodic adv interval milliseconds*0.8
#define PERIODIC_ADV_INT 1000 * 0.8

// > 10 Hz as the imu is sampled every 100 ms
#define IMU_SAMPLE_RATE 15.0f //Hz

// IMU sample time in ms
#define IMU_SAMPLE_TIME  100

// RTH and batter voltage sampling
#define RTH_BAT_SAMPLE_TIME 30000



// acceleration vector from IMU
int16_t avec[3];

// temp and himidity data
uint32_t relh;
int32_t temp;


// Date time info
uint8_t date_time_buffer[6];
sl_sleeptimer_date_t date_time;


// Cow ID
uint8_t cow_id;


//Fist sample Boolean
bool first_sample = true;


// IMU buffer variables
uint8_t imu_buffer[186];
uint8_t imu_index = 0;
uint8_t imu_offset = 0;


// Cow tag struct
typedef struct  cow_tag{
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t battery;
  uint8_t temp;
  uint8_t cow_id;

}cow_t;

cow_t cow_data;



// Time handles and callbacks
sl_sleeptimer_timer_handle_t connection_close_timer;

void connection_close_callback(sl_sleeptimer_timer_handle_t *handle, void *data);

sl_sleeptimer_timer_handle_t imu_sample_handle;

void imu_sample_callback(sl_sleeptimer_timer_handle_t *handle, void *data);

sl_sleeptimer_timer_handle_t rht_sample_handle;

void rht_sample_callback(sl_sleeptimer_timer_handle_t *handle, void *data);


// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// Connection handle
uint8_t connection_handle = 0xff;


/**************************************************************************//**
 * @brief Watchdog initialization
 *****************************************************************************/
void initWDOG(void)
{
  CMU_ClockEnable(cmuClock_WDOG0, true);

  WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;
  CMU_ClockSelectSet(cmuClock_WDOG0, cmuSelect_ULFRCO);
  wdogInit.debugRun = true;
  wdogInit.perSel = wdogPeriod_2k;

  WDOGn_Init(WDOG0, &wdogInit);
}



// Application Init.
void app_init(void)
{

  sl_status_t sc;



  uint32_t resetCause;

  resetCause = RMU_ResetCauseGet();
  RMU_ResetCauseClear();


  // IMU sensor enable
  sc = sensor_imu_enable(true, IMU_SAMPLE_RATE);
  app_assert_status(sc);



  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

// Application Process Action.
void app_process_action(void)
{

}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  int16_t result;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // set max tx power
      sc = sl_bt_advertiser_set_tx_power(advertising_set_handle, 60, &result);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        LEGACY_ADV_INT, // min. adv. interval (milliseconds * 1.6)
        LEGACY_ADV_INT, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
//      app_log("Connection opened \r\n");

      connection_handle = evt->data.evt_connection_opened.connection;

      break;


    // Event that GATT values have been written
    case sl_bt_evt_gatt_server_attribute_value_id:

      size_t data_len;

      if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_date_time){

          sc = sl_bt_gatt_server_read_attribute_value(gattdb_date_time, 0, sizeof(date_time_buffer), &data_len, date_time_buffer);
          app_assert_status(sc);

          // configuring date & time
//          app_log("data len %d \r\n", data_len);
//          app_log("YMDHMS data %d:%d:%d:%d:%d:%d \r\n",date_time_buffer[0],date_time_buffer[1],date_time_buffer[2],date_time_buffer[3],date_time_buffer[4],date_time_buffer[5]);

          date_time.year = (2000 + date_time_buffer[0]) - 1900;
          date_time.month = date_time_buffer[1] - 1;
          date_time.month_day = date_time_buffer[2];

          date_time.hour = date_time_buffer[3];
          date_time.min = date_time_buffer[4];
          date_time.sec = date_time_buffer[5];

          sc = sl_sleeptimer_set_datetime(&date_time);
          app_assert_status(sc);


      }else if((evt->data.evt_gatt_server_attribute_value.attribute == gattdb_cow_id)){

          // TODO - write this data to NVM

          sc = sl_bt_gatt_server_read_attribute_value(gattdb_cow_id, 0, sizeof(gattdb_cow_id), &data_len, &cow_id);
          app_assert_status(sc);

//          app_log("data len %d \r\n", data_len);
//          app_log("cow id data %d \r\n", cow_id);

          sc = sl_sleeptimer_start_timer_ms(&connection_close_timer, 2000, connection_close_callback, (void*)NULL, 0, 0);


          app_assert_status(sc);

      }

      break;


    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
//      app_log("Connection closed start extended advertising & IMU sampling\r\n");

      // set extended adv timing
      sc = sl_bt_advertiser_set_timing(advertising_set_handle,
                                       EXTENDED_ADV_INT,
                                       EXTENDED_ADV_INT,
                                       0,
                                       0);
      app_assert_status(sc);


//      sl_bt_gap_phy_1m    = 0x1,  /**< (0x1) 1M PHY */
//      sl_bt_gap_phy_2m    = 0x2,  /**< (0x2) 2M PHY */
//      sl_bt_gap_phy_coded = 0x4,  /**< (0x4) Coded PHY, 125k (S=8) or 500k (S=2) */
      sc = sl_bt_extended_advertiser_set_phy(advertising_set_handle,sl_bt_gap_phy_coded, sl_bt_gap_phy_coded );
      app_assert_status(sc);

      // Start general advertising
      sc = sl_bt_extended_advertiser_generate_data(advertising_set_handle,
                                                   sl_bt_advertiser_general_discoverable);

      app_assert_status(sc);

      // start extened advertising
      sc = sl_bt_extended_advertiser_start(advertising_set_handle,
                                           sl_bt_extended_advertiser_non_connectable,
                                           SL_BT_EXTENDED_ADVERTISER_INCLUDE_TX_POWER );

      app_assert_status(sc);

      // timer every 100ms to sample IMU
      sc = sl_sleeptimer_start_periodic_timer_ms(&imu_sample_handle, IMU_SAMPLE_TIME , imu_sample_callback, (void*)NULL, 0, 0);


      app_assert_status(sc);

      // timer to update RHT and hour/sec 5 min = 60000*5
      sc = sl_sleeptimer_start_periodic_timer_ms(&rht_sample_handle, RTH_BAT_SAMPLE_TIME, rht_sample_callback, (void*)NULL, 0, 0);


      app_assert_status(sc);


      break;



    case sl_bt_evt_system_external_signal_id:
      if(evt->data.evt_system_external_signal.extsignals == CLOSE_CONNECTION){
          // close the connection after time&date received
          sc = sl_bt_connection_close(connection_handle);
          app_assert_status(sc);

          connection_handle = 0xff;


          initWDOG();

      }else if(evt->data.evt_system_external_signal.extsignals ==  SAMPLE_IMU){

          WDOGn_Feed(WDOG0);

          // Sample IMU accleration vector
          sc = sensor_imu_get_avec(avec);
          //app_assert_status(sc);
          if(sc == SL_STATUS_NOT_READY){
              memset(avec, 0, sizeof(avec));
          }


          if(imu_index > 29){

              if(first_sample){

                  // Set IMU index back to zero
                  imu_index = 0;

                  sc = sl_sleeptimer_get_datetime(&date_time);
                  app_assert_status(sc);

                  sc = sl_sensor_rht_init();
                  app_assert_status(sc);

                  adc_init();

                  sc = sl_sensor_rht_get(&relh, &temp);
                  app_assert_status(sc);
//
//                  app_log("Humidity = %d %%RH" APP_LOG_NL, (uint8_t)(relh / 1000.0f));
//                  app_log("Temperature = %d C" APP_LOG_NL, (uint8_t)(temp / 1000.0f));


                  sl_sensor_rht_deinit();

                  cow_data.cow_id = cow_id;
                  cow_data.battery = (uint8_t)get_adc_sample();
                  cow_data.hour = date_time.hour;
                  cow_data.min = date_time.min;
                  cow_data.sec = date_time.sec;
                  cow_data.temp = (uint8_t)(temp / 1000.0f);

                  adc_deinit();

                  memcpy(&imu_buffer[180], &cow_data, sizeof(cow_data));


                  sc = sl_bt_periodic_advertiser_set_data(advertising_set_handle, sizeof(imu_buffer), imu_buffer);
                  app_assert_status(sc);

                  sc = sl_bt_periodic_advertiser_start(advertising_set_handle, PERIODIC_ADV_INT, PERIODIC_ADV_INT,
                                                       SL_BT_PERIODIC_ADVERTISER_AUTO_START_EXTENDED_ADVERTISING);
                  app_assert_status(sc);


                  memset(imu_buffer, 0, sizeof(imu_buffer));


                  first_sample = false;
              }else{

              // Set IMU index back to zero
              imu_index = 0;

              sc = sl_sleeptimer_get_datetime(&date_time);
              app_assert_status(sc);

              cow_data.min = date_time.min;
              cow_data.sec = date_time.sec;

              memcpy(&imu_buffer[180], &cow_data, sizeof(cow_data));


              sc = sl_bt_periodic_advertiser_set_data(advertising_set_handle, sizeof(imu_buffer), imu_buffer);
              app_assert_status(sc);

              memset(imu_buffer, 0, sizeof(imu_buffer));

              }

          }else{
              imu_offset = imu_index * 6;

              //app_log("%d,%d,%d\r\n", avec[0], avec[1], avec[2]);

              memcpy(&imu_buffer[imu_offset],avec,sizeof(avec));

              imu_index++;
          }


      }else if(evt->data.evt_system_external_signal.extsignals ==  SAMPLE_TEMP){

            sc = sl_sleeptimer_get_datetime(&date_time);
            app_assert_status(sc);

            sc = sl_sensor_rht_init();
            app_assert_status(sc);

            adc_init();

            sc = sl_sensor_rht_get(&relh, &temp);
            app_assert_status(sc);

//            app_log("Humidity = %d %%RH" APP_LOG_NL, (uint8_t)(relh / 1000.0f));
//            app_log("Temperature = %d C" APP_LOG_NL, (uint8_t)(temp / 1000.0f));


            cow_data.cow_id = cow_id;
            cow_data.hour = date_time.hour;
            cow_data.temp = (uint8_t)(temp / 1000.0f);
            cow_data.battery = (uint8_t)get_adc_sample();


            sl_sensor_rht_deinit();

            adc_deinit();

            //sl_sleeptimer_delay_millisecond(50);
      }

      break;
    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

void rht_sample_callback(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)handle;
  (void)data;

  sl_bt_external_signal(SAMPLE_TEMP);
}

void imu_sample_callback(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)handle;
  (void)data;

  sl_bt_external_signal(SAMPLE_IMU);
}


void connection_close_callback(sl_sleeptimer_timer_handle_t *handle, void *data){
  (void)handle;
  (void)data;

  sl_bt_external_signal(CLOSE_CONNECTION);
}

