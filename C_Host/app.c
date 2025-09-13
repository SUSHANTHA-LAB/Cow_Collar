
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "app.h"
#include "gatt_db.h"
#include "ncp_host.h"
#include "app_log.h"
#include "app_log_cli.h"
#include "app_assert.h"
#include "sl_bt_api.h"

// Optstring argument for getopt.
#define OPTSTRING NCP_HOST_OPTSTRING APP_LOG_OPTSTRING "hR"

// Usage info.
#define USAGE APP_LOG_NL "%s " NCP_HOST_USAGE APP_LOG_USAGE " [-h]" APP_LOG_NL

// Options info.
#define OPTIONS                                  \
  "\nOPTIONS\n" NCP_HOST_OPTIONS APP_LOG_OPTIONS \
  "    -h  Print this help message.\n"



#define COW_ID 0x01 // Example cow ID, replace with actual value if needed

/*Main states */
#define DISCONNECTED    0
#define SCANNING        1
#define FIND_SERVICE    2
#define FIND_CHAR       3
#define WRITE_DATA      4
#define PA_SYNC         5
#define DISCONNECTING   6

#define UUID_LEN        16

// UUIDs (converted to little-endian as per BLE spec)
static const uint8_t serviceUUID[UUID_LEN] = {
  0x79, 0x2d, 0xf6, 0x66, 0x9c, 0x19, 0xca, 0x84,
  0xc4, 0x45, 0xcd, 0x93, 0x5f, 0x14, 0x98, 0x86
};

static const uint8_t char1_UUID[UUID_LEN] = {
  0x33, 0xd0, 0x83, 0xd1, 0x99, 0x50, 0x50, 0xa5,
  0x59, 0x44, 0x9d, 0x42, 0x9c, 0x05, 0xb5, 0xb6
};

static const uint8_t char2_UUID[UUID_LEN] = {
  0x61, 0xaa, 0x30, 0x57, 0x9e, 0x60, 0x25, 0x82,
  0x7c, 0x4f, 0x73, 0x0f, 0xdd, 0x4a, 0xab, 0x52
};

/*******************************************************************************
 *    Local Variables
 ******************************************************************************/
static uint8_t conn_handle = 0xFF;
static uint8_t main_state;

static uint32_t service_handle;
static uint16_t char1_handle;
static uint16_t char2_handle;
static uint16_t sync_id;

static uint8_t date_time[6];
static uint8_t cow_id;

static bool conn_close_flag = false;
static bool reprov_flag = false;

static uint8_t prev_id_data[6] = {0};
static FILE *csv_file = NULL;

static timer_t connection_close_timer;
static timer_t reprov_timer;



// ─────────────────────────────────────────────────────────────────────────────
// Helper Functions
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Start a POSIX timer
 */
void start_timer(timer_t *timerid, int timeout_ms, void (*callback)(union sigval))
{
  struct sigevent sev = {
    .sigev_notify = SIGEV_THREAD,
    .sigev_value.sival_ptr = NULL,
    .sigev_notify_function = callback,
    .sigev_notify_attributes = NULL
  };

  struct itimerspec its = {
    .it_value = {
      .tv_sec = timeout_ms / 1000,
      .tv_nsec = (timeout_ms % 1000) * 1000000
    },
    .it_interval = {0, 0}
  };

  timer_create(CLOCK_REALTIME, &sev, timerid);
  timer_settime(*timerid, 0, &its, NULL);
}

/**
 * Log the current local time and store it into the date_time buffer.
 */
void get_local_time()
{
  time_t rawtime;
  struct tm *timeinfo;

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  date_time[0] = (uint8_t)(timeinfo->tm_year + 1900 - 2000); // Year
  date_time[1] = (uint8_t)(timeinfo->tm_mon + 1);            // Month
  date_time[2] = (uint8_t)(timeinfo->tm_mday);               // Day
  date_time[3] = (uint8_t)(timeinfo->tm_hour);               // Hour
  date_time[4] = (uint8_t)(timeinfo->tm_min);                // Minute
  date_time[5] = (uint8_t)(timeinfo->tm_sec);                // Second

  app_log("Local time: %s", asctime(timeinfo));
}



// Parse advertisements looking for advertised periodicSync Service.
static uint8_t parse_adv(uint8_t *data, uint8_t len)
{
  uint8_t adFieldLength;
  uint8_t adFieldType;
  char name[32];

  uint8_t i = 0;

  app_log("packet length %d\r\n", len);

  // Parse advertisement packet
  while (i < len)
  {
    adFieldLength = data[i];
    adFieldType = data[i + 1];
    // Partial ($02) or complete ($03) list of 128-bit UUIDs
    app_log("adField type %d \r\n", adFieldType);
    if ((adFieldType == 0x08) || (adFieldType == 0x09))
    {
      // Type 0x08 = Shortened Local Name
      // Type 0x09 = Complete Local Name
      memcpy(name, &(data[i + 2]), adFieldLength - 1);
      name[adFieldLength - 1] = 0;
      app_log("%s\r\n", name);
    }
    if (adFieldType == 0x06 || adFieldType == 0x07)
    {
      // compare UUID to service UUID
      if (memcmp(&data[i + 2], serviceUUID, 16) == 0)
      {
        return 1;
      }
    }
    // advance to the next AD struct
    i = i + adFieldLength + 1;
  }
  return 0;
}

static void process_periodic_sync_report(const sl_bt_evt_periodic_sync_report_t *report)
{
  uint8_t buffer[191];
  memcpy(buffer, report->data.data, report->data.len);

  int16_t *values = (int16_t *)buffer;

  uint8_t id_data[6];
  memcpy(id_data, &buffer[180], 6);

  // Check for new ID data
  if (memcmp(id_data, prev_id_data, 6) != 0) {
    memcpy(prev_id_data, id_data, 6);

    if (csv_file) {
      fprintf(csv_file, "%d,%d,%d,%d,%d,%d,", id_data[0], id_data[1], id_data[2], id_data[3], id_data[4], id_data[5]);

      for (int i = 0; i < 90; i += 3) {
        fprintf(csv_file, "%d,%d,%d,", values[i], values[i + 1], values[i + 2]);
      }

      fprintf(csv_file, "%d,%d\n", report->counter, report->rssi);
      fflush(csv_file);
    }
  }

  app_log("Counter: %d\r\n", report->counter);
}


void connection_close_callback(union sigval arg);

void reprov_callback(union sigval arg);


/**************************************************************************/ /**
 * Application Init.
*****************************************************************************/
void app_init(int argc, char *argv[])
{
  sl_status_t sc;
  int opt;

  // Process command line options.
  while ((opt = getopt(argc, argv, OPTSTRING)) != -1)
  {
    switch (opt)
    {
    // Print help.
    case 'h':
      app_log(USAGE, argv[0]);
      app_log(OPTIONS);
      exit(EXIT_SUCCESS);

    case 'R':
      app_log("Deprecated option: -R" APP_LOG_NL);
      break;

    // Process options for other modules.
    default:
      sc = ncp_host_set_option((char)opt, optarg);
      if (sc == SL_STATUS_NOT_FOUND)
      {
        sc = app_log_set_option((char)opt, optarg);
      }
      if (sc != SL_STATUS_OK)
      {
        app_log(USAGE, argv[0]);
        exit(EXIT_FAILURE);
      }
      break;
    }
  }

  // Initialize NCP connection.
  sc = ncp_host_init();
  if (sc == SL_STATUS_INVALID_PARAMETER)
  {
    app_log(USAGE, argv[0]);
    exit(EXIT_FAILURE);
  }
  app_assert_status(sc);
  app_log_info("NCP host initialised." APP_LOG_NL);
  app_log_info("Press Crtl+C to quit" APP_LOG_NL APP_LOG_NL);

  get_local_time();

  cow_id = COW_ID;

  csv_file = fopen("ble_data_log.csv", "a");
  if (csv_file && ftell(csv_file) == 0)
  {
    // Write header if file is new
    fprintf(csv_file, "ID,ID,ID,ID,ID,ID,Values->,Counter,RSSI\n");
    fflush(csv_file);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************/ /**
                                                                              * Application Process Action.
                                                                              *****************************************************************************/
void app_process_action(void)
{
  if (conn_close_flag) {
    conn_close_flag = false;

    app_assert_status(sl_bt_scanner_stop());

    app_assert_status(sl_bt_scanner_set_parameters(sl_bt_scanner_scan_mode_passive, 160, 160));
    app_assert_status(sl_bt_sync_scanner_set_sync_parameters(0, 6000, sl_bt_sync_report_all));
    app_assert_status(sl_bt_scanner_start(sl_bt_scanner_scan_phy_coded, sl_bt_scanner_discover_observation));

  } else if (reprov_flag) {
    reprov_flag = false;

    app_assert_status(sl_bt_scanner_stop());

    app_assert_status(sl_bt_scanner_set_parameters(sl_bt_scanner_scan_mode_passive, 160, 160));
    app_assert_status(sl_bt_sync_scanner_set_sync_parameters(0, 6000, sl_bt_sync_report_all));
    app_assert_status(sl_bt_scanner_start(sl_bt_scanner_scan_phy_1m_and_coded, sl_bt_scanner_discover_observation));
  }
}

/**************************************************************************/ /**
                                                                              * Application Deinit.
                                                                              *****************************************************************************/
void app_deinit(void)
{

  if (csv_file)
  {
    fclose(csv_file);
    csv_file = NULL;
  }

  ncp_host_deinit();

}



/******************************************************************************
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  uint16_t max_mtu_out;

  switch (SL_BT_MSG_ID(evt->header))
  {
  // -------------------------------
  // This event indicates the device has started and the radio is ready.
  // Do not call any stack command before receiving this boot event!
  case sl_bt_evt_system_boot_id:

    app_log("BLE Client \r\n");

    sl_bt_gatt_server_set_max_mtu(247, &max_mtu_out);

    sl_bt_scanner_start(sl_bt_scanner_scan_phy_1m_and_coded, sl_bt_scanner_discover_generic);

    main_state = SCANNING;

    break;

  case sl_bt_evt_scanner_legacy_advertisement_report_id:

    if (parse_adv(&(evt->data.evt_scanner_legacy_advertisement_report.data.data[0]), evt->data.evt_scanner_legacy_advertisement_report.data.len) != 0)
    {

      sc = sl_bt_connection_open(
          evt->data.evt_scanner_legacy_advertisement_report.address,
          evt->data.evt_scanner_legacy_advertisement_report.address_type,
          sl_bt_gap_1m_phy,
          &conn_handle);

      if (SL_STATUS_OK == sc)
      {
        sl_bt_scanner_stop();
      }
    }
    break;

  case sl_bt_evt_scanner_extended_advertisement_report_id:

    if (parse_adv(&(evt->data.evt_scanner_extended_advertisement_report.data.data[0]), evt->data.evt_scanner_extended_advertisement_report.data.len) != 0)
    {
      app_log("Found periodic sync service, attempting to open sync\r\n");
      sc = sl_bt_sync_scanner_open(evt->data.evt_scanner_extended_advertisement_report.address,
                                   evt->data.evt_scanner_extended_advertisement_report.address_type,
                                   evt->data.evt_scanner_extended_advertisement_report.adv_sid,
                                   &sync_id);

      app_log_info("cmd_sync_open() sync = 0x%4X\r\n", sc);
    }
    break;

  case sl_bt_evt_connection_opened_id:

    app_log("Connection opened!\r\n");

    main_state = FIND_SERVICE;

    sl_bt_gatt_discover_primary_services_by_uuid(conn_handle,
                                                 16,
                                                 serviceUUID);

    break;

  case sl_bt_evt_connection_closed_id:

    sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);

    main_state = SCANNING;

    break;

  case sl_bt_evt_connection_parameters_id:

    app_log("Conn.parameters: interval %u units\r\n",
            evt->data.evt_connection_parameters.interval);

    break;

  case sl_bt_evt_gatt_service_id:

    if (evt->data.evt_gatt_service.uuid.len == 16)
    {
      if (memcmp(serviceUUID, evt->data.evt_gatt_service.uuid.data, 16) == 0)
      {
        app_log("Service discovered\r\n");
        service_handle = evt->data.evt_gatt_service.service;
      }
    }
    break;

  case sl_bt_evt_gatt_procedure_completed_id:

    switch (main_state)
    {
    case FIND_SERVICE:
      if (service_handle > 0)
      {
        // Service found, next step: search for characteristics
        sl_bt_gatt_discover_characteristics(conn_handle, service_handle);
        main_state = FIND_CHAR;
      }
      else
      {
        // Service is not found: disconnect
        app_log("SPP service not found!\r\n");
        sl_bt_connection_close(conn_handle);
      }
      break;

    case FIND_CHAR:
      if (char1_handle > 0 && char2_handle > 0)
      {

        uint16_t sent_len = 0;

        get_local_time();

        sc = sl_bt_gatt_write_characteristic_value_without_response(conn_handle, char1_handle, sizeof(date_time), date_time, &sent_len);
        if (SL_STATUS_OK == sc)
        {
          app_assert_status(sc);
        }

        sc = sl_bt_gatt_write_characteristic_value_without_response(conn_handle, char2_handle, sizeof(cow_id), &cow_id, &sent_len);
        if (SL_STATUS_OK == sc)
        {
          app_assert_status(sc);
        }

        start_timer(&connection_close_timer, 2000, connection_close_callback);

        main_state = WRITE_DATA;
      }
      else
      {
        // Characteristic is not found: disconnect
        app_log("Char not found closing connection\r\n");
        sl_bt_connection_close(conn_handle);
      }
      break;

    default:
      break;
    }
    break;

  case sl_bt_evt_gatt_characteristic_id:
    if (evt->data.evt_gatt_characteristic.uuid.len == 16)
    {
      if (memcmp(char1_UUID, evt->data.evt_gatt_characteristic.uuid.data, 16) == 0)
      {
        app_log("Char 1 discovered\r\n");
        char1_handle = evt->data.evt_gatt_characteristic.characteristic;
      }
      else if (memcmp(char2_UUID, evt->data.evt_gatt_characteristic.uuid.data, 16) == 0)
      {
        app_log("Char 2 discovered\r\n");
        char2_handle = evt->data.evt_gatt_characteristic.characteristic;
      }
    }
    break;

  case sl_bt_evt_periodic_sync_opened_id:
    /* now that sync is open, we can stop scanning*/
    app_log("evt_sync_opened\r\n");

    app_log("sync %d, adv_phy: %d , interval: %d \r\n", evt->data.evt_periodic_sync_opened.sync, evt->data.evt_periodic_sync_opened.adv_phy, evt->data.evt_periodic_sync_opened.adv_interval);

    sl_bt_scanner_stop();

    main_state = PA_SYNC;

    break;

  case sl_bt_evt_sync_closed_id:

    app_log("periodic sync closed. reason 0x%2X, sync handle %d",
            evt->data.evt_sync_closed.reason,
            evt->data.evt_sync_closed.sync);

    /* restart discovery */
    sl_bt_scanner_start(sl_bt_scanner_scan_phy_coded,
                        sl_bt_scanner_discover_observation);

    main_state = DISCONNECTED;

    start_timer(&reprov_timer, 20000, reprov_callback);

    break;

  case sl_bt_evt_periodic_sync_report_id:

    process_periodic_sync_report(&evt->data.evt_periodic_sync_report);

    break;

  // -------------------------------
  // Default event handler.
  default:
    break;
  }
}

void connection_close_callback(union sigval arg)
{
  // Simulate sl_bt_external_signal(WRITE_DATA);
  conn_close_flag = 1;
}

void reprov_callback(union sigval arg)
{
  // Simulate sl_bt_external_signal(SCANNING);
  if (main_state == DISCONNECTED)
  {
    reprov_flag = 1;
  }
}
