# 🐮 Cow Collar BLE Host Example

This repository contains the `app.c` file for the **Cow Collar Bluetooth Low Energy (BLE) host application** and collar application.  
It is designed to run as the host-side logic for a **Silicon Labs Bluetooth NCP (Network Co-Processor)** device, communicating with a BLE-enabled cow collar.

---

## 🚀 Features

- **🔍 BLE Scanning**  
  Detects advertisements from cow collars broadcasting a specific service UUID.

- **🔗 Connection & Service Discovery**  
  Connects to collars and discovers services and characteristics.

- **📤 Date/Time & Cow ID Write**  
  Sends current date/time and cow ID to the collar via BLE GATT writes.

- **📡 Periodic Advertising Sync**  
  Synchronizes with periodic advertisements for structured sensor data collection.

- **📝 Data Logging**  
  Logs sensor values, cow ID, counter, and RSSI to a CSV file (`ble_data_log.csv`).  
  Only new data (based on cow ID) is written to avoid duplicates.

- **⏱️ POSIX Timers**  
  Uses Linux POSIX timers to simulate Silicon Labs sleeptimer functionality.

---

## ⚙️ Usage

> ⚠️ This repository only contains the `app.c` file. It is **not a complete project**.

To use this application:

1. Clone the **Bluetooth Host Example** (`bt_host_empty`) project from Silicon Labs using Simplicity Studio or from the [Silicon Labs GitHub](https://github.com/SiliconLabs).
2. Replace the `app.c` file in your `bt_host_empty` project with the one from this repository.
3. Build and run the project on your **Linux** machine.

---


