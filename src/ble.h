// ble.h — BLE HID gamepad mode (FPV simulator control) on the ESP32.
//
// When enabled the handset appears as a BLE gamepad ("BertRC"): 4 gimbal axes + 10 toggle
// buttons. Intended as an alternative to the RF/CRSF link for sims, so callers should
// silence CRSF output (crsf_set_output(false)) while it's on to save power. The NimBLE
// stack starts on first enable and stays up (BLE teardown is fragile); disabling just
// stops sending reports.
#pragma once

#include <stdbool.h>

// Enable/disable gamepad reporting. First enable starts the BLE stack + task.
void ble_set(bool on);
bool ble_on();         // reporting currently enabled
bool ble_connected();  // a BLE host is connected
