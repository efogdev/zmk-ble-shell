/*
 * zmk-ble-shell — data push channel public API
 *
 * Characteristic UUID: c901c4eb-5770-4bf1-96b2-2dd287813e6e
 *   Properties: NOTIFY (firmware → client only)
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* Returns true when a BLE client has subscribed to data channel notifications. */
bool zmk_ble_shell_data_connected(void);

/*
 * Enqueue data to be sent via the data channel NOTIFY characteristic.
 * Non-blocking; drops silently if the ring buffer is full or no client
 * is subscribed.
 */
void zmk_ble_shell_data_write(const uint8_t *data, size_t len);
