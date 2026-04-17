/*
 * zmk-ble-shell internal API shared between ble_shell.c and
 * behavior_ble_shell_adv.c.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>

#define SVC_UUID_DATA BT_UUID_128_ENCODE(0xc901c4e9, 0x5770, 0x4bf1, 0x96b2, 0x2dd287813e6e)

bool zmk_ble_shell_connected(void);
void zmk_ble_shell_adv_on_connected(void);
