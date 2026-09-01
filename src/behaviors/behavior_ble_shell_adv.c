/*
 * zmk-ble-shell — "advertise shell service" ZMK behavior
 *
 * When a keymap binding fires this behavior the device starts a connectable
 * legacy advertising session that carries the shell service UUID in its payload.
 * This allows a client to discover the device.
 *
 * The advertising is stopped automatically after
 * CONFIG_ZMK_BLE_SHELL_ADV_TIMEOUT seconds unless a client has already
 * subscribed to shell notifications (in which case the timeout is canceled
 * early via zmk_ble_shell_adv_on_connected()).
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zmk/behavior.h>
#include <zmk/ble.h>

#include "zephyr/bluetooth/uuid.h"

#define DT_DRV_COMPAT zmk_behavior_ble_shell_adv

#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
#include <zmk_adaptive_feedback/adaptive_feedback.h>
ZAF_CUSTOM_EVENT_DEFINE(zbs_mui_adv_evt, "mui-adv");
#endif /* CONFIG_ZMK_ADAPTIVE_FEEDBACK */

#include "ble_shell_private.h"
#include "drivers/behavior.h"

LOG_MODULE_DECLARE(zmk_ble_shell, CONFIG_ZMK_LOG_LEVEL);

static const struct bt_data zbs_adv_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, SVC_UUID_DATA),
};

static const struct bt_data zbs_adv_sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void adv_stop(void);

static void adv_timeout_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);
    LOG_DBG("adv timeout");
    adv_stop();
}

static K_WORK_DELAYABLE_DEFINE(adv_timeout_work, adv_timeout_work_fn);

static void adv_stop(void)
{
    k_work_cancel_delayable(&adv_timeout_work);
    const int err = bt_le_adv_stop();
    if (err && err != -EALREADY) {
        LOG_WRN("adv stop err %d", err);
    }
}

static int adv_start(void)
{
    if (zmk_ble_radio_yielded()) {
        return -EBUSY;
    }

    static const struct bt_le_adv_param param =
        BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE,
            BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);

    int err = bt_le_adv_start(&param, zbs_adv_ad, ARRAY_SIZE(zbs_adv_ad),
                              zbs_adv_sd, ARRAY_SIZE(zbs_adv_sd));
    if (err == -EALREADY) {
        LOG_DBG("adv already running, stopping and restarting with shell data");
        bt_le_adv_stop();
        err = bt_le_adv_start(&param, zbs_adv_ad, ARRAY_SIZE(zbs_adv_ad),
                              zbs_adv_sd, ARRAY_SIZE(zbs_adv_sd));
    }
    if (err) {
        LOG_ERR("adv start err %d", err);
        return err;
    }
    return 0;
}

void zmk_ble_shell_adv_on_connected(void)
{
    if (k_work_delayable_is_pending(&adv_timeout_work)) {
        adv_stop();
    }
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     const struct zmk_behavior_binding_event event)
{
    ARG_UNUSED(binding);
    ARG_UNUSED(event);

    const int err = adv_start();
    if (err) {
        return err;
    }

    LOG_DBG("BLE shell advertising started (%d s timeout)", CONFIG_ZMK_BLE_SHELL_ADV_TIMEOUT);
    k_work_reschedule(&adv_timeout_work, K_SECONDS(CONFIG_ZMK_BLE_SHELL_ADV_TIMEOUT));
#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
    zaf_custom_event_trigger(&zbs_mui_adv_evt);
#endif
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_ble_shell_adv_driver_api = {
    .binding_pressed  = on_keymap_binding_pressed,
    .binding_released = NULL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

#define ZBS_ADV_INST(n)                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL,                              \
                             POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,      \
                             &behavior_ble_shell_adv_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ZBS_ADV_INST)
