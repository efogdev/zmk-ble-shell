/*
 * zmk-ble-shell — BLE GATT shell service
 *
 * Service UUID    : c901c4e9-5770-4bf1-96b2-2dd287813e6e
 * Shell char      : c901c4ea-5770-4bf1-96b2-2dd287813e6e
 *   Properties    : NOTIFY | WRITE_WITHOUT_RESP
 * Data char       : c901c4eb-5770-4bf1-96b2-2dd287813e6e  (if DATA_CHANNEL enabled)
 *   Properties    : NOTIFY
 *
 * Written payload → executed as a Zephyr shell command; output is
 * relayed back via NOTIFY.  While the client has notifications enabled,
 * ZMK log output is also forwarded through the same channel.
 *
 * All existing log backends (USB, UART, RTT, …) are left completely
 * untouched.  This backend is purely additive.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>
#include <errno.h>
#include <sys/unistd.h>

#include <zmk/ble.h>

#include "ble_shell_private.h"
#include <zmk_ble_shell/data_channel.h>
#include <zmk_shell_relay/relay.h>

#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
#include <zmk_adaptive_feedback/adaptive_feedback.h>

ZAF_CUSTOM_EVENT_DEFINE(zbs_conn_evt,    "mui-conn");
ZAF_CUSTOM_EVENT_DEFINE(zbs_disconn_evt, "mui-disconn");
#endif /* CONFIG_ZMK_ADAPTIVE_FEEDBACK */

LOG_MODULE_REGISTER(zmk_ble_shell, CONFIG_ZMK_LOG_LEVEL);

#define ZBS_SVC_UUID BT_UUID_DECLARE_128(SVC_UUID_DATA)

#define ZBS_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xc901c4ea, 0x5770, 0x4bf1, \
                                           0x96b2, 0x2dd287813e6e))

#define ZBS_DATA_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xc901c4eb, 0x5770, 0x4bf1, \
                                           0x96b2, 0x2dd287813e6e))

#define ZBS_ATT_OVERHEAD 3

static volatile bool zbs_notif_enabled;
static bool          zbs_log_first_enable;

#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_DATA_CHANNEL)
static volatile bool zbs_data_notif_enabled;
RING_BUF_DECLARE(zbs_data_rb, CONFIG_ZMK_BLE_SHELL_DATA_TX_BUF_SIZE);
static struct k_spinlock zbs_data_tx_lock;
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_DATA_CHANNEL)
struct zbs_channel {
    struct ring_buf           *rb;
    struct k_spinlock         *lock;
    struct k_work             *flush_work;
    const struct bt_gatt_attr *value_attr;
    const char                *tag;
};

static void zbs_channel_flush(const struct zbs_channel *ch)
{
    struct bt_conn *conn = zmk_ble_active_profile_conn();
    if (!conn) {
        return;
    }

    uint16_t mtu = bt_gatt_get_mtu(conn);
    if (mtu <= ZBS_ATT_OVERHEAD) {
        mtu = 23;
    }
    const uint16_t chunk_max = mtu - ZBS_ATT_OVERHEAD;
    uint8_t chunk[CONFIG_BT_L2CAP_TX_MTU];

    while (true) {
        const k_spinlock_key_t key = k_spin_lock(ch->lock);
        const uint32_t got = ring_buf_get(ch->rb, chunk, chunk_max);
        k_spin_unlock(ch->lock, key);

        if (got == 0) {
            break;
        }

        const int err = bt_gatt_notify(conn, ch->value_attr, chunk, (uint16_t)got);
        if (err == -EAGAIN || err == -ENOMEM) {
            const k_spinlock_key_t rkey = k_spin_lock(ch->lock);
            ring_buf_put(ch->rb, chunk, got);
            k_spin_unlock(ch->lock, rkey);
            k_work_submit(ch->flush_work);
            break;
        } else if (err) {
            LOG_WRN("%s notify err %d", ch->tag, err);
            break;
        }
    }

    bt_conn_unref(conn);
}
#endif /* CONFIG_ZMK_BLE_SHELL_DATA_CHANNEL */

static void zbs_tx_flush_work_handler(struct k_work *work);
static void zbs_cmd_done(const char *cmd, int ret);
static void zbs_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t zbs_write_cmd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

static K_WORK_DEFINE(zbs_tx_flush_work, zbs_tx_flush_work_handler);
static void zbs_shell_data_ready(void)
{
    if (zbs_notif_enabled) {
        k_work_submit(&zbs_tx_flush_work);
    }
}

static const struct zmk_shell_relay_sink zbs_shell_sink = {
    .data_ready = zbs_shell_data_ready,
    .cmd_done   = zbs_cmd_done,
};

#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_DATA_CHANNEL)
static void zbs_data_tx_flush_work_handler(struct k_work *work);
static void zbs_data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static K_WORK_DEFINE(zbs_data_tx_flush_work, zbs_data_tx_flush_work_handler);
#endif

static void zbs_tx_enqueue(const uint8_t *data, const size_t len)
{
    zmk_shell_relay_enqueue(data, len);
}

#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_DATA_CHANNEL)

bool zmk_ble_shell_data_connected(void)
{
    return zbs_data_notif_enabled;
}

void zmk_ble_shell_data_write(const uint8_t *data, const size_t len)
{
    if (len == 0 || !zbs_data_notif_enabled) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&zbs_data_tx_lock);
    ring_buf_put(&zbs_data_rb, data, (uint32_t)len);
    k_spin_unlock(&zbs_data_tx_lock, key);
    k_work_submit(&zbs_data_tx_flush_work);
}

static void zbs_data_ccc_changed(const struct bt_gatt_attr *attr, const uint16_t value)
{
    ARG_UNUSED(attr);
    const bool enabled = (value == BT_GATT_CCC_NOTIFY);
    zbs_data_notif_enabled = enabled;
    LOG_DBG("BLE data channel notifications %s", enabled ? "enabled" : "disabled");
    if (!enabled) {
        const k_spinlock_key_t key = k_spin_lock(&zbs_data_tx_lock);
        ring_buf_reset(&zbs_data_rb);
        k_spin_unlock(&zbs_data_tx_lock, key);
    }
}

#else /* !CONFIG_ZMK_BLE_SHELL_DATA_CHANNEL */

bool zmk_ble_shell_data_connected(void) { return false; }
void zmk_ble_shell_data_write(const uint8_t *data, const size_t len)
{
    ARG_UNUSED(data);
    ARG_UNUSED(len);
}

#endif /* CONFIG_ZMK_BLE_SHELL_DATA_CHANNEL */

#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_PROMPT_EN)
static void zbs_send_prompt(void)
{
    static const char newline[] = "\r\n";
    static const char prompt[]  = "\033[1;32m" CONFIG_ZMK_BLE_SHELL_PROMPT "\033[0m";
    zbs_tx_enqueue((const uint8_t *)newline, sizeof(newline) - 1);
    zbs_tx_enqueue((const uint8_t *)prompt,  sizeof(prompt)  - 1);
    k_work_submit(&zbs_tx_flush_work);
}
#endif /* CONFIG_ZMK_BLE_SHELL_PROMPT_EN */

static uint8_t   zbs_log_fmt_buf[CONFIG_ZMK_BLE_SHELL_LOG_BUF_SIZE];
static uint32_t  zbs_log_format = LOG_OUTPUT_TEXT;

static int zbs_log_line_out(uint8_t *data, const size_t length, void *ctx)
{
    ARG_UNUSED(ctx);
    zbs_tx_enqueue(data, length);
    k_work_submit(&zbs_tx_flush_work);
    return (int)length;
}

LOG_OUTPUT_DEFINE(zbs_log_output, zbs_log_line_out,
                  zbs_log_fmt_buf, sizeof(zbs_log_fmt_buf));

static void zbs_log_process(const struct log_backend *const backend,
                            union log_msg_generic *msg)
{
    ARG_UNUSED(backend);
    if (!zbs_notif_enabled) {
        return;
    }
    const uint32_t flags = LOG_OUTPUT_FLAG_TIMESTAMP | LOG_OUTPUT_FLAG_LEVEL;
    const log_format_func_t fn = log_format_func_t_get(zbs_log_format);
    fn(&zbs_log_output, &msg->log, flags);
}

static void zbs_log_init(const struct log_backend *const backend)
{
    log_backend_deactivate(backend);
}

static int zbs_log_format_set(const struct log_backend *const backend,
                               const uint32_t log_type)
{
    ARG_UNUSED(backend);
    zbs_log_format = log_type;
    return 0;
}

static const struct log_backend_api zbs_log_backend_api = {
    .process    = zbs_log_process,
    .dropped    = NULL,
    .panic      = NULL,
    .init       = zbs_log_init,
    .is_ready   = NULL,
    .format_set = zbs_log_format_set,
};

LOG_BACKEND_DEFINE(log_backend_zbs, zbs_log_backend_api, true);

BT_GATT_SERVICE_DEFINE(zbs_svc,
    BT_GATT_PRIMARY_SERVICE(ZBS_SVC_UUID),
    BT_GATT_CHARACTERISTIC(ZBS_CHAR_UUID,
                           BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
                           NULL, zbs_write_cmd, NULL),
    BT_GATT_CCC(zbs_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_DATA_CHANNEL)
    BT_GATT_CHARACTERISTIC(ZBS_DATA_CHAR_UUID,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           NULL, NULL, NULL),
    BT_GATT_CCC(zbs_data_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif
);

static void zbs_ccc_changed(const struct bt_gatt_attr *attr, const uint16_t value)
{
    ARG_UNUSED(attr);

    const bool enabled = (value == BT_GATT_CCC_NOTIFY);
    zbs_notif_enabled = enabled;

    LOG_DBG("BLE shell notifications %s", enabled ? "enabled" : "disabled");

    if (enabled) {
        zmk_shell_relay_reset();
        zmk_shell_relay_attach(&zbs_shell_sink);
        if (!zbs_log_first_enable) {
            zbs_log_first_enable = true;
            log_backend_enable(&log_backend_zbs, NULL, CONFIG_LOG_MAX_LEVEL);
        } else {
            log_backend_activate(&log_backend_zbs, NULL);
        }
#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_PROMPT_EN)
        zbs_send_prompt();
#endif
#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
        zaf_custom_event_trigger(&zbs_conn_evt);
#endif
#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_ADV_BEHAVIOR)
        zmk_ble_shell_adv_on_connected();
#endif
    } else {
        log_backend_deactivate(&log_backend_zbs);

        zmk_shell_relay_detach();
        zmk_shell_relay_reset();
#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
        zaf_custom_event_trigger(&zbs_disconn_evt);
#endif
    }
}

static ssize_t zbs_write_cmd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, const uint16_t offset, const uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len == 0 || len >= CONFIG_ZMK_BLE_SHELL_CMD_BUF_SIZE) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const int err = zmk_shell_relay_queue_cmd((const char *)buf, len);
    if (err == -ENOSPC) {
        return BT_GATT_ERR(BT_ATT_ERR_PROCEDURE_IN_PROGRESS);
    }
    if (err) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    return (ssize_t)len;
}

static void zbs_cmd_done(const char *cmd, const int ret)
{
    if (ret == -ENOEXEC) {
        char msg[16 + sizeof(": command not found")];
        const int n = snprintk(msg, sizeof(msg),
                               "%s: command not found", cmd);
        if (n > 0) {
            zbs_tx_enqueue((const uint8_t *)msg, (size_t)n);
        }
    } else if (ret != 0) {
        zbs_tx_enqueue((const uint8_t *)"\r\n", 2);
    }

#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_PROMPT_EN)
    zbs_send_prompt();
#endif
}

static void zbs_tx_flush_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (!zbs_notif_enabled) {
        return;
    }

    struct bt_conn *conn = zmk_ble_active_profile_conn();
    if (!conn) {
        return;
    }

    uint16_t mtu = bt_gatt_get_mtu(conn);
    if (mtu <= ZBS_ATT_OVERHEAD) {
        mtu = 23;
    }
    const uint16_t chunk_max = mtu - ZBS_ATT_OVERHEAD;

    while (true) {
        uint8_t *chunk;
        const uint32_t got = zmk_shell_relay_claim(&chunk, chunk_max);
        if (got == 0) {
            break;
        }

        const int err = bt_gatt_notify(conn, &zbs_svc.attrs[2], chunk, (uint16_t)got);
        if (err == -EAGAIN || err == -ENOMEM) {
            zmk_shell_relay_finish(0);
            k_work_submit(&zbs_tx_flush_work);
            break;
        } else if (err) {
            zmk_shell_relay_finish(0);
            LOG_WRN("shell notify err %d", err);
            break;
        }
        zmk_shell_relay_finish(got);
    }

    bt_conn_unref(conn);
}

#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_DATA_CHANNEL)
static const struct zbs_channel zbs_data_ch = {
    .rb         = &zbs_data_rb,
    .lock       = &zbs_data_tx_lock,
    .flush_work = &zbs_data_tx_flush_work,
    .value_attr = &zbs_svc.attrs[5],
    .tag        = "data",
};

static void zbs_data_tx_flush_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (!zbs_data_notif_enabled) {
        return;
    }
    zbs_channel_flush(&zbs_data_ch);
}
#endif /* CONFIG_ZMK_BLE_SHELL_DATA_CHANNEL */
