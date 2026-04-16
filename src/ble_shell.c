/*
 * zmk-ble-shell — BLE GATT shell service
 *
 * Service UUID    : c901c4e9-5770-4bf1-96b2-2dd287813e6e
 * Characteristic  : c901c4ea-5770-4bf1-96b2-2dd287813e6e
 *   Properties    : NOTIFY | WRITE_WITHOUT_RESP
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
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>
#include <errno.h>

#include <zmk/ble.h>

LOG_MODULE_REGISTER(zmk_ble_shell, CONFIG_ZMK_LOG_LEVEL);

#define ZBS_SVC_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xc901c4e9, 0x5770, 0x4bf1, \
                                           0x96b2, 0x2dd287813e6e))
#define ZBS_CHAR_UUID \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xc901c4ea, 0x5770, 0x4bf1, \
                                           0x96b2, 0x2dd287813e6e))

static volatile bool zbs_notif_enabled;
static bool          zbs_log_first_enable;

RING_BUF_DECLARE(zbs_tx_rb, CONFIG_ZMK_BLE_SHELL_TX_BUF_SIZE);
static struct k_spinlock zbs_tx_lock;

static void zbs_tx_enqueue(const uint8_t *data, const size_t len)
{
    if (len == 0) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&zbs_tx_lock);
    ring_buf_put(&zbs_tx_rb, data, (uint32_t)len);
    k_spin_unlock(&zbs_tx_lock, key);
}

static void zbs_tx_flush_work_handler(struct k_work *work);
static void zbs_cmd_exec_work_handler(struct k_work *work);
static void zbs_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t zbs_write_cmd(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len,
                             uint16_t offset, uint8_t flags);

static K_WORK_DEFINE(zbs_tx_flush_work, zbs_tx_flush_work_handler);
static K_WORK_DEFINE(zbs_cmd_exec_work, zbs_cmd_exec_work_handler);

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

static char        zbs_cmd_buf[CONFIG_ZMK_BLE_SHELL_CMD_BUF_SIZE];
static K_MUTEX_DEFINE(zbs_cmd_mutex);

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

static int zbs_log_is_ready(const struct log_backend *const backend)
{
    ARG_UNUSED(backend);
    return -EACCES;
}

static void zbs_log_panic(const struct log_backend *const backend)
{
    ARG_UNUSED(backend);
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
    .panic      = zbs_log_panic,
    .init       = zbs_log_init,
    .is_ready   = zbs_log_is_ready,
    .format_set = zbs_log_format_set,
};

LOG_BACKEND_DEFINE(log_backend_zbs, zbs_log_backend_api, true);

BT_GATT_SERVICE_DEFINE(zbs_svc,
    BT_GATT_PRIMARY_SERVICE(ZBS_SVC_UUID),
    BT_GATT_CHARACTERISTIC(ZBS_CHAR_UUID,
                           BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, zbs_write_cmd, NULL),
    BT_GATT_CCC(zbs_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void zbs_ccc_changed(const struct bt_gatt_attr *attr, const uint16_t value)
{
    ARG_UNUSED(attr);

    const bool enabled = (value == BT_GATT_CCC_NOTIFY);
    zbs_notif_enabled = enabled;

    LOG_DBG("BLE shell notifications %s", enabled ? "enabled" : "disabled");

    if (enabled) {
        if (!zbs_log_first_enable) {
            zbs_log_first_enable = true;
            log_backend_enable(&log_backend_zbs, NULL, CONFIG_LOG_MAX_LEVEL);
        } else {
            log_backend_activate(&log_backend_zbs, NULL);
        }
#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_PROMPT_EN)
        zbs_send_prompt();
#endif
    } else {
        log_backend_deactivate(&log_backend_zbs);

        const k_spinlock_key_t key = k_spin_lock(&zbs_tx_lock);
        ring_buf_reset(&zbs_tx_rb);
        k_spin_unlock(&zbs_tx_lock, key);
    }
}

static ssize_t zbs_write_cmd(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len,
                             const uint16_t offset, const uint8_t flags)
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
    if (k_mutex_lock(&zbs_cmd_mutex, K_NO_WAIT) != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_PROCEDURE_IN_PROGRESS);
    }

    memcpy(zbs_cmd_buf, buf, len);
    uint16_t end = len;
    while (end > 0 &&
           (zbs_cmd_buf[end - 1] == '\n' || zbs_cmd_buf[end - 1] == '\r')) {
        end--;
    }
    zbs_cmd_buf[end] = '\0';

    k_mutex_unlock(&zbs_cmd_mutex);

    k_work_submit(&zbs_cmd_exec_work);
    return (ssize_t)len;
}

static void zbs_cmd_exec_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    char cmd[CONFIG_ZMK_BLE_SHELL_CMD_BUF_SIZE];

    k_mutex_lock(&zbs_cmd_mutex, K_FOREVER);
    strncpy(cmd, zbs_cmd_buf, sizeof(cmd));
    cmd[sizeof(cmd) - 1] = '\0';
    k_mutex_unlock(&zbs_cmd_mutex);

    const struct shell *sh = shell_backend_dummy_get_ptr();

    shell_backend_dummy_clear_output(sh);
    const int ret = shell_execute_cmd(sh, cmd);

    size_t out_len;
    const char *out = shell_backend_dummy_get_output(sh, &out_len);

    if (ret == -ENOEXEC) {
        char msg[CONFIG_ZMK_BLE_SHELL_CMD_BUF_SIZE + 24];
        const int n = snprintk(msg, sizeof(msg),
                               "%s: command not found", cmd);
        if (n > 0) {
            zbs_tx_enqueue((const uint8_t *)msg, (size_t)n);
        }
    } else {
        if (out_len > 0) {
            zbs_tx_enqueue((const uint8_t *)out, out_len);
        }
        if (ret != 0) {
            zbs_tx_enqueue((const uint8_t *)"\r\n", 2);
        }
    }

#if IS_ENABLED(CONFIG_ZMK_BLE_SHELL_PROMPT_EN)
    zbs_send_prompt();
#endif
}

#define ZBS_ATT_OVERHEAD 3

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

    uint8_t chunk[CONFIG_BT_L2CAP_TX_MTU];
    uint32_t got;

    const struct bt_gatt_attr *value_attr = &zbs_svc.attrs[2];
    while (true) {
        const k_spinlock_key_t key = k_spin_lock(&zbs_tx_lock);
        got = ring_buf_get(&zbs_tx_rb, chunk, chunk_max);
        k_spin_unlock(&zbs_tx_lock, key);

        if (got == 0) {
            break;
        }

        const int err = bt_gatt_notify(conn, value_attr, chunk, (uint16_t)got);
        if (err == -EAGAIN || err == -ENOMEM) {
            const k_spinlock_key_t rkey = k_spin_lock(&zbs_tx_lock);
            ring_buf_put(&zbs_tx_rb, chunk, got);
            k_spin_unlock(&zbs_tx_lock, rkey);
            k_work_submit(&zbs_tx_flush_work);
            break;
        } else if (err) {
            LOG_WRN("notify err %d", err);
            break;
        }
    }

    bt_conn_unref(conn);
}
