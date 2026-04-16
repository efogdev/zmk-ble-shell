# zmk-ble-shell

A ZMK module that exposes the Zephyr shell over BLE. Write a command to the characteristic, get the output back as notifications. Log output is forwarded through the same channel while a client is connected.

## How it works

A custom GATT service with one characteristic (`NOTIFY | WRITE_WITHOUT_RESP`):

- **Write** a shell command → executed via `shell_execute_cmd`, output sent back as notifications
- **Enable notifications** → ZMK log output starts streaming through the same channel

Existing log backends (USB, UART, RTT) are untouched.

**Service UUID:** `c901c4e9-5770-4bf1-96b2-2dd287813e6e`  
**Characteristic UUID:** `c901c4ea-5770-4bf1-96b2-2dd287813e6e`

## Setup

Add to your `config/west.yml`:

```yaml
manifest:
  remotes:
    - name: efogdev
      url-base: https://github.com/efogdev
  projects:
    - name: zmk-ble-shell
      remote: efogdev
      revision: main
      path: modules/zmk-ble-shell
```

Enable in your shield/board config:

```kconfig
CONFIG_ZMK_BLE_SHELL=y
```

## Kconfig options

| Option | Default | Description |
|---|---|---|
| `ZMK_BLE_SHELL_TX_BUF_SIZE` | 2048 | TX ring buffer size in bytes |
| `ZMK_BLE_SHELL_CMD_BUF_SIZE` | 256 | Max command length including NUL |
| `ZMK_BLE_SHELL_LOG_BUF_SIZE` | 256 | Log formatting scratch buffer size |
| `ZMK_BLE_SHELL_PROMPT_EN` | y | Send prompt after connect and each command |
| `ZMK_BLE_SHELL_PROMPT` | `SHELL_PROMPT_UART` | Prompt string (ANSI green) |

## Client

Any BLE GATT client works. Enable notifications on the characteristic, then write commands as plain UTF-8 strings (newlines are stripped before execution).

On Linux, `bluetoothctl` + `gatttool` or `bluetoothctl` + `btgatt-client` will do. On macOS/iOS, [nRF Connect](https://www.nordicsemi.com/Products/Development-tools/nrf-connect-for-mobile) is the easiest option.

[Marshmellow UI](https://efog.tech/marshmellow-ui) might work. Or not.