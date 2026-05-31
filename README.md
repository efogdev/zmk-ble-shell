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
    - name: zmk-shell-relay-core
      remote: efogdev
      revision: main
      path: modules/zmk-shell-relay-core
```

> [!IMPORTANT]
> The [`zmk-shell-relay-core`](https://github.com/efogdev/zmk-shell-relay-core)
> module must be present for the shell relay to work. It owns the shared
> internal shell instance and output buffer; without it this module will
> not build.

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

## Behavior: `&zbs_adv`

On Linux, the device is discoverable without any extra steps. On macOS, Windows and Android, Web Bluetooth API will only show the device if it is either paired-but-not-connected, or actively advertising the shell service UUID. This behavior handles the latter: press the bound key to start a short advertising burst so the client can discover the device.

Enable in config:

```kconfig
CONFIG_ZMK_BLE_SHELL_ADV_BEHAVIOR=y
CONFIG_BT_MAX_CONN=2  # required to accept the second connection
```

The behavior is defined automatically via `ble_shell_adv.dtsi`. Use it in your keymap:

```dts
/ {
    keymap {
        compatible = "zmk,keymap";
        default_layer {
            bindings = <&zbs_adv>;  // bind to any key
        };
    };
};
```

Advertising stops after `CONFIG_ZMK_BLE_SHELL_ADV_TIMEOUT` seconds (default: 30) if no client subscribes, or immediately upon subscription.

| Option | Default | Description |
|---|---|---|
| `ZMK_BLE_SHELL_ADV_BEHAVIOR` | n | Enable the behavior |
| `ZMK_BLE_SHELL_ADV_TIMEOUT` | 30 | Seconds to advertise before giving up |

## Client

Any BLE GATT client works. Enable notifications on the characteristic, then write commands as plain UTF-8 strings (newlines are stripped before execution).

On Linux, `bluetoothctl` + `gatttool` or `bluetoothctl` + `btgatt-client` will do. On macOS/iOS, [nRF Connect](https://www.nordicsemi.com/Products/Development-tools/nrf-connect-for-mobile) is the easiest option.

[Marshmellow UI](https://efog.tech/marshmellow-ui) might work. Or not.