# STM32H723_Nucleo_ETH_Motor

Firmware for an **STM32H723ZG** (NUCLEO‑H723ZG) board acting as the real‑time
motion / sensor controller for a mobile robot base. It bridges a host SBC
(over **UDP/Ethernet**) to a **dual‑wheel motor drive** (over **RS485 Modbus
RTU**), and streams IMU, battery‑gauge and power‑rail telemetry back to the
host.

Built with **STM32CubeMX / CubeIDE**, **FreeRTOS** (CMSIS‑RTOS v2) and the
**LwIP** TCP/IP stack.

---

## Overview

```
                +-----------------------------+
   Host SBC <---|  UDP :8080  (LwIP / RMII)   |
   (10.0.0.100) |                             |
                |        STM32H723ZG          |---- I2C  ---> LSM6DSOX IMU
                |   FreeRTOS  +  LwIP         |---- I2C  ---> LIS3MDL magnetometer
                |                             |---- I2C  ---> LTC2944 battery gauge
                |                             |---- ADC  ---> power-rail monitor
                |   RS485 / Modbus RTU :USART2|
                +--------------|--------------+
                               |
                    +----------v----------+
                    | ZLIH65RC motor      |  Slave 1 (left)
                    | controllers         |  Slave 2 (right)
                    +---------------------+
```

* **Host link:** UDP, custom framed protocol with byte‑stuffing + CRC8.
* **Motor link:** RS485 Modbus RTU master, two single‑axis ZLIH65RC drivers
  (selectable for dual‑channel ZLTECH controllers).
* **Sensors:** 6‑axis IMU, 3‑axis magnetometer, coulomb‑counting fuel gauge,
  multi‑rail ADC voltage monitor.
* **Extras:** power button handling that pulses the host SBC power line.

---

## Hardware

| Item              | Detail                                                |
|-------------------|-------------------------------------------------------|
| MCU               | STM32H723ZG (Cortex‑M7, up to 550 MHz)                |
| Board             | NUCLEO‑H723ZG                                         |
| Ethernet PHY      | LAN8742 (RMII), zero‑copy DMA driver                  |
| HSE               | 25 MHz                                                |
| Flash linker      | `STM32H723ZGTX_FLASH.ld` (RAM variant also provided)  |

### Peripheral map

| Peripheral | Use                                   | Key params                       |
|------------|---------------------------------------|----------------------------------|
| USART2     | RS485 Modbus RTU master               | 115200 8N1, RTS on **PD4**       |
| USART1     | Debug / console                       | 115200 8N1                       |
| ETH (RMII) | LwIP / UDP server & client            | static IP, see below             |
| I2C2/4/5   | IMU, magnetometer, battery gauge      | standard mode                    |
| SPI4       | Sensor bus (optional)                 | —                                |
| ADC1/2/3   | Power‑rail & charger voltage sensing  | 16‑bit, VREF 3.3 V               |
| TIM        | FreeRTOS time base / HAL tick         | —                                |

---

## Network configuration

Static addressing (no DHCP):

| Role        | Address       | Port |
|-------------|---------------|------|
| Board (server) | `10.0.0.200` / `255.255.255.0` | **8080** |
| Host (client)  | `10.0.0.100`                   | **8081** |

> Defined in [LWIP/App/lwip.c](LWIP/App/lwip.c) (`IP_ADDRESS`, `NETMASK_ADDRESS`)
> and [Core/Inc/udpserver.h](Core/Inc/udpserver.h)
> (`UDP_SERVER_PORT`, `UDP_CLIENT_PORT`, `CLIENT_IP_*`).

---

## UDP protocol

Frames are byte‑stuffed (`SOF/EOF/ESC`) and carry a packet‑type header with a
CRC8. Defined in [Core/Inc/protocol.h](Core/Inc/protocol.h).

### Framing constants

| Symbol            | Value | Meaning              |
|-------------------|-------|----------------------|
| `DEF_SERIAL_SOF`  | 0xAA  | Start of frame       |
| `DEF_SERIAL_EOF`  | 0x55  | End of frame         |
| `DEF_SERIAL_ESC`  | 0x1B  | Escape byte          |
| `DEF_SERIAL_XOR`  | 0x20  | Escape XOR mask      |
| `CRC_CHECK`       | 1     | 0=off, 1=CRC8, 2=CRC32 |

### Packet types (`packet_type_t`)

| Type | Value | Payload struct          | Direction | Description                   |
|------|-------|-------------------------|-----------|-------------------------------|
| `DEF_IMU_DATA`          | 0x11 | `ImuPacket_t`        | → host | Gyro/accel/mag (9 floats)     |
| `DEF_MOTOR_INFO`        | 0x12 | `MotorInfo_t`        | → host | Single‑controller status      |
| `DEF_MOTOR_SPEED`       | 0x13 | `MotorSpeed_t`       | ← host | Target L/R rpm command        |
| `DEF_TARGET_IP`         | 0x14 | —                    | ← host | Set host/target IP            |
| `DEF_DOUBLE_MOTOR_INFO` | 0x15 | `DoubleMotorInfo_t`  | → host | Dual‑controller status        |
| `DEF_BAT_INFO`          | 0x16 | `BatteryGauge_t`     | → host | Voltage/current/SoC           |
| `DEF_MEASURED_VOLTAGE`  | 0x17 | `MeasuredVoltage_t`  | → host | All power‑rail voltages       |

`PacketHeader_t = { uint8 packet_type; uint16 payload_length; uint8 crc8 }`.

### Key payload fields

* **`ImuPacket_t`** — `gx,gy,gz, ax,ay,az, mag_x,mag_y,mag_z` (float).
* **`MotorSpeed_t`** — `double left_rpm, right_rpm` (host → board command).
* **`DoubleMotorInfo_t`** — per‑side software version, bus voltage, status word,
  error code, encoder count, rpm (0.1 r/min), torque (0.1 A), plus
  `emergency_state` and `init_state`.
* **`BatteryGauge_t`** — `voltage_V`, `current_mA` (+charge/−discharge),
  `temperature_C`, `battery_state`, `soc_percent` (0–100), `soc_valid`.
* **`MeasuredVoltage_t`** — motor / main / 12 V opt / 12 V LED ×2 / 5 V / 3.3 V /
  charger input / filtered charger voltage.

---

## Modbus RTU (RS485)

Master implementation in [Core/Src/modbus_master.c](Core/Src/modbus_master.c).

| Parameter         | Value                                    |
|-------------------|------------------------------------------|
| UART              | USART2, 115200 8N1                        |
| Direction control | RTS on **PD4** (HIGH = TX, LOW = RX)      |
| Mode              | Polling (blocking), CRC16, IDLE‑framed    |
| Timeout           | `MODBUS_TIMEOUT_MS` = 50 ms              |
| Slaves            | `0x01` = motor 1 (left), `0x02` = motor 2 (right) |

Function codes: `0x03` read holding, `0x04` read input, `0x06` write single,
`0x10` write multiple.

**Driver model select** — [Core/Inc/motor_interface.h](Core/Inc/motor_interface.h):

```c
#define MOTOR_CONTROLLER_MODEL 1   // 1 = ZLIH65RC (single-axis ×2)
                                   // else = ZLTECH (dual-axis, one slave)
```

The dual‑driver manager in
[Core/Inc/motor_driver_config.h](Core/Inc/motor_driver_config.h) supports
`SINGLE` / `DUAL` modes with round‑robin polling
(`MotorDriver_GetNextSlaveId`).

---

## Power & sensors

ADC scaling constants — [Core/Inc/adc_calc.h](Core/Inc/adc_calc.h):

| Constant              | Value   | Rail                |
|-----------------------|---------|---------------------|
| `VREF`                | 3.3 V   | ADC reference       |
| `ADC_16`              | 65535   | 16‑bit full scale   |
| `DIVIDER_RATIO_36V`   | 17.67   | Motor / 36 V bus    |
| `DIVIDER_RATIO_12V`   | 8.41    | 12 V rails          |
| `DIVIDER_RATIO_5V`    | 2.0     | 5 V rail            |
| `DIVIDER_RATIO_3V3`   | 2.0     | 3.3 V rail          |

| Sensor    | Bus / addr            | Role                          |
|-----------|-----------------------|-------------------------------|
| LSM6DSOX  | I2C                   | 6‑axis accel + gyro           |
| LIS3MDL   | I2C (0x39 / 0x3D)     | 3‑axis magnetometer           |
| LTC2944   | I2C (0x64)            | Coulomb‑counting battery gauge |

---

## FreeRTOS task structure

Tasks created in [Core/Src/main.c](Core/Src/main.c) (`osThreadNew`); kernel
config in [Core/Inc/FreeRTOSConfig.h](Core/Inc/FreeRTOSConfig.h).

| Task         | Period | Stack    | Priority | Responsibility |
|--------------|--------|----------|----------|----------------|
| `mainTask`   | 50 ms  | 512×4 W  | Normal   | LwIP + UDP init, power‑button state machine, host SBC power pulse |
| `sensorTask` | 10 ms (100 Hz) | 512×4 W | Normal | IMU/mag read, battery‑gauge update (~1 Hz), power‑rail ADC sampling, UDP telemetry |
| `motorTask`  | 50 ms  | 512×4 W  | Normal   | Modbus init, controller state machine, e‑stop handling, motor command + status read/send |

The UDP server runs its own RX/TX worker threads with a bounded queue
(`UDP_QUEUE_SIZE = 20`, `MAX_PACKET_SIZE = 256`) and exposes statistics via
`udp_get_stats()`.

Motor controller sequence (`MotorManagerSeq_e`): `INIT_YET → RW_S_VALUE_INIT_AND_SAVE
→ CONTROL_MODE_SETUP → CONTROL_LOOP / EMERGENCY_LOOP`.

---

## Project structure

```
STM32H723_Nucleo_ETH_Motor/
├── Core/
│   ├── Inc/                       Application headers
│   │   ├── protocol.h             UDP packet types & frame encoding
│   │   ├── udpserver.h            UDP server API (port 8080), queue & stats
│   │   ├── motor_interface.h      Motor control API + model select
│   │   ├── motor_driver_config.h  Single/Dual manager, round-robin polling
│   │   ├── modbus_master.h        RS485 Modbus RTU master
│   │   ├── zlih65rc.h / zltech.h  Motor-controller register maps
│   │   ├── imu.h / lsm6dsox*.h    6-axis IMU driver
│   │   ├── lis3mdl_reg.h          Magnetometer registers
│   │   ├── ltc2944.h              Battery fuel-gauge driver
│   │   ├── adc_calc.h             Power-rail ADC scaling
│   │   ├── power_button.h         Power button / SBC power-pulse logic
│   │   └── FreeRTOSConfig.h       RTOS configuration
│   ├── Src/                       Application source
│   │   ├── main.c                 Entry point + 3 FreeRTOS tasks + peripherals
│   │   ├── udpserver.c            UDP RX/TX threads + encode/decode
│   │   ├── motor_interface.c      Motor control + state machine
│   │   ├── motor_driver_config.c  Multi-motor polling
│   │   ├── modbus_master.c        Modbus RTU CRC16 + framing
│   │   ├── lsm6dsox.c / ltc2944.c Sensor drivers
│   │   └── adc_calc.c             Voltage calculation
│   └── Startup/                   startup_stm32h723zgtx.s
├── Drivers/
│   ├── STM32H7xx_HAL_Driver/      STM32 HAL (ETH, UART, ADC, I2C, SPI, TIM…)
│   ├── CMSIS/                     ARM CMSIS core
│   └── BSP/Components/lan8742/    LAN8742 PHY driver
├── LWIP/
│   ├── App/lwip.c                 LwIP init + static IP (MX_LWIP_Init)
│   └── Target/                    ethernetif.c (RMII DMA), lwipopts.h
├── Middlewares/Third_Party/       FreeRTOS + LwIP sources
├── STM32H723_Nucleo_ETH_Motor.ioc CubeMX device config
├── STM32H723ZGTX_FLASH.ld         Flash linker script
└── README.md
```

---

## Build & flash

Open the project in **STM32CubeIDE** (it already contains `.project` /
`.cproject` and the `Debug/` build folder), then **Build** and **Run/Debug**
using the supplied launch configs:

* `STM32H723_Nucleo_ETH_Motor Debug.launch`
* `STM32H723_Nucleo_ETH.launch`

Device configuration can be regenerated from
`STM32H723_Nucleo_ETH_Motor.ioc` in CubeMX/CubeIDE.

### Quick connectivity test

With the host at `10.0.0.100`, the board responds on UDP `10.0.0.200:8080`.
Telemetry packets (IMU/motor/battery/voltage) are pushed automatically; send
`DEF_MOTOR_SPEED` (0x13) with `MotorSpeed_t` to command wheel speeds.