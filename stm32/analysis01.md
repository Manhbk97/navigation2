# STM32H723_Nucleo_ETH_Motor — Deep-Dive Analysis (analysis01)

> A "from scratch" walkthrough of the firmware: **what each part is for, why it is
> designed that way, and the exact logic/flow/steps** you would follow if you were
> rebuilding this project from an empty CubeIDE workspace.
>
> Companion to `readme.md`. The README is the *reference card*; this document is
> the *story* — purpose, logic, control flow, and the order in which everything
> happens. Source paths are relative to
> `/home/rgt/rgt_navigation2/STM32H723_Nucleo_ETH_Motor`.

---

## 0. The 30-second mental model

This board is a **real-time bridge + telemetry hub** sitting between:

```
   ROS2 / SBC host  ──UDP/Ethernet──►  STM32H723  ──RS485 Modbus──►  2× wheel motor drivers
   (navigation,           (8080/8081)   (this FW)     (USART2)        (left / right, ZLIH65RC)
    velocity cmds)
                                            │
                                            ├── I2C  → IMU + magnetometer  (odom/heading aid)
                                            ├── I2C  → LTC2944 fuel gauge  (battery SoC)
                                            ├── ADC  → power-rail voltages (health monitor)
                                            └── GPIO → power sequencing + SBC power button
```

Its single job: **take wheel-speed commands down from the host, drive the motors
safely, and push back everything the host needs to localize and stay alive**
(IMU, encoders, battery, voltages) — while never letting a comms dropout or an
e-stop turn into a runaway robot.

Everything else in the codebase exists to serve that loop *reliably* and *in
real time*, which is why it is built on **FreeRTOS** (deterministic scheduling)
and **LwIP netconn** (network I/O on dedicated worker threads).

---

## 1. Why these technology choices (the "purpose" layer)

| Choice | Why it's here | What would break without it |
|--------|---------------|------------------------------|
| **STM32H723 (Cortex-M7 @ up to 550 MHz)** | Needs to run a TCP/IP stack, an RTOS, sensor math, and a Modbus master *concurrently* at 100 Hz. An M0/M4 would be tight. | Missed deadlines → jittery odometry, dropped motor commands. |
| **FreeRTOS (CMSIS-RTOS v2)** | Three jobs run at different rates (50 ms control, 10 ms sensors, 50 ms UI/power). Threads + priorities let each meet its own deadline. | A single super-loop would couple the slowest task to the fastest. |
| **LwIP + netconn API** | UDP gives low-latency, fire-and-forget telemetry; the host re-sends commands anyway, so a lost packet is harmless. netconn lets us block in dedicated threads instead of polling. | TCP would add latency + head-of-line blocking for streaming telemetry. |
| **RS485 Modbus RTU** | Industry-standard, multidrop, noise-tolerant motor-bus. Two drivers share one differential pair. | You'd need 2 separate UARTs and a custom protocol. |
| **Custom framed UDP protocol (SOF/EOF/ESC + CRC8)** | Raw UDP has no message boundaries inside a stream of telemetry; byte-stuffing makes frames self-delimiting and CRC catches corruption. | Telemetry would desync on any partial/merged datagram. |
| **Static IP, no DHCP** | A robot base must come up deterministically at a known address with no DHCP server present. | Host wouldn't know where to send commands. |

---

## 2. The big picture: data + control flows

### 2.1 Command path (host → wheels) — the safety-critical one

```
SBC sends DEF_MOTOR_SPEED (0x13, MotorSpeed_t{double L, double R})
        │  UDP :8080
        ▼
udp_rx_thread  (udpserver.c:423)
        │  netconn_recv → fncPacketDecode (de-stuff SOF/EOF/ESC, check CRC8)
        ▼
process_received_packet  (udpserver.c:234, case DEF_MOTOR_SPEED:249)
        │  writes shared state:  sbc_com_t.l_rpm / r_rpm / last_control_time = now
        ▼
StartMotorTask  (main.c:2368, every 50 ms)
        │  reads sbc_com_t, runs the per-motor state machine
        ▼
MotorControlLoop → ProcessMotorSpeed  (motor_interface.c:415 / 376)
        │  MotorSpeedControl() → Modbus_WriteSingleRegister(TARGET_VELOCITY)
        ▼
Modbus master (USART2, RTS=PD4) → motor driver slave 1 / slave 2
```

**The critical decoupling:** the network thread *never* touches the motors. It
only deposits the latest command into `sbc_com_t` and stamps the time. The motor
task owns all driver I/O. This is deliberate — a flood of UDP packets can't
starve or reorder Modbus transactions, and the motor task always acts on the
*freshest* command at its own fixed cadence.

### 2.2 Telemetry path (board → host) — fire-and-forget

```
sensorTask (10ms)  →  IMU/mag read   → udp_queue_send_imu()        0x11
sensorTask (~1Hz)  →  LTC2944 gauge  → udp_queue_send_bat_info()   0x16
motorTask  (50ms)  →  Modbus read    → udp_queue_send_double_motor 0x15
                                          │
                                          ▼  bounded queue (20 deep)
                                    udp_tx_thread (udpserver.c:471)
                                          │  build header + CRC8 + byte-stuff
                                          ▼  netconn_sendto → host :8081
```

Producers (the application tasks) just enqueue; one TX thread serializes all
sends. If the queue is full, the send is *dropped and counted* (`queue_full_count`)
rather than blocking the producer — again, telemetry is allowed to be lossy, the
control loop is not.

---

## 3. Boot sequence — what happens from power-on (the "steps" layer)

This is the order you'd reproduce if starting from scratch. Entry is `main()`.

1. **`MPU_Config()`** (main.c:2474) — configure the Cortex-M7 MPU. A region at
   `0x30000000` (32 KB, D2 SRAM) is made cache-safe for the **Ethernet DMA
   descriptors + LwIP buffers**. This is *the* classic H7 gotcha: without it, the
   Ethernet DMA and the CPU data cache disagree and you get random RX corruption.
2. **HAL + clock init** — `HAL_Init()`, `SystemClock_Config()` (HSE 25 MHz → PLL).
3. **Peripheral init (MX_*)** — GPIO, USART1 (debug), USART2 (Modbus), I2C2/4/5,
   ADC1/2/3, SPI4, ETH, timers. The HAL time base is on **TIM6**, freeing SysTick
   for FreeRTOS.
4. **Power sequencing** — `GPIO_On_Mode()` (main.c:341) brings rails up *in order*
   with delays: main 29.4 V → 5 V → 12 V opt → motor 29.4 V → PHY power → SBC power.
   Order matters: you don't power the Ethernet PHY or motor drivers before their
   supplies are stable, and the SBC is brought up last / under button control.
5. **Sensor power + bus-mode select** — IMU power on; the `IMU_SPI4_SS` pin selects
   SPI vs I2C mode (compile-time `IMU_COM_MODE`).
6. **`osKernelInitialize()` → create 3 tasks → `osKernelStart()`** (main.c:766-772).
   After this, `main()` never returns; the scheduler runs the three task loops.

So the "from scratch" build order is: **clock → MPU/cache → peripherals → power
rails → RTOS → tasks**. Get the MPU and power order wrong and nothing above the
HAL will work reliably.

---

## 4. The three tasks in detail (the "logic" layer)

All three are created in `MX_FREERTOS_Init` and run forever. They share global
state (`sbc_com_t`, `motorN_data`, `measured_voltage_t`, `g_battery`). This is a
design where **each piece of shared state has a single clear writer**, which is
how it stays correct without heavy locking.

### 4.1 `StartMainTask` — UI / power button / housekeeping (50 ms) — main.c:1855

Purpose: bring up the network and run the **SBC power-button state machine**.

Flow each tick:
1. `MX_LWIP_Init()` + `udpserver_init()` **once** at entry (starts the rx/tx threads).
2. `PC_Power_Pulse_Update()` — services any in-progress power-pulse to the SBC.
3. Read the physical button, debounce via `Update_Button_State()`:
   - **CLICKED** → short power pulse (`PWR_SHORT_PRESS_MS`) → "press SBC power".
   - **HELD** → long pulse (`PWR_LONG_PRESS_MS`) → "force SBC shutdown", fired once.
   - **RELEASED** → re-arm.

Why a *pulse* and not a level: the SBC's power button is edge/duration sensitive,
exactly like a PC front-panel button. The firmware emulates a finger press by
driving the line for a measured duration.

### 4.2 `StartSensorTask` — 100 Hz sensors + power health — main.c:2061

Purpose: feed the host the localization + health data it needs, on a strict
`vTaskDelayUntil` 10 ms cadence (no drift).

Per-tick logic (with internal sub-rate counters):
- **Every tick (100 Hz):** read IMU (accel+gyro) and magnetometer, pack into
  `ImuPacket_t`, `udp_queue_send_imu()`. This is the fastest stream because
  odometry/heading fusion on the host wants high-rate IMU.
- **Every ~10 ticks (~10 Hz):** `VoltageMeasurement()` — sweep ADC ranks for all
  power rails (motor / 12 V / main / charger), scale via `calcVoltage()`, and run
  **charger logic**: if the filtered charger voltage < 20 V, disable charging;
  otherwise, when *discharging*, enable the charger input. This is the board
  protecting itself from plug/unplug transients.
- **Every ~100 ticks (~1 Hz):** `Battery_Update_Example()` reads the LTC2944
  coulomb counter → fills `BatteryGauge_t` (V, mA ±, °C, state, SoC %) →
  `udp_queue_send_bat_info()`. Battery state changes slowly, so 1 Hz is plenty.

The sub-rate counter pattern (`gauge_sensor_count`, `voltage_check_count`) is the
idiomatic way to run *multiple rates inside one fixed-rate task* without extra
timers or threads.

### 4.3 `StartMotorTask` — Modbus master + motor state machine (50 ms) — main.c:2368

This is the heart of the system. Steps:

**One-time init (top of the task):**
1. Create `modbus_events` flags, `ModbusMaster_Init(&g_modbus, &huart2)`.
2. `sbc_com_t.time_out = 100` ms — the **command-staleness watchdog window**.
3. `USART2_EnableInterrupts()` — byte-wise RX with T3.5 idle framing.
4. Set slave IDs, set both managers' state to `INIT_YET`.
5. Seed `speed_error` / resolution on both controllers.

**Per-tick loop:**
1. `MotorEmergencyButtonStateUpdate()` — *infer* e-stop from voltage:
   if `motor_voltage <= battery_voltage` **or** `motor_voltage <= 20 V`, the
   e-stop is treated as **pressed** (motor bus was cut). Clever: there's no
   dedicated e-stop GPIO; the firmware detects that the motor rail collapsed.
2. If **not** in e-stop on either side → `GetMotorControllerData()` for both
   slaves (Modbus read of 10 holding regs: version, temp, status, bus V, encoder
   hi/lo, rpm, torque, error) → `SendMotorInfo()` (real data, 0x15).
   If **in** e-stop → `SendFakeMotorInfo()` (zeros for motion fields) so the host
   still gets a heartbeat that clearly says "stopped".
3. `MotorControlLoop(...)` — runs each motor's state machine, then drives speed.
4. **Adaptive cadence:** if either side hasn't reached `CONTROL_LOOP` yet, delay
   **500 ms** (slow, careful init); once both are running, delay **50 ms** (the
   real control rate). This keeps init Modbus traffic gentle and the run loop fast.

---

## 5. The motor state machine (motor_interface.c) — the real control logic

Each motor is independently driven through `MotorManagerSeq_e`:

```
INIT_YET ──► RW_S_VALUE_INIT_AND_SAVE ──► CONTROL_MODE_SETUP ──► CONTROL_LOOP ◄──► EMERGENCY_LOOP
   (0)              (1)                         (2)                   (3)              (4)
```

`ProcessSingleMotorStateMachine()` (motor_interface.c:316):

- **INIT_YET → RW_S_VALUE_INIT_AND_SAVE:** trivial advance (one tick settle).
- **RW_S_VALUE_INIT_AND_SAVE → `ProcessMotorInit()`:** write P/I/F speed gains
  (`SPEED_GAIN_P/I/F`), set velocity mode, **lock the shaft**, then **save to
  EEPROM**. Only on a successful EEPROM write does it advance to setup. *Guarded*
  by an e-stop check and a recovery-delay check at the top of the state — you
  never reconfigure a driver while e-stopped.
- **CONTROL_MODE_SETUP → `ProcessMotorSetup()`:** read back everything
  (resolution, acc/dec/jerk, gains) to confirm the driver accepted the config,
  then advance to `CONTROL_LOOP`.
- **CONTROL_LOOP:** normal running. If e-stop fires → set recovery trigger and
  jump to `EMERGENCY_LOOP`. On recovery, re-assert velocity mode and re-lock the
  shaft if the status word shows it dropped out.
- **EMERGENCY_LOOP:** wait for e-stop to clear, then require it to *stay* clear
  for `EMERGENCY_RECOVERY_COUNT_MAX` (40) consecutive ticks before returning to
  `CONTROL_LOOP`. This **debounce-on-recovery** prevents chattering back into
  motion the instant the voltage flickers.

### 5.1 Speed application — `ProcessMotorSpeed()` (motor_interface.c:376)

Runs only when **both** motors are in `CONTROL_LOOP`. The gate is
`ControlTimeoutCheck()` → `control_valid`:

- **`control_valid == true`** (a fresh host command arrived within 100 ms):
  - If `status_word == 0` (driver disengaged) → re-lock the shaft first.
  - Else → `MotorSpeedControl(-target_rpm * speed_resolution)`. The **negation**
    and **resolution scaling** convert host RPM into the driver's signed,
    resolution-scaled velocity register value (sign = direction convention).
- **`control_valid == false`** (host went silent > 100 ms — the **dead-man's
  switch**):
  - If the motor is essentially stopped (`IsMotorStopped`, threshold 1 RPM) and
    still engaged → **unlock the shaft** (coast/free).
  - Else → command **0 RPM** to actively decelerate to a stop.

This is the single most important safety behavior in the firmware: **if the host
crashes or the network dies, the robot stops on its own within ~100 ms.**

---

## 6. The Modbus master (modbus_master.c) — how a motor write actually happens

Design (per the source header): polling/blocking, RTS-controlled half-duplex,
byte-wise RX interrupt, T3.5 idle gap to detect end-of-frame, table-driven CRC16.

Transaction steps (`ModbusMaster_Transaction_Blocking`):
1. Build the PDU: `[slave][func][addr_hi][addr_lo][...][crc_lo][crc_hi]`.
2. **RTS = HIGH** (PD4) → put the RS485 transceiver in TX.
3. `HAL_UART_Transmit` the frame; wait for TC (transmit complete).
4. **RTS = LOW** → switch to RX. *Timing here is critical* — flip too early and
   you clip your own last byte; too late and you miss the slave's reply.
5. Collect bytes in the RX ISR (`Modbus_RxHandler`), updating `last_rx_tick`.
6. When the bus is idle for ≥ T3.5, treat the frame as complete; validate CRC16
   and the slave/function echo. Errors map to `MODBUS_ERR_*` codes.
7. Up to `MODBUS_TIMEOUT_MS` (50 ms) to get a reply, else `MODBUS_ERR_TIMEOUT`.

Two slaves (`0x01` left, `0x02`/`0x04` right) share the bus; the motor task polls
them sequentially each cycle. (`motor_driver_config.c` adds optional round-robin
polling and SINGLE/DUAL mode for the alternate ZLTECH 2-channel controller,
selected at compile time by `MOTOR_CONTROLLER_MODEL`.)

---

## 7. The UDP protocol (protocol.h + udpserver.c) — framing & integrity

**Why byte-stuffing:** the payload can contain any byte, including `0xAA`/`0x55`.
To keep the frame self-delimiting, any payload byte equal to SOF/EOF/ESC is
replaced by `ESC (0x1B)` followed by `byte XOR 0x20`. The decoder reverses this.

**Frame on the wire:**
```
SOF(0xAA) │ [ stuffed: PacketHeader_t + payload + CRC ] │ EOF(0x55)
PacketHeader_t = { uint8 packet_type; uint16 payload_length; uint8 crc8 }
```

**Decode path** (`fncPacketDecode`, udpserver.c:127 → `process_received_packet`):
1. Strip SOF/EOF, un-stuff ESC sequences.
2. Re-compute CRC8 over the payload, compare to `header->crc8` (drop if mismatch).
3. Verify `payload_length == sizeof(expected struct)` before casting — a basic
   but important guard against malformed/short packets.
4. Dispatch by `packet_type`. Today only `DEF_MOTOR_SPEED` (0x13) actually
   mutates state; the other inbound types are accepted but unused.

**Packet table (recap):** 0x11 IMU, 0x12 motor, 0x13 speed-cmd (in), 0x14 set-IP
(in), 0x15 dual-motor, 0x16 battery, 0x17 voltages. Directions and full struct
layouts are in `readme.md` §UDP protocol.

---

## 8. Shared state & concurrency model (why it's safe without big locks)

| Global | Written by | Read by | Protection |
|--------|-----------|---------|------------|
| `sbc_com_t.l_rpm/r_rpm/last_control_time` | udp_rx_thread | motorTask | single writer; word-sized writes are atomic on M7 |
| `motor1_data` / `motor2_data` | motorTask | motorTask, telemetry build | single owner (motorTask) |
| `measured_voltage_t` | sensorTask | sensorTask, motorTask (e-stop infer) | single writer |
| `g_battery` | sensorTask | sensorTask | single writer |
| UDP TX queue | all tasks (enqueue) | tx_thread (dequeue) | RTOS message queue (thread-safe) |
| `udp_stats` | tx/rx threads | callers | `stats_mutex` |

The design leans on **one writer per datum** plus **a thread-safe queue for the
many-producers-one-consumer TX path**. That's why there are very few explicit
mutexes — and why, if you extend this, you must keep that discipline.

---

## 9. If you were starting from scratch — the build recipe

A concrete order of operations to recreate this firmware:

1. **CubeMX device config**
   - Pick STM32H723ZG / NUCLEO-H723ZG; HSE 25 MHz; clock tree to a stable SYSCLK.
   - Enable: ETH (RMII), USART1, USART2, I2C2/4/5, SPI4, ADC1/2/3, TIM6 (HAL tick).
   - Add FreeRTOS (CMSIS-RTOS v2) and LwIP (static IP `10.0.0.200/24`, no DHCP).
   - **Configure the MPU** for the Ethernet DMA / LwIP RAM region (non-cacheable).
2. **Pin map** — RS485 RTS on PD4, IMU power/CS pins, all rail-enable GPIOs, the
   SBC power-button input and PC_PWR_ON output (see `main.h` pin defines).
3. **Power sequencing** — implement `GPIO_On_Mode()` with the staged delays.
4. **Drivers in** — drop in HAL, LAN8742 PHY, LSM6DSOX/LIS3MDL/LTC2944 vendor
   drivers, and the Modbus master.
5. **Protocol layer** — `protocol.h` structs + `udpserver.c` encode/decode/queue
   + rx/tx threads.
6. **Motor layer** — `zlih65rc.h` register map, `motor_interface.c` state machine,
   `motor_driver_config.c` for single/dual.
7. **Tasks** — wire up the three tasks at 50/10/50 ms; create them in
   `MX_FREERTOS_Init`; `osKernelStart()`.
8. **Bring-up order to debug** — (a) blink / UART1 console, (b) ping `10.0.0.200`,
   (c) confirm IMU telemetry on host, (d) single Modbus read of one driver,
   (e) full state machine reaching `CONTROL_LOOP`, (f) closed-loop speed + dead-man test.

---

## 10. Things to watch / likely failure points

- **MPU / cache region** — #1 cause of "Ethernet works sometimes" on H7.
- **RS485 RTS timing** — wrong TX→RX turnaround clips frames; verify on a scope.
- **`speed_resolution == 0`** — `IsMotorStopped()` and the velocity scaling depend
  on it; it's read from the driver during setup, so a failed setup read can mis-scale.
- **Slave-ID mismatch** — `MOTOR2_SLAVE_ID` is `0x04` in `zlih65rc.h` but the
  driver-config manager and README assume `0x02`. Confirm the *physical* DIP/EEPROM
  ID on each driver matches what the master polls, or motor 2 silently times out.
- **E-stop is inferred from voltage**, not a hard input — if the motor rail is
  noisy near 20 V the state machine can chatter (mitigated by the 40-tick recovery
  debounce, but worth knowing).
- **Telemetry is lossy by design** — never build host logic that assumes every
  packet arrives; rely on the latest value + timestamps.
- **Command dead-man is 100 ms** — the host must send `DEF_MOTOR_SPEED` faster
  than 10 Hz or the robot will repeatedly decelerate / coast.

---

## 11. Quick reference — where to look

| To understand… | Read |
|----------------|------|
| Boot + tasks + power + ADC | `Core/Src/main.c` (`main`, `Start*Task`, `GPIO_On_Mode`, `VoltageMeasurement`) |
| Motor state machine + safety | `Core/Src/motor_interface.c` (`MotorControlLoop`, `ProcessMotorSpeed`, `ProcessSingleMotorStateMachine`) |
| Modbus framing / CRC / RTS | `Core/Src/modbus_master.c` |
| UDP framing / queue / threads | `Core/Src/udpserver.c` |
| Wire structs + packet types | `Core/Inc/protocol.h` |
| Driver registers + tuning consts | `Core/Inc/zlih65rc.h` |
| Single/Dual driver mgmt | `Core/{Inc,Src}/motor_driver_config.*` |
| Network / IP config | `LWIP/App/lwip.c`, `Core/Inc/udpserver.h` |
