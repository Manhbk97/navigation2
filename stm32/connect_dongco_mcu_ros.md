# rgt2_protocol

Software Architecture: rgt2_protocol ↔ STM32H723 MCU
System Overview

┌────────────────────────────────────────────────────────────────────────┐
│                        ROBOT SYSTEM                                    │
│                                                                        │
│  ┌─────────────────────────────────┐    Ethernet (RMII)               │
│  │   STM32H723 MCU (10.0.0.200)   │◄──────────────────────────────┐  │
│  │   FreeRTOS + LwIP               │                               │  │
│  └─────────────────────────────────┘                               │  │
│                                                                     │  │
│  ┌─────────────────────────────────┐    UDP Bidirectional          │  │
│  │  ROS2 Host PC (10.0.0.100)      │◄──── port 8080/8081 ─────────┘  │
│  │  rgt2_protocol node             │                                  │
│  └─────────────────────────────────┘                                  │
│               │                                                        │
│               │ ROS2 Topics                                            │
│               ▼                                                        │
│  ┌─────────────────────────────────┐                                  │
│  │  Navigation2 / Control Nodes    │                                  │
│  │  (nav2, slam, teleop, etc.)     │                                  │
│  └─────────────────────────────────┘                                  │
└────────────────────────────────────────────────────────────────────────┘



Layer Architecture

┌──────────────────────────────────┐   ┌──────────────────────────────────┐
│     HOST PC (ROS2)               │   │     STM32H723 MCU                │
│                                  │   │                                  │
│  ┌────────────────────────────┐  │   │  ┌────────────────────────────┐  │
│  │   Navigation / Control     │  │   │  │   Motor Drivers (RS485)    │  │
│  │   (nav2, teleop, slam)     │  │   │  │   ZLIH65RC  (Modbus RTU)   │  │
│  └──────────┬─────────────────┘  │   │  └──────────▲─────────────────┘  │
│             │ ROS2 Topics        │   │             │ Modbus RTU         │
│  ┌──────────▼─────────────────┐  │   │  ┌──────────┴─────────────────┐  │
│  │   UdpCommunicationNode     │  │   │  │   Motor Interface Layer    │  │
│  │   (rgt2_protocol)          │  │   │  │   motor_interface.c        │  │
│  │                            │  │   │  │   motor_driver_config.c    │  │
│  │  Subscribers:              │  │   │  └──────────▲─────────────────┘  │
│  │  - /cmd_vel → MotorSpeed   │  │   │             │                    │
│  │                            │  │   │  ┌──────────┴─────────────────┐  │
│  │  Publishers:               │  │   │  │   FreeRTOS Tasks           │  │
│  │  - imu/data                │  │   │  │   mainTask                 │  │
│  │  - imu/mag                 │  │   │  │   sensorTask               │  │
│  │  - motor/info              │  │   │  │   motorTask                │  │
│  │  - motor/e_stop            │  │   │  └──────────▲─────────────────┘  │
│  │  - battery/gauge           │  │   │             │                    │
│  │  - enc_diff                │  │   │  ┌──────────┴─────────────────┐  │
│  │  - diagnostics             │  │   │  │   UDP Server Layer         │  │
│  └──────────┬─────────────────┘  │   │  │   udpserver.c (port 8080)  │  │
│             │                    │   │  │   Queue (20 packets)       │  │
│  ┌──────────▼─────────────────┐  │   │  └──────────▲─────────────────┘  │
│  │   PacketCodec              │  │   │             │                    │
│  │   (packet_codec.cpp)       │  │   │  ┌──────────┴─────────────────┐  │
│  │   encode / decode          │  │   │  │   PacketEncode/Decode      │  │
│  │   CRC8 (poly 0x31)         │  │   │  │   fncPacketEncode()        │  │
│  └──────────┬─────────────────┘  │   │  │   fncPacketDecode()        │  │
│             │                    │   │  └──────────▲─────────────────┘  │
│  ┌──────────▼─────────────────┐  │   │             │                    │
│  │   UDP Socket               │  │   │  ┌──────────┴─────────────────┐  │
│  │   TX: 10.0.0.200:8080      │  │   │  │   LwIP TCP/IP Stack        │  │
│  │   RX: 10.0.0.100:8081      │  │   │  │   + LAN8742 PHY (RMII)     │  │
│  └────────────────────────────┘  │   │  └────────────────────────────┘  │
└──────────────────────────────────┘   └──────────────────────────────────┘
Shared Packet Protocol (protocol.h — identical on both sides)
Both projects share the same protocol.h. This is the wire format exchanged over UDP.


Wire Frame Layout:
┌───────┬───────────────────────────────────────────────┬──────┬───────┐
│  SOF  │  Escaped Payload  (Header + Data + CRC8)      │ CRC8 │  EOF  │
│ 0xAA  │  [type][len_lo][len_hi][hdr_crc][...data...]  │      │ 0x55  │
└───────┴───────────────────────────────────────────────┴──────┴───────┘

Byte stuffing (escape):
  If any byte == SOF(0xAA) | EOF(0x55) | ESC(0x1B)
    → transmit [ESC(0x1B), byte XOR 0x20]
Packet Type Table
ID	Direction	Struct	Description
0x11	MCU → ROS2	ImuPacket_t	9x float: gx/gy/gz, ax/ay/az, mag_x/y/z
0x12	MCU → ROS2	MotorInfo_t	Single motor: version, voltage, status, encoder, RPM, torque
0x13	ROS2 → MCU	MotorSpeed_t	double left_rpm, right_rpm (command)
0x14	—	—	Target IP (unused in current code)
0x15	MCU → ROS2	DoubleMotorInfo_t	Dual motor combined status + emergency state
0x16	MCU → ROS2	BatteryGauge_t	voltage, current, temperature, SOC
0x17	MCU → ROS2	MeasuredVoltage_t	9-channel ADC power rail readings
