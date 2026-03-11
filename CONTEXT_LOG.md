# Camera_Board_IMU — Context Log

## Session 3 — 2026-02-20

### Problem: Stream not starting after START_STREAMING
The symptom was: init sequence completed (WHO_AM_I ok, START_STREAM DRDY ok) but the
streaming `while(1)` loop never received further DRDY events.

**Root cause:** `send_command` was called for START_STREAMING with
`rx_size = SIZE_STREAM_INIT_RESPONSE` (46). Inside `send_command` it reads
`rx_size + 1 = 47` bytes. Since `SIZE_STREAM_INIT_RESPONSE` is already the full
46-byte packet, reading 47 bytes over-reads by 1 byte and corrupts the SPI bus state,
leaving NSS timing misaligned for subsequent DRDY pulses.

**Fix applied:** Pass `SIZE_STREAM_INIT_RESPONSE - 1` (= 45) to `send_command` so it
reads exactly 46 bytes (45 payload + 1 echo = 46 total via `rx_size + 1`).

### Changes in this session

| File | Change |
|---|---|
| `imu_handler.h` | Added error code defines (`IMU_ERR_*`) and `IMU_Handler_ProcessStreamPacket` API |
| `imu_handler.c` | Replaced with full ST_Link-test version: header/cmd validation, `decode_stream_pkt` core, both public functions |
| `proto_uart_imu.c` | Written from scratch — COBS+CRC16 orientation event using identical `ui_send_imu_orient` pattern from `mcu_fsm.c` |
| `main.c` | Reconstructed: added `imu_handler.h`/`proto_uart_imu.h` includes, `IMU_Handler_Init` + `ProtoUartIMU_Init` in startup, streaming loop replaced with `IMU_Handler_ProcessPacket` + `ProtoUartIMU_SendOrientation` |

### Streaming loop (final)
```c
while (1) {
    // wait DRDY (1 s timeout)
    HAL_GPIO_WritePin(NSS, RESET);
    HAL_Delay(1);                         // 1 ms NSS setup (was 10 ms)
    HAL_SPI_Receive(&hspi2, buf, 70, ...);
    HAL_GPIO_WritePin(NSS, SET);

    int r = IMU_Handler_ProcessPacket(buf, 70, &sample);  // strips echo byte internally
    if (r == IMU_ERR_OK) {
        ProtoUartIMU_SendOrientation(&sample);            // COBS+CRC16 EVT_IMU_ORIENT
        // optional ASCII debug: Roll/Pitch/Yaw
    }
}
```

---


**Date:** 2026-02-19  
**Project:** `Camera_Board_IMU` (STM32G0C1RETx, STM32CubeIDE 1.16.1)  
**MCU:** STM32G0C1RETX  
**IMU:** TDK InvenSense IIM-46234 (SPI bridge, streaming mode)

---

## Overview

This document records the design decisions and implementation details for the IMU orientation computation and PC-UI communication layer added to the `Camera_Board_IMU` firmware project.

---

## Hardware

| Signal     | MCU Pin | Notes                          |
|------------|---------|--------------------------------|
| IMU RESET  | PA0     | Active-low, output             |
| SPI2 NSS   | PB12    | Software-controlled output     |
| SPI2 SCK   | PB13    | AF0                            |
| SPI2 MISO  | PB14    | AF0                            |
| SPI2 MOSI  | PB15    | AF0                            |
| IMU DRDY   | PB9     | EXTI rising edge, IRQ priority 0 |
| UART3 TX/RX| USART3  | 115200 8N1, debug + PC-UI      |

---

## IIM-46234 SPI Packet Structure

The IIM-46234 communicates via an SPI bridge protocol. All responses include a
leading **echo byte** (the last byte shifted out during TX) followed by the
actual payload.

### Stream Data Packet (`actual_stream = raw_spi_buf + 1`)

| Byte(s) | Field            | Type             | Notes                     |
|---------|-----------------|------------------|---------------------------|
| 0–1     | Header           | `0x24 0x24`      |                           |
| 2       | Length (low)     | uint8            |                           |
| 3       | Command type     | `0xAB`           | Stream data identifier    |
| 4       | Sensor status    | uint8            |                           |
| 5       | Sample counter   | uint8            | Wraps at 255              |
| 6–9     | Timestamp high   | uint32 big-endian| Upper 32 bits of µs clock |
| 10–13   | Timestamp low    | uint32 big-endian| Lower 32 bits of µs clock |
| 14–17   | Accel X          | float32 big-endian | m/s²                    |
| 18–21   | Accel Y          | float32 big-endian | m/s²                    |
| 22–25   | Accel Z          | float32 big-endian | m/s²                    |
| 26–29   | Gyro X           | float32 big-endian | rad/s                   |
| 30–33   | Gyro Y           | float32 big-endian | rad/s                   |
| 34–37   | Gyro Z           | float32 big-endian | rad/s                   |
| 38–41   | Temperature      | float32 big-endian | °C                      |
| 42–43   | Checksum         | uint16 big-endian | Sum of bytes [3..41]     |
| 44      | Footer           | `0x0D`           |                           |
| 45      | Footer           | `0x0A`           |                           |

---

## New Source Files

### `imu_madgwick.h / imu_madgwick.c`

Implements the **Sebastian Madgwick AHRS algorithm** (6-DOF variant, no magnetometer).

- **Input:** Gyroscope (rad/s), Accelerometer (any consistent unit, normalised internally)
- **Output:** Quaternion `(q0=w, q1=x, q2=y, q3=z)` + Euler angles (roll/pitch/yaw, degrees, ZYX convention)
- **Filter gain `beta`:** Default `0.1f`. Higher = faster convergence, more noise sensitivity.
- **Sample rate:** Passed at init time (`100.0f` Hz matches IIM-46234 default ODR).
- The filter starts with an identity quaternion (no rotation assumed).

**Key types:**
```c
typedef struct { float q0, q1, q2, q3; }      Quaternion_t;
typedef struct { float roll, pitch, yaw; }     EulerAngles_t;
typedef struct { float beta, sample_freq; Quaternion_t q; } MadgwickFilter_t;
```

---

### `imu_handler.h / imu_handler.c`

Wraps the packet parser + Madgwick pipeline into a single call.

```c
void IMU_Handler_Init(float sample_freq, float beta);
int  IMU_Handler_ProcessPacket(const uint8_t *raw_spi_buf, uint32_t buf_len, IMU_Sample_t *out);
```

- Strips the echo byte, finds the footer, verifies the checksum.
- Decodes all sensor fields into `IMU_Sample_t`.
- Calls `Madgwick_Update()` and stores `euler` + `quaternion` into the sample struct.
- Returns `1` on success, `0` on any error (footer not found, too short, checksum mismatch).

---

### `proto_uart_imu.h / proto_uart_imu.c`

Binary framing protocol over UART3 for sending orientation data to a PC UI.

#### Frame Format

```
[0x24][0x24][payload_len][msg_type][payload...][chk_hi][chk_lo][0x0D][0x0A]
```

- Header: `0x24 0x24`
- `payload_len`: number of payload bytes (does **not** include checksum or footer)
- Checksum: uint16 sum of `[msg_type + payload bytes]`, big-endian
- Footer: `0x0D 0x0A`

#### Message Types

| Type | ID   | Payload (bytes) | Contents                                |
|------|------|-----------------|----------------------------------------|
| ORIENTATION | `0x01` | 28 | roll, pitch, yaw (3×float32 BE) + q0,q1,q2,q3 (4×float32 BE) |
| RAW_IMU     | `0x02` | 28 | ax,ay,az (3×float32 BE) + gx,gy,gz (3×float32 BE) + temp (float32 BE) |
| STATUS      | `0x03` | var | ASCII string (no null terminator in frame) |

**API:**
```c
void ProtoUartIMU_Init(UART_HandleTypeDef *huart);
void ProtoUartIMU_SendOrientation(const IMU_Sample_t *sample);
void ProtoUartIMU_SendRawIMU(const IMU_Sample_t *sample);
void ProtoUartIMU_SendStatus(const char *msg);
```

---

## Integration into `main.c`

Changes made to `main.c`:

1. Added `#include "imu_handler.h"` and `#include "proto_uart_imu.h"`.
2. After `MX_SPI2_Init()`:
   ```c
   IMU_Handler_Init(100.0f, 0.1f);  // 100 Hz ODR, beta=0.1
   ProtoUartIMU_Init(&huart3);
   ```
3. Inside the streaming `while(1)` loop, after successful legacy decode:
   ```c
   IMU_Sample_t imu_sample;
   if (IMU_Handler_ProcessPacket(stream_packet, 70, &imu_sample)) {
       // Debug print Euler + quaternion to UART terminal
       ProtoUartIMU_SendOrientation(&imu_sample);  // binary frame to PC UI
       ProtoUartIMU_SendRawIMU(&imu_sample);        // binary frame to PC UI
   } else {
       ProtoUartIMU_SendStatus("[IMU] Parse error");
   }
   ```

---

## Design Decisions

| Decision | Rationale |
|----------|-----------|
| Madgwick 6-DOF (no mag) | IIM-46234 does not have an integrated magnetometer; yaw will drift over time but roll/pitch are stable |
| beta = 0.1f | Good balance between convergence speed and noise rejection for typical camera-board vibration profiles |
| sample_freq = 100 Hz | Matches the IIM-46234 default streaming ODR; must be updated if ODR changes |
| Big-endian floats | IIM-46234 SPI bridge outputs all floats in big-endian (network byte order) |
| Gyro units = rad/s | The sensor outputs angular rate in rad/s in float mode — used directly by Madgwick |
| Binary protocol over UART3 | Compact and parse-friendly for a PC-side UI; shares the same UART3 used for debug prints |
| Checksum = simple byte sum | Matches the IIM-46234 native protocol convention already used for SPI commands |

---

## PC-Side Parsing Notes

To decode `ORIENTATION` frames (type `0x01`) on the PC:

1. Wait for two consecutive `0x24` bytes (SOF).
2. Read `payload_len` (1 byte).
3. Read `msg_type` (1 byte).
4. Read `payload_len` bytes of payload.
5. Read 2-byte checksum and verify (`sum of [msg_type, payload...]`).
6. Read `0x0D 0x0A` footer.
7. Unpack 7 × big-endian float32: `roll, pitch, yaw, q0, q1, q2, q3`.

---

## Known Limitations / Future Work

- **Yaw drift:** Without a magnetometer, yaw accumulates gyro drift. Future work: add magnetic reference or external heading source.
- **ODR synchronisation:** `sample_freq` is hardcoded to 100 Hz; it should be derived dynamically from the IIM-46234 timestamp field.
- **Interrupt-driven SPI:** Currently uses polling SPI in the DRDY ISR context (drdy_interrupt flag polled in main loop). For higher ODRs, migrate to DMA-driven SPI.
- **UART3 dual-use:** Both ASCII debug prints and binary protocol frames share UART3. The PC UI must be able to distinguish/skip ASCII lines from binary frames (SOF `0x24 0x24` is unique enough for this).
