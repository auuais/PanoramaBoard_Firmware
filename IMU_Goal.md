# IMU Goal — IIM-46234 High-Rate Orientation

## Primary Goal
Achieve **1 kHz** IMU data rate for accurate, low-latency orientation computation.

**Current status (2026-02-26, batch 14):** Three SPI driver bugs found and fixed
that prevented consecutive commands from succeeding after WHO_AM_I:
1. `spi2_flush()` was not re-asserting `FRXTH=1` — HAL resets it, leaving 16-bit threshold
2. `IMU_NSS_HIGH_US` was 20 µs — sensor needs ≥300 µs after CS HIGH before DRDY fires
3. Phase 2 DRDY poll only checked interrupt flag — missed edges that fired in the race window

With these fixes `imu_configure_odr_lpf` (3 WRITE_REGs) is re-enabled after the second reset,
so ODR=1000 Hz and SELECT_OUT_DATA=0x07 are written explicitly every boot.

---

## Observation (confirmed, reproducible)

| Sequence | Result |
|---|---|
| Reset → WHO_AM_I → SAMPLE_RATE_DIV → START_STREAM | **FAILS** — streams at 50 Hz |
| Reset → WHO_AM_I → Reset → SAMPLE_RATE_DIV → START_STREAM | **WORKS** — streams at 500 Hz |
| Reset → SAMPLE_RATE_DIV (no WHO_AM_I first) | **FAILS** |

The hard reset between WHO_AM_I and the first WRITE_REG is the only reliable way
to get the WRITE_REG accepted.

---

## Investigation History

### ❌ Attempt 1 — SPI Mode mismatch (WRONG hypothesis)
**Hypothesis:** ATMEL `CPOL=1, CPHA=0` → SPI Mode 2. Our STM32 was Mode 3.  
**Disproven:** Datasheet explicitly states Mode 3. Atmel SAM SPI uses **NCPHA**
(Not Clock Phase), so NCPHA=0 = standard CPHA=1 = Mode 3. No change needed.
`SPI_POLARITY_HIGH + SPI_PHASE_2EDGE` = Mode 3 = **correct**.

### ❌ Attempt 2 — DRDY level-check causing false-positives
**Fix applied:** Removed pin-level check — now only uses rising-edge interrupt flag.  
**Result:** Did not fix the core problem.

### ❌ Attempt 3 — No DRDY-low wait between commands
**Fix applied:** Added wait for DRDY to go LOW after every RX read, then clear flag.  
**Result:** Did not fix the core problem.

### ❌ Attempt 4 — SPI RX FIFO flush + 20+8 byte RX clocking (batch 2, 2026-02-25 AM)
**Fixes applied:**
- `spi2_flush()` before every CS assertion — drains RX FIFO, waits BSY=0
- Clock 20 + 8 extra dummy bytes in RX phase, CS stays LOW throughout
- 10 ms delay after WHO_AM_I before WRITE_REG
- SAMPLE_RATE_DIV read-back verify after write
- Duplicate BW_CONFIG write removed from `imu_config.c`

**Log result:** SAMPLE_RATE_DIV RX20 returned `23 23 0A 12...` (looked OK), but
readback READ_REG timed out with DRDY timeout r=-2. BW_CONFIG also timed out.

**Root cause of timeout:** The TX and RX were in **separate CS transactions**
(CS raised after TX, then re-asserted for RX after DRDY). This left the sensor
SPI shift register in an indeterminate state, preventing response to next command.

### ❌ Attempt 5 — CS stays LOW for TX → DRDY-wait → RX (2026-02-25 PM, batch 3)
**Hypothesis:** ATMEL `spi_master_write_read()` keeps CS LOW for the entire
TX + DRDY-wait + RX sequence. Raising CS between TX and RX corrupts the sensor
state machine.

**Fix applied:** Rewrote `send_imu_command_ex` so CS stays LOW from TX start
through DRDY wait through RX, then CS HIGH only after all bytes clocked.  
Also increased inter-command gap to 5 ms and added pin-level DRDY check in
Phase 2 as a belt-and-suspenders.

**Log result:**
```
IMU RX20 [SAMPLE_RATE_DIV]: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
IMU: no 0x2323 hdr [SAMPLE_RATE_DIV]
[ODR] SAMPLE_RATE_DIV write FAILED r=-3
```
All zeros — DRDY fired immediately (no timeout) but response was empty.

**Root cause identified:** **Stale DRDY from the IMU_Reset() boot pulse.**
After reset, the sensor asserts DRDY as a boot-complete signal. `IMU_Reset()`
clears `drdy_interrupt=0` after waiting for DRDY LOW, but the sensor can
re-assert DRDY shortly after. When Phase 2 polls `HAL_GPIO_ReadPin` it sees
HIGH from the boot pulse — not from the command — and immediately proceeds
to RX. The sensor has not yet processed the command so MISO is all zeros.

The pin-level shortcut (`if pin==HIGH → accept`) in Phase 2 was the direct
cause: it bypassed the interrupt-edge requirement and latched the stale
boot-DRDY as valid.

### ✅ Attempt 6 — Two-step DRDY handshake + hardened IMU_Reset (2026-02-25, batch 4)
**Root cause:** Stale DRDY boot pulse accepted as valid command response.

**Fix A — Phase 2 two-step handshake in `send_imu_command_ex`:**
1. **Step A:** Wait for DRDY LOW (flush any stale high state from boot or prior command). Timeout 50 ms.
2. Clear `drdy_interrupt = 0` — discard any edge accumulated before/during TX.
3. **Step B:** Wait for fresh DRDY rising edge via **interrupt flag only** (no pin-level check). Timeout 500 ms.
This ensures only a real command-response DRDY (a new rising edge after our TX) is accepted.

**Fix B — Hardened `IMU_Reset()` post-boot flush:**
- After 300 ms boot wait: wait for DRDY LOW (boot pulse gone, up to 200 ms)
- `HAL_Delay(50)` — extra settling time
- `drdy_interrupt = 0` — clear once
- `HAL_Delay(10)` — wait for any secondary transient edge
- `drdy_interrupt = 0` — clear again

**Status:** Flashed, pending test results.

---

## Current Code State (2026-02-25, batch 4)

### `send_imu_command_ex` SPI flow
```
spi2_flush()
drdy_interrupt = 0           ← clear before TX
CS LOW
delay 5 µs
TX 20 bytes (CS stays LOW)

[Phase 2 — two-step DRDY handshake, CS stays LOW]
  Step A: poll pin LOW (flush stale), timeout 50 ms
  drdy_interrupt = 0          ← discard edges accumulated during TX
  Step B: poll drdy_interrupt (interrupt-only, no pin level), timeout 500 ms

[Phase 3 — RX, CS still LOW]
  Clock (fixed_rx + 8) bytes
CS HIGH
delay 20 µs

drdy_interrupt = 0
HAL_Delay(5)                 ← 5 ms inter-command gap
```

### `IMU_Reset()` post-boot flush
```
PA0 LOW 10 ms → PA0 HIGH → HAL_Delay(300)
Wait DRDY LOW (up to 200 ms)
HAL_Delay(50)
drdy_interrupt = 0
HAL_Delay(10)
drdy_interrupt = 0
Log "IMU: reset"
```

### `mcu_fsm.c` ST_INIT_IMU sequence
```
IMU_Reset()                          ← first reset
WHO_AM_I (send_imu_command_ex)
HAL_Delay(10)
IMU_Reset()                          ← second reset (known-working pattern)
imu_read_config("PRE")               ← read SAMPLE_RATE_DIV, BW_CONFIG, OUT_DATA_FORM
imu_configure_odr_lpf(1, 0x44)       ← WRITE SAMPLE_RATE_DIV=1 (1kHz), BW_CONFIG=0x44
  internally: WRITE_REG SAMPLE_RATE_DIV
              READ_REG  SAMPLE_RATE_DIV  (inline readback verify)
              WRITE_REG BW_CONFIG
imu_read_config("POST")              ← read back all three registers to confirm
IMU_Handler_Init(1000Hz)
START_STREAM
```

### Register readback (`imu_read_config`)
Reads and logs three registers using the existing `imu_read_reg()` helper:

| Register | Page | Addr | Len | Decoded output |
|---|---|---|---|---|
| `SAMPLE_RATE_DIV` | 0 | 0x1A | 2 | `ODR = 1000 / divisor` Hz |
| `BW_CONFIG` | 0 | 0x30 | 1 | accel_bw nibble + gyro_bw nibble |
| `OUT_DATA_FORM` | 0 | 0x19 | 1 | `float IEEE754` or `fixed 2s-comp` |

Expected PRE log (sensor defaults after reset):
```
[CFG PRE] SAMPLE_RATE_DIV=0x0014  ODR=50 Hz
[CFG PRE] BW_CONFIG=0x44  accel_bw=4 gyro_bw=4
[CFG PRE] OUT_DATA_FORM=0x00  (float IEEE754)
```

Expected POST log (after 1 kHz write):
```
[CFG POST] SAMPLE_RATE_DIV=0x0001  ODR=1000 Hz
[CFG POST] BW_CONFIG=0x44  accel_bw=4 gyro_bw=4
[CFG POST] OUT_DATA_FORM=0x00  (float IEEE754)
```

If POST still shows `ODR=50 Hz` the write was silently rejected → the stale-DRDY fix (batch 4) did not fully work and further investigation is needed.

---

## ❌ Root Cause — 50 Hz cap despite sensor running at 1000 Hz (2026-02-25)

The PRE readback showed `SAMPLE_RATE_DIV=0x0001 → ODR=1000 Hz` — the sensor
**was already outputting 1000 Hz** from the factory default (or a previously
saved config). The 50 Hz we observed was entirely a **firmware bottleneck**:

### Bottleneck 1 — `HAL_UART_Transmit` blocking on every DRDY (critical)
`mcu_on_imu_drdy` → `ui_send_imu_orient` → `ui_send_frame` →
`HAL_UART_Transmit(&huart3, buf, len, 50)`.

`HAL_UART_Transmit` is a **synchronous blocking call**. At 115200 baud a
15-byte COBS frame takes `15 × 10 bits / 115200 = 1.30 ms` to clock out.
At 1000 DRDY/s that is **1300 ms of blocking per second** — the MCU spends
more time in UART than it has in total, so DRDY events back up and are
effectively processed at ~1/1.3ms ≈ 50 Hz.

### Bottleneck 2 — `Send_UART3_Debug` ODR log every 16 samples
Inside the ODR estimator block, `Send_UART3_Debug("IMU ODR est...")` called
another blocking `HAL_UART_Transmit` every 16 samples, adding further stalls.

### Bottleneck 3 — dt_s fallback was 0.02 s (50 Hz)
The Kalman fallback `dt_s = 0.02f` assumed 50 Hz even when running at 1 kHz,
corrupting the gyro integration rate.

### Fixes applied (2026-02-25, batch 5)

### Bottleneck 4 — Byte-by-byte SPI header search (Feb 26)

Our `mcu_on_imu_drdy` called `HAL_SPI_TransmitReceive(..., 1 byte)` up to 48
times searching for `0x23 0x23`. Each call on Cortex-M0+ @ 16 MHz costs ~5 µs
HAL overhead. Total per DRDY: **~500–700 µs** out of a 1000 µs budget.

**Fix:** Single 46-byte bulk `HAL_SPI_TransmitReceive`, matching ATMEL's
`spi_master_read(buf, 46)`. SPI time: ~600 µs → **~92 µs**. Header check is
2-position only (byte[0] or byte[1]).

---

## ✅ Root Cause — 48 Hz DRDY despite SAMPLE_RATE_DIV=1 (Feb 26, CONFIRMED)

**Scope showed ~48 Hz DRDY even though `SAMPLE_RATE_DIV` readback = 0x0001.**

The sensor was configured from a previous ATMEL session via `SELECT_OUT_DATA`.
That session set all five outputs enabled (`0x1F` = accel+gyro+temp+delta_vel+
delta_ang). This makes each stream packet:

```
18 base + 4 × 11 outputs = 62 bytes
```

The sensor internally throttles DRDY to fit 62-byte packets at its internal
SPI bus rate. At 4 MHz SPI: 62 bytes × 10 bits / 4 MHz = 155 µs per packet.
With protocol overhead the sensor cannot sustain 1 kHz at 62 bytes → it caps
at ~48 Hz.

**ATMEL code sets `SELECT_OUT_DATA = ACC|GYRO|TEMP = 0x07` by default** in
`IIM4623x_Initialize()`. The ATMEL user then manually selects outputs via menu
key `'d'` before starting streaming. Since the ATMEL demo saved `0x1F` to
sensor flash, our STM was always inheriting that saved config.

### Fix applied (Feb 26, batch 7)

`imu_configure_odr_lpf()` now writes three registers before BW_CONFIG:

| Step | Register | Addr | Value | Effect |
|---|---|---|---|---|
| 1 | `SAMPLE_RATE_DIV` | 0x1A | `0x0001` | ODR = 1000 Hz |
| 2 | **`SELECT_OUT_DATA`** | **0x1C** | **`0x07`** | **acc+gyro+temp → pkt=46 B** |
| 3 | `BW_CONFIG` | 0x30 | `0x44` | LPF BW4 both axes |
| 4 | `SAVE_ALL_CONFIG` | 0x2F | `0x50` | Persist to flash (+150 ms) |

`imu_read_config()` now also reads `SELECT_OUT_DATA` and logs decoded bits +
computed packet size so the value is always visible in PRE/POST logs:

```
[CFG PRE]  SELECT_OUT_DATA=0x1F  (acc=1 gyro=1 temp=1 dvel=1 dang=1)  pkt=62 bytes
[CFG POST] SELECT_OUT_DATA=0x07  (acc=1 gyro=1 temp=1 dvel=0 dang=0)  pkt=46 bytes
```

---

## What We Know For Sure

| Fact | Source |
|---|---|
| SPI Mode 3 (CPOL=1, CPHA=1) | Datasheet DS-000300 Table 5-1 |
| Command TX is always 20 bytes | `SIZE_PACKET_CMD = 20` in iim4623x.h |
| WRITE_REG ACK = 10 bytes | `SIZE_RESP_ACK = 10` in iim4623x.h |
| READ_REG response = 16 + data_len bytes | `SIZE_RESP_READ_REGS_BASE = 16` |
| All commands generate DRDY | ATMEL main.c drdy loop |
| CS must stay LOW for full TX+DRDY-wait+RX | ATMEL `spi_master_write_read()` |
| Sensor emits DRDY boot pulse after reset | Confirmed: stale DRDY → all-zeros RX |
| Hard reset between WHO_AM_I and WRITE_REG required | Reproducible; root unclear |
| SAMPLE_RATE_DIV=1 readback OK but DRDY was 48 Hz | Scope confirmed Feb 26 |
| **SELECT_OUT_DATA=0x1F (all outputs) throttles DRDY to ~48 Hz** | **Root cause confirmed Feb 26** |
| SELECT_OUT_DATA=0x07 (acc+gyro+temp) → pkt=46 B → 1 kHz possible | ATMEL default |
| SAVE_ALL_CONFIG needed to persist writes across reset | `SAVE_ALL_CONFIG reg={0x2F,1,0}` |

---

## ❌ Root Cause — Streaming stopped working after batches 7-9 (Feb 26)

**Symptom:** START_STREAM was sent but NO streaming data arrived (DRDY handler never
received a valid packet). All commands after the first READ_REG (WHO_AM_I) returned
`DRDY timeout r=-2`.

**Root cause:** The init sequence in `ST_INIT_IMU` was issuing **too many consecutive
SPI commands** after WHO_AM_I before START_STREAM:

```
imu_read_config("PRE")       → 4 READ_REG commands
imu_configure_odr_lpf(...)   → WRITE + READ + WRITE + WRITE + WRITE = 5 commands
imu_read_config("POST")      → 4 READ_REG commands
                               = 13 extra commands after WHO_AM_I
```

Log confirmed: `RD_SAMPLE_RATE_DIV` (command 1 of PRE) returned OK, then
`RD_BW_CONFIG` (command 2 of PRE) immediately returned `DRDY timeout`.  
Every subsequent command timed out → START_STREAM was sent into a broken sensor
state → no DRDY pulses → no streaming data.

**Why it worked before:** In earlier batches the init sequence was shorter
(no imu_read_config PRE/POST). Adding them in batch 7 broke the pipeline.

**Why the sensor stops responding after command 1:** Unknown — may be related to
sensor internal state after WHO_AM_I, SPI timing, or the 5 ms inter-command gap
being insufficient for back-to-back reads. The second hardware reset was always
the reliable "clear the slate" mechanism.

### ✅ Fix (batch 10, 2026-02-26)

**Stripped `ST_INIT_IMU` to the confirmed-working bare minimum:**

```
IMU_Reset()              ← first reset
WHO_AM_I (1 command)     ← confirm sensor alive
IMU_Reset()              ← second reset (clears WHO_AM_I side-effects)
IMU_Handler_Init(1000Hz) ← init Madgwick/Kalman (no SPI)
HAL_Delay(10)            ← let post-reset DRDY settle
START_STREAM             ← no ACK, no DRDY wait, returns immediately
→ ST_OPERATION           ← DRDY handler takes over
```

**Removed from init path:**
- `imu_read_config("PRE")` — was 4 READ_REG commands
- `imu_configure_odr_lpf(...)` — was 5 R/W commands
- `imu_read_config("POST")` — was 4 READ_REG commands

All register config (`SELECT_OUT_DATA=0x07`, `SAMPLE_RATE_DIV=1`, `BW_CONFIG=0x44`)
was already persisted to flash by a prior `SAVE_ALL_CONFIG` write.  The PRE log
confirmed `SAMPLE_RATE_DIV=0x0001` so the sensor is already at the correct ODR.

**File changed:** `mcu_fsm.c` → `ST_INIT_IMU` case

---

## ✅ Root Cause — Streaming never started (batch 11, 2026-02-26)

**Two bugs found by direct comparison with ATMEL reference code:**

### Bug 1 — EXTI callback set the wrong flag (critical)

```c
// BROKEN (batch 1–10):
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_9)
        drdy_interrupt = 1;   // only this — main loop never saw it
}

// Main loop checks:
if (g_imu_drdy_flag) {        // ← g_imu_drdy_flag was NEVER set
    g_imu_drdy_flag = 0;
    mcu_on_imu_drdy(&g_ctx);  // ← therefore NEVER called
}
```

`mcu_on_imu_drdy` was **never called** — streaming data was arriving on the SPI
bus on every DRDY pulse but nobody was reading it.

**Fix:** Set **both** flags in the callback:
```c
drdy_interrupt = 1;   // command-mode Phase 2 handshake (send_imu_command_ex)
g_imu_drdy_flag = 1;  // streaming-mode read (mcu_on_imu_drdy via main loop)
```

### Bug 2 — Phase 1 TX used HAL_SPI_TransmitReceive instead of HAL_SPI_Transmit

ATMEL `spi_master_write()` only clocks MOSI — it does not capture MISO during the
command TX phase. Our code used `HAL_SPI_TransmitReceive` which simultaneously
reads MISO into a dummy buffer. This means the sensor's first output bytes (0xFF
idle or early stream header bytes) were silently consumed by the dummy RX buffer
and discarded, leaving the SPI RX FIFO in an inconsistent state.

**Fix:** Use `HAL_SPI_Transmit` for Phase 1 (command TX).

### Bug 3 — Streaming read used HAL_SPI_TransmitReceive instead of HAL_SPI_Receive

ATMEL `spi_master_read()` clocks 0x00 on MOSI while reading. `HAL_SPI_Receive`
does exactly the same thing on STM32 (it clocks 0x00 out automatically).
`HAL_SPI_TransmitReceive` with a separate `s_tx_zeros` array is functionally
identical but requires a static buffer that could be modified. Switched to
`HAL_SPI_Receive` for clarity and correctness.

---

## ✅ Streaming confirmed live — dummy byte fix (batch 12, 2026-02-26)

Log from batch 11 confirmed streaming is working. `[STREAM#N]` lines show:
```
[STREAM#1]: 00 23 23 2E AB 00 01 00 00 00 00 00 0B B8 31 3A F1 EB 7C 3B
[STREAM#2]: 00 23 23 2E AB 00 02 ...
[STREAM#3]: 00 23 23 2E AB 00 03 ...
```

**Finding:** The sensor outputs **1 dummy byte (0x00)** before the `0x23 0x23` header on every DRDY. Total SPI read = **47 bytes** (1 dummy + 46 packet).

| Offset | Value | Meaning |
|---|---|---|
| 0 | `0x00` | Dummy byte — always ignore |
| 1–2 | `0x23 0x23` | Stream packet header |
| 3 | `0x2E` (=46) | Packet size |
| 4 | `0xAB` | CMD_STREAM_DATA |
| 5 | status | Sensor status |
| 6 | counter | Sample counter (increments each DRDY) |
| 7–14 | timestamp | 64-bit µs timestamp |
| 15–42 | data | Accel(12) + Gyro(12) + Temp(4) floats |
| 43–44 | checksum | uint16 big-endian |
| 45–46 | `0x0D 0x0A` | Footer |

**Fix:** Read 47 bytes, always use `&buf[1]` as the packet start. Removed the ambiguous header search — offset is always exactly 1.

---

## Complete Fix Summary (all batches)

| Batch | Fix | File | Effect |
|---|---|---|---|
| 1 | Confirmed SPI Mode 3 correct | — | No change needed |
| 2 | `spi2_flush()` before every CS | `main.c` | Clears stale RX FIFO |
| 3 | CS stays LOW for TX→DRDY→RX | `main.c` `send_imu_command_ex` | Sensor state machine stays intact |
| 4 | Two-step DRDY handshake (LOW→HIGH) | `main.c` Phase 2 | Rejects stale boot pulse |
| 4 | Hardened `IMU_Reset()` post-boot flush | `main.c` | Double-clears drdy_interrupt |
| 5 | UART3 115200 → 921600 baud | `main.c` `MX_USART3_UART_Init` | Frame time 1.3 ms → 0.16 ms |
| 5 | Throttle orient sends ×10 | `mcu_fsm.c` | UART load 1000/s → 100/s |
| 5 | Remove debug logs from hot path | `mcu_fsm.c` | No blocking UART in DRDY handler |
| 6 | 46-byte bulk SPI read | `mcu_fsm.c` `mcu_on_imu_drdy` | Per-DRDY SPI time ~600→92 µs |
| 7 | Write `SELECT_OUT_DATA=0x07` | `imu_config.c` | Fixes 48 Hz → 1 kHz |
| 7 | `SAVE_ALL_CONFIG` after writes | `imu_config.c` | Settings survive reset/power-cycle |
| 7 | `SELECT_OUT_DATA` in `imu_read_config` | `imu_config.c` | PRE/POST log shows packet size |
| 8 | RX clocks min(fixed_rx, 22) bytes | `main.c` | Sensor TX pipeline fully drained |
| 8 | `spi2_flush()` after every CS HIGH | `main.c` | STM32 FIFO clean before next TX |
| 8 | SPI2 FRXTH=1 (8-bit FIFO threshold) | `main.c` `MX_SPI2_Init` | RXNE fires per byte |
| 9 | CS HIGH after TX, CS LOW again for RX | `main.c` `send_imu_command_ex` | Fixes every command after first timing out |
| 10 | Removed ALL register R/W from ST_INIT_IMU | `mcu_fsm.c` | Streaming init is Reset→WHO_AM_I→Reset→START_STREAM only |
| **11** | **EXTI sets `g_imu_drdy_flag` AND `drdy_interrupt`** | **`main.c`** | **`mcu_on_imu_drdy` now actually called — was NEVER called before** |
| **11** | **Phase 1 TX: `HAL_SPI_Transmit` (not TransmitReceive)** | **`main.c` `send_imu_command_ex`** | **Matches ATMEL `spi_master_write` — no spurious MISO capture during TX** |
| **11** | **Stream read: `HAL_SPI_Receive` (not TransmitReceive)** | **`mcu_fsm.c` `mcu_on_imu_drdy`** | **Matches ATMEL `spi_master_read` — clean 46-byte bulk read** |
| **11** | **Log first 3 stream packets `[STREAM#N]:`** | **`mcu_fsm.c`** | **Confirms header and data visible without flooding log** |
| **12** | **Read 47 bytes (1 dummy + 46 packet), skip byte[0]** | **`mcu_fsm.c` `mcu_on_imu_drdy`** | **Parser always gets full 46-byte packet starting at 0x23 0x23** |
| **13** | **Gyro bias subtracted before Madgwick update** | **`imu_handler.c`** | **Eliminates slow orientation drift from static gyro offset** |
| **14** | **`spi2_flush` re-asserts `FRXTH=1` on every call** | **`main.c`** | **HAL resets CR2; without this every 2nd transaction stalls on odd-length** |
| **14** | **`IMU_NSS_HIGH_US` 20 µs → 500 µs** | **`main.c`** | **Sensor needs ≥300 µs after CS HIGH to latch cmd and assert DRDY** |
| **14** | **Phase 2 DRDY: also checks pin level** | **`main.c` `send_imu_command_ex`** | **Catches edge that fires before poll loop starts (race window = ~500 µs)** |
| **14** | **`imu_configure_odr_lpf` re-enabled in `ST_INIT_IMU`** | **`mcu_fsm.c`** | **ODR=1000 Hz and SELECT_OUT_DATA=0x07 written every boot** |
| **14** | **Removed readback + SAVE_ALL_CONFIG from `imu_configure_odr_lpf`** | **`imu_config.c`** | **4 transactions → 3; eliminates extra DRDY cycles that timed out** |

---

## Expected log after batch 14

```
IMU: reset
IMU TX20 [WHO_AM_I]: 24 24 0C 11 ...
IMU RX20 [WHO_AM_I]: 00 23 23 11 ... EA ...
[IMU] WHO_AM_I=0xEA (IIM46234 OK)
IMU: reset
IMU TX20 [SAMPLE_RATE_DIV]: 24 24 0E 12 00 02 1A 00 00 01 ...
IMU RX20 [SAMPLE_RATE_DIV]: 00 23 23 12 00 ...
[ODR] SAMPLE_RATE_DIV=0x0001 (ODR=1000 Hz) OK
IMU TX20 [SELECT_OUT_DATA]: 24 24 0D 12 00 01 1C 00 07 ...
[ODR] SELECT_OUT_DATA=0x07 (acc+gyro+temp, pkt=46 B) OK
IMU TX20 [BW_CONFIG]: 24 24 0D 12 00 01 30 00 44 ...
[ODR] BW_CONFIG=0x44 OK
[IMU] ODR=1000 Hz, SELECT_OUT_DATA=0x07 OK
IMU TX20 [START_STREAM]: 24 24 08 27 ...
IMU: START_STREAM sent
[IMU] Streaming started
```

Then continuous `EVT_IMU_ORIENT` at ~100 Hz (throttled from 1 kHz) with correct roll/pitch.

---

## Next Steps

1. Flash batch 14 — verify `[ODR] SAMPLE_RATE_DIV=0x0001 OK` appears in log.
2. Confirm DRDY on scope is now 1 kHz (not 48–50 Hz).
3. If any WRITE_REG still times out, add `HAL_Delay(1)` between the three writes in `imu_configure_odr_lpf` to give the sensor more inter-command recovery time.
4. If 1 kHz is confirmed, re-add `SAVE_ALL_CONFIG` (once, at first boot) so the sensor survives power-cycle without needing the config path.

```
IMU: reset
IMU TX20 [WHO_AM_I]: ...
[IMU] WHO_AM_I=0xEA (IIM46234 OK)
IMU: reset
[IMU] ODR assumed 1000 Hz (from flash config)
IMU TX20 [START_STREAM]: 24 24 08 27 00 27 0D 0A 00 ...
IMU: START_STREAM sent
[IMU] Streaming started
```

Then continuous `EVT_IMU_ORIENT` frames at ~100 Hz (roll + pitch in centidegrees).

---

## Gyro Bias (measured 2026-02-26, ±250 dps FSR, board stationary)

| Axis | Bias (deg/s) |
|---|---|
| X | +0.421587 |
| Y | −0.133538 |
| Z | +0.064359 |

Subtracted from raw gyro **before** DEG2RAD conversion inside `decode_stream_pkt()` in `imu_handler.c`.  
Hardcoded as default values in `s_gyro_bias_x/y/z`. Call `IMU_Handler_SetGyroBias()` to override at runtime if re-measured.

---

## Next Steps

1. Flash batch 12 — orientation events should now arrive in the UI.
2. Verify `EVT_IMU_ORIENT` frames in UI show sensible roll/pitch values when the board is tilted.
3. If checksum errors appear, verify byte offsets 42–43 in the packet match the checksum calculation (sum of bytes [3..41]).
4. Once stable, re-introduce optional config writes (ODR/BW) as a separate calibration command.

---

## Hardware Context

| Signal | MCU Pin |
|---|---|
| IMU RESET | PA0 (active-low output) |
| SPI2 NSS | PB12 (software, active-low) |
| SPI2 SCK | PB13 |
| SPI2 MISO | PB14 |
| SPI2 MOSI | PB15 |
| IMU DRDY | PB9 (EXTI rising edge, priority 0) |

SPI2: 4 MHz, Mode 3, software NSS, 8-bit, MSB first.

---

## Batch 15 Update (2026-03-11)

### Why this batch was added

Observed on scope: DRDY is actually ~50 Hz (not just UI-estimation error).  
Also, this board needs a hardware reset immediately before `START_STREAM` to start streaming reliably.

That creates a conflict:
- if ODR config is written **before** the final reset, the reset can reload flash-saved values (e.g., 50 Hz),
- but if reset is removed, streaming may fail to start.

### Changes made in this batch

1. **Keep final pre-stream reset, but move runtime ODR writes after it**
   - File: `Core/Src/mcu_fsm.c`
   - `ST_INIT_IMU` now does:
     - `IMU_Reset` -> `WHO_AM_I` -> `IMU_Reset` -> **final `IMU_Reset`** -> `imu_configure_odr_lpf(1, 0x44)` -> `START_STREAM`
   - Removed the old reset that was placed right before `START_STREAM` *after* configuration.
   - Effect: runtime `SAMPLE_RATE_DIV` / `SELECT_OUT_DATA` writes are no longer undone by the final reset.

2. **DRDY command handshake hardened against stale-high state**
   - File: `Core/Src/main.c`, function `send_imu_command_ex`
   - Phase 2 now:
     - fails if DRDY stays HIGH for >50 ms before response window,
     - then waits for fresh DRDY using LOW->HIGH progression (not stale HIGH carry-over).
   - Effect: reduces false command-response acceptance from old DRDY state.

3. **Streaming DRDY event handling changed from flag to counter**
   - Files: `Core/Src/main.c`, `Core/Src/stm32g0xx_it.c`
   - Replaced `g_imu_drdy_flag` with:
     - `g_imu_drdy_count` (pending edges)
     - `g_imu_drdy_edges_total` (total edges seen)
   - Main loop now consumes all pending DRDY edges (instead of collapsing multiple edges into one).
   - Effect: prevents under-processing at higher DRDY rates.

### Verification checklist for this batch

1. Boot log should show ODR config succeeds **after final reset**:
   - `[ODR] SAMPLE_RATE_DIV=0x0001 (ODR=1000 Hz) OK`
   - `[ODR] SELECT_OUT_DATA=0x07 ... OK`
   - `[ODR] BW_CONFIG=0x44 OK`
2. `START_STREAM` should still start reliably.
3. Scope on PB9 DRDY should rise to near 1 kHz.
4. If still ~50 Hz, capture logs around:
   - `IMU: DRDY stuck high before cmd resp`
   - `IMU: DRDY timeout [...]`
   - any `[ODR] ... FAILED` line.

---

## Batch 16 Update (2026-03-11)

### Problem seen in latest run

Startup failed at first command:

```
IMU: reset
IMU TX20 [WHO_AM_I]: ...
IMU: DRDY stuck high before cmd resp [WHO_AM_I]
```

This was caused by a hard-fail condition in `send_imu_command_ex` Phase 2:
if DRDY remained HIGH for 50 ms before the response wait window, command was aborted immediately.

### Fix applied

File: `Core/Src/main.c`, function `send_imu_command_ex`

1. Removed hard failure on pre-existing DRDY HIGH.
2. Added fresh-event gating using `g_imu_drdy_edges_total`:
   - capture `edge_base` before waiting,
   - accept response-ready only when a new edge is seen (`edges_total != edge_base`) or interrupt flag fires.
3. Kept LOW->HIGH observation path (`saw_low && pin==HIGH`) as additional race-safe condition.
4. Added high-held fallback:
   - if DRDY started HIGH and stayed HIGH through timeout window, attempt RX once and let packet validation decide.

### Expected effect

- `WHO_AM_I` no longer fails immediately just because DRDY is already high after reset.
- Command path becomes tolerant to reset/boot DRDY level behavior while still preferring fresh DRDY evidence.

---

## Batch 17 Update (2026-03-11)

### Problem seen after batch 16

Latest run reached:
- `WHO_AM_I` OK
- `SAMPLE_RATE_DIV` write OK
- `SELECT_OUT_DATA` write FAILED (`IMU: no 0x2323 hdr`, all-zero RX)

This indicates command sequencing was improved but still not robust for back-to-back writes without reset.

### Root cause

In `send_imu_command_ex` Phase 2, DRDY edge baseline was captured too late and the stale-high drain loop could consume up to 50 ms before response handling.
If a valid DRDY for the current command arrived in the TX->wait race window, the code could still delay long enough to read zeros/no-header on the next RX transaction.

### Fix applied

File: `Core/Src/main.c`, function `send_imu_command_ex`

1. Capture DRDY edge baseline immediately after TX CS-high (`edge_base_tx`).
2. Keep DRDY/edge state (do not clear it right before waiting), so fast valid events are preserved.
3. In stale-high drain loop, break immediately if `drdy_interrupt` is set or edge counter changed.
4. Use `edge_base_tx` consistently in wait loop for fresh-event detection.

### Expected effect

- Back-to-back write commands (`SAMPLE_RATE_DIV` then `SELECT_OUT_DATA` then `BW_CONFIG`) should no longer lose the second command response.
- ODR config sequence should complete without requiring an extra hardware reset between each command.

---

## Batch 18 Update (2026-03-11)

### Deeper discrepancy identified

The ATMEL reference always reads the **exact response length** in command mode:
- `WRITE_REG` ACK: 10 bytes
- `READ_REG`: `16 + read_length` bytes

The STM32 port had been reading a minimum of 22 bytes for command responses. That does not match either:
- the ATMEL implementation, or
- the datasheet command-mode protocol, which defines packet length explicitly and requires only the real response packet to be read.

This mismatch explains the observed pattern:
- after reset, one command works,
- then the next command is ignored or returns zeros,
- `START_STREAM` appears sent, but no DRDY follows.

### Fix applied

1. In `Core/Src/main.c`, command-mode RX length was changed from `max(fixed_rx, 22)` to `fixed_rx + 1`.
   - `+1` is kept because STM32 logs still show one leading dummy byte (`0x00`) before `0x23 0x23`.
2. In `Core/Src/mcu_fsm.c`, `START_STREAM` is now considered valid only if a new DRDY edge is observed shortly after sending the command.

### Why this is the strongest current hypothesis

- Datasheet section 5.3.2/5.3.4 defines exact ACK/RESPONSE lengths.
- ATMEL code reads exactly those lengths.
- Our failing command was the **second** back-to-back write, which is exactly where an over-read would be expected to strand the protocol state.

---

## Batch 19 Update (2026-03-12)

### Root-cause refinement

After comparing the STM32 implementation against the ATMEL reference and the datasheet timing diagrams again, the most important remaining discrepancy was not SPI mode or base clock.

It was the **command transaction structure**:

- ATMEL path:
  1. send 20-byte command packet
  2. deassert CS
  3. wait for DRDY
  4. start a **new** SPI transaction to read the ACK/RESPONSE

- STM32 path before this batch:
  - kept CS asserted from command TX into response RX for command mode
  - then tried to search for the response header inside that combined transaction

That contradicted:
- datasheet section 5.3.1.1 (`command mode` is half-duplex), and
- ATMEL `serif_tx()` + later `serif_rx()` split.

This also matches the observed symptom pattern:
- first command after reset may succeed,
- next command is ignored or returns zeros,
- reset recovers the sensor state.

### Fix applied

File: `Core/Src/main.c`, function `send_imu_command_ex`

1. Reworked command mode to use **two separate CS windows**:
   - CS low -> 20-byte command TX -> CS high
   - wait for fresh DRDY
   - CS low -> response RX -> CS high
2. Added explicit no-response handling for:
   - `START_STREAMING (0x27)`
   - `STOP_STREAMING (0x28)`
   - `SET_UTC_TIME (0x2D)`
3. Response wait now uses a fresh DRDY edge baseline:
   - `g_imu_drdy_edges_total` captured before command TX
   - timeout if no new response-ready indication arrives
4. Added explicit post-response wait for DRDY to finish its response window before returning.
5. Added explicit `Tnext` guard (`>= 0.3 ms`) after response RX.
6. Kept SPI clock at prescaler `/2` (~8 MHz), which is safely inside the datasheet's `4-12 MHz` window.

### Expected effect

- Back-to-back command reads/writes should no longer require hardware reset between commands.
- The "echo of the last TX byte" should no longer contaminate response parsing, because command TX and response RX are now separate SPI transactions.

---

## Batch 20 Update (2026-03-12)

### Additional bug found in streaming path

The long-run ODR degradation on the host side (`1000 -> 765 -> 619`) was inconsistent with the first verified stream packets, which already proved the sensor itself was outputting 1 kHz.

The streaming read path in `Core/Src/mcu_fsm.c` used:

- `HAL_SPI_Receive()` in SPI master 2-line mode

On STM32 HAL, that is unsafe here because the master still transmits bytes while receiving. For this IMU stream read, the host must clock out **zeros**, as the ATMEL code does. Sending stale buffer contents on MOSI during stream reads can corrupt long-running streaming behavior.

The stream path also was not explicitly flushing SPI RX/OVR state around each 47-byte read.

### Fix applied

File: `Core/Src/mcu_fsm.c`

1. Added `imu_stream_spi_flush()`:
   - re-assert 8-bit RX threshold
   - wait for `BSY=0`
   - drain RX FIFO
   - clear OVR if present
2. Replaced `HAL_SPI_Receive()` with explicit:
   - `HAL_SPI_TransmitReceive(... zeros ..., rx_buf, 47, ...)`
3. Added SPI flush before and after each stream transaction.

### Expected effect

- Stream reads now match the ATMEL reference behavior more closely.
- Host-side packet alignment should be more stable over time.
- The later false low ODR estimates should be reduced or eliminated if they were caused by SPI RX/FIFO corruption rather than true sensor ODR.

### Verification limitation on this machine

The updated firmware was compiled and flashed successfully after these changes.

However, the full UART log replay loop could not be completed locally in this environment because the currently visible VCP (`COM5`) is not exposing the firmware's UI/log UART traffic, so no post-flash run log was obtainable from this machine for this batch.

---

## Batch 21 Update (2026-03-12)

### New finding from saved log + UI ODR mismatch

The saved `log.txt` only captures the startup window and shows:

- first parsed stream packets at `dts_us ~= 1000`
- first ODR log at `1000 Hz`

At the same time, the UI later reports about `615 Hz` in the live orientation panel.

Those two observations can both be true because:

1. the firmware logs the human-readable `"[IMU] Sensor ODR ~= ..."` line only once, on the first completed ODR window
2. later ODR updates are still sent to the UI as `EVT_IMU_ODR`, but are not logged again

### Root issue found in DRDY handling

The PB9 IMU DRDY signal was being handled inconsistently:

- board behavior observed on hardware: **idle HIGH, active LOW pulse**
- comments in `imu_config.c` already referred to the **next falling-edge DRDY**
- but firmware GPIO/IRQ handling was still configured for **rising-edge EXTI**
- `send_imu_command_ex()` also treated `HIGH` after RX as abnormal, which contradicts the observed idle state

That mismatch can cause both:

1. fragile command-response synchronization
2. unstable long-run stream servicing / ODR estimation

### Fix applied

Files:

- `Core/Src/main.c`
- `Core/Src/stm32g0xx_it.c`

Changes:

1. PB9 EXTI changed from `GPIO_MODE_IT_RISING` to `GPIO_MODE_IT_FALLING`
2. EXTI handler changed to consume `FPR1` falling-pending for PB9 instead of `RPR1` rising-pending
3. Added explicit helpers for DRDY active/idle semantics:
   - active = pin LOW
   - idle = pin HIGH
4. `send_imu_command_ex()` response wait now accepts either:
   - a fresh DRDY falling edge
   - or the DRDY pin already LOW
5. post-RX command wait now waits for DRDY to return to **idle HIGH**, instead of incorrectly treating HIGH as the stuck state

### Expected effect

- command-mode synchronization should now match the actual electrical DRDY behavior
- false `"DRDY stayed high after RX"` diagnostics are eliminated
- later ODR collapse caused by wrong-edge synchronization should be reduced or eliminated

### Still open after this batch

This batch fixes a concrete firmware inconsistency.

If the ODR still drops after this change, the next likely bucket is not startup configuration but steady-state servicing throughput:

- SPI stream read + packet parse + Madgwick + Kalman + UART event traffic per sample

At that point the next debug step should be to log:

- DRDY edges seen
- valid packets parsed
- parse failures
- current sample_counter delta histogram

so we can separate true sensor-rate change from host-side servicing loss.

---

## Batch 22 Update (2026-03-12)

### Correction to Batch 21

The falling-edge PB9 change regressed command-mode bring-up immediately:

- fresh log showed `WHO_AM_I` timeout at startup
- `edge=0 pin=1` means no usable response-ready indication was captured

This proves the previous assumption was too aggressive.

### Refined interpretation

Hardware observation still stands:

- DRDY line is normally HIGH
- it pulses LOW when active

But for **command-mode response synchronization**, the host-side usable event is the **return-to-HIGH** edge, not the falling edge itself.

That matches the earlier successful runs:

- command responses were captured when PB9 was handled on rising edge
- switching PB9 command/stream bookkeeping to falling edge broke `WHO_AM_I`

### Fix applied

Files:

- `Core/Src/main.c`
- `Core/Src/stm32g0xx_it.c`

Changes:

1. Reverted PB9 EXTI from `GPIO_MODE_IT_FALLING` back to `GPIO_MODE_IT_RISING`
2. Reverted EXTI handler to use `RPR1` / `RPIF9` for PB9
3. Reverted command wait loop so response readiness is again based on:
   - fresh PB9 edge count change, or
   - DRDY interrupt flag

### Conclusion after this correction

The PB9 line may be low-active electrically, but the firmware should not use falling-edge EXTI as the response-ready trigger for command mode on this board. Rising-edge response-ready handling is the behavior that matches the working handshake.

---

## Batch 23 Update (2026-03-12)

### What the latest log + UI behavior mean

The saved boot log still shows:

- valid `WHO_AM_I`
- default profile confirmed at 1 kHz
- first parsed stream packets at `dts_us = 1000`
- first ODR window at `1000 Hz`

But the UI later falls toward about `615 Hz`.

That pattern is now most consistent with a **steady-state servicing throughput limit**, not a startup configuration error.

### Why this is the likely root cause

In the previous stream path, every parsed sample did all of the following on a Cortex-M0+ with software floating point:

1. Madgwick quaternion update
2. quaternion-to-Euler conversion inside `imu_handler.c`
   - `atan2f`
   - `asinf`
   - `atan2f`
3. second post-filter accel-angle + Kalman stage inside `mcu_fsm.c`
   - `atan2f`
   - `sqrtf`
   - `atan2f`
   - two `Kalman1D_Update()` calls

That is redundant work.

Madgwick is already the fusion filter. The second software Kalman stage was adding more floating-point cost to an already expensive path.

### Fix applied

Files:

- `Core/Src/imu_handler.c`
- `Core/Src/mcu_fsm.c`

Changes:

1. Removed per-sample `Madgwick_GetEuler()` call from `imu_handler.c`
2. Removed the second post-Madgwick accel-angle + Kalman fusion stage from `mcu_fsm.c`
3. Kept Madgwick quaternion integration running on **every** valid sample
4. Added a reduced-rate quaternion-to-roll/pitch conversion path in `mcu_fsm.c`
   - roll/pitch are now derived from quaternion every `4` samples (`250 Hz` at a `1 kHz` stream)
   - shadow roll/pitch values are still available continuously for VSYNC use
5. UI orientation event remains throttled to `100 Hz`

### Intended effect

- preserve high-rate quaternion fusion
- drastically reduce per-sample software trig cost
- remove redundant filtering
- recover sustained stream servicing closer to the sensor's 1 kHz capability

### Tradeoff

Roll/pitch shadow updates are now decimated relative to the raw sensor stream.

That is intentional: the FPGA consumer is frame-synchronous via VSYNC, so updating roll/pitch at `250 Hz` is still comfortably above frame rate while avoiding the previous M0+ overload.

---

## Batch 24 Update (2026-03-16)

### Problem clarified by scope

Hardware scope measurement on PB9 shows DRDY at about `48.5 Hz`, while the UI was showing about `615 Hz`.

Those are not the same metric.

The old UI field labeled `ODR` was populated from the firmware's packet-derived estimate based on:

- stream packet timestamp delta
- stream packet sample-counter delta

It was **not** the physical PB9 DRDY edge rate.

### Fix applied

Firmware now publishes a separate IMU stats event with three different rates:

1. `DRDY_HZ`
   - measured from PB9 edge count over wall-clock time
   - this is the physical interrupt cadence reaching the MCU
2. `PKT_TS_HZ`
   - measured from packet `timestamp` and `sample_counter`
   - this is the logical/sample-domain rate implied by packet contents
3. `PARSE_OK_HZ`
   - measured from number of valid parsed packets per wall-clock second
   - this is the effective packet-processing rate inside the MCU

### Files changed

- `Core/Src/mcu_fsm.c`
- `PC_MCU_COM/Program.cs`

### UI change

The IMU status text now shows:

- `DRDY <x> Hz`
- `TS <y> Hz`
- `Parse <z> Hz`

instead of a single ambiguous `ODR` field.

### Why this matters

This split makes it immediately visible whether a mismatch comes from:

- real sensor/DRDY output behavior
- packet metadata behavior
- MCU parsing/service throughput

So if scope says `48.5 Hz`, the UI should now also show something close to:

- `DRDY ~48 Hz`
- `Parse ~48 Hz`

while `TS` may still differ if the sensor packet metadata implies a different internal sample cadence.

---

## Batch 25 Update (2026-03-16)

### New interpretation of `DRDY 48 Hz / TS 581 Hz / Parse 48 Hz`

That triple no longer points to a bad `SAMPLE_RATE_DIV` write by itself.

It matches the IMU's documented streaming FIFO behavior:

- the sensor samples internally at the configured ODR and pushes packets into a 40-entry queue
- the host was servicing exactly one packet per observed DRDY event
- once the host falls behind, the queue can stay non-empty and delivered DRDY transitions no longer reflect every internal sample
- packet `sample_counter` then jumps by more than 1, so packet timestamps still imply a much higher internal rate than the physical/event-delivered packet rate

In short: the STM32 stream reader was still behaving like an edge-for-edge reader, while the sensor behaves like a queued producer.

### Fix applied

Files:

- `Core/Src/mcu_fsm.c`
- `Core/Src/main.c`

Changes:

1. **Burst-drain the IMU FIFO when backlog is detected**
   - `mcu_on_imu_drdy()` now processes more than one stream packet per service pass.
   - When a parsed packet shows `sample_counter` delta `> 1`, the firmware immediately reads additional packets in the same pass, up to a bounded burst (`IMU_STREAM_BURST_MAX = 16`).
   - This is intended to catch the host back up to the IMU queue instead of waiting for more DRDY edges that may not come at the internal sample rate once backlog exists.

2. **Move IMU servicing ahead of UI/FSM work in the main loop**
   - Pending IMU DRDY packets are now consumed before UI RX parsing and FSM stepping.
   - This reduces latency between the EXTI event and the SPI read in steady-state streaming mode.

3. **Remove trig from the hot path unless it is actually needed**
   - Quaternion fusion still runs on every valid sample.
   - Quaternion-to-roll/pitch conversion now runs only when:
     - a UI orientation send is due, or
     - FPGA VSYNC data is pending.
   - This removes unnecessary `atan2f/asinf` cost from most stream packets.

### Expected effect

- physical DRDY and parse rate should move much closer to the true internal sample rate
- `sample_counter` jumps should shrink back toward `1`
- `TS` should move toward `DRDY`/`Parse`
- if the remaining bottleneck is purely compute budget, this batch should still improve the sustainable rate materially because the trig work is now demand-driven instead of packet-driven

---

## Batch 26 Update (2026-03-16)

### Timing alignment with datasheet and ATMEL reference

Files:

- Core/Inc/main.h
- Core/Src/main.c
- Core/Src/mcu_fsm.c

Changes:

1. Replaced the old ad-hoc STM32 IMU timing constants with shared timing values derived from the datasheet command timing model and the ATMEL reference SPI cadence:
   - IMU_NSS_SETUP_US = 1
   - IMU_NSS_HOLD_US = 1
   - IMU_NSS_HIGH_US = 7
   - IMU_CMD_TWAIT_US_NORMAL = 50
   - IMU_CMD_TWAIT_US_START = 600
   - IMU_CMD_TNEXT_US = 300

2. Removed the blanket IMU_POST_CMD_DELAY_MS = 10 behavior from the STM32 command path.
   - No-response commands now wait only for their protocol-specific gap.
   - Normal command sequencing now ends with Tnext instead of a large millisecond delay.

3. Updated the streaming read path to use the same shared NSS setup/hold timing instead of hardcoded 5 us / 5 us guards.

### Why this matters

The previous STM32 path mixed protocol timing and local software guard delays. That made it hard to compare directly against either the datasheet or the ATMEL implementation. This batch makes the STM32 transaction timing explicit:

- command Twait is no longer approximated by a blanket 10 ms
- inter-command gap now matches the manual Tnext = 0.3 ms
- stream chip-select timing now uses the same constants as command mode

This does not by itself prove the root cause, but it removes one clear timing discrepancy so subsequent logs can be compared more directly against ATMEL behavior.

---

## Batch 27 Update (2026-03-16)

### STM32 SPI session rework for IMU command and stream paths

Files:

- Core/Inc/main.h
- Core/Src/main.c
- Core/Src/mcu_fsm.c

Changes:

1. Replaced the previous per-byte HAL SPI loops in command mode with burst session helpers:
   - IMU_SPI_WriteSession() for the 20-byte command TX phase
   - IMU_SPI_ReadSession() for the response/stream RX phase

2. Added IMU_SPI_ResetSession() and call it before and after each IMU SPI phase.
   - waits for the peripheral to drain
   - disables SPI
   - clears overrun state
   - re-enables SPI and restores FRXTH

3. Command response parsing no longer trusts ixed_rx + 2 alone.
   - RX now clocks ixed_rx + IMU_CMD_RX_SLACK
   - parser searches for a complete packet with both header and footer
   - if only a shifted header is present, log now reports it as an incomplete packet instead of accepting a partial payload

4. Stream reads now use the same SPI read-session helper as command responses.

### Why this matters

ATMEL behaves like a clean burst-oriented SPI session machine. The STM32 path was still performing command TX/RX as many 1-byte HAL calls inside a transaction, while the logs showed the response ending one byte early after the echoed leading byte. That is exactly the kind of session desynchronization that can explain:

- leading echo byte
- next command requiring reset
- stream cadence collapse if packets are not fully consumed

This batch removes that per-byte session behavior so the next log can tell us whether the root issue is genuine STM32 SPI state carry-over or something lower on the wire.

---

## Batch 28 Update (2026-03-17)

### Real root cause isolated: PB12 NSS ownership bug

Files:

- Core/Src/stm32g0xx_hal_msp.c
- Core/Src/main.c

Changes:

1. Removed PB12 from the SPI2 alternate-function pin setup in `HAL_SPI_MspInit()`.
2. Restored PB12 as an explicit GPIO-controlled chip-select in the IMU path.
3. Kept raw SPI register transfers for the IMU session so PB12 low/high boundaries are fully software-defined.

### Verified result

After this fix:

- the leading echoed command byte disappeared from command responses
- back-to-back command reads worked without reset
- the default 1 kHz flash profile was read back cleanly
- startup stream packets showed `dts_us = 1000`

This matched the scope observation that the previous STM32 build had PB12 held low continuously while the ATMEL reference toggled NSS for each transaction. The PB12 AF override was therefore a genuine protocol bug, not a cosmetic GPIO difference.

---

## Batch 29 Update (2026-03-17)

### Sustained 1 kHz restored by reducing MCU fusion load, not by changing IMU ODR

Files:

- Core/Src/imu_handler.c
- Core/Src/mcu_fsm.c
- .cproject

Changes:

1. Added sustained IMU stats logging once per second:
   - `DRDY`
   - packet timestamp-derived rate (`TS`)
   - parse rate (`Parse`)

2. Used a parse-only diagnostic run to prove the transport path can sustain full rate:
   - with fusion bypassed, STM32 reached about `1000 Hz` for `DRDY/TS/Parse`
   - this proved the IMU was already configured correctly and the remaining limit was MCU compute budget

3. Switched the Debug configuration from unoptimized `-O0` to `-Os` in `.cproject`.

4. Added fusion decimation in `imu_handler.c` so packet ingest remains at 1 kHz while the Madgwick update runs at a lower effective rate.

### Measured outcome

Observed progression:

- full-rate fusion, old debug build: about `403 Hz`
- full-rate fusion, optimized build: about `462 Hz`
- fusion decimation x4, optimized build: about `907 Hz`
- fusion decimation x8, optimized build: about `998-999 Hz`

Final steady-state log now shows:

- `DRDY ~= 998-999 Hz`
- `TS = 1000 Hz`
- `Parse ~= 998-999 Hz`

### Practical note

The IMU ODR did not need another explicit register change. The flash-backed profile was already at factory 1 kHz defaults. The remaining issue was purely host-side processing load on the STM32G0 when running floating-point fusion on every packet.
