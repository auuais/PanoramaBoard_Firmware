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