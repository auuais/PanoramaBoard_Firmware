# IR Camera NV Command Context — Tenum 640 (UART1, STM32G0)

## Hardware Setup

| Item | Detail |
|---|---|
| Camera model | Tenum 640 IR thermal |
| Interface | UART1 (`huart1`) — 8-bit packets, idle-line detection |
| Multiplexer | 6 cameras selected via GPIOC pins PC0/PC1/PC2 (3-bit mux) |
| Current IR camera index | `current_ir_camera` (global `uint8_t`) |
| RX buffer | `ir_camera_rx_buffer[128]` |
| RX flag | `volatile bool ir_camera_data_received_flag` |
| RX byte count | `volatile uint16_t ir_camera_received_bytes` |

---

## Packet Format

All packets follow this structure (both TX and RX):

```
[0x01] [msg_id] [param_len] [param_0 .. param_N-1] [checksum]
```

- **Checksum** = `(~sum_of_all_preceding_bytes) + 1` (two's complement of byte sum)
- Computed by `ComputeChecksum(buf, len)` in `main.c`

---

## Command Reference (relevant subset)

| Command | msg_id TX | Description | Response msg_id |
|---|---|---|---|
| Handshake | `0x07` | Test camera alive | `0x02` ACK or `0x00` Text |
| Verbose Mode | `0xFF` | Enable text responses | `0x02` ACK |
| Factory Reset NV | `0xB3` | Restore all NV to defaults | `0x02` ACK (echo `0xB3` at byte[4]) |
| Set NV | `0xB0` | Write one NV parameter | `0x02` ACK (normal) or `0x06` NACK (reboot required) |
| Get NV | `0xB5` | Read one NV parameter | `0x45` VALUE packet, len=2, big-endian uint16 |
| Set Mux | `0xD7` | Set digital video mux | `0x02` ACK (no readback needed) |

### Factory Reset NV — TX packet
```
[0x01][0xB3][0x00][checksum]   // 4 bytes total
```
ACK response pattern (found by scanning rx buffer):
```
rx[i]==0x01 && rx[i+1]==0x02 && rx[i+3]==0x00 && rx[i+4]==0xB3
```
Timeout: **1000 ms**

### Set NV — TX packet (`0xB0`)
```
[0x01][0xB0][0x04][id_hi][id_lo][val_hi][val_lo][checksum]   // 8 bytes
```
- `id` and `val` both big-endian uint16
- ACK `0x02`: value active immediately
- NACK `0x06`: value stored to NV flash, requires **power cycle** to activate (NV17, NV18)

### Get NV — TX packet (`0xB5`)
```
[0x01][0xB5][0x02][id_hi][id_lo][checksum]   // 6 bytes
```
Response VALUE packet:
```
[0x01][0x45][0x02][val_hi][val_lo][checksum]
```
Parsed by `IR_FindPacket(0x45, 0x02, &v)` → big-endian uint16 into `v`

---

## NV Parameter Table (configured in `Initialize_IR_Camera`)

| NV# | Description | Value set | Response | Notes |
|---|---|---|---|---|
| 4 | RS-170 | 0 | ACK `0x02` | Disable RS-170 |
| 5 | PDVO enable | 1 | ACK `0x02` | Enable parallel 8-bit post-AGC output |
| 6 | (output format) | 0 | ACK `0x02` | |
| 7 | Video mux NV | 9 | ACK `0x02` | Also set via `IR_SetMux(9)` using `0xD7` |
| 14 | (sync) | 1 | ACK `0x02` | |
| 16 | Frame rate | 1 | ACK `0x02` | 1 = 30 Hz |
| 17 | Genlock Direction | 1 | **NACK `0x06`** | Input/slave direction — requires power cycle |
| 18 | Genlock Mode | 0 | **NACK `0x06`** | Slave mode — requires power cycle |
| 76 | (SuperFrame/sync) | 0 | ACK `0x02` | Must be 0 before setting NV17/18 |

**NV17 and NV18 always return NACK `0x06`** — this is correct and expected. The camera stores the values to NV flash and needs a reboot to apply them. `IR_SetNV` treats `0x06` as success.

---

## UART RX Architecture

### Key principle
The UART is always listening via `HAL_UARTEx_ReceiveToIdle_IT`. The callback **always re-arms** after each received packet:

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        ir_camera_data_received_flag = true;
        ir_camera_received_bytes = Size;
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof ir_camera_rx_buffer);
    }
}
```

### IR_ArmRx
Called before each `IR_SetNV` / `IR_GetNV`. Clears the flag so `IR_WaitPacket` waits for the response to **this specific command**, not a stale one. Does NOT abort or flush — just resets state and re-arms:

```c
static void IR_ArmRx(void) {
    ir_camera_data_received_flag = false;
    ir_camera_received_bytes = 0;
    memset(ir_camera_rx_buffer, 0, sizeof ir_camera_rx_buffer);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof ir_camera_rx_buffer);
}
```

### ir_flush_rx
Available but **NOT called from IR_ArmRx**. Calling `HAL_UART_AbortReceive` or flushing from inside `IR_ArmRx` breaks the handshake because it kills the live `ReceiveToIdle_IT` and prevents the handshake `0x07` response from being received.

---

## Initialize_IR_Camera Sequence

```
select_ir_camera(n)          HAL_Delay(100)
IR_FactoryResetNV()          → log OK or FAILED, HAL_Delay(10)
Send 0xFF verbose mode       HAL_Delay(100) → process response, clear flag, CONTINUE
Send 0x07 handshake          HAL_Delay(100) → parse ACK/Text → handshake_success
--- if !handshake_success → return RET_ERR_TX ---
IR_SetMux(9)                 0xD7 command, no readback
IR_SetNV(7, 9)               NV#7 = 9
IR_SetNV(6, 0)               NV#6 = 0
IR_SetNV(14, 1)              NV#14 = 1
IR_SetNV(4, 0)               NV#4 RS-170 off
IR_SetNV(5, 1)               NV#5 PDVO on
IR_SetNV(76, 0)              NV#76 = 0
GetNV verify: NV4,5,7,76     → mcu_ui_log
IR_SetNV(16, 1)              NV#16 30fps
GetNV verify: NV16           → mcu_ui_log
IR_SetNV(17, 1)              NV#17 Genlock Dir (NACK 0x06 = OK)
IR_SetNV(18, 0)              NV#18 Genlock Mode (NACK 0x06 = OK)
GetNV verify: NV17,18        → mcu_ui_log (will show -1 since reboot needed)
mcu_ui_log "init complete"
return RET_OK
```

---

## Log Output Interpretation

### Good output (camera working)
```
IR cam 0: FactoryResetNV OK
IR Camera 0 ACK received
--- STEP 2: Here Performing Handshake ---
Camera 0 Text: Rel: X1.P55.03.10.17-1-g1b174d0
--- !!!!!!!!!!!!! Handshake successful ---
Verify PDVO: NV4=0 NV5=1 NV7=9 NV76=0
Verify FrameRate: NV16=1
SetNV#17 NACK(reboot needed) — stored OK
SetNV#18 NACK(reboot needed) — stored OK
Verify Genlock: NV17=-1 NV18=-1      ← -1 is correct: GetNV returns -1 until reboot
IR Camera 0 initialization complete
```

### NV17/18 returning -1 on GetNV is EXPECTED
The camera stores genlock settings to NV flash but only activates them after power cycle. `IR_GetNV` returns -1 (VALUE packet `0x45` not found) because the camera has not yet applied the new values — it won't until it reboots.

### Failure patterns
| Log line | Cause |
|---|---|
| `FactoryResetNV FAILED (no ACK)` | Camera not responding to `0xB3` — check UART connection / mux select |
| `handshake failed - No valid response` | `HAL_UARTEx_ReceiveToIdle_IT` not active; callback broken; camera offline |
| `SetNV#N timeout` | `ir_camera_data_received_flag` never set — UART RX not re-armed |
| `SetNV#N unknown resp(...): XX XX ...` | Camera returned unexpected packet ID — check hex dump |
| `GetNV#N unknown resp(...)` | Response packet ID is not `0x45` — may need different VALUE packet ID |

---

## Bugs Fixed in This Chat Session

| # | Bug | Root Cause | Fix |
|---|---|---|---|
| 1 | `return RET_OK` after verbose ACK | Early exit before any NV command ran | Removed return, added comment |
| 2 | `HAL_Delay(10)` instead of `100` | Too short — camera not ready | Restored `HAL_Delay(100)` |
| 3 | `NV17=4294967295` in log | `%u` format on signed `int` printing `(uint32_t)-1` | Changed all NV verify format to `%d` |
| 4 | SetNV no ACK on cameras 1,3 | Stale buffer from previous command | `IR_ArmRx` clears flag before each command |
| 5 | `HAL_UART_AbortReceive` in `IR_ArmRx` | Killed live ReceiveToIdle_IT → handshake deaf | Reverted to original: no abort in IR_ArmRx |
| 6 | Callback removed re-arm | UART never re-armed after abort | Reverted: callback always re-arms |
| 7 | NV17/18 always timeout | Camera returns NACK `0x06` not ACK `0x02` | Accept `0x06` as success in `IR_SetNV` |
| 8 | `IR_FactoryResetNV` no status check | Silent fail/pass | Added `mcu_ui_log` on success and failure |

---

## Current State of Key Functions in main.c

### `IR_FactoryResetNV` — line ~933
- Sends `[0x01][0xB3][0x00][cksum]`
- 1000 ms timeout polling `ir_camera_data_received_flag`
- Scans for ACK pattern `0x01 0x02 ... 0x00 0xB3`
- Logs `"FactoryResetNV OK"` or `"FAILED"` via `mcu_ui_log`

### `IR_ArmRx` — line ~987
- Clears flag, clears buffer, calls `HAL_UARTEx_ReceiveToIdle_IT`
- **No flush, no abort** — original working pattern

### `IR_SetNV` — line ~1026
- `IR_ArmRx()` → `Send_IR_Camera_Command(0xB0, ...)` → `IR_WaitPacket(500)`
- Accepts `0x02` (ACK) and `0x06` (NACK/reboot-required) as success
- Logs 8-byte hex dump on unknown response

### `IR_GetNV` — line ~1060
- Signature: `int IR_GetNV(int id, int *out)` — returns `int`, `*out` is `int`
- `IR_ArmRx()` → `Send_IR_Camera_Command(0xB5, ...)` → `IR_WaitPacket(500)`
- Looks for VALUE packet `IR_FindPacket(0x45, 0x02, &v)`
- Logs 8-byte hex dump on failure

### `HAL_UARTEx_RxEventCallback` — line ~1271
- Sets `ir_camera_data_received_flag = true`, `ir_camera_received_bytes = Size`
- **Always re-arms** `HAL_UARTEx_ReceiveToIdle_IT` — this is critical

### `Initialize_IR_Camera` — line ~1097
- Calls `IR_FactoryResetNV()` and checks return
- Does NOT call `ir_flush_rx` — original had none here
- Does NOT return early after verbose ACK — continues to handshake and NV writes

---

## ui_log Routing

All NV verify lines use `mcu_ui_log(&g_ctx, ...)` (defined in `mcu_fsm.c`, declared in `mcu_fsm.h`) which builds a `MSG_LOG` COBS frame and sends it over UART3 to the PC UI log window.

`Send_UART3_Debug(...)` routes through `ui_logf` → same `MSG_LOG` frame.

`mcu_ui_log` is declared public in `mcu_fsm.h` so `main.c` can call it with `&g_ctx`.
