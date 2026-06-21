# Customer ICD Migration Plan

Date: 2026-06-21

Legacy fallback anchor:
- Commit: `1136214 Snapshot legacy firmware before ICD migration`
- Tag: `legacy-firmware-before-icd`

## Goal
Convert STM32 panorama-board firmware from the local engineering COBS/CRC protocol toward the customer ICD RS-422 protocol while preserving enough internal logic to keep the FPGA, EO/IR cameras, IMU, and status checks working.

## ICD Source
Workbook:
`C:\Users\USER\Documents\KakaoTalk Downloads\SAPC_ICD_v0.89_TBT_??_260518.xlsx`

Important ICD facts extracted so far:
- Transport label: RS-422.
- Endian: little endian.
- SOM: `0x70717883`.
- EOM: `0x83787170`.
- Frame fields: SOM, Message ID, Sequence Number, Databyte Count, Payload, CRC, EOM.
- CRC: CRC-CCITT, initial value `0xFFFF`, ICD text says CRC over Payload field.
- PIPA receives mode/camera/panorama/maintenance/UTC messages.
- PIPA transmits status, maintenance status, parameter, PBIT, CBIT, IBIT, and ACK messages.

## Assumptions Until Customer Clarifies
The ICD does not appear to fully specify these items, so use current working firmware defaults initially:
- UART3 remains the customer/control UART at `460800 8N1`, no flow control.
- CRC coverage is payload only.
- Invalid frames are discarded silently except optional debug logging in our UI build.
- ACK is sent once a command is syntactically valid and accepted for execution.
- Long-running commands update later status/BIT messages rather than blocking ACK.
- CBIT is 1 Hz in operational/customer-active state.
- Existing safe node health checks are reused for CBIT/IBIT and must not reinitialize cameras destructively.

## Firmware Implementation Stages
1. Preserve legacy fallback commit/tag.
2. Add ICD raw-frame parser alongside legacy parser:
   - Detect SOM/EOM raw frames in UART3 byte stream.
   - Parse little-endian header fields.
   - Validate payload length and CRC-CCITT.
3. Add ICD frame transmitter:
   - Shared helper to send message ID, sequence, payload, CRC, EOM.
4. Map ICD commands to existing safe control functions:
   - `0x2201` mode control -> current mode/FPGA mode handling.
   - `0x2202` camera control -> current EO/IR camera power and selected camera control where applicable.
   - `0x2203` panorama control -> current FPGA mode/config mapping where applicable.
   - `0x2204` maintenance -> power/control/config operations that currently exist.
   - `0x2205` UTC info -> store latest UTC info; no destructive action.
5. Implement outgoing ICD messages:
   - `0x2210` STATUS periodic/available status.
   - `0x2211` maintenance status.
   - `0x2212` parameter report.
   - `0x2213` PBIT.
   - `0x2214` CBIT at 1 Hz.
   - `0x2215` IBIT on request.
   - ACK IDs `0x22A1`.. as defined by ICD.
6. Once customer ICD path is stable, decide whether to remove or compile-disable the local engineering COBS/CRC protocol.

## Acceptance Criteria
- Firmware accepts and validates all customer-to-PIPA ICD command frames.
- Firmware emits valid ICD ACK and status/BIT frames with correct sequence and CRC.
- CBIT/IBIT/PBIT use safe checks and do not reset/reinitialize cameras unless explicitly commanded.
- Current FPGA/camera/IMU functionality remains internally usable.
- Raw byte logs from our UI match customer software expectations during integration.
