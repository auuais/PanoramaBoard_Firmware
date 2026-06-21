# Legacy Fallback Snapshot

Date: 2026-06-21

This commit freezes the current STM32 panorama-board firmware before starting the customer ICD migration.

Scope preserved here:
- Existing binary COBS/CRC UI protocol and MCU FSM.
- Current safe node status checks, firmware version response, camera power control, reset-cause logging, and IR handshake timing.
- Current FPGA/camera/IMU integration behavior used by the local PC_MCU_COM UI.

Purpose:
- Use this commit as the known fallback if the ICD protocol migration needs to be reverted or compared.
- Do not treat this firmware as customer-ICD compliant. ICD work starts after this fallback snapshot.

Traceability:
- Repository: PanoramaBoard_Firmware
- Branch at snapshot time: main
- Remote: https://github.com/auuais/PanoramaBoard_Firmware.git
