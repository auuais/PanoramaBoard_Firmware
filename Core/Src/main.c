/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Master firmware - MCU FSM + UI framing + IMU DRDY hook
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "mcu_fsm.h"
#include "proto_uart.h"
#include "fpga_i2c.h"
#include "fpga_regs.h"
#include "imu_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define RET_OK            0
#define RET_ERR_TX       -1
#define RET_ERR_TIMEOUT  -2
#define RET_ERR_RX       -3
#define RET_ERR_SPI      -4

/* IIM-46234 command packets are always exactly 20 bytes (from ATMEL ref SIZE_PACKET_CMD) */
#define IMU_CMD_PACKET_SIZE  20u


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART3_CMD_READ_TEMP   0xA1  // send this single byte from serial monitor

static uint8_t uart3_cmd_byte __attribute__((unused)) = 0;


volatile int drdy_interrupt = 0;

#include <stdbool.h>
bool g_debug_enabled = false;

extern UART_HandleTypeDef huart3;

// same IDs as your FSM
#define MSG_LOG   0x7F

// ------------------------------
// UI (UART3) framing buffers
// ------------------------------
#define UI_RX_DMA_MAX      128u
#define UI_ACC_MAX         512u
#define UI_PAYLOAD_MAX     256u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;  // IR cameras
UART_HandleTypeDef huart2;  // Day cameras
UART_HandleTypeDef huart3;  // PC UI (binary COBS+CRC)

/* USER CODE BEGIN PV */

// ---- FSM context ----
static McuContext g_ctx;

// ---- IMU DRDY flag (set in EXTI ISR) ----
volatile uint8_t g_imu_drdy_flag = 0;
volatile uint8_t g_vsync_flag = 0;

// ---- UART3 UI RX ----
// DMA/Idle chunk buffer:
static uint8_t  ui_rx_chunk[UI_RX_DMA_MAX];
static volatile uint16_t ui_rx_chunk_len = 0;
static volatile uint8_t  ui_rx_chunk_ready = 0;

// Accumulator so we can parse frames split across idle events:
static uint8_t  ui_acc[UI_ACC_MAX];
static uint16_t ui_acc_len = 0;


typedef struct {
  volatile int16_t roll_cd;
  volatile int16_t pitch_cd;
  volatile uint16_t rate_hz;
  volatile uint16_t seq;
  volatile bool pending;
} fpga_orient_shadow_t;

static fpga_orient_shadow_t g_fpga_orient = {0};

fpga_i2c_t g_fpga;

void fpga_init(void)
{
    fpga_i2c_init(&g_fpga, &hi2c3, 0x42);
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI2_Init(void);
void Enable_All_Camera_Power(void);
void IMU_Reset(void);
void IMU_SoftReset(void);

/* IMU SPI helpers */
static void delay_us(uint32_t us);
static int imu_drdy_ready(void);
void build_cmd20(const uint8_t *cmd, uint32_t cmd_len, uint8_t out20[IMU_CMD_PACKET_SIZE]);

void Send_UART3_Debug(const char *format, ...);

int send_imu_command(uint8_t *tx_packet, uint32_t tx_size, uint32_t rx_size, const char *cmd_name);
int send_imu_command_ex(uint8_t *tx_packet, uint32_t tx_size, uint32_t rx_size, const char *cmd_name, uint8_t *rx_out);

// Day Camera (VISCA) Functions
void select_day_camera(uint8_t camera_num);
int send_visca_command(uint8_t* cmd, uint8_t cmd_len, int parse_version);
int wait_for_visca_ack_and_completion(uint32_t timeout_ms, int parse_version);
void parse_and_print_version_response(uint8_t* data, uint8_t len);
int test_day_camera_controls(uint8_t camera_num);

// IR Camera Functions
void select_ir_camera(uint8_t camera_num);
int Send_IR_Camera_Command(uint8_t message_id, uint8_t *params, uint32_t param_size);
void Process_IR_Camera_Received_Data(uint8_t* data, uint16_t length);
void Translate_And_Send_IR_Camera_Command(uint16_t param_id, uint16_t value);
int Initialize_IR_Camera(uint8_t camera_num);
bool Validate_IR_Parameter(uint16_t param_id, uint16_t value);
uint8_t ComputeChecksum(uint8_t* message_buffer, uint32_t length);


/* USER CODE BEGIN PFP */
static void ui_process_bytes(const uint8_t *data, uint16_t len);
static void ui_acc_reset(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Day camera (VISCA) variables
uint8_t visca_rx_buffer[64];
volatile uint8_t visca_rx_idx = 0;
volatile uint8_t visca_rx_ready = 0;
uint8_t visca_rx_byte;
uint8_t current_day_camera = 0;  // Currently selected day camera (0-5)

// IR camera variables
volatile bool ir_camera_data_received_flag = false;
volatile uint16_t ir_camera_received_bytes = 0;
uint8_t ir_camera_rx_buffer[128];
uint8_t current_ir_camera = 0;  // Currently selected IR camera (0-5)



void Enable_All_Camera_Power(void)
{
    // Port C GPIO outputs (SW_EN signals)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    // Port A GPIO outputs (SW_EN signals)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    // Port D GPIO outputs (SW_EN signals)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
}





#include "proto_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi2;


// same IDs as your FSM
#define MSG_LOG   0x7F

static void ui_logf(const char *fmt, ...) {
    char text[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    // payload format: [type][seq][lenL][lenH][...data...]
    uint8_t payload[4 + 200];
    uint16_t n = (uint16_t)strlen(text);
    if (n > 200) n = 200;

    payload[0] = MSG_LOG;
    payload[1] = 0;              // seq unused for logs
    payload[2] = (uint8_t)(n & 0xFF);
    payload[3] = (uint8_t)((n >> 8) & 0xFF);
    memcpy(&payload[4], text, n);

    uint8_t frame[512];
    size_t flen = uart_frame_build(payload, (size_t)(4 + n), frame, sizeof(frame));
    if (flen) {
        HAL_UART_Transmit(&huart3, frame, (uint16_t)flen, 50);
    }
}

// Backward-compatible name used by older code paths:
void Send_UART3_Debug(const char *format, ...) {
    char text[200];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    ui_logf("%s", text);
}

/* -------------------------------------------------------------------------- */
/* Microsecond busy-wait (Cortex-M0+ has no DWT cycle counter).              */
/* Calibrated for 16 MHz SYSCLK: ~4 cycles per NOP iteration.               */
/* -------------------------------------------------------------------------- */
static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 4u;   /* 16MHz / 4 cycles ≈ 4 iters/µs */
    while (count--) { __NOP(); }
}

/* -------------------------------------------------------------------------- */
/* spi2_flush — drain stale bytes from the SPI2 RX FIFO and wait for BSY=0. */
/*                                                                            */
/* WHY: After each CS transaction the STM32 SPI peripheral may have bytes    */
/* sitting in the RX FIFO (received during TX or from a partial read).       */
/* If not drained before the next transaction, HAL_SPI_TransmitReceive will  */
/* return those old bytes as if they were the new response.                  */
/*                                                                            */
/* On STM32G0 SPI: DR read drains RXNE. We loop while RXNE is set.          */
/* -------------------------------------------------------------------------- */
static void spi2_flush(void)
{
    /* Re-assert FRXTH=1 every time: HAL_SPI_Init (and any HAL re-init path)
     * resets CR2, which clears FRXTH back to 0 (16-bit threshold).
     * With FRXTH=0 RXNE only fires after 2 bytes, so the last byte of any
     * odd-length transfer stalls forever.  Always force 8-bit threshold. */
    hspi2.Instance->CR2 |= SPI_CR2_FRXTH;

    /* Wait for BSY=0 (any in-progress transfer to finish) */
    uint32_t t0 = HAL_GetTick();
    while (hspi2.Instance->SR & SPI_SR_BSY) {
        if ((HAL_GetTick() - t0) >= 10u) break;
    }
    /* Drain RX FIFO: read DR while RXNE is set */
    while (hspi2.Instance->SR & SPI_SR_RXNE) {
        volatile uint8_t dummy = *(__IO uint8_t *)&hspi2.Instance->DR;
        (void)dummy;
    }
    /* Re-check BSY after drain */
    t0 = HAL_GetTick();
    while (hspi2.Instance->SR & SPI_SR_BSY) {
        if ((HAL_GetTick() - t0) >= 5u) break;
    }
}

/* -------------------------------------------------------------------------- */
/* IMU NSS timing constants (derived from ATMEL reference system.c):         */
/*   SPI_DLYBS  = 0x40 (64 peripheral clocks before first SPCK)             */
/*   SPI_DLYBCT = 0x20 (32 peripheral clocks between byte transfers)        */
/*   SPI clock  = 6 MHz                                                      */
/*                                                                            */
/* On our STM32 with software NSS we must replicate DLYBS manually.          */
/* DLYBCT is handled by the HAL byte-by-byte SPI engine automatically.       */
/* -------------------------------------------------------------------------- */
#define IMU_NSS_SETUP_US     5u    /* delay after CS low before first clock  */
#define IMU_NSS_HOLD_US      5u    /* delay after last clock before CS high  */
#define IMU_NSS_HIGH_US    500u    /* CS high hold: sensor needs ~300 µs to
                                    * latch the command and assert DRDY.
                                    * 20 µs was too short — Phase 2 DRDY poll
                                    * started before DRDY ever fired.        */

/* Maximum number of dummy bytes to clock before giving up on header sync.    */
/* Stream packets may arrive after up to ~5 dummy bytes, but use 48 for      */
/* extra margin against clock-stretch or any SPI pipeline latency.           */
#define IMU_HEADER_SEARCH_MAX  48u

/* Post-response inter-command delay in ms (datasheet minimum ~0.3 ms)        */
/* Note: send_imu_command_ex uses 5 ms directly; this constant is kept for   */
/* legacy callers (START_STREAM path).                                        */
#define IMU_POST_CMD_DELAY_MS   1u

/* Total RX buffer size for one SPI response: largest response (READ_REG     */
/* base 16 + up to 10 data bytes = 26) + 8 flush bytes = 34.                */
#define IMU_RX_TOTAL  34u

/* WRITE_REG ACK response size (per ATMEL IIM4623x ref: SIZE_RESP_ACK = 10)  */
#define IMU_SIZE_RESP_ACK       10u

/* READ_REG response base overhead (per ATMEL ref: SIZE_RESP_READ_REGS_BASE) */
/* Full read-reg response = SIZE_RESP_READ_REGS_BASE + data_length bytes     */
#ifndef SIZE_RESP_READ_REGS_BASE
#define SIZE_RESP_READ_REGS_BASE  16u
#endif

/* DRDY pin: PB9 (rising-edge EXTI sets drdy_interrupt; also checked as level) */
#define IMU_DRDY_PORT   GPIOB
#define IMU_DRDY_PIN    GPIO_PIN_9

void IMU_Reset(void) {
    spi2_flush();   /* clear any stuck SPI state before hardware reset */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   /* CS idle high */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(300);

    /* Harden post-boot DRDY flush:
     * The sensor asserts DRDY as a boot-complete signal.  This pulse can
     * still be HIGH (or re-assert) when we return and immediately start
     * a command.  Phase 2 of send_imu_command_ex uses a LOW→HIGH two-step
     * handshake which handles this, but we also flush here as a belt-and-
     * suspenders: wait for pin LOW (boot pulse fully gone), then wait a
     * further 50 ms for any secondary transient to settle, then clear the
     * interrupt flag twice (catches any edge that fired during the wait). */
    {
        uint32_t t0 = HAL_GetTick();
        while (HAL_GPIO_ReadPin(IMU_DRDY_PORT, IMU_DRDY_PIN) == GPIO_PIN_SET) {
            if ((HAL_GetTick() - t0) >= 200u) break;
        }
        HAL_Delay(50);           /* extra settling time after pin goes LOW */
        drdy_interrupt = 0;      /* clear once … */
        HAL_Delay(10);
        drdy_interrupt = 0;      /* … and again to catch any edge fired during the delay */
    }
    Send_UART3_Debug("IMU: reset\r\n");
}

/* -------------------------------------------------------------------------- */
/* IMU_SoftReset — send STOP_STREAMING (0x28) to return to command mode      */
/* without a hardware reset. Config registers survive.                        */
/*                                                                            */
/* STOP_STREAMING is a "common command" (cmd_type=0x28).  It follows the     */
/* same two-phase TX → DRDY-wait → RX flow as all other commands.            */
/* The sensor sends back a 10-byte ACK (same as SIZE_RESP_ACK).              */
/* -------------------------------------------------------------------------- */
void IMU_SoftReset(void) {
    static uint8_t stop_stream_pkt[] = {
        0x24, 0x24,   /* header */
        0x08,         /* total_size = 8 */
        0x28,         /* CMD_TYPE_STOP_STREAMING */
        0x00, 0x28,   /* checksum big-endian (sum of byte[3] = 0x28) */
        0x0D, 0x0A    /* footer */
    };

    /* Use send_imu_command_ex:
     *   - TX 20-byte padded frame, CS HIGH after TX
     *   - Wait DRDY (level or interrupt) with 500 ms timeout
     *   - Receive ACK (10 bytes) via header-scan on new CS assertion
     *   - 1 ms post-command gap
     */
    send_imu_command_ex(stop_stream_pkt, sizeof(stop_stream_pkt),
                        IMU_SIZE_RESP_ACK, "STOP_STREAMING", NULL);

    drdy_interrupt = 0;
    Send_UART3_Debug("IMU: soft reset (STOP_STREAMING)\r\n");
}

/* -------------------------------------------------------------------------- */
/* build_cmd20 — zero-pad any command to the mandatory 20-byte frame size.   */
/* -------------------------------------------------------------------------- */
void build_cmd20(const uint8_t *cmd, uint32_t cmd_len, uint8_t out20[IMU_CMD_PACKET_SIZE])
{
    memset(out20, 0, IMU_CMD_PACKET_SIZE);
    uint32_t copy_len = (cmd_len < IMU_CMD_PACKET_SIZE) ? cmd_len : IMU_CMD_PACKET_SIZE;
    memcpy(out20, cmd, copy_len);
}

/* -------------------------------------------------------------------------- */
/* imu_drdy_ready — returns non-zero when a new DRDY rising edge has fired.  */
/* ONLY uses the interrupt flag set by the EXTI ISR — never the pin level.   */
/* Checking pin level causes false-positives when DRDY is still high from    */
/* the previous command's response (sensor hasn't de-asserted yet).          */
static int imu_drdy_ready(void) __attribute__((unused));
static int imu_drdy_ready(void)
{
    if (drdy_interrupt) {
        drdy_interrupt = 0;
        return 1;
    }
    return 0;
}

/* -------------------------------------------------------------------------- */
/* imu_rx_one — clock one 0x00 byte on MOSI, return received byte.           */
/* SPI must already be in an active CS transaction.                           */
/* -------------------------------------------------------------------------- */
static HAL_StatusTypeDef imu_rx_one(uint8_t *out) __attribute__((unused));
static HAL_StatusTypeDef imu_rx_one(uint8_t *out)
{
    uint8_t tx = 0x00u;
    return HAL_SPI_TransmitReceive(&hspi2, &tx, out, 1, 10);
}

/* -------------------------------------------------------------------------- */
/* send_imu_command_ex — IIM-46234 SPI command/response                      */
/*                                                                            */
/* ATMEL reference pattern (system.c spi_master_write_read):                 */
/*   CS LOW → TX 20 bytes → [DRDY fires while CS is still LOW] → RX → HIGH  */
/*                                                                            */
/* CS must stay LOW for the ENTIRE TX + DRDY-wait + RX sequence.             */
/* Raising CS between TX and RX leaves the sensor's SPI shift register in an */
/* indeterminate state and it will not respond to the next command.           */
/*                                                                            */
/*  Phase 1 — FLUSH + CS LOW → TX 20 bytes (CS stays LOW)                   */
/*  Phase 2 — DRDY poll with CS still LOW (max 500 ms)                       */
/*  Phase 3 — RX with CS still LOW, then CS HIGH                             */
/*  Phase 4 — 5 ms inter-command gap (sensor state machine returns to idle)  */
/*                                                                            */
/*  Special cases:                                                            */
/*   START_STREAM 0x27: no response, raise CS after TX immediately.          */
/*   STOP_STREAM  0x28: uses normal TX+DRDY+RX flow.                         */
/*                                                                            */
/*  Fixed response sizes (ATMEL response_handler):                           */
/*   WRITE_REG  0x12 → IMU_SIZE_RESP_ACK = 10                               */
/*   READ_REG   0x11 → SIZE_RESP_READ_REGS_BASE(16) + data_len              */
/*   GET_VERSION 0x20 → 20                                                   */
/*   SELF_TEST  0x2B → 16                                                    */
/*   All others       → IMU_SIZE_RESP_ACK = 10                              */
/* -------------------------------------------------------------------------- */
int send_imu_command_ex(uint8_t *tx_packet, uint32_t tx_size,
                        uint32_t rx_size, const char *cmd_name,
                        uint8_t *rx_out)
{
    uint8_t tx20[IMU_CMD_PACKET_SIZE];
    if (tx_packet == NULL || tx_size == 0) return RET_ERR_SPI;

    build_cmd20(tx_packet, tx_size, tx20);

    /* Log TX */
    {
        char dbg[120];
        int off = snprintf(dbg, sizeof(dbg), "IMU TX20 [%s]:", cmd_name);
        for (int i = 0; i < (int)IMU_CMD_PACKET_SIZE && off < (int)(sizeof(dbg) - 4); i++)
            off += snprintf(dbg + off, sizeof(dbg) - (size_t)off, " %02X", tx20[i]);
        snprintf(dbg + off, sizeof(dbg) - (size_t)off, "\r\n");
        Send_UART3_Debug(dbg);
    }

    uint8_t cmd_type    = tx20[3];
    int is_start_stream = (cmd_type == 0x27u);

    /* Determine fixed RX size per ATMEL response_handler() */
    uint32_t fixed_rx;
    switch (cmd_type) {
    case 0x11u: /* READ_REG: base + data_len; data_len stored at tx_packet[5] */
        fixed_rx = (uint32_t)SIZE_RESP_READ_REGS_BASE + (uint32_t)tx_packet[5];
        break;
    case 0x20u: /* GET_VERSION */
        fixed_rx = 20u;
        break;
    case 0x2Bu: /* SELF_TEST */
        fixed_rx = 16u;
        break;
    default:    /* WRITE_REG 0x12, STOP_STREAM 0x28, etc. */
        fixed_rx = (uint32_t)IMU_SIZE_RESP_ACK;
        break;
    }
    if (rx_size > 0u && rx_size != fixed_rx) fixed_rx = rx_size; /* caller override */
    if (fixed_rx > 70u) fixed_rx = 70u;

    /* ================================================================ */
    /* PHASE 1: CS LOW → TX 20 bytes → CS HIGH                         */
    /*                                                                   */
    /* ATMEL spi_master_write() clocks MOSI only — TX, no MISO capture.*/
    /* Using HAL_SPI_Transmit (not TransmitReceive) matches exactly.    */
    /* The sensor ONLY begins processing the command and asserting DRDY  */
    /* AFTER it sees CS go HIGH.                                         */
    /* ================================================================ */
    drdy_interrupt = 0;

    spi2_flush();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);  /* CS LOW  */
    delay_us(IMU_NSS_SETUP_US);

    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi2, tx20, IMU_CMD_PACKET_SIZE, 100);

    delay_us(IMU_NSS_HOLD_US);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);    /* CS HIGH — sensor latches cmd */
    delay_us(IMU_NSS_HIGH_US);
    spi2_flush();

    if (st != HAL_OK) {
        Send_UART3_Debug("IMU: TX fail [%s] hal=%d\r\n", cmd_name, (int)st);
        return RET_ERR_SPI;
    }

    /* ================================================================ */
    /* START_STREAM: no ACK. CS is already HIGH. Done.                  */
    /* ================================================================ */
    if (is_start_stream) {
        Send_UART3_Debug("IMU: START_STREAM sent\r\n");
        HAL_Delay(IMU_POST_CMD_DELAY_MS);
        return RET_OK;
    }

    /* ================================================================ */
    /* PHASE 2: Wait for DRDY (CS is HIGH — sensor is processing cmd). */
    /*                                                                   */
    /* DRDY is a rising-edge pulse. Two ways we can see it:             */
    /*   (a) interrupt flag: EXTI ISR set drdy_interrupt=1              */
    /*   (b) pin level HIGH: edge fired but interrupt was already        */
    /*       processed before we got here (race window = ~500 µs)       */
    /*                                                                   */
    /* Step A: wait for any stale HIGH to fall (max 50 ms).             */
    /*         A stale HIGH means the previous command's DRDY hasn't    */
    /*         de-asserted yet — discard it.                            */
    /* Step B: clear interrupt flag, then wait for fresh DRDY.          */
    /*         Accept EITHER rising-edge interrupt OR pin HIGH.         */
    /* ================================================================ */
    {
        /* Step A — drain stale DRDY HIGH left from previous command */
        uint32_t t0 = HAL_GetTick();
        while (HAL_GPIO_ReadPin(IMU_DRDY_PORT, IMU_DRDY_PIN) == GPIO_PIN_SET) {
            if ((HAL_GetTick() - t0) >= 50u) break;
        }
        drdy_interrupt = 0;   /* discard any edge accumulated so far */

        /* Step B — wait for the fresh DRDY from THIS command */
        int drdy_ok = 0;
        t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 500u) {
            if (drdy_interrupt) {
                drdy_interrupt = 0;
                drdy_ok = 1;
                break;
            }
            /* Also accept pin level HIGH — catches the race where the
             * edge fires and the EXTI ISR runs before we enter this loop,
             * which would have set and then cleared drdy_interrupt already. */
            if (HAL_GPIO_ReadPin(IMU_DRDY_PORT, IMU_DRDY_PIN) == GPIO_PIN_SET) {
                drdy_interrupt = 0;
                drdy_ok = 1;
                break;
            }
        }
        if (!drdy_ok) {
            Send_UART3_Debug("IMU: DRDY timeout [%s]\r\n", cmd_name);
            return RET_ERR_TIMEOUT;
        }
    }

    /* ================================================================ */
    /* PHASE 3: CS LOW → RX max(fixed_rx, 22) bytes → CS HIGH          */
    /* Matches ATMEL spi_master_read(rbuf, rSize) — separate CS pulse.  */
    /* Clock at least 22 bytes (SIZE_PACKET_CMD+2) to fully drain the   */
    /* sensor's TX shift register before raising CS again.              */
    /* ================================================================ */
    {
        const uint32_t DRAIN_MIN = (uint32_t)IMU_CMD_PACKET_SIZE + 2u;
        uint32_t rx_total = (fixed_rx > DRAIN_MIN) ? fixed_rx : DRAIN_MIN;
        if (rx_total > IMU_RX_TOTAL) rx_total = IMU_RX_TOTAL;

        uint8_t rx_buf[IMU_RX_TOTAL]   = {0};
        uint8_t tx_zeros[IMU_RX_TOTAL] = {0};

        spi2_flush();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);  /* CS LOW — RX */
        delay_us(IMU_NSS_SETUP_US);

        HAL_StatusTypeDef rx_st = HAL_SPI_TransmitReceive(&hspi2, tx_zeros, rx_buf,
                                                          (uint16_t)rx_total, 100);
        delay_us(IMU_NSS_HOLD_US);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);    /* CS HIGH */
        delay_us(IMU_NSS_HIGH_US);
        spi2_flush();

        if (rx_st != HAL_OK) {
            uint32_t sr = hspi2.Instance->SR;
            Send_UART3_Debug("IMU: RX fail [%s] hal=%d SR=0x%04lX\r\n",
                             cmd_name, (int)rx_st, (unsigned long)sr);
            return RET_ERR_SPI;
        }

        /* Log first 20 raw bytes */
        {
            char dbg[120];
            int off = snprintf(dbg, sizeof(dbg), "IMU RX20 [%s]:", cmd_name);
            for (int i = 0; i < 20 && off < (int)(sizeof(dbg) - 4); i++)
                off += snprintf(dbg + off, sizeof(dbg) - (size_t)off, " %02X", rx_buf[i]);
            snprintf(dbg + off, sizeof(dbg) - (size_t)off, "\r\n");
            Send_UART3_Debug(dbg);
        }

        /* Find 0x23 0x23 header */
        uint8_t *pkt_start = NULL;
        for (uint32_t i = 0u; i + 1u < rx_total; i++) {
            if (rx_buf[i] == 0x23u && rx_buf[i+1u] == 0x23u) {
                pkt_start = &rx_buf[i];
                break;
            }
        }
        if (pkt_start == NULL) {
            Send_UART3_Debug("IMU: no 0x2323 hdr [%s]\r\n", cmd_name);
            return RET_ERR_RX;
        }

        uint32_t avail = (uint32_t)(rx_total - (uint32_t)(pkt_start - rx_buf));
        uint8_t *pkt   = pkt_start;

        if (avail < fixed_rx) {
            Send_UART3_Debug("IMU: partial pkt avail=%lu need=%lu [%s]\r\n",
                             (unsigned long)avail, (unsigned long)fixed_rx, cmd_name);
        }

        /* Footer check */
        if (avail >= fixed_rx &&
            (pkt[fixed_rx-2u] != 0x0Du || pkt[fixed_rx-1u] != 0x0Au))
            Send_UART3_Debug("IMU: bad footer [%s] %02X %02X\r\n",
                             cmd_name, pkt[fixed_rx-2u], pkt[fixed_rx-1u]);

        /* Checksum */
        if (avail >= fixed_rx && fixed_rx >= 7u) {
            uint16_t stored = ((uint16_t)pkt[fixed_rx-4u] << 8) | pkt[fixed_rx-3u];
            uint16_t calc = 0u;
            for (uint32_t ci = 3u; ci < (fixed_rx - 4u); ci++) calc += pkt[ci];
            if (stored != calc)
                Send_UART3_Debug("IMU: chk FAIL [%s] stored=%04X calc=%04X\r\n",
                                 cmd_name, (unsigned)stored, (unsigned)calc);
        }

        /* WRITE_REG error byte */
        if (cmd_type == 0x12u && avail >= 5u && pkt[4] != 0x00u) {
            Send_UART3_Debug("IMU: WRITE_REG err=0x%02X [%s]\r\n", pkt[4], cmd_name);
            return RET_ERR_RX;
        }

        if (rx_out && avail >= fixed_rx) memcpy(rx_out, pkt, fixed_rx);
    }

    /* ================================================================ */
    /* PHASE 4: Inter-command gap — 5 ms minimum.                       */
    /* Clear any DRDY that fired during the RX read itself.             */
    /* ================================================================ */
    drdy_interrupt = 0;
    HAL_Delay(5u);
    return RET_OK;
}

/* Backward-compatible wrapper */
int send_imu_command(uint8_t *tx_packet, uint32_t tx_size,
                     uint32_t rx_size, const char *cmd_name)
{
    return send_imu_command_ex(tx_packet, tx_size, rx_size, cmd_name, NULL);
}



// ===================== DAY CAMERA (VISCA) FUNCTIONS =====================
void select_day_camera(uint8_t camera_num) {
    if (camera_num > 5) camera_num = 5;
    current_day_camera = camera_num;

    // Set day camera multiplexer control pins (A=PC3, B=PB1, C=PB2)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, (camera_num & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // A
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (camera_num & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // B
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (camera_num & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // C

    HAL_Delay(10);
    Send_UART3_Debug("Selected day camera %d\r\n", camera_num);
}

void parse_and_print_version_response(uint8_t* data, uint8_t len) {
    if (len < 10) {
        Send_UART3_Debug("Day camera incomplete version response\r\n");
        return;
    }
    uint8_t address = (data[0] & 0x0F);
    (void)address;
    uint16_t model_code = (data[2] << 8) | data[3];
    uint8_t fw_major = data[4];
    uint8_t fw_minor = data[5];

    Send_UART3_Debug("Day Camera %d - Model: 0x%04X, FW: %d.%03d\r\n",
                     current_day_camera, model_code, fw_major, fw_minor);
}

// was: void wait_for_visca_ack_and_completion(...)
int wait_for_visca_ack_and_completion(uint32_t timeout_ms, int parse_version)
{
    uint32_t start = HAL_GetTick();

    // Example: wait until visca_rx_ready set by RX ISR/parser
    while (!visca_rx_ready) {
        if ((HAL_GetTick() - start) > timeout_ms) {
        	Send_UART3_Debug("RET Timeout");
        	return RET_ERR_TIMEOUT;

        }
    }

    // Optional: parse/validate response buffer here and return RET_ERR_RX if invalid
    // If parse_version != 0, parse version fields and confirm.

//    Send_UART3_Debug("ACK Received! \r\n");
    return RET_OK;

}


int send_visca_command(uint8_t* cmd, uint8_t cmd_len, int parse_version)
{
    if (cmd == NULL || cmd_len == 0) return RET_ERR_TX;

    visca_rx_idx = 0;
    visca_rx_ready = 0;
    memset(visca_rx_buffer, 0, sizeof(visca_rx_buffer));

    // Start RX first (but RX might already be active => HAL_BUSY is OK)
    HAL_StatusTypeDef rxst = HAL_UART_Receive_IT(&huart2, &visca_rx_byte, 1);

    if (rxst == HAL_BUSY) {
        // Option A: just accept BUSY (RX already active)
        // do nothing
    } else if (rxst != HAL_OK) {
        // Option B (more aggressive): abort and try once more
        (void)HAL_UART_AbortReceive_IT(&huart2);
        rxst = HAL_UART_Receive_IT(&huart2, &visca_rx_byte, 1);
        if (rxst != HAL_OK && rxst != HAL_BUSY){

        	return RET_ERR_RX;
        }
    }

    // TX
    if (HAL_UART_Transmit(&huart2, cmd, cmd_len, 1000) != HAL_OK){
    	return RET_ERR_TX;
    	Send_UART3_Debug("Transmit Error! \r\n");
    }


    // Wait for ACK/completion (0 OK, <0 fail/timeout)
    return wait_for_visca_ack_and_completion(1000, parse_version);
}


// Return 0 on success, negative on failure
int test_day_camera_controls(uint8_t camera_num)
{
    select_day_camera(camera_num);
    HAL_Delay(10);

    Send_UART3_Debug("Testing Day Camera %d controls...\r\n", camera_num);

    static uint8_t visca_version_inquiry[] = { 0x81, 0x09, 0x00, 0x02, 0xFF };

    // If send_visca_command has a return value, use it.
    // If it returns void today, change it to return int (0 success / <0 fail).
    int r = send_visca_command(visca_version_inquiry,
                              (uint8_t)sizeof(visca_version_inquiry),
                              1);

    HAL_Delay(10);

    if (r != 0) {
        Send_UART3_Debug("Day Camera %d: VISCA version inquiry send FAIL (r=%d)\r\n", camera_num, r);
        return RET_ERR_TX;
    }

    // Future improvement: wait for VISCA ACK/response flag with timeout and return -2 on timeout.
    Send_UART3_Debug("CAMERA %d OK!! \r\n\n", camera_num);
    return RET_OK;

}



// ===================== IR CAMERA FUNCTIONS =====================
void select_ir_camera(uint8_t camera_num) {
    if (camera_num > 5) camera_num = 5;
    current_ir_camera = camera_num;

        // Set IR camera multiplexer control pins (A=PC0, B=PC1, C=PC2)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (camera_num & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // A
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (camera_num & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // B
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, (camera_num & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // C

    HAL_Delay(10);
    Send_UART3_Debug("Selected IR camera %d\r\n", camera_num);
}

uint8_t ComputeChecksum(uint8_t* message_buffer, uint32_t length) {
    uint8_t sum = 0;
    for(uint32_t i = 0; i < length; i++) sum += message_buffer[i];
    return (uint8_t)((~sum) + 1);
}

// 2) IR camera UART: change to return int
int Send_IR_Camera_Command(uint8_t message_id, uint8_t *params, uint32_t param_size)
{
    uint8_t command_buffer[32];

    if (param_size > (sizeof(command_buffer) - 4u))
        return RET_ERR_TX;

    command_buffer[0] = 0x01;
    command_buffer[1] = message_id;
    command_buffer[2] = (uint8_t)param_size;

    if (param_size > 0 && params != NULL) {
        memcpy(&command_buffer[3], params, param_size);
    }

    uint32_t body_length = 3u + param_size;
    command_buffer[body_length] = ComputeChecksum(command_buffer, body_length);

    if (HAL_UART_Transmit(&huart1, command_buffer, (uint16_t)(body_length + 1u), 100) != HAL_OK)
        return RET_ERR_TX;

    return RET_OK;
}


bool Validate_IR_Parameter(uint16_t param_id, uint16_t value) {
    switch(param_id) {
        case PARAM_ID_NUC_COMMAND:   return value <= 2;
        case PARAM_ID_ROTATION:      return value <= 3;
        case PARAM_ID_FRAMERATE:     return value <= 8;
        case PARAM_ID_PALETTE:       return value >= 1 && value <= 6;
        case PARAM_ID_POLARITY:      return value <= 1;
        case PARAM_ID_D_ZOOM:        return value <= 12;
        case PARAM_ID_TNR_ENABLE:    return value <= 1;
        case PARAM_ID_POWER_RESET:   return value <= 1;
        default: return false;
    }
}

void Translate_And_Send_IR_Camera_Command(uint16_t param_id, uint16_t value) {
    uint8_t payload[4];
    switch(param_id) {
    case PARAM_ID_POLARITY:
        if (value == 1) {
            Send_IR_Camera_Command(0x29, NULL, 0);
            Send_UART3_Debug("IR Camera %d: Set Polarity WHITE HOT\r\n", current_ir_camera);
        } else {
            Send_IR_Camera_Command(0x28, NULL, 0);
            Send_UART3_Debug("IR Camera %d: Set Polarity BLACK HOT\r\n", current_ir_camera);
        }
        break;
    case PARAM_ID_D_ZOOM:
        payload[0] = 0x00;
        payload[1] = (uint8_t)value;
        Send_IR_Camera_Command(0xA4, payload, 2);
        Send_UART3_Debug("IR Camera %d: Set Digital Zoom: %d\r\n", current_ir_camera, value);
        break;
    case PARAM_ID_PALETTE:
        payload[0] = 0x00;
        payload[1] = value;
        Send_IR_Camera_Command(0x58, payload, 2);
        Send_UART3_Debug("IR Camera %d: Set Palette: %d\r\n", current_ir_camera, value);
        break;
    default:
        Send_UART3_Debug("IR Camera %d: Unknown Param ID 0x%04X\r\n", current_ir_camera, param_id);
        break;
    }
}

void Process_IR_Camera_Received_Data(uint8_t* data, uint16_t length) {
    uint16_t current_pos = 0;
    while (current_pos < length) {
        if (data[current_pos] == 0x01) {
            if (current_pos + 3 < length) {
                uint8_t message_id = data[current_pos + 1];
                uint8_t param_len = data[current_pos + 2];
                uint32_t total_msg_len = 4 + param_len;
                if (current_pos + total_msg_len <= length) {
                    uint8_t* msg_ptr = &data[current_pos];
                    if (msg_ptr[total_msg_len - 1] == ComputeChecksum(msg_ptr, total_msg_len - 1)) {
                        if (message_id == 0x00) {
                            Send_UART3_Debug("IR Camera %d Text: %.*s\r\n", current_ir_camera, param_len, (char*)&msg_ptr[3]);
                        } else if (message_id == 0x02) {
                            Send_UART3_Debug("IR Camera %d ACK received\r\n\n\n", current_ir_camera);
                        }
                        current_pos += total_msg_len; continue;
                    }
                }
            }
        }
        current_pos++;
    }
}


int IR_FactoryResetNV(void)
{
    uint8_t cmd[4];
    cmd[0] = 0x01;
    cmd[1] = 0xB3;
    cmd[2] = 0x00;
    cmd[3] = ComputeChecksum(cmd, 3);

    // Arm RX cleanly
    ir_camera_data_received_flag = false;
    ir_camera_received_bytes = 0;
    memset(ir_camera_rx_buffer, 0, sizeof ir_camera_rx_buffer);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof ir_camera_rx_buffer);

    HAL_UART_Transmit(&huart1, cmd, sizeof(cmd), 200);

    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 1000) {
        if (!ir_camera_data_received_flag) continue;
        ir_camera_data_received_flag = false;

        for (uint16_t i = 0; i + 5 < ir_camera_received_bytes; i++) {
            if (ir_camera_rx_buffer[i]     == 0x01 &&
                ir_camera_rx_buffer[i + 1] == 0x02 &&   // ACK
                ir_camera_rx_buffer[i + 3] == 0x00 &&
                ir_camera_rx_buffer[i + 4] == 0xB3) {
                mcu_ui_log(&g_ctx, "IR cam %d: FactoryResetNV OK\r\n", current_ir_camera);
                return 0; // SUCCESS
            }
        }
    }
    mcu_ui_log(&g_ctx, "IR cam %d: FactoryResetNV FAILED (no ACK)\r\n", current_ir_camera);
    return -1; // FAILED
}


//IR Init Helpers

/* ir_flush_rx: explicit drain only — called ONCE before verbose mode command.
 * Must NOT be called from IR_ArmRx — doing so kills the live ReceiveToIdle_IT
 * that the callback re-arms, breaking the handshake.                        */
static void ir_flush_rx(UART_HandleTypeDef *huart)
{
    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    __HAL_UART_CLEAR_OREFLAG(huart);
    uint8_t dummy;
    while (HAL_UART_Receive(huart, &dummy, 1, 0) == HAL_OK) {}
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

/* IR_ArmRx: matches original working code exactly.
 * Just resets the data-ready flag and re-arms ReceiveToIdle_IT.
 * The callback always re-arms after each received packet, so the UART
 * is always listening.  IR_ArmRx clears the flag so IR_WaitPacket waits
 * for the response to THIS specific command, not a stale one.           */
static void IR_ArmRx(void) {
    ir_camera_data_received_flag = false;
    ir_camera_received_bytes = 0;
    memset(ir_camera_rx_buffer, 0, sizeof ir_camera_rx_buffer);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof ir_camera_rx_buffer);
}

static int IR_WaitPacket(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms) {
        if (ir_camera_data_received_flag) return 0;
    }
    return -1;
}

/* Parse packets of form:
   [0]=0x01, [1]=msg_id, [2]=len, [3..3+len-1]=payload, [last]=checksum
*/
static int IR_FindPacket(uint8_t wanted_id, uint8_t wanted_len, uint16_t *out_payload_u16) {
    uint16_t n = ir_camera_received_bytes;
    for (uint16_t i = 0; i + 4u <= n; i++) {
        if (ir_camera_rx_buffer[i] != 0x01) continue;

        uint8_t id  = ir_camera_rx_buffer[i+1];
        uint8_t len = ir_camera_rx_buffer[i+2];
        uint16_t total = (uint16_t)(4u + len);
        if (i + total > n) continue;

        uint8_t *p = &ir_camera_rx_buffer[i];
        if (p[total-1] != ComputeChecksum(p, total-1)) continue;

        if (id == wanted_id && (wanted_len == 0xFF || len == wanted_len)) {
            if (out_payload_u16 && len >= 2) {
                *out_payload_u16 = ((uint16_t)p[3] << 8) | p[4];
            }
            return 0;
        }
    }
    return -1;
}

static int IR_SetNV(uint16_t id, uint16_t val) {
    uint8_t p[4] = { (uint8_t)(id>>8), (uint8_t)(id&0xFF), (uint8_t)(val>>8), (uint8_t)(val&0xFF) };

    IR_ArmRx();
    Send_IR_Camera_Command(0xB0, p, 4);

    if (IR_WaitPacket(500) != 0) {
        Send_UART3_Debug("SetNV#%u timeout\r\n", (unsigned)id);
        return -1;
    }

    /* ACK (0x02): setting accepted immediately.                          */
    if (IR_FindPacket(0x02, 0xFF, NULL) == 0) return 0;

    /* NACK (0x06): camera accepted the write but requires a power-cycle  *
     * to activate (e.g. NV17 Genlock Direction, NV18 Genlock Mode).      *
     * Treat as success — the value IS stored; reboot will apply it.      */
    if (IR_FindPacket(0x06, 0xFF, NULL) == 0) {
        Send_UART3_Debug("SetNV#%u NACK(reboot needed) — stored OK\r\n", (unsigned)id);
        return 0;
    }

    /* Unknown response — log first 8 bytes for diagnostics */
    {
        char dbg[80];
        int off = snprintf(dbg, sizeof(dbg), "SetNV#%u unknown resp(%u):", (unsigned)id, ir_camera_received_bytes);
        for (int i = 0; i < 8 && i < (int)ir_camera_received_bytes && off < (int)sizeof(dbg)-4; i++)
            off += snprintf(dbg+off, sizeof(dbg)-(size_t)off, " %02X", ir_camera_rx_buffer[i]);
        snprintf(dbg+off, sizeof(dbg)-(size_t)off, "\r\n");
        Send_UART3_Debug(dbg);
    }
    return -1;
}

static int IR_GetNV(int id, int *out) {
    uint8_t p[2] = { (uint8_t)(id>>8), (uint8_t)(id&0xFF) };

    IR_ArmRx();
    Send_IR_Camera_Command(0xB5, p, 2);

    if (IR_WaitPacket(500) != 0) {
        Send_UART3_Debug("GetNV#%d timeout\r\n", id);
        return -1;
    }

    uint16_t v = 0;
    if (IR_FindPacket(0x45, 0x02, &v) == 0) { *out = (int)v; return 0; }

    /* Log first 8 bytes so we can identify the actual response packet ID */
    {
        char dbg[80];
        int off = snprintf(dbg, sizeof(dbg), "GetNV#%d unknown resp(%u):", id, ir_camera_received_bytes);
        for (int i = 0; i < 8 && i < (int)ir_camera_received_bytes && off < (int)sizeof(dbg)-4; i++)
            off += snprintf(dbg+off, sizeof(dbg)-(size_t)off, " %02X", ir_camera_rx_buffer[i]);
        snprintf(dbg+off, sizeof(dbg)-(size_t)off, "\r\n");
        Send_UART3_Debug(dbg);
    }
    return -1;
}

static int IR_SetMux(uint16_t sel) {
    uint8_t p[2] = { sel>>8, sel&0xFF };
    Send_IR_Camera_Command(0xD7, p, 2);               // Digital Video Mux Select
    // ACK to 0xD7 will arrive; no value to read back
    return 0;
}




// Return 0 on success, negative on failure
int Initialize_IR_Camera(uint8_t camera_num) {
    select_ir_camera(camera_num);
    HAL_Delay(100);

    Send_UART3_Debug("Initializing IR Camera %d...\r\n", camera_num);

    // Factory reset NV — restores all NV parameters to defaults before writing ours
    {
        int r = IR_FactoryResetNV();
        if (r != 0) {
            mcu_ui_log(&g_ctx, "IR cam %d: FactoryResetNV FAILED — continuing anyway\r\n", camera_num);
        }
        HAL_Delay(10);
    }

    // Enable Verbose Mode
    uint8_t verbose_params[] = {0x00, 0x01};
    Send_IR_Camera_Command(0xFF, verbose_params, sizeof(verbose_params));
    HAL_Delay(100);
    if (ir_camera_data_received_flag) {
        Process_IR_Camera_Received_Data(ir_camera_rx_buffer, ir_camera_received_bytes);
        ir_camera_data_received_flag = false;
        // Do NOT return — verbose mode response is expected; NV commands must still run
        //Return only when NV commands not required
        return RET_OK;
    }

    Send_UART3_Debug("\r\n--- STEP 2: Here Performing Handshake ---\r\n");
    Send_IR_Camera_Command(0x07, NULL, 0);
    HAL_Delay(100);

    bool handshake_success = false;
    if (ir_camera_data_received_flag) {
        // Parse and validate the response
        uint16_t current_pos = 0;
        while (current_pos < ir_camera_received_bytes) {
            if (ir_camera_rx_buffer[current_pos] == 0x01) {
                if (current_pos + 3 < ir_camera_received_bytes) {
                    uint8_t message_id = ir_camera_rx_buffer[current_pos + 1];
                    uint8_t param_len = ir_camera_rx_buffer[current_pos + 2];
                    uint32_t total_msg_len = 4 + param_len;

                    if (current_pos + total_msg_len <= ir_camera_received_bytes) {
                        uint8_t* msg_ptr = &ir_camera_rx_buffer[current_pos];
                        // Validate checksum
                        if (msg_ptr[total_msg_len - 1] == ComputeChecksum(msg_ptr, total_msg_len - 1)) {
                            if (message_id == 0x02) { // ACK message
                                handshake_success = true;
                                Send_UART3_Debug("\r\n--- !!!!!!!!!!!!! Handshake successful - Valid ACK received ---\r\n\n\n");
                                break;
                            } else if (message_id == 0x00) { // Text message
                                Send_UART3_Debug("Camera %d Text: %.*s\r\n", camera_num, param_len, (char*)&msg_ptr[3]);
                                handshake_success = true; // Text response also indicates camera is responding
                            }
                        }
                    }
                }
            }
            current_pos++;
        }
        ir_camera_data_received_flag = false;
    }

    if (!handshake_success) {
        Send_UART3_Debug("\r\n--- ERROR: Camera %d handshake failed - No valid response ---\r\n\n\n", camera_num);
        return RET_ERR_TX; // Exit if handshake failed
    }

    // Enable NTSC Analog Video
//    uint8_t payload[] = {0x00, 0x04, 0x00, 0x01};
//    Send_IR_Camera_Command(0xB0, payload, 4);
//    HAL_Delay(100);
//    if (ir_camera_data_received_flag) {
//        Process_IR_Camera_Received_Data(ir_camera_rx_buffer, ir_camera_received_bytes);
//        ir_camera_data_received_flag = false;
//    }





   IR_SetMux(9);
   IR_SetNV(7, 9);
   IR_SetNV(6,0);
   IR_SetNV(14,1);


    // --- Parallel 8-bit post-AGC on PDVO ---
    IR_SetNV(4, 0);        // NV#4 RS-170 = disable
    IR_SetNV(5, 1);        // NV#5 PDVO = enable
    IR_SetNV(76, 0);






    // Verify PDVO settings
    int nv4=-1,nv5=-1,nv7=-1,nv76=-1;
    IR_GetNV(4,&nv4); IR_GetNV(5,&nv5); IR_GetNV(7,&nv7); IR_GetNV(76,&nv76);
    mcu_ui_log(&g_ctx, "Verify PDVO: NV4=%d NV5=%d NV7=%d NV76=%d\r\n", nv4,nv5,nv7,nv76);

    // --- 30 fps ---
    IR_SetNV(16,1);       // NV#16 Frame Rate = 1 => 30 Hz
    int nv16=-1; IR_GetNV(16,&nv16);
    mcu_ui_log(&g_ctx, "Verify FrameRate: NV16=%d\r\n", nv16);

    // --- Genlock slave / input ---
    IR_SetNV(17, 1);       // NV#17 Genlock Direction = input (slave mode)
    IR_SetNV(18, 0);       // NV#18 Genlock Mode = slave

    int nv17=-1,nv18=-1;
    IR_GetNV(17,&nv17); IR_GetNV(18,&nv18);
    mcu_ui_log(&g_ctx, "Verify Genlock: NV17=%d NV18=%d\r\n", nv17, nv18);

    mcu_ui_log(&g_ctx, "IR Camera %d initialization complete\r\n", camera_num);
    return RET_OK;
}



void Read_HDC3022_Sensor(float* temp, float* hum) {
    uint8_t sensor_data_buffer[6];
    uint8_t trigger_cmd[] = {0x24, 0x00};
    if (HAL_I2C_Master_Transmit(&hi2c2, (0x44 << 1), trigger_cmd, 2, 1000) != HAL_OK) {
        Send_UART3_Debug("I2C Transmit Error\r\n");
        *temp = -999.0f;
        *hum = -999.0f;
        return;
    }
    HAL_Delay(20);
    if (HAL_I2C_Master_Receive(&hi2c2, (0x44 << 1), sensor_data_buffer, 6, 1000) != HAL_OK) {
        Send_UART3_Debug("I2C Receive Error\r\n");
        *temp = -999.0f;
        *hum = -999.0f;
        return;
    }
    uint16_t raw_temp = (sensor_data_buffer[0] << 8) | sensor_data_buffer[1];
    uint16_t raw_hum = (sensor_data_buffer[3] << 8) | sensor_data_buffer[4];
    *temp = 175.0f * ((float)raw_temp / 65535.0f) - 45.0f;
    *hum = 100.0f * ((float)raw_hum / 65535.0f);
}

void mcu_on_new_attitude(int16_t roll_cd, int16_t pitch_cd, uint16_t rate_hz) {
  g_fpga_orient.roll_cd = roll_cd;
  g_fpga_orient.pitch_cd = pitch_cd;
  g_fpga_orient.rate_hz = rate_hz;
  g_fpga_orient.seq++;
  g_fpga_orient.pending = true;
}

/* fpga_service_orient is implemented in mcu_fsm.c */



// ------------------------------------------------------------
// EXTI callback
// PB9: IMU DRDY
// PC4: FPGA VSYNC
// ------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_9) { // PB9 DRDY
        drdy_interrupt = 1;       /* used by send_imu_command_ex Phase 2 (command mode) */
        g_imu_drdy_flag = 1;      /* used by main loop -> mcu_on_imu_drdy (streaming mode) */
    } else if (GPIO_Pin == GPIO_PIN_4) { // PC4 VSYNC
        g_vsync_flag = 1;
    }
}

// ------------------------------------------------------------
// UART Rx Event callback (IDLE)
// UART3 is UI binary stream (COBS delimited by 0x00)
// ------------------------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {  // Day camera (VISCA)
        if (visca_rx_idx < sizeof(visca_rx_buffer)) {
            visca_rx_buffer[visca_rx_idx++] = visca_rx_byte;
            if (visca_rx_byte == 0xFF) visca_rx_ready = 1;
        }
        HAL_UART_Receive_IT(&huart2, &visca_rx_byte, 1);
//        Send_UART3_Debug("day cam call triggered\r\n");
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {  // IR camera
        ir_camera_data_received_flag = true;
        ir_camera_received_bytes = Size;
        /* Always re-arm — keeps the UART listening between NV commands and
         * during the handshake / verbose-mode response window.             */
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof ir_camera_rx_buffer);
    }
    else if (huart->Instance == USART3) { // UI protocol (COBS + CRC)
        ui_rx_chunk_len = Size;
        ui_rx_chunk_ready = 1;

        // re-arm for next chunk
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, ui_rx_chunk, UI_RX_DMA_MAX);
    }
}


// ------------------------------------------------------------
// UI accumulator helpers
// ------------------------------------------------------------
static void ui_acc_reset(void)
{
    ui_acc_len = 0;
    memset(ui_acc, 0, sizeof(ui_acc));
}

// Feed bytes into accumulator, split frames by 0x00 delimiter,
// parse each COBS frame, then forward payload to mcu_on_ui_payload().
static void ui_process_bytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = data[i];

        if (b == 0x00) {
            // delimiter -> parse current frame if any
            if (ui_acc_len > 0) {
                uint8_t payload[UI_PAYLOAD_MAX];
                size_t pay_len = uart_frame_parse(ui_acc, ui_acc_len, payload, sizeof(payload));
                if (pay_len > 0) {
                    mcu_on_ui_payload(&g_ctx, payload, (uint16_t)pay_len);
                }
            }
            ui_acc_reset();
            continue;
        }

        // normal byte
        if (ui_acc_len < UI_ACC_MAX) {
            ui_acc[ui_acc_len++] = b;
        } else {
            // overflow -> drop
            ui_acc_reset();
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, ir_camera_rx_buffer, sizeof(ir_camera_rx_buffer));
  MX_USART2_UART_Init();
  HAL_UART_Receive_IT(&huart2, &visca_rx_byte, 1);

  MX_USART3_UART_Init();
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, ui_rx_chunk, UI_RX_DMA_MAX);

  MX_I2C2_Init();
  MX_I2C3_Init();
  fpga_i2c_init(&g_fpga, &hi2c3, FPGA_I2C_ADDR7);


  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */

  // Init FSM context
  mcu_fsm_init(&g_ctx);

  // Arm UART3 UI receiver
  ui_acc_reset();
  (void)HAL_UARTEx_ReceiveToIdle_IT(&huart3, ui_rx_chunk, UI_RX_DMA_MAX);


//  Send_UART3_Debug("waiting for INIT");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    // Consume UI R if ready
    if (ui_rx_chunk_ready) {
        ui_rx_chunk_ready = 0;
        if (ui_rx_chunk_len > 0) {
            ui_process_bytes(ui_rx_chunk, ui_rx_chunk_len);
        }
        ui_rx_chunk_len = 0;
    }

    // Progress FSM (contains blocking delays by design)
        mcu_fsm_step(&g_ctx);


    // Consume IMU DRDY events
    if (g_imu_drdy_flag) {
        g_imu_drdy_flag = 0;
        mcu_on_imu_drdy(&g_ctx);
    }

    if (g_vsync_flag) {
        g_vsync_flag = 0;
        mcu_on_vsync(&g_ctx);
    }

    // NOTE: No HAL_Delay() here. The main loop must be free-running so IMU
    // DRDY events at 416 Hz (every 2.4 ms) are processed with minimum latency.
    // Any HAL_Delay() here would add that many ms of lag to EVERY IMU sample.
  }
  /* USER CODE END WHILE */
}

/* ===================== CLOCK / PERIPH INIT ===================== */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}



static void MX_I2C3_Init(void)
{
  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */

  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00503D58;  // start same as I2C2 (adjust later if needed)
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;   /* CPOL=1 — clock idles HIGH */
  hspi2.Init.CLKPhase    = SPI_PHASE_2EDGE;    /* CPHA=1 → SPI Mode 3 (datasheet confirmed; Atmel NCPHA=0 = standard CPHA=1) */
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  /* ~4 MHz (ATMEL ref uses 6 MHz) */
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* Force RXFIFO threshold to 8-bit (1 byte).
   * STM32G0 SPI FIFO default threshold is 16-bit — RXNE only fires after
   * 2 bytes are received.  For 8-bit transfers this causes HAL_SPI_TransmitReceive
   * to stall on the last byte of any odd-length transfer (10-byte ACK,
   * 18-byte READ_REG response, 22-byte drain, etc.).
   * Setting FRXTH=1 makes RXNE fire after every single byte.            */
  hspi2.Instance->CR2 |= SPI_CR2_FRXTH;

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;   /* 921600: ~0.16 ms/frame vs 1.3 ms at 115200 — needed for 1kHz IMU */
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC11 PC12 PC13 PC0
                           PC1 PC2 PC3 PC8
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PB9 interrupt pin (DRDY)
      GPIO_InitStruct.Pin = GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      // PC4 interrupt pin (FPGA VSYNC)
      GPIO_InitStruct.Pin = GPIO_PIN_4;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

      // Enable EXTI4_15 interrupt for PB9
      HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);


      // PA0 reset pin (output)
          GPIO_InitStruct.Pin = GPIO_PIN_0;
          GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
          GPIO_InitStruct.Pull = GPIO_PULLUP;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
          HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

          // PB12 NSS pin (output)
          GPIO_InitStruct.Pin = GPIO_PIN_12;
          GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
          GPIO_InitStruct.Pull = GPIO_PULLUP;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
          HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

          // SPI2 pins (PB13, PB14, PB15)
          GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
          GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull = GPIO_NOPULL;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
          GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
          HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  /*Configure GPIO pins : PD0 PD1 PD3 PD4
                           PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
