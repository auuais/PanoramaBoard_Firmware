// mcu_fsm.c
// UART3 is reserved for the binary UI protocol (COBS + CRC16).
// Any "logs" must be sent as MSG_LOG frames (not raw printf).

#include "mcu_fsm.h"
#include "main.h"
#include "proto_uart.h"
#include "fpga_i2c.h"
#include "fpga_regs.h"
#include "imu_handler.h"
#include "imu_config.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stddef.h>
#include <limits.h>

//
static void fpga_service_orient(void);
static int imu_stream_read_raw(uint8_t *dst);
static void ui_send_log(McuContext *ctx, const char *s);


// ---- externs from existing codebase ----
extern void Enable_All_Camera_Power(void);
extern int Power_Cycle_All_Cameras(void);
extern void IMU_Reset(void);
extern void IMU_SoftReset(void);
extern int send_imu_command(uint8_t *tx_packet, uint32_t tx_size, uint32_t rx_size, const char *cmd_name);
extern int send_imu_command_ex(uint8_t *tx_packet, uint32_t tx_size, uint32_t rx_size, const char *cmd_name, uint8_t *rx_out);
extern void Read_HDC3022_Sensor(float* temp, float* hum);
extern void Send_UART3_Debug(const char *format, ...);
extern int test_day_camera_controls(uint8_t camera_num);
extern int Initialize_IR_Camera(uint8_t camera_num);
extern int Apply_IR_Camera_Param(uint8_t camera_sel, uint16_t param_id, uint16_t value);
extern int Apply_EO_Camera_Param(uint8_t camera_sel, uint16_t param_id, uint16_t value);
extern SPI_HandleTypeDef hspi2;

// UART3 (PC UI)
extern UART_HandleTypeDef huart3;

// Optional global debug mirror (so legacy code can keep calling Send_UART3_Debug safely)
extern volatile bool g_debug_enabled;
extern fpga_i2c_t g_fpga;
extern void IMU_Handler_SetSampleFreq(float sample_freq);
extern volatile uint32_t g_imu_drdy_count;
extern volatile uint32_t g_imu_drdy_edges_total;

// ---- config ----
#define UI_TX_MAX   512
#define UI_PAY_MAX  256

// ---- Message types ----
#define MSG_CMD   0x01
#define MSG_RESP  0x02
#define MSG_EVT   0x03
#define MSG_LOG   0x7F

// ---- Event subtypes ----
#define EVT_IMU_ORIENT  0x20   // roll_cd (int16 LE) + pitch_cd (int16 LE)
#define EVT_IMU_ODR     0x21   // rate_hz (uint16 LE)
#define EVT_IMU_STATS   0x22   // drdy_hz + pkt_ts_hz + parse_ok_hz (uint16 LE each)
#define EVT_CFG_BANK    0x30   // [sub][bank][mode][yaw/pitch/roll offsets + overlap + feather]
#define EVT_CFG_APPLY   0x31   // [sub][bank][ok][detail]
#define EVT_LOG_CTRL    0x40   // [sub][cmd]

// ---- Command IDs ----
#define CMD_CHECK_NODE   0x10
#define CMD_SET_MODE     0x11
#define CMD_SET_DEBUG    0x12
#define CMD_INIT_SYSTEM  0x13
#define CMD_CAMERA_RESET 0x14
#define CMD_PING         0x15
#define CMD_GET_CONFIG   0x30
#define CMD_APPLY_CONFIG 0x31
#define CMD_CAMERA_CONTROL 0x32

// ---- Node types (for CHECK) ----
#define NODE_SYSTEM 0x00
#define NODE_IMU   0x01
#define NODE_TEMP  0x02
#define NODE_FPGA  0x03
#define NODE_DAY   0x04
#define NODE_IR    0x05

// ---- Error codes ----
#define ERR_OK     0x00
#define ERR_BUSY   0x01
#define ERR_BADARG 0x02
#define ERR_FAIL   0x03

// ---- Parameter/config protocol ----
#define CFG_BANK_EO      0x00
#define CFG_BANK_IR      0x01
#define CFG_CAM_COUNT    6
#define CFG_BANK_PAYLOAD_SIZE 42u

// ---- Orientation safety clamp (hard limits, not artificial) ----
#define ORIENT_CLAMP_DEG      (90.0f)
#define IMU_STREAM_BURST_MAX  16u
#define UI_ORIENT_DECIM       10u
#define LOG_CTRL_CLEAR        0x01u
#define LOG_CTRL_SAVE         0x02u
#define LOG_CAPTURE_SAVE_DELAY_MS 45000u
#define FPGA_DEBUG_TOP_MODE_ONLY 0u

// ---- IMU command packets (matching ATMEL IIM4623x reference exactly) ----
// WHO_AM_I = ReadRegister(addr=0x00, len=1, page=0)
// IIM4623x_SetCMD_ReadRegister: SIZE_CMD_READ_REGS=12, checksum=sum(bytes[3..7])
//   bytes[3..7] = {0x11, 0x00, 0x01, 0x00, 0x00} -> checksum = 0x0012
static uint8_t who_am_i_packet[] = {
    0x24, 0x24,       // header
    0x0C,             // total size = 12
    0x11,             // CMD_TYPE_READ_REG
    0x00,             // reserved
    0x01,             // length = 1 byte
    0x00,             // first_addr = 0x00 (WHO_AM_I register)
    0x00,             // page_id = 0
    0x00, 0x12,       // checksum big-endian (0x11+0x00+0x01+0x00+0x00 = 0x12)
    0x0D, 0x0A        // footer
};

// START_STREAMING = Common command (cmd_type=0x27)
// IIM4623x_SetCMD_Common: SIZE_CMD_COMMON=8, checksum=sum(bytes[3..3])
//   bytes[3] = 0x27 -> checksum = 0x0027
static uint8_t start_streaming_packet[] = {
    0x24, 0x24,       // header
    0x08,             // total size = 8
    0x27,             // CMD_TYPE_START_STREAMING
    0x00, 0x27,       // checksum big-endian (0x27)
    0x0D, 0x0A        // footer
};

// STOP_STREAMING (cmd_type=0x28) is handled by IMU_SoftReset() in main.c


static volatile int16_t s_roll_cd;
static volatile int16_t s_pitch_cd;
static volatile int16_t s_rate_hz;
static volatile bool s_vsync_pending = false;
static uint16_t s_orient_tx_seq = 0;
static uint32_t s_odr_accum_us = 0;
static uint16_t s_odr_samples = 0;
static uint8_t s_stream_valid_log_sent = 0u;
static uint8_t s_stream_odr_log_sent = 0u;
static uint8_t s_stream_diag_count = 0;
static uint8_t s_prev_sample_counter = 0;
static uint64_t s_prev_timestamp_us = 0;
static uint16_t s_ui_throttle = 0u;
static uint32_t s_stats_window_start_ms = 0u;
static uint32_t s_stats_edges_base = 0u;
static uint32_t s_stats_parse_ok = 0u;
static uint32_t s_stats_ts_delta_us = 0u;
static uint32_t s_stats_ts_delta_count = 0u;
static float s_roll_deg_filt = 0.0f;
static float s_pitch_deg_filt = 0.0f;
static bool s_orient_filter_init = false;

/* -------------------------------------------------------------------------- */
/* Light output-stage smoothing and 0.1 degree quantization.                  */
/*                                                                             */
/* Madgwick IS the filter. Its quaternion integrator already smooths the      */
/* gyro + accel fusion ??we do not need a second software filter on top.      */
/* A very small EMA here targets only the final visible flutter and is tuned  */
/* to stay well below 10 ms of added lag.                                     */
/*                                                                             */
/* Output behavior:                                                            */
/*   - EMA time constant is about 5 ms                                        */
/*   - Rounds centidegrees to the nearest 10 cd (= 0.1 degree)                */
/*   - Any residual flutter smaller than +-5 cd (+-0.05 degree) is absorbed   */
/* -------------------------------------------------------------------------- */
#define QUANT_STEP_CD            10   /* 0.1 degree = 10 centidegrees */
#define ORIENT_SMOOTH_TAU_MS      10.0f

/**
 * @brief Round val_cd to the nearest QUANT_STEP_CD increment.
 *
 * Works correctly for positive and negative values.
 * Example: 137 cd ??125 cd,  138 cd ??150 cd,  -137 cd ??-125 cd
 */
static inline int16_t quantize_cd(int16_t val_cd)
{
    /* Add half-step with correct sign for rounding, then truncate */
    int16_t half = QUANT_STEP_CD / 2;
    if (val_cd >= 0) {
        return (int16_t)(((val_cd + half) / QUANT_STEP_CD) * QUANT_STEP_CD);
    } else {
        return (int16_t)(((val_cd - half) / QUANT_STEP_CD) * QUANT_STEP_CD);
    }
}

typedef struct {
    uint16_t yaw_cd[CFG_CAM_COUNT];
    int16_t pitch_cd[CFG_CAM_COUNT];
    int16_t roll_cd[CFG_CAM_COUNT];
    uint16_t overlap_px;
    int32_t feather_strength_q16;
} ParamBankQ16;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t crc32;
    uint8_t mode;
    uint8_t reserved0[3];
    ParamBankQ16 eo;
    ParamBankQ16 ir;
    uint32_t write_count;
    uint32_t reserved1;
} ParamStoreBlob;

#define PARAM_STORE_MAGIC      0x5041524Du /* "PARM" */
#define PARAM_STORE_VERSION    0x0003u
#define PARAM_FLASH_PAGE_COUNT 1u

enum {
    CFG_APPLY_OK = 0,
    CFG_APPLY_ERR_BADARG = 1,
    CFG_APPLY_ERR_I2C_WRITE = 2,
    CFG_APPLY_ERR_I2C_READ = 3,
    CFG_APPLY_ERR_I2C_VERIFY = 4,
    CFG_APPLY_ERR_FLASH = 5
};

static ParamStoreBlob s_cfg_store;

static inline int32_t float_to_q16_sat(float v)
{
    double scaled = (double)v * 65536.0;
    if (scaled > (double)INT32_MAX) return INT32_MAX;
    if (scaled < (double)INT32_MIN) return INT32_MIN;
    return (int32_t)lround(scaled);
}

static uint32_t crc32_sw(const uint8_t* data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static uint32_t cfg_crc_payload(const ParamStoreBlob* cfg)
{
    const uint8_t* start = (const uint8_t*)&cfg->mode;
    const size_t size = sizeof(ParamStoreBlob) - offsetof(ParamStoreBlob, mode);
    return crc32_sw(start, size);
}

static uint32_t cfg_flash_address(void)
{
    uint32_t flash_kb = (uint32_t)(*((uint16_t*)FLASHSIZE_BASE));
    uint32_t flash_bytes = flash_kb * 1024u;
    return FLASH_BASE + flash_bytes - FLASH_PAGE_SIZE;
}

static bool cfg_mode_valid(uint8_t mode)
{
    if (mode == 0x00u || mode == 0x01u || mode == 0x02u || mode == 0x03u || mode == 0x06u || mode == 0x13u) {
        return true;
    }
    if (mode >= 0x07u && mode <= 0x12u) {
        return true;
    }
    return false;
}

static bool cfg_mode_uses_vsync(uint8_t mode)
{
    return (mode == 0x01u || mode == 0x02u);
}

static void cfg_set_bank_defaults(ParamBankQ16* bank, uint8_t which)
{
    if (which == CFG_BANK_IR) {
        static const uint16_t kIrYawCd[CFG_CAM_COUNT]   = {  9000u, 15000u, 21000u, 27000u, 33000u, 39000u };
        static const int16_t  kIrPitchCd[CFG_CAM_COUNT] = {     0,     0,     0,     0,     0,     0 };
        static const int16_t  kIrRollCd[CFG_CAM_COUNT]  = {     0,     0,     0,     0,     0,     0 };
        memcpy(bank->yaw_cd, kIrYawCd, sizeof(kIrYawCd));
        memcpy(bank->pitch_cd, kIrPitchCd, sizeof(kIrPitchCd));
        memcpy(bank->roll_cd, kIrRollCd, sizeof(kIrRollCd));
        bank->overlap_px = 30u;
        bank->feather_strength_q16 = float_to_q16_sat(1.0f);
        return;
    }

    static const uint16_t kEoYawCd[CFG_CAM_COUNT]   = {     0u,  6000u, 12000u, 18000u, 24000u, 30000u };
    static const int16_t  kEoPitchCd[CFG_CAM_COUNT] = {  -500,   500,  -500,   500,  -500,   500 };
    static const int16_t  kEoRollCd[CFG_CAM_COUNT]  = {     0,     0,     0,     0,     0,     0 };
    memcpy(bank->yaw_cd, kEoYawCd, sizeof(kEoYawCd));
    memcpy(bank->pitch_cd, kEoPitchCd, sizeof(kEoPitchCd));
    memcpy(bank->roll_cd, kEoRollCd, sizeof(kEoRollCd));
    bank->overlap_px = 100u;
    bank->feather_strength_q16 = float_to_q16_sat(1.0f);
}

static void cfg_set_defaults(ParamStoreBlob* cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = PARAM_STORE_MAGIC;
    cfg->version = PARAM_STORE_VERSION;
    cfg->length = (uint16_t)sizeof(*cfg);
    cfg->mode = 0x01u;
    cfg_set_bank_defaults(&cfg->eo, CFG_BANK_EO);
    cfg_set_bank_defaults(&cfg->ir, CFG_BANK_IR);
    cfg->write_count = 0;
    cfg->crc32 = cfg_crc_payload(cfg);
}

static bool cfg_bank_values_valid(const ParamBankQ16* bank)
{
    if (!bank) return false;

    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i) {
        if (bank->pitch_cd[i] < -500 || bank->pitch_cd[i] > 500) return false;
        if (bank->roll_cd[i] < -500 || bank->roll_cd[i] > 500) return false;
    }

    if (bank->overlap_px > 1920u) return false;

    const int32_t min_feather_q16 = float_to_q16_sat(0.1f);
    const int32_t max_feather_q16 = float_to_q16_sat(10.0f);
    if (bank->feather_strength_q16 < min_feather_q16 || bank->feather_strength_q16 > max_feather_q16) return false;

    return true;
}

static bool cfg_is_valid(const ParamStoreBlob* cfg)
{
    if (cfg->magic != PARAM_STORE_MAGIC) return false;
    if (cfg->version != PARAM_STORE_VERSION) return false;
    if (cfg->length != (uint16_t)sizeof(*cfg)) return false;
    if (!cfg_mode_valid(cfg->mode)) return false;
    if (!cfg_bank_values_valid(&cfg->eo)) return false;
    if (!cfg_bank_values_valid(&cfg->ir)) return false;
    return (cfg->crc32 == cfg_crc_payload(cfg));
}

static bool cfg_load_from_flash(void)
{
    const uint32_t base = cfg_flash_address();
    ParamStoreBlob flash_cfg;
    memcpy(&flash_cfg, (const void*)base, sizeof(flash_cfg));
    if (cfg_is_valid(&flash_cfg)) {
        s_cfg_store = flash_cfg;
        return true;
    }
    cfg_set_defaults(&s_cfg_store);
    return false;
}

static bool cfg_save_to_flash(void)
{
    s_cfg_store.magic = PARAM_STORE_MAGIC;
    s_cfg_store.version = PARAM_STORE_VERSION;
    s_cfg_store.length = (uint16_t)sizeof(s_cfg_store);
    s_cfg_store.crc32 = cfg_crc_payload(&s_cfg_store);

    const uint32_t base = cfg_flash_address();
    const uint32_t page = (base - FLASH_BASE) / FLASH_PAGE_SIZE;

    if (HAL_FLASH_Unlock() != HAL_OK) return false;

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = page;
    erase.NbPages = PARAM_FLASH_PAGE_COUNT;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        (void)HAL_FLASH_Lock();
        return false;
    }

    const uint8_t* src = (const uint8_t*)&s_cfg_store;
    for (uint32_t off = 0; off < (uint32_t)sizeof(s_cfg_store); off += 8u) {
        uint64_t word = 0xFFFFFFFFFFFFFFFFull;
        memcpy(&word, src + off, 8u);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, base + off, word) != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return false;
        }
    }

    (void)HAL_FLASH_Lock();

    ParamStoreBlob verify;
    memcpy(&verify, (const void*)base, sizeof(verify));
    return cfg_is_valid(&verify) && (memcmp(&verify, &s_cfg_store, sizeof(verify)) == 0);
}

static ParamBankQ16* cfg_bank_mut(uint8_t bank)
{
    return (bank == CFG_BANK_IR) ? &s_cfg_store.ir : &s_cfg_store.eo;
}

static const ParamBankQ16* cfg_bank_const(uint8_t bank)
{
    return (bank == CFG_BANK_IR) ? &s_cfg_store.ir : &s_cfg_store.eo;
}

static uint16_t read_le_u16_local(const uint8_t* p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t read_le_i16_local(const uint8_t* p)
{
    return (int16_t)read_le_u16_local(p);
}

static void put_le_u16_local(uint8_t* dst, uint16_t v)
{
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_le_i16_local(uint8_t* dst, int16_t v)
{
    put_le_u16_local(dst, (uint16_t)v);
}

static bool cfg_bank_from_payload(const uint8_t* p, uint16_t n, ParamBankQ16* out)
{
    if (!p || !out) return false;
    if (n < CFG_BANK_PAYLOAD_SIZE) return false;

    uint16_t off = 0u;
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        out->yaw_cd[i] = read_le_u16_local(&p[off]);
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        out->pitch_cd[i] = read_le_i16_local(&p[off]);
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        out->roll_cd[i] = read_le_i16_local(&p[off]);
    }
    out->overlap_px = read_le_u16_local(&p[off]);
    off += 2u;
    out->feather_strength_q16 = (int32_t)(
        ((uint32_t)p[off + 0u]) |
        ((uint32_t)p[off + 1u] << 8) |
        ((uint32_t)p[off + 2u] << 16) |
        ((uint32_t)p[off + 3u] << 24));
    return true;
}

static void cfg_bank_copy_aligned(ParamBankQ16* dst, const ParamBankQ16* src)
{
    memcpy(dst, src, sizeof(*dst));
}

static bool fpga_write_verify_reg8(uint8_t reg, uint8_t value, uint8_t* detail_out)
{
    uint8_t rd = 0;
    HAL_StatusTypeDef st = fpga_write_reg8(&g_fpga, reg, value, 200);
    if (st != HAL_OK) {
        if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_WRITE;
        return false;
    }
    st = fpga_read_reg8(&g_fpga, reg, &rd, 200);
    if (st != HAL_OK) {
        if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_READ;
        return false;
    }
    if (rd != value) {
        if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_VERIFY;
        return false;
    }
    if (detail_out) *detail_out = CFG_APPLY_OK;
    return true;
}

static bool fpga_write_verify_i32(uint8_t reg, int32_t value, uint8_t* detail_out)
{
    uint8_t b[4];
    b[0] = (uint8_t)(value & 0xFF);
    b[1] = (uint8_t)((value >> 8) & 0xFF);
    b[2] = (uint8_t)((value >> 16) & 0xFF);
    b[3] = (uint8_t)((value >> 24) & 0xFF);

    for (uint8_t i = 0; i < 4u; ++i) {
        if (!fpga_write_verify_reg8((uint8_t)(reg + i), b[i], detail_out)) {
            return false;
        }
    }
    return true;
}

static bool fpga_write_verify_u16(uint8_t reg, uint16_t value, uint8_t* detail_out)
{
    uint8_t b[2];
    uint8_t rd[2] = {0};

    b[0] = (uint8_t)(value & 0xFFu);
    b[1] = (uint8_t)((value >> 8) & 0xFFu);

    for (uint8_t i = 0; i < 2u; ++i) {
        HAL_StatusTypeDef st = fpga_write_reg8(&g_fpga, (uint8_t)(reg + i), b[i], 200);
        if (st != HAL_OK) {
            if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_WRITE;
            return false;
        }
    }

    for (uint8_t i = 0; i < 2u; ++i) {
        HAL_StatusTypeDef st = fpga_read_reg8(&g_fpga, (uint8_t)(reg + i), &rd[i], 200);
        if (st != HAL_OK) {
            if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_READ;
            return false;
        }
    }

    if (memcmp(b, rd, sizeof(b)) != 0) {
        if (detail_out) *detail_out = CFG_APPLY_ERR_I2C_VERIFY;
        return false;
    }
    return true;
}

static uint8_t fpga_push_bank(uint8_t bank, const ParamBankQ16* cfg)
{
    const uint8_t base = fpga_cfg_bank_base(bank);
    uint8_t detail = CFG_APPLY_OK;
    ParamBankQ16 local_cfg;

    cfg_bank_copy_aligned(&local_cfg, cfg);

    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i) {
        if (!fpga_write_verify_u16((uint8_t)(base + FPGA_REG_CFG_YAW_CD(i)), local_cfg.yaw_cd[i], &detail)) return detail;
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i) {
        if (!fpga_write_verify_u16((uint8_t)(base + FPGA_REG_CFG_PITCH_CD(i)), (uint16_t)local_cfg.pitch_cd[i], &detail)) return detail;
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i) {
        if (!fpga_write_verify_u16((uint8_t)(base + FPGA_REG_CFG_ROLL_CD(i)), (uint16_t)local_cfg.roll_cd[i], &detail)) return detail;
    }
    if (!fpga_write_verify_u16((uint8_t)(base + FPGA_REG_CFG_OVERLAP_U16), local_cfg.overlap_px, &detail)) return detail;
    if (!fpga_write_verify_i32((uint8_t)(base + FPGA_REG_CFG_FEATHER_Q16), local_cfg.feather_strength_q16, &detail)) return detail;

    return CFG_APPLY_OK;
}

static uint8_t fpga_push_mode(uint8_t mode)
{
    uint8_t detail = CFG_APPLY_OK;
    if (!fpga_write_verify_reg8(FPGA_REG_MODE_CURRENT, mode, &detail)) return detail;
    return CFG_APPLY_OK;
}

static uint8_t fpga_push_all_runtime_config(void)
{
    uint8_t detail = fpga_push_mode(s_cfg_store.mode);
    if (detail != CFG_APPLY_OK) return detail;
    detail = fpga_push_bank(CFG_BANK_EO, &s_cfg_store.eo);
    if (detail != CFG_APPLY_OK) return detail;
    detail = fpga_push_bank(CFG_BANK_IR, &s_cfg_store.ir);
    return detail;
}



// ===================== UI TX =====================
static void ui_send_frame(uint8_t msg_type, uint8_t seq, const uint8_t *payload, uint16_t plen)
{
    uint8_t buf[UI_TX_MAX];
    uint8_t tmp[UI_PAY_MAX];

    // Inner payload = [type][seq][lenL][lenH][payload...]
    if ((uint32_t)plen + 4u > sizeof(tmp)) return;

    tmp[0] = msg_type;
    tmp[1] = seq;
    tmp[2] = (uint8_t)(plen & 0xFF);
    tmp[3] = (uint8_t)((plen >> 8) & 0xFF);
    if (plen) memcpy(&tmp[4], payload, plen);

    size_t frame_len = uart_frame_build(tmp, (size_t)(plen + 4u), buf, sizeof(buf));
    if (frame_len) {
        (void)HAL_UART_Transmit(&huart3, buf, (uint16_t)frame_len, 50);
    }
}

static void ui_send_resp_simple(uint8_t seq, uint8_t err)
{
    uint8_t p[1] = { err };
    ui_send_frame(MSG_RESP, seq, p, sizeof(p));
}

static void ui_send_camera_apply_result(uint8_t seq,
                                        uint8_t node_type,
                                        uint8_t camera_id,
                                        uint16_t param_id,
                                        uint16_t value,
                                        uint8_t ok,
                                        uint8_t err)
{
    uint8_t resp[8];
    resp[0] = err;
    resp[1] = node_type;
    resp[2] = camera_id;
    resp[3] = ok;
    resp[4] = (uint8_t)(param_id & 0xFF);
    resp[5] = (uint8_t)((param_id >> 8) & 0xFF);
    resp[6] = (uint8_t)(value & 0xFF);
    resp[7] = (uint8_t)((value >> 8) & 0xFF);
    ui_send_frame(MSG_RESP, seq, resp, sizeof(resp));
}

static void ui_send_log(McuContext *ctx, const char *s)
{
    if (!ctx->debug_enabled) return;
    uint16_t n = (uint16_t)strlen(s);
    if (n > 200) n = 200;
    ui_send_frame(MSG_LOG, 0, (const uint8_t*)s, n);
}

/* Public version ??callable from main.c and any other translation unit.
 * Accepts a printf-style format so callers don't need a local snprintf.
 * Usage:  mcu_ui_log(&g_ctx, "Verify Genlock: NV17=%u NV18=%u\r\n", nv17, nv18); */
void mcu_ui_log(McuContext *ctx, const char *fmt, ...)
{
    if (!ctx || !ctx->debug_enabled) return;
    char buf[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uint16_t n = (uint16_t)strlen(buf);
    if (n > 200) n = 200;
    ui_send_frame(MSG_LOG, 0, (const uint8_t*)buf, n);
}

static void ui_send_init_ack(uint8_t seq, uint8_t node, uint8_t id, uint8_t ok, uint8_t err)
{
    uint8_t resp[4] = { err, node, id, ok };
    ui_send_frame(MSG_RESP, seq, resp, sizeof(resp));
}

// ---- IMU orientation event (roll + pitch in centidegrees, int16 LE) ----
// Frame payload: [EVT_IMU_ORIENT][roll_lo][roll_hi][pitch_lo][pitch_hi]
static void ui_send_imu_orient(int16_t roll_cd, int16_t pitch_cd)
{
    uint8_t p[5];
    p[0] = EVT_IMU_ORIENT;
    p[1] = (uint8_t)( roll_cd        & 0xFF);
    p[2] = (uint8_t)((roll_cd  >> 8) & 0xFF);
    p[3] = (uint8_t)( pitch_cd       & 0xFF);
    p[4] = (uint8_t)((pitch_cd >> 8) & 0xFF);
    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void ui_send_imu_odr(uint16_t rate_hz)
{
    uint8_t p[3];
    p[0] = EVT_IMU_ODR;
    p[1] = (uint8_t)(rate_hz & 0xFF);
    p[2] = (uint8_t)((rate_hz >> 8) & 0xFF);
    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void ui_send_imu_stats(uint16_t drdy_hz, uint16_t pkt_ts_hz, uint16_t parse_ok_hz)
{
    uint8_t p[7];
    p[0] = EVT_IMU_STATS;
    p[1] = (uint8_t)(drdy_hz & 0xFF);
    p[2] = (uint8_t)((drdy_hz >> 8) & 0xFF);
    p[3] = (uint8_t)(pkt_ts_hz & 0xFF);
    p[4] = (uint8_t)((pkt_ts_hz >> 8) & 0xFF);
    p[5] = (uint8_t)(parse_ok_hz & 0xFF);
    p[6] = (uint8_t)((parse_ok_hz >> 8) & 0xFF);
    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void put_le_i32(uint8_t* dst, int32_t v)
{
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
    dst[2] = (uint8_t)((v >> 16) & 0xFF);
    dst[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void ui_send_cfg_bank(uint8_t bank)
{
    ParamBankQ16 cfg_local;
    cfg_bank_copy_aligned(&cfg_local, cfg_bank_const(bank));
    uint8_t p[3 + CFG_BANK_PAYLOAD_SIZE];
    uint16_t off = 3u;
    p[0] = EVT_CFG_BANK;
    p[1] = bank;
    p[2] = s_cfg_store.mode;

    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        put_le_u16_local(&p[off], cfg_local.yaw_cd[i]);
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        put_le_i16_local(&p[off], cfg_local.pitch_cd[i]);
    }
    for (uint8_t i = 0; i < CFG_CAM_COUNT; ++i, off += 2u) {
        put_le_i16_local(&p[off], cfg_local.roll_cd[i]);
    }
    put_le_u16_local(&p[off], cfg_local.overlap_px);
    off += 2u;
    put_le_i32(&p[off], cfg_local.feather_strength_q16);

    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void ui_send_cfg_apply_result(uint8_t bank, uint8_t ok, uint8_t detail)
{
    uint8_t p[4];
    p[0] = EVT_CFG_APPLY;
    p[1] = bank;
    p[2] = ok;
    p[3] = detail;
    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void ui_send_log_ctrl(uint8_t cmd)
{
    uint8_t p[2];
    p[0] = EVT_LOG_CTRL;
    p[1] = cmd;
    ui_send_frame(MSG_EVT, 0, p, sizeof(p));
}

static void mcu_service_log_capture(McuContext *ctx)
{
    if (ctx == NULL || !ctx->log_capture_active || ctx->log_capture_save_sent) {
        return;
    }

    if ((HAL_GetTick() - ctx->log_capture_start_ms) >= LOG_CAPTURE_SAVE_DELAY_MS) {
        ui_send_log_ctrl(LOG_CTRL_SAVE);
        ctx->log_capture_save_sent = true;
    }
}

static float orient_output_alpha(void)
{
    float rate_hz = (s_rate_hz > 0) ? (float)s_rate_hz : 1000.0f;
    float dt_ms = 1000.0f / rate_hz;
    float alpha = dt_ms / (ORIENT_SMOOTH_TAU_MS + dt_ms);

    if (alpha < 0.02f) alpha = 0.02f;
    if (alpha > 1.0f) alpha = 1.0f;
    return alpha;
}


// ===================== FPGA helpers =====================
//static int fpga_write16(uint16_t reg, const uint8_t *data, uint8_t len)
//{
//    return fpga_write(reg, data, len);
//}

static int fpga_rw_smoke_test(void)
{
  uint8_t rd = 0;
  const uint8_t wr = 0xA5;
  HAL_StatusTypeDef st;

  st = fpga_is_ready(&g_fpga, 2u, 50u);
  if (st != HAL_OK) {
    uint32_t e = HAL_I2C_GetError(g_fpga.hi2c);
    Send_UART3_Debug("FPGA ready fail HAL=%d I2Cerr=0x%08lX addr7=0x%02X\r\n",
                     (int)st, (unsigned long)e, (unsigned)g_fpga.addr7);
    return -10;
  }

  st = fpga_write_reg8(&g_fpga, FPGA_REG_MODE_CURRENT, wr, 200);
  if (st != HAL_OK) {
    uint32_t e = HAL_I2C_GetError(g_fpga.hi2c);
    Send_UART3_Debug("FPGA write fail HAL=%d I2Cerr=0x%08lX reg=0x%02X wr=0x%02X\r\n",
                     (int)st, (unsigned long)e, FPGA_REG_MODE_CURRENT, wr);
    return -11;
  }

  st = fpga_read_reg8(&g_fpga, FPGA_REG_MODE_CURRENT, &rd, 200);
  if (st != HAL_OK) {
    uint32_t e = HAL_I2C_GetError(g_fpga.hi2c);
    Send_UART3_Debug("FPGA read fail HAL=%d I2Cerr=0x%08lX reg=0x%02X\r\n",
                     (int)st, (unsigned long)e, FPGA_REG_MODE_CURRENT);
    return -12;
  }

  Send_UART3_Debug("FPGA smoke OK wr=0x%02X rd=0x%02X\r\n", wr, rd);
  return (rd == wr) ? 0 : -2;
}


// ===================== FSM =====================
void mcu_fsm_init(McuContext *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = ST_WAIT_UI_INIT;   // <-- NEW: wait for UI init command
    ctx->debug_enabled = true;
    g_debug_enabled = true;
    ctx->log_capture_active = false;
    ctx->log_capture_save_sent = false;
    ctx->log_capture_start_ms = 0u;

    const bool loaded = cfg_load_from_flash();
    if (!loaded) {
        (void)cfg_save_to_flash();
    }

    s_vsync_pending = false;
    s_orient_tx_seq = 0;
}

void mcu_fsm_start_init(McuContext *ctx)
{
    if (ctx == NULL) return;

    // ctx->init_seq is already set by the caller (CMD_INIT_SYSTEM handler)
    // to the UI's sequence number ??do NOT reset it here.
    ctx->day_idx = 0u;
    ctx->ir_idx = 0u;
    ctx->imu_streaming = false;
    ctx->log_capture_active = false;
    ctx->log_capture_save_sent = false;
    ctx->log_capture_start_ms = 0u;
    ctx->state = ST_BOOT;
    // No extra ACK here ??the caller (CMD_INIT_SYSTEM handler) already sent it
}


void mcu_fsm_step(McuContext *ctx)
{
    mcu_service_log_capture(ctx);

	switch (ctx->state) {




    case ST_WAIT_UI_INIT: {
        // Do nothing until UI requests init

    	ctx->state = ST_BOOT;
        break;
    }


    case ST_BOOT: {
        ui_send_log_ctrl(LOG_CTRL_CLEAR);
        ctx->log_capture_active = true;
        ctx->log_capture_save_sent = false;
        ctx->log_capture_start_ms = HAL_GetTick();
        ctx->state = ST_INIT_POWER;
        break;
    }

    case ST_INIT_POWER: {
        Enable_All_Camera_Power();
        HAL_Delay(1000);
        ctx->state = ST_INIT_IMU;
        break;
    }

    case ST_INIT_IMU: {
        // ----------------------------------------------------------------
        // INIT SEQUENCE (batch 14):
        //   Reset ??WHO_AM_I ??Reset ??imu_configure_odr_lpf ??START_STREAM
        //
        // SPI fixes applied (batch 14):
        //   - spi2_flush() now re-asserts FRXTH=1 on every call
        //   - IMU_NSS_HIGH_US raised from 20 µs to 500 µs so the sensor
        //     has time to latch the command before DRDY is polled
        //   - Phase 2 DRDY also checks pin level, not just interrupt flag,
        //     to catch the race where the edge fires before the poll loop
        //
        // With these fixes multiple consecutive commands work, so ODR
        // and SELECT_OUT_DATA are written explicitly every boot ??we no
        // longer rely on whatever was saved to flash.
        // ----------------------------------------------------------------

        /* ---- First reset: clean boot ---- */
        IMU_Reset();
        HAL_Delay(10);
        /* ---- WHO_AM_I ??confirm sensor is alive ---- */
        uint8_t wai_rx[SIZE_RESP_READ_REGS_BASE + 2u] = {0};
        int r = send_imu_command_ex(who_am_i_packet, sizeof(who_am_i_packet),
                                    (uint32_t)(SIZE_RESP_READ_REGS_BASE + 1u), "WHO_AM_I",
                                    wai_rx);

        if (r == 0) {
            uint8_t wai_val = 0;
            for (int i = 0; i < (int)sizeof(wai_rx) - 1; i++) {
                if (wai_rx[i] == 0xEAu || wai_rx[i] == 0xE6u) {
                    wai_val = wai_rx[i];
                    break;
                }
            }
            char msg[80];
            snprintf(msg, sizeof(msg), "[IMU] WHO_AM_I=0x%02X (%s)\r\n", wai_val,
                     (wai_val == 0xEAu) ? "IIM46234 OK" :
                     (wai_val == 0xE6u) ? "IIM46230 OK" : "UNKNOWN");
            ui_send_log(ctx, msg);
        }

        uint8_t ok = (r == 0) ? 1 : 0;
        ui_send_init_ack(ctx->init_seq, NODE_IMU, 0, ok, ok ? ERR_OK : ERR_FAIL);
        if (r != 0) {
            ui_send_log(ctx, "[IMU] WHO_AM_I failed -- IMU offline\r\n");
            ctx->imu_streaming = false;
            ctx->day_idx = 0;
            ctx->state = ST_INIT_DAY;
            break;
        }

        float imu_odr_hz = 1000.0f;
        {
            int rc = imu_ensure_default_profile();
            if (rc == 0) {
                ui_send_log(ctx, "[IMU] Flash-backed default profile verified\r\n");
            } else {
                ui_send_log(ctx, "[IMU] WARN: Default profile verify/restore failed\r\n");
            }
        }

        {
            float t = 0.0f, h = 0.0f;
            for (int i = 0; i < 3; i++) {
                Read_HDC3022_Sensor(&t, &h);
                HAL_Delay(10);
            }

            uint8_t ok_temp = (t > -900.0f && h > -900.0f) ? 1 : 0;
            ui_send_init_ack(ctx->init_seq, NODE_TEMP, 0, ok_temp,
                             ok_temp ? ERR_OK : ERR_FAIL);

            {
                int16_t  t_cd = (int16_t)lroundf(t * 100.0f);
                uint16_t h_cp = (uint16_t)lroundf(h * 100.0f);
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "Temperature %.2f C, Humidity %.2f %%\r\n",
                         t_cd / 100.0f,
                         h_cp / 100.0f);
                Send_UART3_Debug(msg);
            }
        }

        /* This board still behaves reliably only when START_STREAM is the
         * first command after reset. Keep the final reset immediately before
         * sending START_STREAM. */
        IMU_Reset();
        HAL_Delay(10);

        IMU_Handler_Init(imu_odr_hz, 0.12f);
        s_stream_valid_log_sent = 0u;
        s_stream_odr_log_sent = 0u;
        s_stream_diag_count = 0u;
        s_prev_sample_counter = 0u;
        s_prev_timestamp_us = 0u;
        s_odr_accum_us = 0u;
        s_odr_samples = 0u;
        s_ui_throttle = 0u;
        s_roll_deg_filt = 0.0f;
        s_pitch_deg_filt = 0.0f;
        s_orient_filter_init = false;
        s_stats_window_start_ms = HAL_GetTick();
        s_stats_edges_base = g_imu_drdy_edges_total;
        s_stats_parse_ok = 0u;
        s_stats_ts_delta_us = 0u;
        s_stats_ts_delta_count = 0u;
        HAL_Delay(50u);

        int rs = send_imu_command(start_streaming_packet,
                                  sizeof(start_streaming_packet),
                                  0, "START_STREAM");

        if (rs == 0) {
            ctx->imu_streaming = true;
            ui_send_log(ctx, "[IMU] START_STREAM sent; waiting for first DRDY packet\r\n");
        } else {
            ctx->imu_streaming = false;
            uint8_t err_payload[2] = { NODE_IMU, ERR_FAIL };
            ui_send_frame(MSG_EVT, 0, err_payload, sizeof(err_payload));
            ui_send_log(ctx, "[IMU] START_STREAM failed\r\n");
        }

        ctx->state = ST_INIT_DAY;
        ui_send_log(ctx, "Entered OPERATION\r\n");
        break;
    }


    case ST_INIT_DAY: {
        if (ctx->day_idx < 6) {
            HAL_Delay(20);
            int r = test_day_camera_controls(ctx->day_idx);   // change it to return status if it doesn?�t
            uint8_t ok = (r == 0) ? 1 : 0;
            ui_send_init_ack(ctx->init_seq,NODE_DAY, ctx->day_idx, ok, ok ? ERR_OK : ERR_FAIL);
            ctx->day_idx++;
        } else {
            ctx->ir_idx = 0;
            ctx->state = ST_INIT_IR;
        }
        break;
    }


    case ST_INIT_IR: {
    	HAL_Delay(1000);
    	if (ctx->ir_idx < 6) {
        	int r = Initialize_IR_Camera(ctx->ir_idx);   // change to return status if it doesn?�t
            uint8_t ok = (r == 0) ? 1 : 0;
            ui_send_init_ack(ctx->init_seq,NODE_IR, ctx->ir_idx, ok, ok ? ERR_OK : ERR_FAIL);

            ctx->ir_idx++;

        } else {
//            ctx->state = ST_INIT_FPGA_CAL;
        	ctx->state = ST_INIT_TEMP;
        }
        break;
    }


//    case ST_INIT_FPGA_CAL: {
//        ui_send_log(ctx, "FPGA init build: full-regmap diag v9\r\n");
//        int st = fpga_rw_smoke_test();
//
//        if (st == 0) {
//            ui_send_log(ctx, "FPGA I2C ping OK (0xA5)\r\n");
//            Send_UART3_Debug("FPGA I2C ping OK (0xA5)\r\n\n\n");
//            uint8_t detail = CFG_APPLY_OK;
//            uint32_t t0 = HAL_GetTick();
//            uint8_t probe_detail = CFG_APPLY_OK;
//
//            {
//                static const uint8_t probe_regs[4]  = { 0x20u, 0x21u, 0x22u, 0x23u };
//                static const uint8_t probe_vals[4]  = { 0x5Au, 0xA5u, 0x3Cu, 0xC3u };
//                char msg[96];
//                for (uint8_t i = 0; i < 4u; ++i) {
//                    snprintf(msg, sizeof(msg), "FPGA probe reg 0x%02X\r\n", (unsigned)probe_regs[i]);
//                    ui_send_log(ctx, msg);
//                    if (!fpga_write_verify_reg8(probe_regs[i], probe_vals[i], &probe_detail)) {
//                        snprintf(msg, sizeof(msg), "FPGA probe reg 0x%02X FAIL detail=%u\r\n",
//                                 (unsigned)probe_regs[i], (unsigned)probe_detail);
//                        ui_send_log(ctx, msg);
//                        detail = probe_detail;
//                        break;
//                    }
//                }
//            }
//
//            if (detail == CFG_APPLY_OK) {
//                #if FPGA_DEBUG_TOP_MODE_ONLY
//                ui_send_log(ctx, "FPGA bank push skipped (debug top)\r\n");
//                ui_send_log(ctx, "FPGA push stage: mode\r\n");
//                detail = fpga_push_mode(s_cfg_store.mode);
//                #else
//                ui_send_log(ctx, "FPGA push stage: EO bank\r\n");
//                detail = fpga_push_bank_chunked(ctx, CFG_BANK_EO, &s_cfg_store.eo);
//
//                if (detail == CFG_APPLY_OK) {
//                    ui_send_log(ctx, "FPGA push stage: IR bank\r\n");
//                    detail = fpga_push_bank_chunked(ctx, CFG_BANK_IR, &s_cfg_store.ir);
//                }
//
//                if (detail == CFG_APPLY_OK) {
//                    ui_send_log(ctx, "FPGA push stage: mode\r\n");
//                    detail = fpga_push_mode(s_cfg_store.mode);
//                }
//                #endif
//            }
//
//            if (detail == CFG_APPLY_OK) {
//                char msg[96];
//                snprintf(msg, sizeof(msg), "FPGA runtime config push OK (%lums)\r\n",
//                         (unsigned long)(HAL_GetTick() - t0));
//                ui_send_log(ctx, msg);
//                ui_send_init_ack(ctx->init_seq,NODE_FPGA, 0, 1, ERR_OK);
//            } else {
//                char msg[96];
//                snprintf(msg, sizeof(msg), "FPGA runtime config push FAIL detail=%u (%lums)\r\n",
//                         (unsigned)detail,
//                         (unsigned long)(HAL_GetTick() - t0));
//                ui_send_log(ctx, msg);
//                ui_send_init_ack(ctx->init_seq,NODE_FPGA, 0, 0, ERR_FAIL);
//            }
//
//        } else {
//            ui_send_log(ctx, "FPGA I2C ping FAIL\r\n");
//            Send_UART3_Debug("FPGA I2C ping FAIL\r\n");
//
//             ui_send_init_ack(ctx->init_seq,NODE_FPGA, 0, 0, ERR_FAIL);
//        }
//
//        // Do NOT say Entered OPERATION here; we are not in OPERATION yet.
//        ctx->state = ST_INIT_TEMP;
//        break;
//    }

    case ST_INIT_TEMP: {
        float t = 0.0f, h = 0.0f;

        // Speedy init: do a few quick reads (or just 1). This avoids ~1s boot delay.
        // Keep 3 reads to stabilize without blocking too long.
        for (int i = 0; i < 3; i++) {
            Read_HDC3022_Sensor(&t, &h);
            HAL_Delay(10);
        }

        // basic validity check (matches your error behavior)
        uint8_t ok = (t > -900.0f && h > -900.0f) ? 1 : 0;

        // If you already have node ACK helper:
         ui_send_init_ack(ctx->init_seq,NODE_TEMP, 0, ok, ok ? ERR_OK : ERR_FAIL);

        // Optional debug print
        int16_t  t_cd = (int16_t)lroundf(t * 100.0f);
        uint16_t h_cp = (uint16_t)lroundf(h * 100.0f);

        char msg[80];
        snprintf(msg, sizeof(msg),
                 "Temperature %.2f C, Humidity %.2f %%\r\n",
                 t_cd / 100.0f,
                 h_cp / 100.0f);
        Send_UART3_Debug(msg);

        ctx->state = ST_OPERATION;
        ui_send_log(ctx, "Entered OPERATION\r\n");
        break;
    }

    case ST_OPERATION: {
        fpga_service_orient();
        break;
    }

    case ST_ERROR:
    default: {
        break;
    }
    }
}

// ===================== IMU DRDY handler =====================
// Called from main loop for each pending DRDY edge counted from EXTI PB9.

static void quat_to_roll_pitch_deg(const Quaternion_t *q, float *roll_deg, float *pitch_deg)
{
    const float RAD2DEG = 180.0f / 3.14159265358979f;
    const float q0 = q->q0;
    const float q1 = q->q1;
    const float q2 = q->q2;
    const float q3 = q->q3;

    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    float sinp = 2.0f * (q0 * q2 - q3 * q1);

    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;
    *pitch_deg = asinf(sinp) * RAD2DEG;
}

static void update_shadow_roll_pitch(const Quaternion_t *q)
{
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float alpha = 1.0f;

    quat_to_roll_pitch_deg(q, &roll_deg, &pitch_deg);

    if (roll_deg  >  ORIENT_CLAMP_DEG) roll_deg  =  ORIENT_CLAMP_DEG;
    if (roll_deg  < -ORIENT_CLAMP_DEG) roll_deg  = -ORIENT_CLAMP_DEG;
    if (pitch_deg >  ORIENT_CLAMP_DEG) pitch_deg =  ORIENT_CLAMP_DEG;
    if (pitch_deg < -ORIENT_CLAMP_DEG) pitch_deg = -ORIENT_CLAMP_DEG;

    if (!s_orient_filter_init) {
        s_roll_deg_filt = roll_deg;
        s_pitch_deg_filt = pitch_deg;
        s_orient_filter_init = true;
    } else {
        alpha = orient_output_alpha();
        s_roll_deg_filt += alpha * (roll_deg - s_roll_deg_filt);
        s_pitch_deg_filt += alpha * (pitch_deg - s_pitch_deg_filt);
    }

    s_roll_cd = quantize_cd((int16_t)lroundf(s_roll_deg_filt * 100.0f));
    s_pitch_cd = quantize_cd((int16_t)lroundf(s_pitch_deg_filt * 100.0f));
}

/* Confirmed from log (2026-02-26, batch 11):
 * The sensor outputs 1 dummy byte (0x00) before the 46-byte stream packet.
 * Total SPI read = 47 bytes.  Packet starts at byte[1] (0x23 0x23 header).
 *   byte[0]       = 0x00  (dummy / SPI pipeline flush byte ??ignore)
 *   byte[1..2]    = 0x23 0x23  (stream packet header)
 *   byte[3]       = packet size byte (0x2E = 46)
 *   byte[4..46]   = remainder of 46-byte stream packet
 *
 * SELECT_OUT_DATA = 0x07 (accel+gyro+temp, 7 outputs):
 *   SIZE_PACKET_BASE_DATA(18) + 4×7 = 46 bytes payload
 */
#define IMU_STREAM_DUMMY_BYTES  1u               /* one byte of alignment slack */
#define IMU_STREAM_PKT_BYTES    46u              /* actual packet bytes */
#define IMU_STREAM_READ_TOTAL   (IMU_STREAM_DUMMY_BYTES + IMU_STREAM_PKT_BYTES)  /* 47 */

static uint8_t s_imu_rx_buf[IMU_STREAM_READ_TOTAL];

static int imu_stream_read_raw(uint8_t *dst)
{
    return IMU_SPI_ReadSession(dst, IMU_STREAM_READ_TOTAL, 10u);
}

static int mcu_process_imu_stream_packet(McuContext *ctx, uint8_t *delta_counter_out)
{
    if (delta_counter_out) *delta_counter_out = 0u;

    if (imu_stream_read_raw(s_imu_rx_buf) != 0) return -1;

    uint8_t *pkt = NULL;
    uint32_t pkt_avail = IMU_STREAM_PKT_BYTES;

    if (s_imu_rx_buf[0] == 0x23u && s_imu_rx_buf[1] == 0x23u) {
        pkt = &s_imu_rx_buf[0];
    } else if (s_imu_rx_buf[1] == 0x23u && s_imu_rx_buf[2] == 0x23u) {
        pkt = &s_imu_rx_buf[1];
    } else {
        return 0;
    }

    // ---- Parse packet + run Madgwick ----
    IMU_Sample_t sample;
    int parse_result = IMU_Handler_ProcessStreamPacket(pkt, pkt_avail, &sample);
    if (parse_result != IMU_ERR_OK) {
        /* Discard silently ??no Send_UART3_Debug here, blocking UART kills 1kHz */
        return 0;
    }

    s_stats_parse_ok++;

    if (!s_stream_valid_log_sent) {
        s_stream_valid_log_sent = 1u;
        ui_send_log(ctx, "[IMU] First stream packet parsed\r\n");
    }

    uint64_t timestamp_us =
        ((uint64_t)sample.timestamp_high << 32) | (uint64_t)sample.timestamp_low;
    uint8_t delta_counter = 0u;
    uint32_t delta_us = 0u;

    if (s_prev_timestamp_us != 0u) {
        delta_counter = (uint8_t)(sample.sample_counter - s_prev_sample_counter);
        delta_us = (uint32_t)(timestamp_us - s_prev_timestamp_us);
        if (delta_counter > 0u && delta_us > 0u && delta_us < 1000000u) {
            s_stats_ts_delta_us += delta_us;
            s_stats_ts_delta_count += delta_counter;
            uint32_t per_sample_us = delta_us / (uint32_t)delta_counter;
            if (per_sample_us > 0u) {
                s_odr_accum_us += per_sample_us;
                s_odr_samples++;
                if (s_odr_samples >= 32u) {
                    uint32_t avg_us = s_odr_accum_us / s_odr_samples;
                    if (avg_us > 0u) {
                        uint16_t rate_hz = (uint16_t)((1000000u + (avg_us / 2u)) / avg_us);
                        s_rate_hz = (int16_t)rate_hz;
                        IMU_Handler_SetSampleFreq((float)rate_hz);
                        ui_send_imu_odr(rate_hz);
                        if (!s_stream_odr_log_sent) {
                            char odr_msg[64];
                            s_stream_odr_log_sent = 1u;
                            snprintf(odr_msg, sizeof(odr_msg), "[IMU] Sensor ODR ~= %u Hz\r\n",
                                     (unsigned)rate_hz);
                            ui_send_log(ctx, odr_msg);
                        }
                    }
                    s_odr_accum_us = 0u;
                    s_odr_samples = 0u;
                }
            }
        }
    }
    if (delta_counter_out) *delta_counter_out = delta_counter;

    if (s_stream_diag_count < 8u) {
        char diag_msg[128];

        if (s_stream_diag_count == 0u) {
            snprintf(diag_msg, sizeof(diag_msg),
                     "[IMU] pkt0 sc=%u ts=%lu\r\n",
                     (unsigned)sample.sample_counter,
                     (unsigned long)timestamp_us);
        } else {
            uint8_t delta_counter = (uint8_t)(sample.sample_counter - s_prev_sample_counter);
            uint32_t delta_us = (uint32_t)(timestamp_us - s_prev_timestamp_us);
            snprintf(diag_msg, sizeof(diag_msg),
                     "[IMU] pkt%u sc=%u dsc=%u dts_us=%lu\r\n",
                     (unsigned)s_stream_diag_count,
                     (unsigned)sample.sample_counter,
                     (unsigned)delta_counter,
                     (unsigned long)delta_us);
        }
        ui_send_log(ctx, diag_msg);
        s_stream_diag_count++;
    }
    s_prev_sample_counter = sample.sample_counter;
    s_prev_timestamp_us = timestamp_us;

    {
        uint32_t now_ms = HAL_GetTick();
        uint32_t elapsed_ms = now_ms - s_stats_window_start_ms;
        if (elapsed_ms >= 1000u) {
            uint32_t edge_now = 0u;

            __disable_irq();
            edge_now = g_imu_drdy_edges_total;
            __enable_irq();

            s_stats_window_start_ms = now_ms;
            s_stats_edges_base = edge_now;
            s_stats_parse_ok = 0u;
            s_stats_ts_delta_us = 0u;
            s_stats_ts_delta_count = 0u;
        }
    }

    s_ui_throttle++;
    {
        const bool ui_due = (s_ui_throttle >= UI_ORIENT_DECIM);
        if (s_vsync_pending || ui_due) {
            update_shadow_roll_pitch(&sample.quaternion);
        }

        if (ui_due) {
            s_ui_throttle = 0u;
            ui_send_imu_orient((int16_t)s_roll_cd, (int16_t)s_pitch_cd);
        }
    }

    return 1;
}

void mcu_on_imu_drdy(McuContext *ctx)
{
    if (ctx->state != ST_OPERATION) return;

    /* The IMU streams into a 40-entry FIFO in streaming mode. If we process only
     * one packet per DRDY event and fall behind, the queue can stay non-empty and
     * the delivered DRDY edge rate collapses well below the internal sample rate.
     * When sample_counter jumps by N, immediately drain up to N-1 more packets to
     * catch the host back up to the FIFO. */
    uint32_t processed = 0u;
    uint32_t burst_budget = 1u;

    while (processed < IMU_STREAM_BURST_MAX && burst_budget > 0u) {
        uint8_t delta_counter = 0u;
        int packet_status = 0;

        burst_budget--;
        packet_status = mcu_process_imu_stream_packet(ctx, &delta_counter);
        if (packet_status <= 0) break;

        processed++;

        if (delta_counter > 1u && processed < IMU_STREAM_BURST_MAX) {
            uint32_t backlog = (uint32_t)(delta_counter - 1u);
            uint32_t room = IMU_STREAM_BURST_MAX - processed;
            if (backlog > room) backlog = room;
            burst_budget += backlog;
        }
    }
}

void mcu_on_vsync(McuContext *ctx)
{
    if (ctx == NULL) return;
    if (ctx->state != ST_OPERATION) return;
    if (!cfg_mode_uses_vsync(s_cfg_store.mode)) return;
    s_vsync_pending = true;
}


static void fpga_service_orient(void)
{
    if (!s_vsync_pending) return;
    s_vsync_pending = false;

    const int16_t roll  = s_roll_cd;
    const int16_t pitch = s_pitch_cd;
    uint8_t payload[4];

    payload[0] = (uint8_t)(roll & 0xFF);
    payload[1] = (uint8_t)((roll >> 8) & 0xFF);
    payload[2] = (uint8_t)(pitch & 0xFF);
    payload[3] = (uint8_t)((pitch >> 8) & 0xFF);

    (void)s_rate_hz;
    s_orient_tx_seq++;

    /* VSYNC path is latency-sensitive: push only roll/pitch in one burst. */
    (void)fpga_write_regs(&g_fpga,
                          FPGA_REG_ORIENT_ROLL_CD8,
                          payload,
                          (uint16_t)sizeof(payload),
                          20);
}




// ===================== UI command handler =====================
// p/n are the *decoded* bytes from proto_uart (CRC OK, COBS OK).
// Format: [type][seq][lenL][lenH][...payload...]
void mcu_on_ui_payload(McuContext *ctx, const uint8_t *p, uint16_t n)
{
    if (n < 4) return;

    uint8_t  msg_type = p[0];
    uint8_t  seq      = p[1];
    uint16_t plen     = (uint16_t)p[2] | ((uint16_t)p[3] << 8);

    if ((uint16_t)(plen + 4u) != n) return;

    const uint8_t *d = &p[4];

    // Allow CMD_INIT_SYSTEM and CMD_GET_CONFIG from any state.
    // For any other command, reply BUSY until OPERATION.
    if (ctx->state != ST_OPERATION) {
        if (msg_type == MSG_CMD && plen >= 1) {
            uint8_t maybe_cmd = d[0];
            if (maybe_cmd != CMD_INIT_SYSTEM && maybe_cmd != CMD_GET_CONFIG && maybe_cmd != CMD_PING) {
                ui_send_resp_simple(seq, ERR_BUSY);
                return;
            }
            // else: allow regardless of state
        } else if (msg_type == MSG_CMD) {
            ui_send_resp_simple(seq, ERR_BADARG);
            return;
        }
    }


    if (msg_type != MSG_CMD || plen < 1) {
        ui_send_resp_simple(seq, ERR_BADARG);
        return;
    }

    uint8_t cmd_id = d[0];

    switch (cmd_id) {

    case CMD_INIT_SYSTEM: {
        // Accept INIT when command byte exists; tolerate future payload extensions.
        if (plen < 1u) {
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }
        mcu_ui_log(ctx, "INIT cmd rx: seq=%u plen=%u\r\n", (unsigned)seq, (unsigned)plen);

        // Store the UI's seq so all subsequent init ACKs from mcu_fsm_step()
        // carry the correct sequence number back to the UI.
        ctx->init_seq = seq;

        // Always ACK the command itself
        ui_send_init_ack(seq, NODE_SYSTEM, 0, 1, ERR_OK);
        ui_send_log(ctx, (ctx->state == ST_WAIT_UI_INIT) ? "INIT: started\r\n" : "INIT: restart\r\n");
        Send_UART3_Debug("INIT: requested (state=%d)\r\n", (int)ctx->state);
        mcu_fsm_start_init(ctx);

        break;
    }




    case CMD_GET_CONFIG: {
        ui_send_cfg_bank(CFG_BANK_EO);
        ui_send_cfg_bank(CFG_BANK_IR);
        ui_send_resp_simple(seq, ERR_OK);
        break;
    }

    case CMD_SET_DEBUG: {
        if (plen < 2) { ui_send_resp_simple(seq, ERR_BADARG); break; }
        ctx->debug_enabled = (d[1] != 0);
        g_debug_enabled = ctx->debug_enabled;
        ui_send_resp_simple(seq, ERR_OK);
        break;
    }

    case CMD_SET_MODE: {
      if (plen < 2) { ui_send_resp_simple(seq, ERR_BADARG); break; }
      uint8_t mode_id = d[1];

      if (!cfg_mode_valid(mode_id)) { ui_send_resp_simple(seq, ERR_BADARG); break; }

      uint8_t detail = fpga_push_mode(mode_id);
      if (detail != CFG_APPLY_OK) { ui_send_resp_simple(seq, ERR_FAIL); break; }

      ParamStoreBlob prev = s_cfg_store;
      s_cfg_store.mode = mode_id;
      s_cfg_store.write_count = prev.write_count + 1u;
      if (!cfg_save_to_flash()) {
          s_cfg_store = prev;
          (void)fpga_push_mode(prev.mode);
          ui_send_resp_simple(seq, ERR_FAIL);
          break;
      }

      ui_send_resp_simple(seq, ERR_OK);
      ui_send_cfg_bank(CFG_BANK_EO);
      ui_send_cfg_bank(CFG_BANK_IR);
      break;
    }

    case CMD_CAMERA_RESET: {
        mcu_ui_log(ctx, "Camera power cycle requested\r\n");
        if (Power_Cycle_All_Cameras() != 0) {
            mcu_ui_log(ctx, "Camera power cycle failed\r\n");
            ui_send_resp_simple(seq, ERR_FAIL);
            break;
        }
        mcu_ui_log(ctx, "Camera power cycle complete\r\n");
        ui_send_resp_simple(seq, ERR_OK);
        break;
    }

    case CMD_PING: {
        ui_send_resp_simple(seq, ERR_OK);
        break;
    }

    case CMD_APPLY_CONFIG: {
        if (plen < (uint16_t)(2u + CFG_BANK_PAYLOAD_SIZE)) {
            ui_send_cfg_apply_result(0xFFu, 0u, CFG_APPLY_ERR_BADARG);
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        const uint8_t bank = d[1];
        if (bank != CFG_BANK_EO && bank != CFG_BANK_IR) {
            ui_send_cfg_apply_result(bank, 0u, CFG_APPLY_ERR_BADARG);
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        ParamBankQ16 parsed;
        if (!cfg_bank_from_payload(&d[2], (uint16_t)(plen - 2u), &parsed)) {
            ui_send_cfg_apply_result(bank, 0u, CFG_APPLY_ERR_BADARG);
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }
        if (!cfg_bank_values_valid(&parsed)) {
            ui_send_cfg_apply_result(bank, 0u, CFG_APPLY_ERR_BADARG);
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        ParamStoreBlob prev = s_cfg_store;
        *cfg_bank_mut(bank) = parsed;

        uint8_t detail = fpga_push_bank(bank, cfg_bank_const(bank));
        if (detail != CFG_APPLY_OK) {
            s_cfg_store = prev;
            ui_send_cfg_apply_result(bank, 0u, detail);
            ui_send_resp_simple(seq, ERR_FAIL);
            break;
        }

        s_cfg_store.write_count = prev.write_count + 1u;
        if (!cfg_save_to_flash()) {
            s_cfg_store = prev;
            (void)fpga_push_bank(bank, cfg_bank_const(bank));
            ui_send_cfg_apply_result(bank, 0u, CFG_APPLY_ERR_FLASH);
            ui_send_resp_simple(seq, ERR_FAIL);
            break;
        }

        ui_send_cfg_apply_result(bank, 1u, CFG_APPLY_OK);
        ui_send_cfg_bank(bank);
        ui_send_resp_simple(seq, ERR_OK);
        break;
    }

    case CMD_CAMERA_CONTROL: {
        if (plen < 4u) {
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        const uint8_t node_type = d[1];
        const uint8_t camera_sel = d[2];
        const uint8_t count = d[3];
        const uint16_t expected = (uint16_t)(4u + ((uint16_t)count * 4u));
        uint8_t overall_err = ERR_OK;

        if (count == 0u || plen != expected) {
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        if (node_type != NODE_DAY && node_type != NODE_IR) {
            ui_send_resp_simple(seq, ERR_BADARG);
            break;
        }

        for (uint8_t i = 0u; i < count; ++i) {
            const uint16_t off = (uint16_t)(4u + ((uint16_t)i * 4u));
            const uint16_t param_id = (uint16_t)d[off] | ((uint16_t)d[off + 1u] << 8);
            const uint16_t value = (uint16_t)d[off + 2u] | ((uint16_t)d[off + 3u] << 8);
            int rc = -3;

            if (node_type == NODE_DAY) {
                rc = Apply_EO_Camera_Param(camera_sel, param_id, value);
            } else {
                rc = Apply_IR_Camera_Param(camera_sel, param_id, value);
            }

            {
                const uint8_t ok = (rc == 0) ? 1u : 0u;
                const uint8_t err = ok ? ERR_OK : ((rc == -2) ? ERR_BUSY : ERR_FAIL);
                const uint8_t resp_id = (camera_sel == 0u) ? 0xFFu : (uint8_t)(camera_sel - 1u);
                ui_send_camera_apply_result(seq, node_type, resp_id, param_id, value, ok, err);
                if (!ok) overall_err = err;
            }
        }

        ui_send_resp_simple(seq, overall_err);
        break;
    }

    case CMD_CHECK_NODE: {
        if (plen < 3) { ui_send_resp_simple(seq, ERR_BADARG); break; }

        uint8_t node_type = d[1];
        uint8_t id        = d[2];

        uint8_t err = ERR_OK;
        uint8_t ok  = 0;
        bool response_sent = false;

        switch (node_type) {
        case NODE_FPGA:
            ok = (fpga_rw_smoke_test() == 0) ? 1 : 0;
            err = ok ? ERR_OK : ERR_FAIL;
            break;

        case NODE_DAY:
            if (id >= 6) { err = ERR_BADARG; break; }
            ok = (test_day_camera_controls(id) == 0) ? 1 : 0;
            err = ok ? ERR_OK : ERR_FAIL;
            break;

        case NODE_IR:
            if (id >= 6) { err = ERR_BADARG; break; }
            ok = (Initialize_IR_Camera(id) == 0) ? 1 : 0;
            err = ok ? ERR_OK : ERR_FAIL;
            break;

        case NODE_IMU: {
            IMU_Reset();
            HAL_Delay(10);
            int r = send_imu_command(who_am_i_packet, sizeof(who_am_i_packet), SIZE_RESP_READ_REGS_BASE, "WHO_AM_I");
            ok = (r == 0) ? 1 : 0;
            err = ok ? ERR_OK : ERR_FAIL;
            break;
        }

        case NODE_TEMP: {
            float t = 0.0f, h = 0.0f;
            Read_HDC3022_Sensor(&t, &h);

            // basic validity check (matches your error behavior)
            ok = (t > -900.0f && h > -900.0f) ? 1 : 0;

            // Return extended response for TEMP:
            // [ERR_OK][node_type][id][ok][t_centi_deg_L][t_centi_deg_H][h_centi_pct_L][h_centi_pct_H]
            int16_t t_cd = (int16_t)lroundf(t * 100.0f);
            uint16_t h_cp = (uint16_t)lroundf(h * 100.0f);

            uint8_t resp[8];
            resp[0] = ERR_OK;
            resp[1] = node_type;
            resp[2] = id;
            resp[3] = ok;
            resp[4] = (uint8_t)(t_cd & 0xFF);
            resp[5] = (uint8_t)((t_cd >> 8) & 0xFF);
            resp[6] = (uint8_t)(h_cp & 0xFF);
            resp[7] = (uint8_t)((h_cp >> 8) & 0xFF);

            ui_send_frame(MSG_RESP, seq, resp, sizeof(resp));
            response_sent = true;
//            char msg[64];
//                    snprintf(msg, sizeof(msg),
//                             "Temperature %.2f C, Humidity %.2f %%\r\n",
//                             t_cd / 100.0f,
//                             h_cp / 100.0f);
//
//                    Send_UART3_Debug(msg);

            break;
        }

        default:
        err = ERR_BADARG;
        break;
       }

        if (!response_sent) {
            uint8_t resp[4] = { err, node_type, id, ok };
            ui_send_frame(MSG_RESP, seq, resp, sizeof(resp));
        }

        break;
    }

    default:
        ui_send_resp_simple(seq, ERR_BADARG);
        break;
    }
}




