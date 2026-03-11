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
#include "orientation_kalman.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stddef.h>
#include <limits.h>

//
static void fpga_service_orient(void);


// ---- externs from existing codebase ----
extern void Enable_All_Camera_Power(void);
extern void IMU_Reset(void);
extern void IMU_SoftReset(void);
extern int send_imu_command(uint8_t *tx_packet, uint32_t tx_size, uint32_t rx_size, const char *cmd_name);
extern int send_imu_command_ex(uint8_t *tx_packet, uint32_t tx_size, uint32_t rx_size, const char *cmd_name, uint8_t *rx_out);
extern void Read_HDC3022_Sensor(float* temp, float* hum);
extern void Send_UART3_Debug(const char *format, ...);
extern int test_day_camera_controls(uint8_t camera_num);
extern int Initialize_IR_Camera(uint8_t camera_num);
extern SPI_HandleTypeDef hspi2;

// UART3 (PC UI)
extern UART_HandleTypeDef huart3;

// Optional global debug mirror (so legacy code can keep calling Send_UART3_Debug safely)
extern volatile bool g_debug_enabled;
extern fpga_i2c_t g_fpga;
extern void IMU_Handler_SetSampleFreq(float sample_freq);

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
#define EVT_CFG_BANK    0x30   // [sub][bank][mode][8*i32 LE quantized params]
#define EVT_CFG_APPLY   0x31   // [sub][bank][ok][detail]

// ---- Command IDs ----
#define CMD_CHECK_NODE   0x10
#define CMD_SET_MODE     0x11
#define CMD_SET_DEBUG    0x12
#define CMD_INIT_SYSTEM  0x13
#define CMD_GET_CONFIG   0x30
#define CMD_APPLY_CONFIG 0x31

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
#define CFG_FLOAT_COUNT  8

// ---- Orientation safety clamp (hard limits, not artificial) ----
#define ORIENT_CLAMP_DEG   (90.0f)

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
static uint32_t s_last_drdy_tick = 0;
static uint32_t s_drdy_accum_ms = 0;
static uint16_t s_drdy_samples = 0;
static Kalman1D_t s_kf_roll;
static Kalman1D_t s_kf_pitch;

/* -------------------------------------------------------------------------- */
/* 0.25° quantization — zero lag, eliminates sub-quarter-degree jitter        */
/*                                                                             */
/* Madgwick IS the filter. Its quaternion integrator already smooths the      */
/* gyro + accel fusion — we do not need a second software filter on top.      */
/* Adding EMA on the Euler output just creates lag that makes 30 Hz frame     */
/* correction feel sluggish.                                                  */
/*                                                                             */
/* What quantization does:                                                     */
/*   - Rounds centidegrees to the nearest 25 cd (= 0.25°)                    */
/*   - Any jitter smaller than ±12 cd (±0.12°) is silently absorbed          */
/*   - True motion larger than 0.25° passes through at full speed             */
/*   - Zero samples of lag — output reflects this exact sample                */
/* -------------------------------------------------------------------------- */
#define QUANT_STEP_CD   25   /* 0.25 degree = 25 centidegrees */

/**
 * @brief Round val_cd to the nearest QUANT_STEP_CD increment.
 *
 * Works correctly for positive and negative values.
 * Example: 137 cd → 125 cd,  138 cd → 150 cd,  -137 cd → -125 cd
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
    int32_t cyl_h_fov_q16;
    int32_t cyl_v_fov_q16;
    int32_t pano_crop_h_q16;
    int32_t pano_crop_w_q16;
    int32_t pitch_to_translation_raw_q16;
    int32_t yaw_to_translation_q16;
    int32_t overlap_px_i32;
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
#define PARAM_STORE_VERSION    0x0002u
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

static inline int32_t div_round_i64_local(int64_t num, int64_t den)
{
    if (den == 0) return 0;
    if (num >= 0) return (int32_t)((num + den / 2) / den);
    return (int32_t)((num - den / 2) / den);
}

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

static int32_t cfg_bank_resize_q16(uint8_t bank)
{
    return (bank == CFG_BANK_IR) ? (1 << 16) : (1 << 15); // IR=1.0, EO=0.5
}

static void cfg_set_bank_defaults(ParamBankQ16* bank, uint8_t which)
{
    if (which == CFG_BANK_IR) {
        bank->cyl_h_fov_q16 = float_to_q16_sat(65.0f);
        bank->cyl_v_fov_q16 = float_to_q16_sat(52.0f);
        bank->pano_crop_h_q16 = float_to_q16_sat(0.65f);
        bank->pano_crop_w_q16 = float_to_q16_sat(0.9f);
        bank->pitch_to_translation_raw_q16 = float_to_q16_sat(-8.72f);
        bank->yaw_to_translation_q16 = float_to_q16_sat(-8.70f);
        bank->overlap_px_i32 = 30;
        bank->feather_strength_q16 = float_to_q16_sat(1.0f);
        return;
    }

    bank->cyl_h_fov_q16 = float_to_q16_sat(77.0f);
    bank->cyl_v_fov_q16 = float_to_q16_sat(51.0f);
    bank->pano_crop_h_q16 = float_to_q16_sat(0.70f);
    bank->pano_crop_w_q16 = float_to_q16_sat(0.725f);
    bank->pitch_to_translation_raw_q16 = float_to_q16_sat(-9.80f);
    bank->yaw_to_translation_q16 = float_to_q16_sat(17.84f);
    bank->overlap_px_i32 = 100;
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

static bool cfg_is_valid(const ParamStoreBlob* cfg)
{
    if (cfg->magic != PARAM_STORE_MAGIC) return false;
    if (cfg->version != PARAM_STORE_VERSION) return false;
    if (cfg->length != (uint16_t)sizeof(*cfg)) return false;
    if (!cfg_mode_valid(cfg->mode)) return false;
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

static bool cfg_bank_from_float_payload(const uint8_t* p, uint16_t n, ParamBankQ16* out)
{
    if (!p || !out) return false;
    if (n < (uint16_t)(CFG_FLOAT_COUNT * 4u)) return false;

    float vals[CFG_FLOAT_COUNT];
    for (uint8_t i = 0; i < CFG_FLOAT_COUNT; ++i) {
        memcpy(&vals[i], p + ((uint16_t)i * 4u), 4u);
    }

    out->cyl_h_fov_q16 = float_to_q16_sat(vals[0]);
    out->cyl_v_fov_q16 = float_to_q16_sat(vals[1]);
    out->pano_crop_h_q16 = float_to_q16_sat(vals[2]);
    out->pano_crop_w_q16 = float_to_q16_sat(vals[3]);
    out->pitch_to_translation_raw_q16 = float_to_q16_sat(vals[4]);
    out->yaw_to_translation_q16 = float_to_q16_sat(vals[5]);
    out->overlap_px_i32 = (int32_t)lroundf(vals[6]);
    out->feather_strength_q16 = float_to_q16_sat(vals[7]);
    return true;
}

static int32_t cfg_pitch_to_translation_effective_q16(uint8_t bank_id, const ParamBankQ16* bank)
{
    const int32_t resize_q16 = cfg_bank_resize_q16(bank_id);
    return div_round_i64_local(((int64_t)bank->pitch_to_translation_raw_q16) << 16, resize_q16);
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

static uint8_t fpga_push_bank(uint8_t bank, const ParamBankQ16* cfg)
{
    const uint8_t base = fpga_cfg_bank_base(bank);
    uint8_t detail = CFG_APPLY_OK;

    if (!fpga_write_verify_i32((uint8_t)(base + FPGA_REG_CFG_CYL_H_FOV_Q16), cfg->cyl_h_fov_q16, &detail)) return detail;
    if (!fpga_write_verify_i32((uint8_t)(base + FPGA_REG_CFG_CYL_V_FOV_Q16), cfg->cyl_v_fov_q16, &detail)) return detail;
    if (!fpga_write_verify_i32((uint8_t)(base + FPGA_REG_CFG_CROP_H_Q16), cfg->pano_crop_h_q16, &detail)) return detail;
    if (!fpga_write_verify_i32((uint8_t)(base + FPGA_REG_CFG_CROP_W_Q16), cfg->pano_crop_w_q16, &detail)) return detail;
    if (!fpga_write_verify_i32((uint8_t)(base + FPGA_REG_CFG_PITCH_TR_Q16), cfg_pitch_to_translation_effective_q16(bank, cfg), &detail)) return detail;
    if (!fpga_write_verify_i32((uint8_t)(base + FPGA_REG_CFG_YAW_TR_Q16), cfg->yaw_to_translation_q16, &detail)) return detail;
    if (!fpga_write_verify_i32((uint8_t)(base + FPGA_REG_CFG_OVERLAP_I32), cfg->overlap_px_i32, &detail)) return detail;
    if (!fpga_write_verify_i32((uint8_t)(base + FPGA_REG_CFG_FEATHER_Q16), cfg->feather_strength_q16, &detail)) return detail;

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

static void ui_send_log(McuContext *ctx, const char *s)
{
    if (!ctx->debug_enabled) return;
    uint16_t n = (uint16_t)strlen(s);
    if (n > 200) n = 200;
    ui_send_frame(MSG_LOG, 0, (const uint8_t*)s, n);
}

/* Public version — callable from main.c and any other translation unit.
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

static void put_le_i32(uint8_t* dst, int32_t v)
{
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
    dst[2] = (uint8_t)((v >> 16) & 0xFF);
    dst[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void ui_send_cfg_bank(uint8_t bank)
{
    const ParamBankQ16* cfg = cfg_bank_const(bank);
    uint8_t p[3 + (CFG_FLOAT_COUNT * 4)];
    p[0] = EVT_CFG_BANK;
    p[1] = bank;
    p[2] = s_cfg_store.mode;

    put_le_i32(&p[3 + (0 * 4)], cfg->cyl_h_fov_q16);
    put_le_i32(&p[3 + (1 * 4)], cfg->cyl_v_fov_q16);
    put_le_i32(&p[3 + (2 * 4)], cfg->pano_crop_h_q16);
    put_le_i32(&p[3 + (3 * 4)], cfg->pano_crop_w_q16);
    put_le_i32(&p[3 + (4 * 4)], cfg->pitch_to_translation_raw_q16);
    put_le_i32(&p[3 + (5 * 4)], cfg->yaw_to_translation_q16);
    put_le_i32(&p[3 + (6 * 4)], cfg->overlap_px_i32);
    put_le_i32(&p[3 + (7 * 4)], cfg->feather_strength_q16);

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


// ===================== FPGA helpers =====================
//static int fpga_write16(uint16_t reg, const uint8_t *data, uint8_t len)
//{
//    return fpga_write(reg, data, len);
//}

static int fpga_rw_smoke_test(void)
{
  uint8_t rd = 0;
  const uint8_t wr = 0xA5;

  HAL_StatusTypeDef st = fpga_write_read_reg8(&g_fpga, FPGA_REG_MODE_CURRENT, wr, &rd, 200);
  if (st != HAL_OK) {
    uint32_t e = HAL_I2C_GetError(g_fpga.hi2c);
    Send_UART3_Debug("FPGA smoke fail HAL=%d I2Cerr=0x%08lX\r\n", (int)st, (unsigned long)e);
    return -1;
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
    // to the UI's sequence number — do NOT reset it here.

    // Only allow init start from idle/wait state
    if (ctx->state == ST_WAIT_UI_INIT) {
        ctx->state = ST_BOOT;
        // No extra ACK here — the caller (CMD_INIT_SYSTEM handler) already sent it
    }
}


void mcu_fsm_step(McuContext *ctx)
{
	switch (ctx->state) {




    case ST_WAIT_UI_INIT: {
        // Do nothing until UI requests init

//    	ctx->state = ST_BOOT;
        break;
    }


    case ST_BOOT: {
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
        //   Reset → WHO_AM_I → Reset → imu_configure_odr_lpf → START_STREAM
        //
        // SPI fixes applied (batch 14):
        //   - spi2_flush() now re-asserts FRXTH=1 on every call
        //   - IMU_NSS_HIGH_US raised from 20 µs to 500 µs so the sensor
        //     has time to latch the command before DRDY is polled
        //   - Phase 2 DRDY also checks pin level, not just interrupt flag,
        //     to catch the race where the edge fires before the poll loop
        //
        // With these fixes multiple consecutive commands work, so ODR
        // and SELECT_OUT_DATA are written explicitly every boot — we no
        // longer rely on whatever was saved to flash.
        // ----------------------------------------------------------------

        /* ---- First reset: clean boot ---- */
        IMU_Reset();

        /* ---- WHO_AM_I — confirm sensor is alive ---- */
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

        /* ---- Second reset: clears WHO_AM_I side-effects ---- */
        HAL_Delay(10);
        IMU_Reset();

        /* ---- Configure ODR=1000 Hz and SELECT_OUT_DATA=0x07 ----
         * Write every boot — do not rely on flash state.
         * imu_configure_odr_lpf writes: SAMPLE_RATE_DIV, SELECT_OUT_DATA,
         * BW_CONFIG.  SAVE_ALL_CONFIG is intentionally removed (see
         * imu_config.c) to reduce the number of SPI transactions.      */
        float imu_odr_hz = 1000.0f;
        {
            int rc = imu_configure_odr_lpf(IMU_ODR_DIV_1000HZ, IMU_BW_DEFAULT);
            if (rc == 0) {
                ui_send_log(ctx, "[IMU] ODR=1000 Hz, SELECT_OUT_DATA=0x07 OK\r\n");
            } else {
                imu_odr_hz = 50.0f;   /* fall back to ~sensor default */
                ui_send_log(ctx, "[IMU] WARN: ODR config failed, using flash defaults\r\n");
            }
        }

        IMU_Handler_Init(imu_odr_hz, 0.9f);
        Kalman1D_Init(&s_kf_roll,  0.001f, 0.003f, 0.5f, 0.0f);
        Kalman1D_Init(&s_kf_pitch, 0.001f, 0.003f, 0.5f, 0.0f);

        /* ---- START_STREAM — no DRDY/ACK, returns immediately ---- */
        HAL_Delay(10);
        IMU_Reset();
        int rs = send_imu_command(start_streaming_packet,
                                  sizeof(start_streaming_packet),
                                  0, "START_STREAM");

        if (rs == 0) {
            ctx->imu_streaming = true;
            ui_send_log(ctx, "[IMU] Streaming started\r\n");
        } else {
            ctx->imu_streaming = false;
            uint8_t err_payload[2] = { NODE_IMU, ERR_FAIL };
            ui_send_frame(MSG_EVT, 0, err_payload, sizeof(err_payload));
            ui_send_log(ctx, "[IMU] START_STREAM failed\r\n");
        }

        ctx->day_idx = 0;
        ctx->state = ST_INIT_DAY;
        break;
    }


    case ST_INIT_DAY: {
        if (ctx->day_idx < 6) {
            HAL_Delay(20);
            int r = test_day_camera_controls(ctx->day_idx);   // change it to return status if it doesn’t
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
        	int r = Initialize_IR_Camera(ctx->ir_idx);   // change to return status if it doesn’t
            uint8_t ok = (r == 0) ? 1 : 0;
            ui_send_init_ack(ctx->init_seq,NODE_IR, ctx->ir_idx, ok, ok ? ERR_OK : ERR_FAIL);

            ctx->ir_idx++;

        } else {
            ctx->state = ST_INIT_FPGA_CAL;
        }
        break;
    }


    case ST_INIT_FPGA_CAL: {
        int st = fpga_rw_smoke_test();

        if (st == 0) {
            ui_send_log(ctx, "FPGA I2C ping OK (0xA5)\r\n");
            Send_UART3_Debug("FPGA I2C ping OK (0xA5)\r\n\n\n");
            uint8_t detail = fpga_push_all_runtime_config();
            if (detail == CFG_APPLY_OK) {
                ui_send_log(ctx, "FPGA runtime config push OK\r\n");
                ui_send_init_ack(ctx->init_seq,NODE_FPGA, 0, 1, ERR_OK);
            } else {
                char msg[96];
                snprintf(msg, sizeof(msg), "FPGA runtime config push FAIL detail=%u\r\n", (unsigned)detail);
                ui_send_log(ctx, msg);
                ui_send_init_ack(ctx->init_seq,NODE_FPGA, 0, 0, ERR_FAIL);
            }

        } else {
            ui_send_log(ctx, "FPGA I2C ping FAIL\r\n");
            Send_UART3_Debug("FPGA I2C ping FAIL\r\n");

             ui_send_init_ack(ctx->init_seq,NODE_FPGA, 0, 0, ERR_FAIL);
        }

        // Do NOT say Entered OPERATION here; we are not in OPERATION yet.
        ctx->state = ST_INIT_TEMP;
        break;
    }

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
// Called from main loop when g_imu_drdy_flag is set (EXTI PB9).

/* Local microsecond delay for NSS timing in streaming read path */
static void fsm_delay_us(uint32_t us) {
    volatile uint32_t count = us * 4u;
    while (count--) { __NOP(); }
}

/* Confirmed from log (2026-02-26, batch 11):
 * The sensor outputs 1 dummy byte (0x00) before the 46-byte stream packet.
 * Total SPI read = 47 bytes.  Packet starts at byte[1] (0x23 0x23 header).
 *   byte[0]       = 0x00  (dummy / SPI pipeline flush byte — ignore)
 *   byte[1..2]    = 0x23 0x23  (stream packet header)
 *   byte[3]       = packet size byte (0x2E = 46)
 *   byte[4..46]   = remainder of 46-byte stream packet
 *
 * SELECT_OUT_DATA = 0x07 (accel+gyro+temp, 7 outputs):
 *   SIZE_PACKET_BASE_DATA(18) + 4×7 = 46 bytes payload
 */
#define IMU_STREAM_DUMMY_BYTES  1u               /* leading dummy byte  */
#define IMU_STREAM_PKT_BYTES    46u              /* actual packet bytes */
#define IMU_STREAM_READ_TOTAL   (IMU_STREAM_DUMMY_BYTES + IMU_STREAM_PKT_BYTES)  /* 47 */

static uint8_t s_imu_rx_buf[IMU_STREAM_READ_TOTAL];

void mcu_on_imu_drdy(McuContext *ctx)
{
    if (ctx->state != ST_OPERATION) return;

    /* ------------------------------------------------------------------ */
    /* Bulk streaming read:                                                 */
    /*   CS LOW → HAL_SPI_Receive 47 bytes → CS HIGH                      */
    /* byte[0] = dummy, byte[1..46] = 46-byte stream packet (0x23 header) */
    /* ------------------------------------------------------------------ */

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);  /* CS LOW  */
    fsm_delay_us(5u);

    HAL_StatusTypeDef spi_st = HAL_SPI_Receive(
        &hspi2,
        s_imu_rx_buf,
        IMU_STREAM_READ_TOTAL,
        10u);   /* 47 bytes @ 4 MHz ≈ 94 µs; 10 ms timeout is plenty */

    fsm_delay_us(5u);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);    /* CS HIGH */

    if (spi_st != HAL_OK) return;

    /* Packet always starts at byte[1] — validate header */
    uint8_t *pkt      = &s_imu_rx_buf[IMU_STREAM_DUMMY_BYTES];
    uint32_t pkt_avail = IMU_STREAM_PKT_BYTES;

    if (pkt[0] != 0x23u || pkt[1] != 0x23u) {
        /* Unexpected — header not where expected, discard */
        return;
    }

    // ---- Parse packet + run Madgwick ----
    IMU_Sample_t sample;
    int parse_result = IMU_Handler_ProcessStreamPacket(pkt, pkt_avail, &sample);
    if (parse_result != IMU_ERR_OK) {
        /* Discard silently — no Send_UART3_Debug here, blocking UART kills 1kHz */
        return;
    }

    // ---- DRDY interval measurement (ms) -> ODR ----
    uint32_t now = HAL_GetTick();
    float dt_s = 0.0f;
    if (s_last_drdy_tick != 0) {
        uint32_t delta_ms = now - s_last_drdy_tick;
        if (delta_ms > 0 && delta_ms < 1000) { // ignore outliers
            dt_s = delta_ms * 0.001f;
            s_drdy_accum_ms += delta_ms;
            s_drdy_samples++;
            if (s_drdy_samples >= 16) {
                uint32_t avg_ms = s_drdy_accum_ms / s_drdy_samples;
                if (avg_ms > 0) {
                    uint16_t rate_hz = (uint16_t)((1000u + (avg_ms / 2u)) / avg_ms); // rounded
                    s_rate_hz = (int16_t)rate_hz;
                    IMU_Handler_SetSampleFreq((float)rate_hz);
                    ui_send_imu_odr(rate_hz);
                    /* NOTE: Send_UART3_Debug removed from here — it calls
                     * HAL_UART_Transmit (blocking) every 16 samples which
                     * adds ~1.3 ms stall at 115200 and prevents 1kHz processing. */
                }
                s_drdy_accum_ms = 0;
                s_drdy_samples = 0;
            }
        }
    }
    s_last_drdy_tick = now;
    if (dt_s <= 0.0f) dt_s = 0.001f; // fallback 1 kHz

    // ---- Extract roll and pitch (yaw discarded — no magnetometer) ----
    float roll_deg  = sample.euler.roll;
    float pitch_deg = sample.euler.pitch;

    // ---- Kalman fuse gyro+accel (gyro deg/s, accel-derived angles) ----
    float acc_roll_deg  = atan2f(sample.accel_y, sample.accel_z) * (180.0f / 3.14159265358979f);
    float acc_pitch_deg = atan2f(-sample.accel_x, sqrtf(sample.accel_y*sample.accel_y + sample.accel_z*sample.accel_z)) * (180.0f / 3.14159265358979f);
    float kf_roll_deg  = Kalman1D_Update(&s_kf_roll, sample.gyro_x, acc_roll_deg, dt_s);
    float kf_pitch_deg = Kalman1D_Update(&s_kf_pitch, sample.gyro_y, acc_pitch_deg, dt_s);

    // ---- Safety clamp ±90° ----
    if (roll_deg  >  90.0f) roll_deg  =  90.0f;
    if (roll_deg  < -90.0f) roll_deg  = -90.0f;
    if (pitch_deg >  90.0f) pitch_deg =  90.0f;
    if (pitch_deg < -90.0f) pitch_deg = -90.0f;

    if (kf_roll_deg  >  90.0f) kf_roll_deg  =  90.0f;
    if (kf_roll_deg  < -90.0f) kf_roll_deg  = -90.0f;
    if (kf_pitch_deg >  90.0f) kf_pitch_deg =  90.0f;
    if (kf_pitch_deg < -90.0f) kf_pitch_deg = -90.0f;

    // ---- Convert to centidegrees (int16) ----
    int16_t roll_cd  = (int16_t)lroundf(kf_roll_deg  * 100.0f);
    int16_t pitch_cd = (int16_t)lroundf(kf_pitch_deg * 100.0f);

    // ---- Quantize to 0.25° steps ----
    roll_cd  = quantize_cd(roll_cd);
    pitch_cd = quantize_cd(pitch_cd);

    // ---- Update shadow registers ----
    s_roll_cd      = roll_cd;
    s_pitch_cd     = pitch_cd;

    // ---- Throttle UI sends to 100 Hz max ----------------------------------------
    // HAL_UART_Transmit is BLOCKING. At 921600 baud a ~15 byte frame costs ~0.16 ms.
    // At 1000 Hz that is 160 ms/s of blocking — acceptable but only if we throttle.
    // Sending every 10th sample = 100 Hz to the UI, zero DRDY samples are dropped
    // (Kalman still runs at full 1 kHz), and UART time is capped at 16 ms/s.
    // -------------------------------------------------------------------------------
    static uint16_t s_ui_throttle = 0u;
    s_ui_throttle++;
    if (s_ui_throttle >= 10u) {
        s_ui_throttle = 0u;
        ui_send_imu_orient(roll_cd, pitch_cd);
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

    uint8_t payload[8];

    const int16_t roll  = s_roll_cd;
    const int16_t pitch = s_pitch_cd;
    const uint16_t rate = (uint16_t)s_rate_hz;
    const uint16_t seq  = ++s_orient_tx_seq;

    payload[0] = (uint8_t)(roll & 0xFF);
    payload[1] = (uint8_t)((roll >> 8) & 0xFF);
    payload[2] = (uint8_t)(pitch & 0xFF);
    payload[3] = (uint8_t)((pitch >> 8) & 0xFF);
    payload[4] = (uint8_t)(rate & 0xFF);
    payload[5] = (uint8_t)((rate >> 8) & 0xFF);
    payload[6] = (uint8_t)(seq & 0xFF);
    payload[7] = (uint8_t)((seq >> 8) & 0xFF);

    uint8_t detail = CFG_APPLY_OK;
    for (uint8_t i = 0; i < (uint8_t)sizeof(payload); ++i) {
        if (!fpga_write_verify_reg8((uint8_t)(FPGA_REG_ORIENT_ROLL_CD8 + i), payload[i], &detail)) {
            break;
        }
    }
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
            if (maybe_cmd != CMD_INIT_SYSTEM && maybe_cmd != CMD_GET_CONFIG) {
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

        if (ctx->state == ST_WAIT_UI_INIT) {
            Send_UART3_Debug("INIT: started\r\n");
            mcu_fsm_start_init(ctx);
            break;
        }

        // If already running (or in init), do not block. Just re-sync the UI.
        ui_send_log(ctx, "INIT: resync (already running)\r\n");

        // Option 1 (fastest): report “ready” for nodes (snapshot)
        // (You can make these more truthful if you store per-node init results.)
        ui_send_init_ack(seq, NODE_IMU,  0, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_DAY,  0, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_DAY,  1, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_DAY,  2, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_DAY,  3, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_DAY,  4, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_DAY,  5, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_IR,   0, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_IR,   1, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_IR,   2, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_IR,   3, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_IR,   4, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_IR,   5, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_FPGA, 0, 1, ERR_OK);
        ui_send_init_ack(seq, NODE_TEMP, 0, 1, ERR_OK);

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

      ui_send_cfg_bank(CFG_BANK_EO);
      ui_send_cfg_bank(CFG_BANK_IR);
      ui_send_resp_simple(seq, ERR_OK);
      break;
    }

    case CMD_APPLY_CONFIG: {
        if (plen < (uint16_t)(2u + (CFG_FLOAT_COUNT * 4u))) {
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
        if (!cfg_bank_from_float_payload(&d[2], (uint16_t)(plen - 2u), &parsed)) {
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
