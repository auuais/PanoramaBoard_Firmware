#include "proto_uart.h"
#include <string.h>

uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Standard COBS
size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max) {
    if (!in || !out || out_max == 0) return 0;

    size_t read_index = 0, write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < len) {
        if (write_index >= out_max) return 0;

        if (in[read_index] == 0) {
            out[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            out[write_index++] = in[read_index++];
            code++;
            if (code == 0xFF) {
                out[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    if (code_index >= out_max) return 0;
    out[code_index] = code;

    return write_index;
}

size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max) {
    if (!in || !out) return 0;
    size_t read_index = 0, write_index = 0;

    while (read_index < len) {
        uint8_t code = in[read_index];
        if (code == 0) return 0;
        read_index++;

        for (uint8_t i = 1; i < code; i++) {
            if (read_index >= len) return 0;
            if (write_index >= out_max) return 0;
            out[write_index++] = in[read_index++];
        }
        if (code != 0xFF && read_index < len) {
            if (write_index >= out_max) return 0;
            out[write_index++] = 0;
        }
    }
    return write_index;
}

size_t uart_frame_build(const uint8_t *payload, size_t payload_len,
                        uint8_t *out, size_t out_max) {
    if (!payload || !out) return 0;

    // temp: payload + crc16
    uint8_t tmp[512];
    if (payload_len + 2 > sizeof(tmp)) return 0;

    memcpy(tmp, payload, payload_len);
    uint16_t crc = crc16_ccitt_false(payload, payload_len);
    tmp[payload_len + 0] = (uint8_t)(crc & 0xFF);       // little-endian CRC
    tmp[payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    size_t enc_len = cobs_encode(tmp, payload_len + 2, out, out_max);
    if (enc_len == 0) return 0;
    if (enc_len + 1 > out_max) return 0;
    out[enc_len] = 0x00; // delimiter
    return enc_len + 1;
}

size_t uart_frame_parse(const uint8_t *cobs_data, size_t cobs_len,
                        uint8_t *payload_out, size_t payload_out_max) {
    if (!cobs_data || !payload_out) return 0;

    uint8_t tmp[512];
    if (cobs_len > sizeof(tmp)) return 0;

    size_t dec_len = cobs_decode(cobs_data, cobs_len, tmp, sizeof(tmp));
    if (dec_len < 3) return 0; // need at least 1 byte payload + 2 CRC

    size_t payload_len = dec_len - 2;
    if (payload_len > payload_out_max) return 0;

    uint16_t crc_rx = (uint16_t)tmp[payload_len] | ((uint16_t)tmp[payload_len + 1] << 8);
    uint16_t crc_calc = crc16_ccitt_false(tmp, payload_len);
    if (crc_rx != crc_calc) return 0;

    memcpy(payload_out, tmp, payload_len);
    return payload_len;
}
