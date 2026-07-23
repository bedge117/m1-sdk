/*
 * Thermometer — DS18B20 1-Wire temperature probe on the M1 header.
 *
 * Uses the firmware's generic 1-Wire primitive (m1_ow_*) to talk to a Maxim
 * DS18B20. All device-specific logic — the ROM/convert/scratchpad command
 * sequence, CRC-8, and raw-to-temperature math — lives here; the firmware only
 * provides the microsecond-timed reset/read/write.
 *
 * Wiring (all on the 10-pin header group):
 *   DS18B20 VDD (red)     -> header pin 9  (+3.3V)
 *   DS18B20 DATA (yellow) -> header pin 14 (PC2)      == app pin id 8
 *   DS18B20 GND (black)   -> header pin 18 (GND)
 *   4.7k resistor between DATA (pin 14) and VDD (pin 9)   <- required pull-up
 */

#include "m1app.h"

M1_APP_MANIFEST("Thermometer", 1024);

#define OW_PIN   8      /* app pin id 8 = PC2 = header physical pin 14 */

enum { RD_OK = 0, RD_NO_SENSOR = -1, RD_CRC = -2, RD_STUCK_LOW = -3, RD_NO_RESP = -4 };

/* Maxim/Dallas CRC-8 (reflected poly 0x8C). */
static uint8_t crc8(const uint8_t *d, int n)
{
    uint8_t crc = 0;
    for (int i = 0; i < n; i++) {
        uint8_t b = d[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (uint8_t)((crc ^ b) & 1);
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            b >>= 1;
        }
    }
    return crc;
}

/* Read one DS18B20 (single drop, Skip ROM). raw = 1/16 deg on success. */
static int ds18b20_read(int16_t *raw)
{
    uint8_t sp[9], all_and = 0xFF, all_or = 0x00;

    if (!m1_ow_reset()) return RD_NO_SENSOR;   /* line idle high, no device */
    m1_ow_write_byte(0xCC);                    /* Skip ROM */
    m1_ow_write_byte(0x44);                    /* Convert T */
    m1app_delay(800);                          /* 12-bit conversion ~750 ms */

    if (!m1_ow_reset()) return RD_NO_SENSOR;
    m1_ow_write_byte(0xCC);
    m1_ow_write_byte(0xBE);                    /* Read Scratchpad */
    for (int i = 0; i < 9; i++) {
        sp[i] = m1_ow_read_byte();
        all_and &= sp[i];
        all_or  |= sp[i];
    }

    if (all_or  == 0x00) return RD_STUCK_LOW;  /* every bit low: short / wrong pin / no 3V3 */
    if (all_and == 0xFF) return RD_NO_RESP;    /* every bit high: nothing driving DATA */
    if (crc8(sp, 8) != sp[8]) return RD_CRC;

    *raw = (int16_t)((sp[1] << 8) | sp[0]);
    return RD_OK;
}

static void draw_err(u8g2_t *u8g2, const char *l1, const char *l2, const char *l3)
{
    u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
    u8g2_DrawStr(u8g2, 2, 30, l1);
    u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
    if (l2) u8g2_DrawStr(u8g2, 2, 44, l2);
    if (l3) u8g2_DrawStr(u8g2, 2, 53, l3);
}

int32_t app_main(void *context)
{
    (void)context;
    u8g2_t *u8g2 = m1app_get_u8g2();

    /* Power the probe from the header 3.3V rail. The 3.3V and 5V rails are
     * mutually exclusive, so make sure 5V is off first, then let it settle. */
    ext_power_5V_set(0);
    ext_power_3V_set(1);
    m1app_delay(100);
    m1_ow_init(OW_PIN);

    for (;;) {
        int16_t raw = 0;
        int rc = ds18b20_read(&raw);

        int c10 = (raw * 10) / 16;       /* tenths of a degree, signed */
        int f10 = (c10 * 9) / 5 + 320;

        m1app_display_begin();
        do {
            u8g2_SetDrawColor(u8g2, 1);
            u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
            u8g2_DrawStr(u8g2, 2, 11, "Thermometer");
            u8g2_DrawHLine(u8g2, 0, 14, 128);

            if (rc == RD_NO_SENSOR) {
                draw_err(u8g2, "No sensor found",
                         "DATA=pin14 VDD=pin9 GND=pin18",
                         "4.7k between DATA and VDD");
            } else if (rc == RD_STUCK_LOW) {
                draw_err(u8g2, "DATA line stuck LOW",
                         "wrong pin? DATA/GND swapped?",
                         "3.3V rail / short to GND");
            } else if (rc == RD_NO_RESP) {
                draw_err(u8g2, "No response on DATA",
                         "check DATA wire + 4.7k pull-up", 0);
            } else if (rc == RD_CRC) {
                draw_err(u8g2, "Bad read (CRC)",
                         "noisy line / weak pull-up", 0);
            } else {
                char line[20];
                const char *cs = (c10 < 0) ? "-" : "";
                const char *fs = (f10 < 0) ? "-" : "";
                int ca = (c10 < 0) ? -c10 : c10;
                int fa = (f10 < 0) ? -f10 : f10;

                u8g2_SetFont(u8g2, u8g2_font_10x20_mr);
                snprintf(line, sizeof(line), "%s%d.%d C", cs, ca / 10, ca % 10);
                u8g2_DrawStr(u8g2, 10, 40, line);

                u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
                snprintf(line, sizeof(line), "%s%d.%d F", fs, fa / 10, fa % 10);
                u8g2_DrawStr(u8g2, 10, 55, line);
            }

            u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
            u8g2_DrawStr(u8g2, 100, 62, "Back:exit");
        } while (m1app_display_flush());

        if (game_poll_button(50) == M1APP_BTN_BACK) break;
    }

    m1_ow_deinit();
    ext_power_3V_set(0);
    return 0;
}
