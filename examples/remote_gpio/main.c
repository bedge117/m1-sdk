/*
 * Remote GPIO — drive one M1's header output pins from another, over ESP-NOW.
 *
 * Open the app on two M1s. One picks Controller, the other Receiver, and they
 * pair over the peer link. Toggling a pin on the Controller drives that
 * physical header pin high/low on the Receiver — wire an LED or relay to a
 * header pin and switch it from across the room.
 *
 * Pins are the 12 user header signal pins (PE2, PE4, PE5, PE6, PD12, PD13,
 * PA14, PA13, PC2, PC3, PD0, PD1). The Receiver configures them as outputs on
 * link and parks them safe (and restores SWD) on exit.
 *
 * Protocol (mirrors the Connect Four peer app):
 *   msgs = ['R'][subtype][body]; filtered by magic + opponent MAC.
 *   RG_STATE body = [seq:1][mask:2 LE] — the full 12-bit pin bitmap. The
 *   Controller resends on every change (3x) and as a ~700ms heartbeat, so a
 *   dropped frame self-heals. Receiver applies newest seq (wrap-safe).
 */

#include "m1app.h"

M1_APP_MANIFEST("Remote GPIO", 1024);

#define RG_CHANNEL   1
#define RG_MAX_PEERS 8
#define RG_MAGIC     'R'
enum { RG_LINK = 1, RG_ACK = 2, RG_STATE = 3, RG_QUIT = 4 };

#define GRID_ROWS 6            /* 2 columns x 6 rows = up to 12 pins */
#define HEARTBEAT_MS 700

/* buffered ESP-NOW RX (single task -> file-scope cursor is safe) */
static uint8_t s_buf[1024];
static int     s_len, s_count, s_idx, s_off;

/* ------------------------------------------------------------------ */
static int rg_next(uint8_t from[6], uint8_t *sub, const uint8_t **body, int *blen)
{
    for (;;) {
        if (s_idx >= s_count) {
            s_len = m1_esp_client_now_recv(s_buf, sizeof(s_buf));
            if (s_len < 1 || s_buf[0] == 0) { s_count = 0; return 0; }
            s_count = s_buf[0]; s_idx = 0; s_off = 1;
        }
        while (s_idx < s_count) {
            if (s_off + 8 > s_len) { s_idx = s_count; break; }
            uint8_t fm[6];
            memcpy(fm, &s_buf[s_off], 6); s_off += 6;
            int len = s_buf[s_off] | (s_buf[s_off + 1] << 8); s_off += 2;
            if (s_off + len > s_len) { s_idx = s_count; break; }
            const uint8_t *p = &s_buf[s_off];
            s_off += len; s_idx++;
            if (len >= 2 && p[0] == RG_MAGIC) {
                memcpy(from, fm, 6); *sub = p[1]; *body = p + 2; *blen = len - 2;
                return 1;
            }
        }
    }
}

static void rg_send_state(const uint8_t *mac, uint8_t seq, uint16_t mask)
{
    uint8_t m[5] = { RG_MAGIC, RG_STATE, seq, (uint8_t)(mask & 0xFF), (uint8_t)(mask >> 8) };
    for (int i = 0; i < 3; i++) { m1_esp_client_now_send(mac, m, sizeof(m)); m1app_delay(8); }
}

static void rg_send_simple(const uint8_t *mac, uint8_t sub)
{
    uint8_t m[2] = { RG_MAGIC, sub };
    m1_esp_client_now_send(mac, m, sizeof(m));
}

/* short pin name: "Pin PE2" -> "PE2" */
static const char *rg_pin_name(uint8_t id)
{
    const char *n = m1_gpio_ext_app_name(id);
    if (n[0] == 'P' && n[1] == 'i' && n[2] == 'n' && n[3] == ' ') n += 4;
    return n;
}

/* ------------------------------------------------------------------ */
/* Draw the pin grid. cursor<0 hides the selection box (Receiver view). */
static void rg_draw(u8g2_t *u8g2, uint8_t count, uint16_t mask, int cursor,
                    const char *status)
{
    m1app_display_begin();
    do {
        u8g2_SetDrawColor(u8g2, 1);
        u8g2_SetFont(u8g2, u8g2_font_5x8_tr);
        u8g2_DrawStr(u8g2, 2, 7, status);
        u8g2_DrawHLine(u8g2, 0, 10, 128);

        for (uint8_t id = 0; id < count; id++) {
            int col = id / GRID_ROWS;              /* 0 = left, 1 = right */
            int row = id % GRID_ROWS;
            int x = col ? 66 : 2;
            int y = 20 + row * 7;                  /* baseline */
            int on = (mask >> id) & 1;

            if (cursor == (int)id) {               /* highlight selected pin */
                u8g2_DrawBox(u8g2, x - 1, y - 7, 62, 8);
                u8g2_SetDrawColor(u8g2, 0);
            }
            /* state indicator: filled = ON */
            u8g2_DrawFrame(u8g2, x, y - 6, 6, 6);
            if (on) u8g2_DrawBox(u8g2, x + 1, y - 5, 4, 4);
            u8g2_DrawStr(u8g2, x + 10, y, rg_pin_name(id));
            u8g2_SetDrawColor(u8g2, 1);
        }
    } while (m1app_display_flush());
}

/* ------------------------------------------------------------------ */
static int rg_mode_select(void)
{
    u8g2_t *u8g2 = m1app_get_u8g2();
    static const char *opt[2] = { "Controller",     "Receiver"        };
    static const char *sub[2] = { "sends pin state", "drives its pins" };
    int sel = 0;
    for (;;) {
        m1app_display_begin();
        do {
            u8g2_SetDrawColor(u8g2, 1);
            u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
            u8g2_DrawStr(u8g2, 2, 12, "Remote GPIO");
            for (int i = 0; i < 2; i++) {
                int y = 28 + i * 18;
                if (i == sel) { u8g2_DrawBox(u8g2, 0, y - 9, 128, 18); u8g2_SetDrawColor(u8g2, 0); }
                u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
                u8g2_DrawStr(u8g2, 6, y, opt[i]);
                u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
                u8g2_DrawStr(u8g2, 6, y + 8, sub[i]);
                u8g2_SetDrawColor(u8g2, 1);
            }
        } while (m1app_display_flush());
        m1app_button_t b = game_poll_button(120);
        if (b == M1APP_BTN_BACK) return -1;
        if (b == M1APP_BTN_UP   && sel > 0) sel--;
        if (b == M1APP_BTN_DOWN && sel < 1) sel++;
        if (b == M1APP_BTN_OK) return sel;
    }
}

/* Controller side: pick a receiver from the peer list and invite it. */
static int rg_pick_peer(const uint8_t my_mac[6], uint8_t mac[6])
{
    (void)my_mac;
    u8g2_t *u8g2 = m1app_get_u8g2();
    m1app_now_peer_t peers[RG_MAX_PEERS];
    int npeers = 0, sel = 0;
    uint32_t last = 0;

    for (;;) {
        uint32_t now = m1app_get_tick();
        if (now - last >= 500) {
            last = now;
            m1_esp_client_now_announce();
            int n = m1_esp_client_now_get_peers(peers, RG_MAX_PEERS);
            npeers = (n > 0) ? n : 0;
            if (sel >= npeers) sel = npeers ? npeers - 1 : 0;
        }
        m1app_display_begin();
        do {
            u8g2_SetDrawColor(u8g2, 1);
            u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
            u8g2_DrawStr(u8g2, 2, 11, "Pick a receiver");
            u8g2_SetFont(u8g2, u8g2_font_5x8_tr);
            int y = 24;
            for (int i = 0; i < npeers && i < 3; i++) {
                char line[28];
                snprintf(line, sizeof(line), "%c%s %02X %ddB",
                         (i == sel) ? '>' : ' ', peers[i].name, peers[i].mac[5], peers[i].rssi);
                u8g2_DrawStr(u8g2, 2, y, line); y += 10;
            }
            if (npeers == 0) u8g2_DrawStr(u8g2, 2, 24, " (searching...)");
            u8g2_DrawBox(u8g2, 0, 53, 128, 11);
            u8g2_SetDrawColor(u8g2, 0);
            u8g2_DrawStr(u8g2, 3, 62, "OK:Link"); u8g2_DrawStr(u8g2, 98, 62, "Back");
            u8g2_SetDrawColor(u8g2, 1);
        } while (m1app_display_flush());

        m1app_button_t b = game_poll_button(80);
        if (b == M1APP_BTN_BACK) return 0;
        if (b == M1APP_BTN_UP   && sel > 0) sel--;
        if (b == M1APP_BTN_DOWN && sel < npeers - 1) sel++;
        if (b == M1APP_BTN_OK && npeers > 0) {
            memcpy(mac, peers[sel].mac, 6);
            uint32_t t0 = m1app_get_tick();
            int acked = 0;
            while (!acked && (m1app_get_tick() - t0) < 2500) {
                rg_send_simple(mac, RG_LINK);
                uint32_t w = m1app_get_tick();
                while (!acked && (m1app_get_tick() - w) < 300) {
                    uint8_t fm[6], sb; const uint8_t *bd; int bl;
                    while (rg_next(fm, &sb, &bd, &bl))
                        if (sb == RG_ACK && memcmp(fm, mac, 6) == 0) acked = 1;
                    m1app_delay(10);
                }
            }
            if (acked) return 1;
            m1_message_box(u8g2, "Remote GPIO", "No answer", peers[sel].name, " OK ");
        }
    }
}

/* Receiver side: wait for a controller to link. */
static int rg_wait_controller(const uint8_t my_mac[6], uint8_t mac[6])
{
    (void)my_mac;
    u8g2_t *u8g2 = m1app_get_u8g2();
    uint32_t last = 0;
    for (;;) {
        uint32_t now = m1app_get_tick();
        if (now - last >= 500) { last = now; m1_esp_client_now_announce(); }

        uint8_t fm[6], sb; const uint8_t *bd; int bl;
        while (rg_next(fm, &sb, &bd, &bl)) {
            if (sb == RG_LINK) {
                memcpy(mac, fm, 6);
                rg_send_simple(mac, RG_ACK);
                return 1;
            }
        }
        m1app_display_begin();
        do {
            u8g2_SetDrawColor(u8g2, 1);
            u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
            u8g2_DrawStr(u8g2, 2, 22, "Receiver ready");
            u8g2_SetFont(u8g2, u8g2_font_5x8_tr);
            u8g2_DrawStr(u8g2, 2, 38, "Waiting for a controller");
            u8g2_DrawStr(u8g2, 2, 50, "to link...   Back:cancel");
        } while (m1app_display_flush());

        if (game_poll_button(120) == M1APP_BTN_BACK) return 0;
    }
}

/* ------------------------------------------------------------------ */
static void rg_controller(const uint8_t *mac)
{
    u8g2_t *u8g2 = m1app_get_u8g2();
    uint8_t count  = m1_gpio_ext_app_count();
    uint16_t mask  = 0;
    uint8_t  seq   = 0;
    int      cursor = 0;
    uint32_t beat  = 0;

    rg_send_state(mac, seq, mask);   /* push initial (all-off) state */

    for (;;) {
        uint8_t from[6], sub; const uint8_t *body; int blen;
        while (rg_next(from, &sub, &body, &blen)) {
            if (memcmp(from, mac, 6) != 0) continue;
            if (sub == RG_QUIT) {
                m1_message_box(u8g2, "Remote GPIO", "Receiver", "left", " OK ");
                return;
            }
        }

        uint32_t now = m1app_get_tick();
        if (now - beat >= HEARTBEAT_MS) { beat = now; rg_send_state(mac, seq, mask); }

        rg_draw(u8g2, count, mask, cursor, "Controller  OK=toggle");

        m1app_button_t b = game_poll_button(60);
        if (b == M1APP_BTN_NONE) continue;
        if (b == M1APP_BTN_BACK) { rg_send_simple(mac, RG_QUIT); return; }

        int col = cursor / GRID_ROWS, row = cursor % GRID_ROWS;
        if      (b == M1APP_BTN_UP)    row = (row + GRID_ROWS - 1) % GRID_ROWS;
        else if (b == M1APP_BTN_DOWN)  row = (row + 1) % GRID_ROWS;
        else if (b == M1APP_BTN_LEFT || b == M1APP_BTN_RIGHT) col ^= 1;
        else if (b == M1APP_BTN_OK) {
            if (cursor < count) {
                mask ^= (uint16_t)(1u << cursor);
                seq++;
                rg_send_state(mac, seq, mask);
                m1_buzzer_notification();
            }
        }
        int ni = col * GRID_ROWS + row;
        if (ni < count) cursor = ni;                 /* skip empty grid slots */
        else if (col) cursor = row;                  /* right col empty -> left */
    }
}

static void rg_receiver(const uint8_t *mac)
{
    u8g2_t *u8g2 = m1app_get_u8g2();
    uint8_t count = m1_gpio_ext_app_count();
    uint16_t applied = 0;
    uint8_t  last_seq = 0;
    int      have = 0;

    for (uint8_t i = 0; i < count; i++)
        m1_gpio_ext_app_mode(i, M1APP_GPIO_MODE_OUTPUT);

    for (;;) {
        uint8_t from[6], sub; const uint8_t *body; int blen;
        while (rg_next(from, &sub, &body, &blen)) {
            if (memcmp(from, mac, 6) != 0) continue;
            if (sub == RG_QUIT) {
                m1_gpio_ext_app_release();
                m1_message_box(u8g2, "Remote GPIO", "Controller", "left", " OK ");
                return;
            }
            if (sub == RG_STATE && blen >= 3) {
                uint8_t seq = body[0];
                uint16_t mask = (uint16_t)body[1] | ((uint16_t)body[2] << 8);
                if (!have || (int8_t)(seq - last_seq) >= 0) {   /* newest wins (wrap-safe) */
                    for (uint8_t i = 0; i < count; i++)
                        m1_gpio_ext_app_write(i, (mask >> i) & 1);
                    applied = mask; last_seq = seq; have = 1;
                }
            }
        }

        rg_draw(u8g2, count, applied, -1, "Receiver  driving pins");

        if (game_poll_button(60) == M1APP_BTN_BACK) {
            m1_gpio_ext_app_release();
            rg_send_simple(mac, RG_QUIT);
            return;
        }
    }
}

/* ------------------------------------------------------------------ */
int32_t app_main(void *context)
{
    (void)context;
    u8g2_t *u8g2 = m1app_get_u8g2();
    game_rand_seed();

    if (!m1_esp_client_ping()) {
        m1_message_box(u8g2, "Remote GPIO", "ESP32 not ready", "Initialize it first", " OK ");
        return 0;
    }
    uint8_t my_mac[6];
    if (!m1_esp_client_now_start(RG_CHANNEL, "M1-GPIO", my_mac)) {
        m1_message_box(u8g2, "Remote GPIO", "ESP-NOW start", "failed", " OK ");
        return 0;
    }

    for (;;) {
        int mode = rg_mode_select();
        if (mode < 0) break;

        uint8_t mac[6];
        if (mode == 0) {                 /* Controller */
            if (rg_pick_peer(my_mac, mac)) rg_controller(mac);
        } else {                         /* Receiver */
            if (rg_wait_controller(my_mac, mac)) rg_receiver(mac);
        }
    }

    m1_esp_client_now_stop();
    return 0;
}
