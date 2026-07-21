/*
 * Remote GPIO — drive AND monitor one M1's header pins from another, over ESP-NOW.
 *
 * Open the app on two M1s. One picks Controller, the other Receiver, and they
 * pair over the peer link. On the Controller, each of the 12 user header pins
 * can be cycled OFF -> ON -> INPUT with OK:
 *   - OFF / ON  : the pin is a physical OUTPUT on the Receiver (drive an LED,
 *                 relay, etc.) — square icon, filled when ON.
 *   - INPUT     : the pin is a physical INPUT on the Receiver; the Receiver
 *                 reads its level and reports it back live — circle icon,
 *                 filled when the Receiver reads HIGH.
 *
 * Pins are the 12 user header signal pins (PE2, PE4, PE5, PE6, PD12, PD13,
 * PA14, PA13, PC2, PC3, PD0, PD1). The Receiver parks them safe (and restores
 * SWD) on exit.
 *
 * Protocol (mirrors the Connect Four peer app):
 *   msgs = ['R'][subtype][body]; filtered by magic + peer MAC.
 *   RG_STATE  (ctrl->recv) = [seq:1][dir:2 LE][out:2 LE]   full pin config
 *   RG_INPUT  (recv->ctrl) = [levels:2 LE]                 live pin read-back
 *   Controller resends RG_STATE on every change (3x) and as a ~700ms heartbeat;
 *   Receiver streams RG_INPUT ~5x/sec. Newest seq wins (wrap-safe).
 */

#include "m1app.h"

M1_APP_MANIFEST("Remote GPIO", 1024);

#define RG_CHANNEL   1
#define RG_MAX_PEERS 8
#define RG_MAGIC     'R'
enum { RG_LINK = 1, RG_ACK = 2, RG_STATE = 3, RG_QUIT = 4, RG_INPUT = 5 };

#define GRID_ROWS      6       /* 2 columns x 6 rows = up to 12 pins */
#define HEARTBEAT_MS   700
#define INPUT_TX_MS    200

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

static void rg_send_state(const uint8_t *mac, uint8_t seq, uint16_t dir, uint16_t out)
{
    uint8_t m[7] = { RG_MAGIC, RG_STATE, seq,
                     (uint8_t)(dir & 0xFF), (uint8_t)(dir >> 8),
                     (uint8_t)(out & 0xFF), (uint8_t)(out >> 8) };
    for (int i = 0; i < 3; i++) { m1_esp_client_now_send(mac, m, sizeof(m)); m1app_delay(8); }
}

static void rg_send_input(const uint8_t *mac, uint16_t levels)
{
    uint8_t m[4] = { RG_MAGIC, RG_INPUT, (uint8_t)(levels & 0xFF), (uint8_t)(levels >> 8) };
    m1_esp_client_now_send(mac, m, sizeof(m));
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
/* dir bit = 1 -> pin is an INPUT (circle icon, `levels` shows read-back);
 * dir bit = 0 -> pin is an OUTPUT (square icon, `out` shows drive level).
 * cursor < 0 hides the selection box (Receiver view).                       */
static void rg_draw(u8g2_t *u8g2, uint8_t count, uint16_t dir, uint16_t out,
                    uint16_t levels, int cursor, const char *status)
{
    m1app_display_begin();
    do {
        u8g2_SetDrawColor(u8g2, 1);
        u8g2_SetFont(u8g2, u8g2_font_5x8_tr);
        u8g2_DrawStr(u8g2, 2, 7, status);
        u8g2_DrawHLine(u8g2, 0, 10, 128);

        for (uint8_t id = 0; id < count; id++) {
            int col = id / GRID_ROWS;
            int row = id % GRID_ROWS;
            int x = col ? 66 : 2;
            int y = 20 + row * 7;
            int is_in = (dir >> id) & 1;
            int cx = x + 3, cy = y - 3;         /* icon centre */

            if (cursor == (int)id) {
                u8g2_DrawBox(u8g2, x - 1, y - 7, 62, 8);
                u8g2_SetDrawColor(u8g2, 0);
            }

            if (is_in) {                        /* INPUT: circle, filled if read HIGH */
                if ((levels >> id) & 1) u8g2_DrawDisc(u8g2, cx, cy, 3, U8G2_DRAW_ALL);
                else                    u8g2_DrawCircle(u8g2, cx, cy, 3, U8G2_DRAW_ALL);
            } else {                            /* OUTPUT: square, filled if driven HIGH */
                u8g2_DrawFrame(u8g2, x, y - 6, 6, 6);
                if ((out >> id) & 1) u8g2_DrawBox(u8g2, x + 1, y - 5, 4, 4);
            }
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
    static const char *sub[2] = { "drive + monitor", "runs the pins"   };
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
            if (sb == RG_LINK) { memcpy(mac, fm, 6); rg_send_simple(mac, RG_ACK); return 1; }
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
    uint8_t  count = m1_gpio_ext_app_count();
    uint16_t dir = 0, out = 0, levels = 0;
    uint8_t  seq = 0;
    int      cursor = 0;
    uint32_t beat = 0;

    rg_send_state(mac, seq, dir, out);      /* push initial (all outputs, off) */

    for (;;) {
        uint8_t from[6], sub; const uint8_t *body; int blen;
        while (rg_next(from, &sub, &body, &blen)) {
            if (memcmp(from, mac, 6) != 0) continue;
            if (sub == RG_QUIT) {
                m1_message_box(u8g2, "Remote GPIO", "Receiver", "left", " OK ");
                return;
            }
            if (sub == RG_INPUT && blen >= 2)
                levels = (uint16_t)body[0] | ((uint16_t)body[1] << 8);
        }

        uint32_t now = m1app_get_tick();
        if (now - beat >= HEARTBEAT_MS) { beat = now; rg_send_state(mac, seq, dir, out); }

        rg_draw(u8g2, count, dir, out, levels, cursor, "Ctrl  OK:off/on/in");

        m1app_button_t b = game_poll_button(60);
        if (b == M1APP_BTN_NONE) continue;
        if (b == M1APP_BTN_BACK) { rg_send_simple(mac, RG_QUIT); return; }

        int col = cursor / GRID_ROWS, row = cursor % GRID_ROWS;
        if      (b == M1APP_BTN_UP)    row = (row + GRID_ROWS - 1) % GRID_ROWS;
        else if (b == M1APP_BTN_DOWN)  row = (row + 1) % GRID_ROWS;
        else if (b == M1APP_BTN_LEFT || b == M1APP_BTN_RIGHT) col ^= 1;
        else if (b == M1APP_BTN_OK) {
            if (cursor < count) {
                uint16_t bit = (uint16_t)(1u << cursor);
                int d = (dir >> cursor) & 1, o = (out >> cursor) & 1;
                if      (!d && !o) { out |= bit; }               /* off -> on */
                else if (!d &&  o) { dir |= bit; out &= ~bit; }  /* on -> input */
                else               { dir &= ~bit; out &= ~bit; } /* input -> off */
                seq++;
                rg_send_state(mac, seq, dir, out);
                m1_buzzer_notification();
            }
        }
        int ni = col * GRID_ROWS + row;
        if (ni < count) cursor = ni;
        else if (col)   cursor = row;
    }
}

static void rg_receiver(const uint8_t *mac)
{
    u8g2_t *u8g2 = m1app_get_u8g2();
    uint8_t  count = m1_gpio_ext_app_count();
    uint16_t a_dir = 0xFFFF, a_out = 0xFFFF;   /* force first apply */
    uint16_t cur_dir = 0, cur_out = 0, levels = 0;
    uint8_t  last_seq = 0;
    int      have = 0;
    uint32_t tx = 0;

    /* start with everything an output, low */
    for (uint8_t i = 0; i < count; i++)
        m1_gpio_ext_app_mode(i, M1APP_GPIO_MODE_OUTPUT);
    a_dir = 0; a_out = 0;

    for (;;) {
        uint8_t from[6], sub; const uint8_t *body; int blen;
        while (rg_next(from, &sub, &body, &blen)) {
            if (memcmp(from, mac, 6) != 0) continue;
            if (sub == RG_QUIT) {
                m1_gpio_ext_app_release();
                m1_message_box(u8g2, "Remote GPIO", "Controller", "left", " OK ");
                return;
            }
            if (sub == RG_STATE && blen >= 5) {
                uint8_t seq = body[0];
                if (!have || (int8_t)(seq - last_seq) >= 0) {
                    cur_dir = (uint16_t)body[1] | ((uint16_t)body[2] << 8);
                    cur_out = (uint16_t)body[3] | ((uint16_t)body[4] << 8);
                    last_seq = seq; have = 1;
                }
            }
        }

        /* apply only when the config actually changed */
        if (cur_dir != a_dir || cur_out != a_out) {
            for (uint8_t i = 0; i < count; i++) {
                uint16_t bit = (uint16_t)(1u << i);
                if (cur_dir & bit) {
                    if (!(a_dir & bit)) m1_gpio_ext_app_mode(i, M1APP_GPIO_MODE_INPUT);
                } else {
                    if (a_dir & bit)    m1_gpio_ext_app_mode(i, M1APP_GPIO_MODE_OUTPUT);
                    m1_gpio_ext_app_write(i, (cur_out & bit) ? 1 : 0);
                }
            }
            a_dir = cur_dir; a_out = cur_out;
        }

        /* read every pin and stream levels back to the controller */
        levels = 0;
        for (uint8_t i = 0; i < count; i++)
            if (m1_gpio_ext_app_read(i)) levels |= (uint16_t)(1u << i);

        uint32_t now = m1app_get_tick();
        if (now - tx >= INPUT_TX_MS) { tx = now; rg_send_input(mac, levels); }

        rg_draw(u8g2, count, a_dir, a_out, levels, -1, "Receiver  live");

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
        if (mode == 0) { if (rg_pick_peer(my_mac, mac))       rg_controller(mac); }
        else           { if (rg_wait_controller(my_mac, mac)) rg_receiver(mac);   }
    }

    m1_esp_client_now_stop();
    return 0;
}
