/*
 * Connect Four — 2-player over the ESP-NOW peer link
 *
 * A drop-and-connect game for two M1 units. Open the app on both devices,
 * pick your opponent from the peer list, and take turns dropping discs.
 * First to line up four in a row (horizontal, vertical, or diagonal) wins.
 *
 * Protocol (mirrors the built-in peer Tic-Tac-Toe):
 *   - Every message is  ['C'][subtype][body...]  so stray frames are ignored.
 *   - The inviter is Player 1 (filled disc) and moves first; the invitee is
 *     Player 2 (ring). A mutual invite is broken by MAC comparison.
 *   - After every move the mover broadcasts the FULL board a few times
 *     (ESP-NOW frames can drop); a move counter makes repeats idempotent.
 */

#include "m1app.h"

M1_APP_MANIFEST("Connect Four", 1024);

/* ---- board / game ---- */
#define COLS        7
#define ROWS        6
#define CF_CHANNEL  1
#define CF_MAX_PEERS 8

#define CF_MAGIC    'C'
enum { CF_INVITE = 1, CF_ACCEPT = 2, CF_STATE = 3, CF_QUIT = 4 };

enum { CELL_EMPTY = 0, CELL_P1 = 1, CELL_P2 = 2 };
enum { ST_PLAY = 0, ST_P1WIN = 1, ST_P2WIN = 2, ST_DRAW = 3 };

/* ---- board pixel layout (128x64) ---- */
#define STATUS_Y   7
#define BOARD_OX   15
#define BOARD_OY   15
#define CELL_W     14
#define CELL_H     8
#define DISC_R     3

/* Buffered ESP-NOW RX: hand back one 'C' message at a time. The app runs as a
 * single task, so file-scope cursors are safe (not reentrant). */
static uint8_t s_buf[1024];
static int     s_len, s_count, s_idx, s_off;

/* ------------------------------------------------------------------ */
/* Return the next buffered Connect-Four message, draining the ESP as
 * needed. true if one was returned; false when nothing is pending.
 * now_recv format:  [count:1] then per-msg [mac:6][len:2 LE][data].     */
static int cf_next(uint8_t from[6], uint8_t *sub, const uint8_t **body, int *blen)
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
            if (len >= 2 && p[0] == CF_MAGIC) {
                memcpy(from, fm, 6);
                *sub  = p[1];
                *body = p + 2;
                *blen = len - 2;
                return 1;
            }
            /* not one of ours — skip */
        }
    }
}

/* Broadcast the full board to the opponent, a few times for reliability. */
static void cf_send_state(const uint8_t *mac, const uint8_t *board,
                          uint8_t turn, uint8_t mc, uint8_t status)
{
    uint8_t m[2 + COLS * ROWS + 3];
    m[0] = CF_MAGIC; m[1] = CF_STATE;
    memcpy(&m[2], board, COLS * ROWS);
    m[2 + COLS * ROWS + 0] = turn;
    m[2 + COLS * ROWS + 1] = mc;
    m[2 + COLS * ROWS + 2] = status;
    for (int i = 0; i < 3; i++) {
        m1_esp_client_now_send(mac, m, sizeof(m));
        m1app_delay(8);
    }
}

/* Win / draw check. Scans every cell for a run of four in the four forward
 * directions. Returns ST_P1WIN / ST_P2WIN / ST_DRAW / ST_PLAY. */
static uint8_t cf_status(const uint8_t *b)
{
    static const int dr[4] = { 0, 1, 1, 1 };
    static const int dc[4] = { 1, 0, 1, -1 };
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            uint8_t v = b[r * COLS + c];
            if (v == CELL_EMPTY) continue;
            for (int d = 0; d < 4; d++) {
                int k;
                for (k = 1; k < 4; k++) {
                    int rr = r + dr[d] * k, cc = c + dc[d] * k;
                    if (rr < 0 || rr >= ROWS || cc < 0 || cc >= COLS) break;
                    if (b[rr * COLS + cc] != v) break;
                }
                if (k == 4) return (v == CELL_P1) ? ST_P1WIN : ST_P2WIN;
            }
        }
    }
    for (int c = 0; c < COLS; c++)
        if (b[c] == CELL_EMPTY) return ST_PLAY;   /* top row has room */
    return ST_DRAW;
}

/* Drop a disc into a column. Returns the landing row, or -1 if full. */
static int cf_drop(uint8_t *b, int col, uint8_t mark)
{
    for (int r = ROWS - 1; r >= 0; r--) {
        if (b[r * COLS + col] == CELL_EMPTY) {
            b[r * COLS + col] = mark;
            return r;
        }
    }
    return -1;
}

/* ------------------------------------------------------------------ */
/* Draw the board + HUD. `result` non-NULL draws the end-of-game overlay. */
static void cf_draw(u8g2_t *u8g2, const uint8_t *board, int cursor,
                    int show_cursor, const char *status_txt, const char *result)
{
    m1app_display_begin();
    do {
        u8g2_SetDrawColor(u8g2, 1);
        u8g2_SetFont(u8g2, u8g2_font_5x8_tr);

        /* status line */
        if (status_txt)
            u8g2_DrawStr(u8g2, 2, STATUS_Y, status_txt);

        /* column cursor marker (little triangle pointing at the board) */
        if (show_cursor) {
            int cx = BOARD_OX + cursor * CELL_W + CELL_W / 2;
            u8g2_DrawTriangle(u8g2, cx - 3, 9, cx + 3, 9, cx, BOARD_OY - 1);
        }

        /* board frame */
        u8g2_DrawFrame(u8g2, BOARD_OX - 1, BOARD_OY - 1,
                       COLS * CELL_W + 2, ROWS * CELL_H + 2);

        /* discs */
        for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) {
                int cx = BOARD_OX + c * CELL_W + CELL_W / 2;
                int cy = BOARD_OY + r * CELL_H + CELL_H / 2;
                uint8_t v = board[r * COLS + c];
                if (v == CELL_P1)
                    u8g2_DrawDisc(u8g2, cx, cy, DISC_R, U8G2_DRAW_ALL);
                else if (v == CELL_P2)
                    u8g2_DrawCircle(u8g2, cx, cy, DISC_R, U8G2_DRAW_ALL);
                else
                    u8g2_DrawPixel(u8g2, cx, cy);   /* faint empty marker */
            }
        }

        /* end-of-game overlay */
        if (result) {
            int w = 104, x = (128 - w) / 2, y = 19, h = 28;
            u8g2_SetDrawColor(u8g2, 0);
            u8g2_DrawBox(u8g2, x, y, w, h);
            u8g2_SetDrawColor(u8g2, 1);
            u8g2_DrawFrame(u8g2, x, y, w, h);
            u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
            int tw = u8g2_GetStrWidth(u8g2, result);
            u8g2_DrawStr(u8g2, (128 - tw) / 2, y + 12, result);
            u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
            const char *hint = "OK=rematch  Back=exit";
            int hw = u8g2_GetStrWidth(u8g2, hint);
            u8g2_DrawStr(u8g2, (128 - hw) / 2, y + 24, hint);
        }
    } while (m1app_display_flush());
}

/* ------------------------------------------------------------------ */
/* Lobby: discover peers and pair up. On success fills mac[6] and sets
 * *is_p1 (inviter=P1, first move). Returns 0 if the user backs out.      */
static int cf_lobby(const uint8_t my_mac[6], uint8_t mac[6], int *is_p1)
{
    u8g2_t *u8g2 = m1app_get_u8g2();
    m1app_now_peer_t peers[CF_MAX_PEERS];
    int      npeers = 0, sel = 0;
    uint32_t last_poll = 0;

    for (;;) {
        uint32_t now = m1app_get_tick();

        /* someone invited us -> accept and play P2 */
        uint8_t from[6], sub; const uint8_t *body; int blen;
        while (cf_next(from, &sub, &body, &blen)) {
            if (sub == CF_INVITE) {
                uint8_t ack[2] = { CF_MAGIC, CF_ACCEPT };
                m1_esp_client_now_send(from, ack, sizeof(ack));
                memcpy(mac, from, 6);
                *is_p1 = 0;
                return 1;
            }
        }

        if ((now - last_poll) >= 500) {
            last_poll = now;
            m1_esp_client_now_announce();
            int n = m1_esp_client_now_get_peers(peers, CF_MAX_PEERS);
            npeers = (n > 0) ? n : 0;
            if (sel >= npeers) sel = npeers ? npeers - 1 : 0;
        }

        /* draw lobby */
        m1app_display_begin();
        do {
            u8g2_SetDrawColor(u8g2, 1);
            u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
            u8g2_DrawStr(u8g2, 2, 9, "Connect Four");
            u8g2_SetFont(u8g2, u8g2_font_5x8_tr);
            char hdr[24];
            snprintf(hdr, sizeof(hdr), "Pick opponent (%d)", npeers);
            u8g2_DrawStr(u8g2, 2, 20, hdr);

            int y = 31;
            for (int i = 0; i < npeers && i < 3; i++) {
                char line[28];
                snprintf(line, sizeof(line), "%c%s %02X %ddB",
                         (i == sel) ? '>' : ' ',
                         peers[i].name, peers[i].mac[5], peers[i].rssi);
                u8g2_DrawStr(u8g2, 2, y, line);
                y += 10;
            }
            if (npeers == 0)
                u8g2_DrawStr(u8g2, 2, 31, " (searching...)");

            u8g2_SetDrawColor(u8g2, 1);
            u8g2_DrawBox(u8g2, 0, 53, 128, 11);
            u8g2_SetDrawColor(u8g2, 0);
            u8g2_DrawStr(u8g2, 3, 62, "OK:Invite");
            u8g2_DrawStr(u8g2, 98, 62, "Back");
            u8g2_SetDrawColor(u8g2, 1);
        } while (m1app_display_flush());

        /* input */
        m1app_button_t btn = game_poll_button(80);
        if (btn == M1APP_BTN_BACK)  return 0;
        if (btn == M1APP_BTN_UP   && sel > 0)            sel--;
        if (btn == M1APP_BTN_DOWN && sel < npeers - 1)   sel++;
        if (btn == M1APP_BTN_OK   && npeers > 0) {
            /* invite selected peer; wait ~2.5s for an ACCEPT -> we're P1 */
            memcpy(mac, peers[sel].mac, 6);
            uint8_t inv[2] = { CF_MAGIC, CF_INVITE };
            uint32_t t0 = m1app_get_tick();
            int accepted = 0;
            while (!accepted && (m1app_get_tick() - t0) < 2500) {
                m1_esp_client_now_send(mac, inv, sizeof(inv));
                uint32_t w = m1app_get_tick();
                while (!accepted && (m1app_get_tick() - w) < 350) {
                    uint8_t fm[6], sb; const uint8_t *bd; int bl;
                    while (cf_next(fm, &sb, &bd, &bl)) {
                        if (sb == CF_ACCEPT && memcmp(fm, mac, 6) == 0) {
                            accepted = 1;
                        } else if (sb == CF_INVITE && memcmp(fm, mac, 6) == 0
                                   && memcmp(my_mac, fm, 6) > 0) {
                            /* both invited: higher MAC yields and plays P2 */
                            uint8_t ack[2] = { CF_MAGIC, CF_ACCEPT };
                            m1_esp_client_now_send(fm, ack, sizeof(ack));
                            *is_p1 = 0;
                            return 1;
                        }
                    }
                    m1app_delay(10);
                }
            }
            if (accepted) { *is_p1 = 1; return 1; }
            m1_message_box(u8g2, "Connect Four", "No answer", peers[sel].name, " OK ");
        }
    }
}

/* ------------------------------------------------------------------ */
static void cf_play(const uint8_t *mac, int is_p1)
{
    u8g2_t *u8g2 = m1app_get_u8g2();
    uint8_t board[COLS * ROWS];
    uint8_t turn      = CELL_P1;          /* P1 always moves first */
    uint8_t movecount = 0;
    uint8_t status    = ST_PLAY;
    uint8_t my_mark   = is_p1 ? CELL_P1 : CELL_P2;
    int     cursor    = COLS / 2;         /* center column */

    memset(board, 0, sizeof(board));

    if (is_p1) cf_send_state(mac, board, turn, movecount, status);   /* kick off */

    for (;;) {
        /* ingest opponent messages */
        uint8_t from[6], sub; const uint8_t *body; int blen;
        while (cf_next(from, &sub, &body, &blen)) {
            if (memcmp(from, mac, 6) != 0) continue;   /* opponent only */
            if (sub == CF_QUIT) {
                m1_message_box(u8g2, "Connect Four", "Opponent", "left", " OK ");
                return;
            }
            if (sub == CF_STATE && blen >= COLS * ROWS + 3) {
                uint8_t mc = body[COLS * ROWS + 1];
                if (mc == 0 || mc > movecount) {   /* newer state or reset */
                    memcpy(board, body, COLS * ROWS);
                    turn      = body[COLS * ROWS + 0];
                    movecount = mc;
                    status    = body[COLS * ROWS + 2];
                    if (mc == 0) cursor = COLS / 2;
                }
            }
        }

        int my_turn = (status == ST_PLAY && turn == my_mark);

        char stbuf[24];
        const char *st;
        if (status == ST_PLAY) {
            snprintf(stbuf, sizeof(stbuf), "P%d (%s) %s",
                     is_p1 ? 1 : 2, is_p1 ? "disc" : "ring",
                     my_turn ? "- your turn" : "- wait");
            st = stbuf;
        } else {
            st = "Game over";
        }

        const char *result = 0;
        if      (status == ST_P1WIN) result = (my_mark == CELL_P1) ? "You win!" : "You lose";
        else if (status == ST_P2WIN) result = (my_mark == CELL_P2) ? "You win!" : "You lose";
        else if (status == ST_DRAW)  result = "Draw";

        cf_draw(u8g2, board, cursor, my_turn, st, result);

        /* input */
        m1app_button_t btn = game_poll_button(60);
        if (btn == M1APP_BTN_NONE) continue;

        if (btn == M1APP_BTN_BACK) {
            uint8_t q[2] = { CF_MAGIC, CF_QUIT };
            m1_esp_client_now_send(mac, q, sizeof(q));
            return;
        }

        if (status != ST_PLAY) {
            if (btn == M1APP_BTN_OK) {
                /* rematch: fresh game, P1 first, tell the peer */
                memset(board, 0, sizeof(board));
                turn = CELL_P1; movecount = 0; status = ST_PLAY;
                cursor = COLS / 2;
                cf_send_state(mac, board, turn, movecount, status);
                m1_buzzer_notification();
            }
            continue;
        }

        if (!my_turn) continue;

        if (btn == M1APP_BTN_LEFT)  cursor = (cursor + COLS - 1) % COLS;
        else if (btn == M1APP_BTN_RIGHT) cursor = (cursor + 1) % COLS;
        else if (btn == M1APP_BTN_OK) {
            if (cf_drop(board, cursor, my_mark) >= 0) {
                movecount++;
                turn   = (my_mark == CELL_P1) ? CELL_P2 : CELL_P1;
                status = cf_status(board);
                cf_send_state(mac, board, turn, movecount, status);
                m1_buzzer_notification();
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Local "pass & play" — both players share one device, no radio. */
static void cf_local(void)
{
    u8g2_t *u8g2 = m1app_get_u8g2();
    uint8_t board[COLS * ROWS];
    uint8_t turn   = CELL_P1;
    uint8_t status = ST_PLAY;
    int     cursor = COLS / 2;

    memset(board, 0, sizeof(board));

    for (;;) {
        char stbuf[24];
        const char *st;
        if (status == ST_PLAY) {
            snprintf(stbuf, sizeof(stbuf), "P%d (%s) - your move",
                     turn, (turn == CELL_P1) ? "disc" : "ring");
            st = stbuf;
        } else {
            st = "Game over";
        }

        const char *result = 0;
        if      (status == ST_P1WIN) result = "P1 (disc) wins!";
        else if (status == ST_P2WIN) result = "P2 (ring) wins!";
        else if (status == ST_DRAW)  result = "Draw";

        cf_draw(u8g2, board, cursor, status == ST_PLAY, st, result);

        m1app_button_t btn = game_poll_button(80);
        if (btn == M1APP_BTN_NONE) continue;
        if (btn == M1APP_BTN_BACK) return;

        if (status != ST_PLAY) {
            if (btn == M1APP_BTN_OK) {
                memset(board, 0, sizeof(board));
                turn = CELL_P1; status = ST_PLAY; cursor = COLS / 2;
                m1_buzzer_notification();
            }
            continue;
        }

        if (btn == M1APP_BTN_LEFT)  cursor = (cursor + COLS - 1) % COLS;
        else if (btn == M1APP_BTN_RIGHT) cursor = (cursor + 1) % COLS;
        else if (btn == M1APP_BTN_OK) {
            if (cf_drop(board, cursor, turn) >= 0) {
                status = cf_status(board);
                turn   = (turn == CELL_P1) ? CELL_P2 : CELL_P1;
                m1_buzzer_notification();
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Choose peer link vs local play. Returns 0=peer, 1=local, -1=exit.  */
static int cf_mode_select(void)
{
    u8g2_t *u8g2 = m1app_get_u8g2();
    static const char *opt[2] = { "Peer Link",       "Pass & Play"    };
    static const char *sub[2] = { "2 M1s, ESP-NOW",  "1 M1, hot-seat" };
    int sel = 0;

    for (;;) {
        m1app_display_begin();
        do {
            u8g2_SetDrawColor(u8g2, 1);
            u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
            u8g2_DrawStr(u8g2, 2, 12, "Connect Four");

            for (int i = 0; i < 2; i++) {
                int y = 26 + i * 18;
                if (i == sel) {
                    u8g2_SetDrawColor(u8g2, 1);
                    u8g2_DrawBox(u8g2, 0, y - 9, 128, 18);
                    u8g2_SetDrawColor(u8g2, 0);
                }
                u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
                u8g2_DrawStr(u8g2, 6, y, opt[i]);
                u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
                u8g2_DrawStr(u8g2, 6, y + 8, sub[i]);
                u8g2_SetDrawColor(u8g2, 1);
            }
        } while (m1app_display_flush());

        m1app_button_t btn = game_poll_button(120);
        if (btn == M1APP_BTN_BACK) return -1;
        if (btn == M1APP_BTN_UP   && sel > 0) sel--;
        if (btn == M1APP_BTN_DOWN && sel < 1) sel++;
        if (btn == M1APP_BTN_OK) return sel;
    }
}

/* ------------------------------------------------------------------ */
int32_t app_main(void *context)
{
    (void)context;
    u8g2_t *u8g2 = m1app_get_u8g2();
    game_rand_seed();

    for (;;) {
        int mode = cf_mode_select();
        if (mode < 0) break;                 /* backed out of the menu */

        if (mode == 1) {                     /* local pass & play */
            cf_local();
            continue;
        }

        /* peer link — bring up the ESP-NOW radio */
        uint8_t my_mac[6];
        if (!m1_esp_client_ping()) {
            m1_message_box(u8g2, "Connect Four", "ESP32 not ready",
                           "Initialize it first", " OK ");
            continue;
        }
        if (!m1_esp_client_now_start(CF_CHANNEL, "M1-C4", my_mac)) {
            m1_message_box(u8g2, "Connect Four", "ESP-NOW start", "failed", " OK ");
            continue;
        }

        for (;;) {
            uint8_t mac[6];
            int is_p1 = 0;
            if (!cf_lobby(my_mac, mac, &is_p1))
                break;                       /* backed out of the lobby */
            cf_play(mac, is_p1);             /* returns when the match ends */
        }

        m1_esp_client_now_stop();
    }

    return 0;
}
