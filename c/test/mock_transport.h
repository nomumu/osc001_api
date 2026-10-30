/* モックトランスポート。
 * 実機で確定した応答挙動を再現する:
 *   - 全 op は byte0 に ACK エコー(op8 の有効索引は byte0=chunk 索引、
 *     無効索引は byte0=0x08)
 *   - op2 は "DPScope SE"、op6 は byte0=1、op8 chunk6 応答 [38..63] にフッター語
 * 仮想時計(sleep_ms が進める)なのでテストは実時間で待たない。 */
#ifndef OSC001_MOCK_TRANSPORT_H
#define OSC001_MOCK_TRANSPORT_H

#include <string.h>
#include "../include/osc001.h"

typedef struct {
    osc001_transport_t t;
    /* 制御 */
    int fill_ff;             /* op8 データを 0xFF 埋めにする */
    int la_mode;             /* op8 データを 0xA5 ニブルパターンにする */
    uint16_t footer_base;    /* chunk6 フッター語の基準値 */
    int op6_fire_after;      /* op5 がこの回数に達するまで op6 は 0(トリガ未成立)。0 = 常に発火 */
    int mon_enable;          /* op9 に mon[] の値を返す(0 ならエコーのみ) */
    int mon[16];             /* op9 セレクタ → m */
    int op8_const;           /* 0 以外なら op8 データを全てこの値にする(code = 値 − 0x7F) */
    int n_op9;
    /* 観測 */
    int n_send;
    uint8_t last_report[OSC001_REPORT_LEN];
    uint8_t last_op5[20];    /* 直近 op5 の byte0..19 */
    int op5_count;
    uint8_t last_op16[10];   /* 直近 op16 の byte0..9 */
    /* 内部 */
    uint64_t vtime;
    uint8_t resp[OSC001_REPORT_LEN];
    int resp_ready;
} mock_dev;

static int mock_open(void *u) { (void)u; return 0; }
static void mock_close(void *u) { (void)u; }

static int mock_send(void *u, const uint8_t *report)
{
    mock_dev *m = (mock_dev *)u;
    int i;
    memcpy(m->last_report, report, OSC001_REPORT_LEN);
    m->n_send++;
    m->vtime += 1;   /* 仮想時計を進める(arm のタイムスタンプが 0 と衝突しないように) */
    memset(m->resp, 0, OSC001_REPORT_LEN);
    m->resp[0] = report[0];
    switch (report[0]) {
    case 2:
        memcpy(m->resp + 1, "DPScope SE", 10);
        memcpy(m->resp + 11, "@x", 2);   /* byte11 以降は前の応答の残骸(実機でも 0 とは限らない) */
        break;
    case 3:
        m->resp[1] = 0x05; m->resp[2] = 0x01;   /* V1.05 相当(値は固定のダミー) */
        break;
    case 5:
        memcpy(m->last_op5, report, 20);
        m->op5_count++;
        break;
    case 6:
        m->resp[0] = (m->op5_count >= m->op6_fire_after) ? 1 : 0;   /* 取得完了/発火 */
        break;
    case 8: {
        int c = report[1];
        if (c <= 6) {
            if (m->fill_ff) {                    /* : chunk が 0xFF で埋まる */
                memset(m->resp, 0xFF, OSC001_REPORT_LEN);
                m->resp_ready = 1;
                return 0;
            }
            m->resp[0] = m->op8_const ? (uint8_t)m->op8_const   /* 一定値モードは byte0 も同じ値 */
                                      : (uint8_t)c;             /* 有効索引: byte0=chunk 索引 */
            for (i = 1; i < OSC001_REPORT_LEN; i++)
                m->resp[i] = m->op8_const ? (uint8_t)m->op8_const :
                             m->la_mode ? 0xA5 : (uint8_t)((c * 64 + i) & 0xFF);
            if (c == 6) {                        /* フッター本体 */
                for (i = 0; i < OSC001_FOOTER_WORDS; i++) {
                    uint16_t wv = (uint16_t)(m->footer_base + i);
                    m->resp[OSC001_FOOTER_OFFSET + 2 * i] = (uint8_t)(wv & 0xFF);
                    m->resp[OSC001_FOOTER_OFFSET + 2 * i + 1] = (uint8_t)(wv >> 8);
                }
            }
        } else {
            m->resp[0] = OSC001_OP_ECHO;         /* 無効索引 */
            for (i = 1; i < OSC001_REPORT_LEN; i++)
                m->resp[i] = (uint8_t)((6 * 64 + i) & 0xFF);
        }
        break;
    }
    case 9:
        m->n_op9++;
        if (m->mon_enable) {
            int a = m->mon[report[1] & 15], b = m->mon[report[2] & 15];
            m->resp[0] = (uint8_t)(a >> 8); m->resp[1] = (uint8_t)a;
            m->resp[2] = (uint8_t)(b >> 8); m->resp[3] = (uint8_t)b;
        }
        break;
    case 16:
        memcpy(m->last_op16, report, 10);
        break;
    default:
        break;
    }
    m->resp_ready = 1;
    return 0;
}

static int mock_recv(void *u, uint8_t *report, int timeout_ms)
{
    mock_dev *m = (mock_dev *)u;
    (void)timeout_ms;
    if (!m->resp_ready)
        return 0;
    memcpy(report, m->resp, OSC001_REPORT_LEN);
    m->resp_ready = 0;
    return 1;
}

static uint64_t mock_now_ms(void *u) { return ((mock_dev *)u)->vtime; }
static void mock_sleep_ms_(int ms) { /* now_ms の user が使えないので send/recv 側で進める */
    (void)ms;
}

static void mock_init(mock_dev *m)
{
    memset(m, 0, sizeof *m);
    m->footer_base = 0x1000;
    m->t.open = mock_open;
    m->t.close = mock_close;
    m->t.send = mock_send;
    m->t.recv = mock_recv;
    m->t.now_ms = mock_now_ms;
    m->t.sleep_ms = mock_sleep_ms_;
    m->t.open_errmsg = 0;
    m->t.user = m;
}

#endif /* OSC001_MOCK_TRANSPORT_H */
