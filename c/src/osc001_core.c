/* libosc001 コア — プロトコル+スコープ層。OS 非依存(C99 のみ、トランスポート注入)。
 * python/osc001.py(実機検証済み)の移植。数式・ byte 配置・ガードは同一。
 * 相違点: stale 判定のハッシュは md5 でなく FNV-1a(セッション内一貫性が
 * あれば足りるため。stale 判定自体はヒューリスティック)。 */

#include "osc001.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── 表──────────────────────────── */

static const uint8_t DEFAULT_BYTES[20] = {
    /* byte0 は op。送信時に立てる */
    0x00,
    0x05,  /* byte1  CH1 タップ(5=÷8、6=×1) */
    0x08,  /* byte2  CH2 タップ(8=÷8、9=×1) */
    0xAA,  /* byte3  取得イネーブル */
    0xFF,  /* byte4  時間基軸上位 ┐ X=65536−(b4·256+b5) */
    0x00,  /* byte5  時間基軸下位 ┘ */
    0x00,  /* byte6  bit0=1 で除数 X/2(byte7 無視) */
    0x00,  /* byte7  2^(byte7&7) だけ追加分周(下位 3 ビットのみ有効) */
    0x01,  /* byte8  CH1 ゲイン段 0..5 */
    0x01,  /* byte9  CH2 ゲイン段 0..5 */
    0x80,  /* byte10 CH1 Position */
    0x80,  /* byte11 CH2 Position */
    0x00,  /* byte12 ハードトリガ 0=フリーラン / 1=トリガ待ち */
    0x00,  /* byte13 トリガ極性 0=立下り / 1=立上り */
    0x00,  /* byte14 */
    0x4D,  /* byte15 トリガレベル(段・Position の前の値 m/2 − 128 と比較) */
    0x00, 0x0A, 0x05,            /* byte16..18 ET モード/2MHz 分周比/その半分 */
    0x01   /* byte19 トリガ経路(下位 2 ビット): 1=CH1 ÷8 / 2=CH1 ×1 / 3=EXT */
};

/* (V/div, tap, ゲイン段) — 粗い順 */
typedef struct { double vdiv; int tap; int gain; } range_row_t;
static const range_row_t RANGES[6] = {
    { 2.0, 8, 2 }, { 1.0, 8, 1 }, { 0.5, 8, 0 },
    { 0.2, 1, 2 }, { 0.1, 1, 1 }, { 0.05, 1, 0 }
};

/* 0 V のときの m(op9 の値)の既定。[ch-1][tap: 0=×1, 1=÷8]。2026-10-18 の実機 */
static const double M0_DEFAULT[2][2] = {
    { 379.3, 495.0 },   /* CH1 */
    { 388.0, 496.5 }    /* CH2 */
};
static const int MON_SEL[2][2] = { { 6, 5 }, { 9, 8 } };   /* [ch-1][tap: 0=×1, 1=÷8] */
/* レンジ別の既定 Position 補正(掛けないと段 0 で上レールに張り付く) */
static const int POSITION_CORR[6] = { 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00 }; /* 段 0..5 */
/* 中間レベルが code0 へ来る byte 値。段 3..5 は到達不能 */
static const int POSITION_CENTER[3] = { 210, 146, 19 };  /* 段 0/1/2(round 済み) */
/* 0 V のコード = m0/2^段 − 127 + POSITION_SLOPE[段]·Position */
static const double POSITION_SLOPE[6] = { -2.0, -1.0, -0.5, -0.25, -0.125, -0.0625 };
#define X1_CLAMP_V 3.35
#define TRIG_AUTO_MIN_MS 100
#define BYTE3_REQUIRED 0xAA

/* LA */
#define LA_B47_DEFAULT 0x0001BEFFu  /* byte4..7(LE ff be 01 00)。実意味不明 */

/* フッター */
static const int FOOTER_PARITY_IDX[2] = { 5, 7 };
static const int FOOTER_FP_IDX[10] = { 0, 1, 2, 3, 4, 6, 8, 9, 10, 12 };

/* ── handle ───────────────────────────────────────────────────────── */

struct osc001_handle {
    osc001_transport_t t;
    int timeout_ms;
    uint8_t b[20];              /* op5 ペイロード(byte1..19 を使用) */
    uint64_t armed_at;          /* 0=未 arm */
    int ready_seen;             /* arm 後に op6 の完了を見た */
    double m0[2][2];            /* 0 V の m。[ch-1][tap: 0=×1, 1=÷8] */
    double m0_gain[2][2][6];    /* レンジ別の実測(calibrate_zero)。m0_gain_set が立っていれば優先 */
    int m0_gain_set[2][2][6];
    osc001_trigger_t trig;      /* source=NONE なら byte12/13/15/19 に触らない */
    /* LA */
    int la_configured;
    uint8_t la_frame[9];        /* op16 の args=wire byte1..9 */
    uint64_t la_armed_at;
    int la_armed;               /* la_arm 済みで、op5/abort をまだ挟んでいない */
    int la_edge_pending;        /* 未発火のエッジ arm が残っている(op5 前に解除が要る) */
    uint8_t la_b3;              /* op16 byte3(レート段 3/2/1、0=byte4..7 の除数を使う) */
    uint8_t la_b47[4];          /* op16 byte4..7 */
    double la_rate;             /* 実効 fs */
    int la_fired;
    /* read_deep のフッター指紋状態 */
    int fp_have_prev;
    uint16_t fp_prev_parity[2];
    uint64_t fp_prev_hash;
    int fp_have_base;
    uint16_t fp_base[10];
    char err[256];
};

static void set_err(osc001_handle *h, const char *msg)
{
    strncpy(h->err, msg, sizeof h->err - 1);
    h->err[sizeof h->err - 1] = '\0';
}

const char *osc001_errmsg(const osc001_handle *h) { return h->err; }
void osc001_set_timeout(osc001_handle *h, int ms) { h->timeout_ms = ms; }

/* ── open/close ───────────────────────────────────────────────────── */

osc001_handle *osc001_open(const osc001_transport_t *transport,
                           char *errbuf, size_t errlen)
{
    osc001_handle *h;
    if (errbuf && errlen) errbuf[0] = '\0';
    if (!transport || !transport->send || !transport->recv ||
        !transport->now_ms || !transport->sleep_ms) {
        if (errbuf && errlen)
            snprintf(errbuf, errlen, "トランスポートのコールバック不足");
        return NULL;
    }
    h = (osc001_handle *)malloc(sizeof *h);
    if (!h) {
        if (errbuf && errlen)
            snprintf(errbuf, errlen, "メモリ確保失敗");
        return NULL;
    }
    memset(h, 0, sizeof *h);
    h->t = *transport;
    h->timeout_ms = 2000;
    memcpy(h->b, DEFAULT_BYTES, sizeof h->b);
    memcpy(h->m0, M0_DEFAULT, sizeof h->m0);
    osc001_trigger_init(&h->trig);
    osc001_la_set_rate(h, OSC001_LA_RATE_HZ);
    if (h->t.open && h->t.open(h->t.user) != 0) {
        const char *why = (h->t.open_errmsg ? h->t.open_errmsg(h->t.user) : NULL);
        if (errbuf && errlen)
            snprintf(errbuf, errlen, "open 失敗: %s", why ? why : "(理由不明)");
        free(h);
        return NULL;
    }
    /* 残応答のドレイン(Python _drain(0.2) と同じ) */
    uint8_t junk[OSC001_REPORT_LEN];
    while (h->t.recv(h->t.user, junk, 200) == 1)
        ;
    return h;
}

void osc001_close(osc001_handle *h)
{
    if (!h)
        return;
    if (h->t.close)
        h->t.close(h->t.user);
    free(h);
}

/* ── 低レベルトランザクション ─────────────────────────────────────── */

static int txn(osc001_handle *h, int op, const uint8_t *args, int nargs,
               uint8_t *resp, int timeout_ms)
{
    uint8_t buf[OSC001_REPORT_LEN];
    int i;
    if (nargs > OSC001_REPORT_LEN - 1)
        return OSC001_ERR_ARG;
    memset(buf, 0, sizeof buf);
    buf[0] = (uint8_t)op;
    for (i = 0; i < nargs; i++)
        buf[1 + i] = args[i];
    if (h->t.send(h->t.user, buf) != 0) {
        set_err(h, "write 失敗");
        return OSC001_ERR_IO;
    }
    int r = h->t.recv(h->t.user, resp, timeout_ms);
    if (r == 0)
        return OSC001_ERR_TIMEOUT;
    if (r < 0) {
        set_err(h, "read 失敗");
        return OSC001_ERR_IO;
    }
    return OSC001_OK;
}

int osc001_raw_command(osc001_handle *h, int op,
                       const uint8_t *args, int nargs, uint8_t *resp64)
{
    uint8_t local[OSC001_REPORT_LEN];
    int r = txn(h, op, args, nargs, local, h->timeout_ms);
    if (resp64)
        memcpy(resp64, local, OSC001_REPORT_LEN);
    return r;
}

/* ── 情報 ─────────────────────────────────────────────────────────── */

int osc001_info(osc001_handle *h, char *out, size_t outlen)
{
    uint8_t r[OSC001_REPORT_LEN];
    int rc = txn(h, 2, NULL, 0, r, h->timeout_ms);
    if (rc != OSC001_OK) {
        set_err(h, "op2 無応答");
        return rc;
    }
    size_t k = 0;
    for (int i = 1; i <= OSC001_INFO_LEN && r[i] != 0; i++) {
        if (r[i] >= 32 && r[i] < 127 && k + 1 < outlen)
            out[k++] = (char)r[i];
    }
    if (outlen)
        out[k < outlen ? k : outlen - 1] = '\0';
    return OSC001_OK;
}

int osc001_version(osc001_handle *h, int *byte1, int *byte2)
{
    uint8_t r[OSC001_REPORT_LEN];
    int rc = txn(h, 3, NULL, 0, r, h->timeout_ms);
    if (rc != OSC001_OK) {
        set_err(h, "op3 無応答");
        return rc;
    }
    if (byte1) *byte1 = r[1];
    if (byte2) *byte2 = r[2];
    return OSC001_OK;
}

/* ── 設定 ─────────────────────────────────────────────────────────── */

int osc001_set_raw(osc001_handle *h, int index, int byte_value)
{
    if (index < 1 || index > 19) {
        set_err(h, "index は 1..19");
        return OSC001_ERR_ARG;
    }
    h->b[index] = (uint8_t)(byte_value & 0xFF);
    return OSC001_OK;
}

int osc001_get_raw(const osc001_handle *h, int index)
{
    if (index < 1 || index > 19)
        return -1;
    return h->b[index];
}

static int ch_index(int ch) { return (ch == 1) ? 0 : 1; }

static int tap_of_byte(int tapbyte)   /* 1=×1、8=÷8、-1=使用不可値 */
{
    if (tapbyte == 6 || tapbyte == 9) return 1;
    if (tapbyte == 5 || tapbyte == 8) return 8;
    return -1;
}

int osc001_set_position(osc001_handle *h, int ch, int byte_value)
{
    if (ch != 1 && ch != 2) { set_err(h, "ch は 1/2"); return OSC001_ERR_ARG; }
    h->b[(ch == 1) ? 10 : 11] = (uint8_t)(byte_value & 0xFF);
    return OSC001_OK;
}

static int apply_tap(osc001_handle *h, int ch, int tap)
{
    /* TAPBYTE: ch1: tap1→6 tap8→5 / ch2: tap1→9 tap8→8(添字は tap 値 1/8) */
    static const int TAPBYTE[2][9] = {
        { 0, 6, 0, 0, 0, 0, 0, 0, 5 },
        { 0, 9, 0, 0, 0, 0, 0, 0, 8 }
    };
    if (tap != 1 && tap != 8) { set_err(h, "tap は 1(×1)か 8(÷8)"); return OSC001_ERR_ARG; }
    h->b[(ch == 1) ? 1 : 2] = (uint8_t)TAPBYTE[ch_index(ch)][tap];
    return OSC001_OK;
}

int osc001_set_gain(osc001_handle *h, int ch, int gain, int tap, int position)
{
    if (ch != 1 && ch != 2) { set_err(h, "ch は 1/2"); return OSC001_ERR_ARG; }
    if (gain < 0 || gain > 5) { set_err(h, "gain は 0..5"); return OSC001_ERR_ARG; }
    h->b[(ch == 1) ? 8 : 9] = (uint8_t)gain;
    if (tap != 0) {
        int rc = apply_tap(h, ch, tap);
        if (rc != OSC001_OK) return rc;
    }
    if (position == OSC001_POS_AUTO) {
        h->b[(ch == 1) ? 10 : 11] = (uint8_t)POSITION_CORR[gain];
    } else if (position == OSC001_POS_CENTER) {
        if (gain > 2) {
            set_err(h, "段 3..5 は Position で中央(code 0)へ到達不能");
            return OSC001_ERR_ARG;
        }
        h->b[(ch == 1) ? 10 : 11] = (uint8_t)POSITION_CENTER[gain];
    } else if (position != OSC001_POS_KEEP) {
        h->b[(ch == 1) ? 10 : 11] = (uint8_t)(position & 0xFF);
    }
    return OSC001_OK;
}

int osc001_set_range(osc001_handle *h, int ch, double v_per_div,
                     int tap, int position)
{
    int i, found = -1;
    for (i = 0; i < 6; i++) {
        if (fabs(RANGES[i].vdiv - v_per_div) < 1e-9) { found = i; break; }
    }
    if (found < 0) {
        set_err(h, "range は 6 ラベル(2.0/1.0/0.5/0.2/0.1/0.05)のいずれか");
        return OSC001_ERR_ARG;
    }
    int t = RANGES[found].tap;
    if (tap != 0) {
        if (tap != 1 && tap != 8) { set_err(h, "tap は 0(自動)/1(×1)/8(÷8)"); return OSC001_ERR_ARG; }
        t = tap;
    }
    return osc001_set_gain(h, ch, RANGES[found].gain, t, position);
}

/* fs 式 */
static double x_value(const osc001_handle *h)
{
    return (double)(0x10000u - (unsigned)(h->b[4] * 256 + h->b[5]));
}

static double divisor(const osc001_handle *h)
{
    double x = x_value(h);
    if (h->b[6] & 1)
        return x / 2.0;
    return (double)(1 << (h->b[7] & 0x07)) * x;
}

double osc001_sample_rate(const osc001_handle *h)
{
    double d = divisor(h);
    if (d < OSC001_D_MIN) d = OSC001_D_MIN;
    double fs = OSC001_CLOCK_HZ / (d + OSC001_RATE_OFFSET);
    return (fs > OSC001_RATE_MAX) ? OSC001_RATE_MAX : fs;
}

/* Hz 指定 → 最も近い (fs, b4, b5, b6, b7)。python rate_settings と同一手順 */
static int rate_settings(double hz, double *fs_out,
                         int *b4, int *b5, int *b6, int *b7)
{
    double target;
    int have = 0, bi4 = 0, bi5 = 0, bi6 = 0, bi7 = 0, bxc = 0;
    double bkey1 = 0.0, bfs = 0.0;
    int i6, i7, k;
    if (!(hz > 0.0))
        return OSC001_ERR_ARG;
    target = OSC001_CLOCK_HZ / hz - OSC001_RATE_OFFSET;
    if (target < OSC001_D_MIN) target = OSC001_D_MIN;
    for (i6 = 0; i6 <= 1; i6++) {
        for (i7 = 0; i7 <= 7; i7++) {
            double shift = i6 ? 0.5 : (double)(1 << i7);
            long x = lround(target / shift);
            if (x < 1) x = 1;
            if (x > 65280) x = 65280;
            for (k = -1; k <= 1; k++) {
                long xc = x + k;
                double d, fs, key1;
                unsigned v;
                if (xc < 1 || xc > 65280) continue;
                d = shift * (double)xc;
                if (d < OSC001_D_MIN) d = OSC001_D_MIN;
                fs = OSC001_CLOCK_HZ / (d + OSC001_RATE_OFFSET);
                if (fs > OSC001_RATE_MAX) fs = OSC001_RATE_MAX;
                key1 = fabs(fs - hz);
                /* key=(|fs−hz|, −xc): 同率なら X の大きい(目目の細かい)組 */
                if (!have || key1 < bkey1 ||
                    (key1 == bkey1 && -(long)xc < (long)-bxc)) {
                    have = 1;
                    bkey1 = key1; bfs = fs; bxc = (int)xc;
                    v = (0x10000u - (unsigned)xc) & 0xFFFFu;
                    bi4 = (int)(v >> 8); bi5 = (int)(v & 0xFF);
                    bi6 = i6; bi7 = i7;
                }
            }
        }
    }
    if (!have)
        return OSC001_ERR_ARG;
    *fs_out = bfs; *b4 = bi4; *b5 = bi5; *b6 = bi6; *b7 = bi7;
    return OSC001_OK;
}

double osc001_set_sample_rate(osc001_handle *h, double hz)
{
    double fs;
    int b4, b5, b6, b7;
    int rc = rate_settings(hz, &fs, &b4, &b5, &b6, &b7);
    if (rc != OSC001_OK) {
        set_err(h, "hz が不正(>0)");
        return (double)rc;
    }
    h->b[4] = (uint8_t)b4; h->b[5] = (uint8_t)b5;
    h->b[6] = (uint8_t)b6; h->b[7] = (uint8_t)b7;
    return fs;
}

double osc001_v_per_lsb(const osc001_handle *h, int ch)
{
    int tap, gain;
    if (ch != 1 && ch != 2) return 0.0;
    tap = tap_of_byte(h->b[(ch == 1) ? 1 : 2]);
    gain = h->b[(ch == 1) ? 8 : 9];
    if (tap < 0 || gain > 5) return 0.0;
    return (tap == 8 ? OSC001_VCOUNT_DIV8 : OSC001_VCOUNT_X1) * (double)(1 << gain);
}

int osc001_zero_m0(const osc001_handle *h, int ch, int tap, int gain, double *m0)
{
    int ci, ti;
    if ((ch != 1 && ch != 2) || (tap != 1 && tap != 8) || gain < 0 || gain > 5)
        return OSC001_ERR_ARG;
    ci = ch_index(ch);
    ti = (tap == 8) ? 1 : 0;
    *m0 = h->m0_gain_set[ci][ti][gain] ? h->m0_gain[ci][ti][gain] : h->m0[ci][ti];
    return OSC001_OK;
}

int osc001_set_zero_m0(osc001_handle *h, int ch, int tap, double m0)
{
    int ci, ti;
    if ((ch != 1 && ch != 2) || (tap != 1 && tap != 8)) return OSC001_ERR_ARG;
    ci = ch_index(ch);
    ti = (tap == 8) ? 1 : 0;
    h->m0[ci][ti] = m0;
    memset(h->m0_gain_set[ci][ti], 0, sizeof h->m0_gain_set[ci][ti]);
    return OSC001_OK;
}

int osc001_zero_code(const osc001_handle *h, int ch, double *code)
{
    int tap, gain, rc;
    double m0;
    if (ch != 1 && ch != 2) return OSC001_ERR_ARG;
    tap = tap_of_byte(h->b[(ch == 1) ? 1 : 2]);
    gain = h->b[(ch == 1) ? 8 : 9];
    if (tap < 0) return OSC001_ERR_ARG;
    rc = osc001_zero_m0(h, ch, tap, gain, &m0);
    if (rc != OSC001_OK) return rc;
    *code = m0 / (double)(1 << gain) - 127.0 +
            POSITION_SLOPE[gain] * h->b[(ch == 1) ? 10 : 11];
    return OSC001_OK;
}

int osc001_volts_to_code(const osc001_handle *h, int ch, double volts, double *code)
{
    double z;
    int rc = osc001_zero_code(h, ch, &z);
    if (rc != OSC001_OK) return rc;
    *code = z + volts / osc001_v_per_lsb(h, ch);
    return OSC001_OK;
}

/* ── ハードトリガ ─────────────────────────────────────────────────── */

void osc001_trigger_init(osc001_trigger_t *cfg)
{
    memset(cfg, 0, sizeof *cfg);
    cfg->source = OSC001_TRIG_NONE;
    cfg->edge = OSC001_EDGE_RISING;
    cfg->level_raw = -1;
    cfg->mode = OSC001_TRIG_NORMAL;
}

/* cfg → byte12/13/15/19(python _trigger_bytes と同じ)。換算不能なら ERR_ARG と理由 */
static int trigger_bytes(const osc001_handle *h, const osc001_trigger_t *t,
                         uint8_t out[4], char *why, size_t whylen)
{
    int b13 = (t->edge == OSC001_EDGE_RISING) ? 1 : 0;
    if (t->source == OSC001_TRIG_EXT) {
        out[0] = 1; out[1] = (uint8_t)b13; out[2] = OSC001_TRIG_LEVEL_EXT; out[3] = 3;
        return OSC001_OK;
    }
    int tap = tap_of_byte(h->b[1]);
    if (tap < 0) {
        snprintf(why, whylen, "byte1=%d は使用不可のタップ値", h->b[1]);
        return OSC001_ERR_ARG;
    }
    int raw = t->level_raw;
    if (raw < 0) {
        /* 比較器は段・Position の前を見る: byte15 = m/2 − 128(段 1・Position 0x80 のサンプル値 + 0x7F) */
        double m = h->m0[ch_index(1)][(tap == 8) ? 1 : 0] +
                   t->level_v / ((tap == 8) ? OSC001_VCOUNT_DIV8 : OSC001_VCOUNT_X1);
        raw = (int)lround(m / 2.0) - 128;
        if (raw < OSC001_TRIG_LEVEL_MIN || raw > OSC001_TRIG_LEVEL_MAX) {
            snprintf(why, whylen, "トリガレベル %.3g V は比較器の範囲外(生バイト %d)",
                     t->level_v, raw);
            return OSC001_ERR_ARG;
        }
    }
    out[0] = 1; out[1] = (uint8_t)b13; out[2] = (uint8_t)raw;
    out[3] = (uint8_t)(tap == 8 ? 1 : 2);
    return OSC001_OK;
}

static int apply_trigger(osc001_handle *h)
{
    uint8_t tb[4];
    char why[160];
    int rc;
    if (h->trig.source == OSC001_TRIG_NONE)
        return OSC001_OK;
    rc = trigger_bytes(h, &h->trig, tb, why, sizeof why);
    if (rc != OSC001_OK) { set_err(h, why); return rc; }
    h->b[12] = tb[0]; h->b[13] = tb[1]; h->b[15] = tb[2]; h->b[19] = tb[3];
    return OSC001_OK;
}

int osc001_set_trigger(osc001_handle *h, const osc001_trigger_t *cfg)
{
    uint8_t tb[4];
    char why[160];
    int rc;
    if (!cfg || cfg->source == OSC001_TRIG_NONE) {
        osc001_trigger_init(&h->trig);
        h->b[12] = 0;
        return OSC001_OK;
    }
    if (cfg->source != OSC001_TRIG_CH1 && cfg->source != OSC001_TRIG_EXT) {
        set_err(h, "source は NONE / CH1 / EXT(CH2 のハードトリガは無い)");
        return OSC001_ERR_ARG;
    }
    if (cfg->edge != OSC001_EDGE_RISING && cfg->edge != OSC001_EDGE_FALLING) {
        set_err(h, "edge は RISING / FALLING");
        return OSC001_ERR_ARG;
    }
    if (cfg->mode != OSC001_TRIG_NORMAL && cfg->mode != OSC001_TRIG_AUTO) {
        set_err(h, "mode は NORMAL / AUTO");
        return OSC001_ERR_ARG;
    }
    if (cfg->auto_timeout_ms < 0) {
        set_err(h, "auto_timeout_ms は 0(既定)以上");
        return OSC001_ERR_ARG;
    }
    if (cfg->level_raw >= 0 &&
        (cfg->level_raw < OSC001_TRIG_LEVEL_MIN || cfg->level_raw > OSC001_TRIG_LEVEL_MAX)) {
        set_err(h, "level_raw は 0x01..0xFE(<0 で level_v を換算)");
        return OSC001_ERR_ARG;
    }
    rc = trigger_bytes(h, cfg, tb, why, sizeof why);   /* 失敗なら設定は変えない */
    if (rc != OSC001_OK) { set_err(h, why); return rc; }
    h->trig = *cfg;
    h->b[12] = tb[0]; h->b[13] = tb[1]; h->b[15] = tb[2]; h->b[19] = tb[3];
    return OSC001_OK;
}

int osc001_get_trigger(const osc001_handle *h, osc001_trigger_t *out)
{
    *out = h->trig;
    return OSC001_OK;
}

int osc001_trigger_level_raw(const osc001_handle *h)
{
    uint8_t tb[4];
    char why[160];
    if (h->trig.source == OSC001_TRIG_NONE)
        return -1;
    if (trigger_bytes(h, &h->trig, tb, why, sizeof why) != OSC001_OK)
        return OSC001_ERR_ARG;
    return tb[2];
}

/* ── 警告(python warnings() の移植)───────────────────────────────── */

size_t osc001_warnings(const osc001_handle *h, char *buf, size_t buflen)
{
    size_t used = 0;
    int ch;
    if (buflen) buf[0] = '\0';
/* buf が溢れた後は捨て書き(snprintf は欲しかった長さを返すので used は増え続ける) */
#define WARN(...) do { \
        char tmp_[512]; \
        size_t room = (used < buflen) ? (buflen - used) : 0; \
        int n_ = snprintf(buf ? buf + (used < buflen ? used : buflen) : tmp_, \
                          buf ? room : sizeof tmp_, __VA_ARGS__); \
        if (n_ > 0) used += (size_t)n_; \
    } while (0)
    {
        for (ch = 1; ch <= 2; ch++) {
            int tapbyte = h->b[(ch == 1) ? 1 : 2];
            int tap = tap_of_byte(tapbyte);
            if (tap < 0) {
                WARN("byte%d=%d: 使用不可のタップ値\n",
                     (ch == 1) ? 1 : 2, tapbyte);
                continue;
            }
            if (tap == 1) {
                double span = 256.0 * osc001_v_per_lsb(h, ch);
                if (span > X1_CLAMP_V * 1.5)
                    WARN("CH%d: ×1 タップの全スパン %.2gV は入力 %.2gV でクランプ\n",
                         ch, span, X1_CLAMP_V);
            }
        }
        {
            int b3 = h->b[3];
            if (!(b3 & 0x80))
                WARN("byte3=%02x: bit7=0 で取得が失敗し chunk0..5 が 0xFF 埋め"
                     "(全サンプル +128)。致命的\n", b3);
            else if ((b3 & 0x03) == 0)
                WARN("byte3=%02x: bits1:0=00 で DC レベルが -73..-127 へ下がり"
                     "下レールにクリップする\n", b3);
            else if ((b3 & 0x03) == 3)
                WARN("byte3=%02x: bits1:0=11 で信号が乗らない(ほぼ DC)。\n", b3);
            else if (b3 != BYTE3_REQUIRED)
                WARN("byte3=%02x: 正常に動くが既定の %02x と異なる。"
                     "bit3/bit5 の意味は未解明\n", b3, BYTE3_REQUIRED);
        }
        for (ch = 1; ch <= 2; ch++) {
            int g = h->b[(ch == 1) ? 8 : 9];
            if (g > 2)
                WARN("CH%d: ゲイン段 %d は Position をいくら振っても中央(code 0)に"
                     "届かない\n", ch, g);
        }
        if (h->b[4] == 0)
            WARN("byte4=0: 不正値(波形崩壊)。X=%.0f は範囲外\n", x_value(h));
        if (h->b[7] > 7)
            WARN("byte7=%d: 実機は下位 3 ビットだけ使い %d と同一の動作になる\n",
                 h->b[7], h->b[7] & 0x07);
        if (divisor(h) < OSC001_D_MIN)
            WARN("fs は実測上限 ≒%.0f SPS/ch で頭打ち(除数 D=%.1f < 実効下限 %.1f。"
                 ")\n", OSC001_RATE_MAX, divisor(h), OSC001_D_MIN);
        if (h->trig.source == OSC001_TRIG_EXT) {
            WARN("トリガ EXT: T 入力は浮かせると電圧が定まらず誤トリガし得る"
                 "(T を信号か GND に繋ぐ)\n");
        } else if (h->trig.source == OSC001_TRIG_CH1) {
            uint8_t tb[4];
            char why[160];
            if (trigger_bytes(h, &h->trig, tb, why, sizeof why) != OSC001_OK)
                WARN("トリガ CH1: %s\n", why);
        }
    }
#undef WARN
    return used;
}

/* ── 取得(スコープ)────────────────────────────────────────────────── */

static int la_clear(osc001_handle *h);

static int send_arm(osc001_handle *h);

int osc001_arm(osc001_handle *h)
{
    int rc = apply_trigger(h);
    if (rc != OSC001_OK) return rc;
    return send_arm(h);
}

static int send_arm(osc001_handle *h)
{
    uint8_t args[19], resp[OSC001_REPORT_LEN];
    int i;
    if (h->la_edge_pending) {   /* 未発火のエッジ arm を残して op5 へ戻ると暴走する */
        int rc0 = la_clear(h);
        if (rc0 != OSC001_OK) return rc0;
    }
    for (i = 0; i < 19; i++)
        args[i] = h->b[1 + i];
    int rc = txn(h, 5, args, 19, resp, h->timeout_ms);
    if (rc != OSC001_OK) {
        set_err(h, "op5 無応答");
        return rc;
    }
    h->armed_at = h->t.now_ms(h->t.user);
    h->ready_seen = 0;
    h->la_armed = 0;            /* 以後 op6 はスコープ側の状態を返す */
    return OSC001_OK;
}

int osc001_wait_ready(osc001_handle *h, int timeout_ms)
{
    uint64_t t0 = h->t.now_ms(h->t.user);
    uint8_t resp[OSC001_REPORT_LEN];
    while ((int64_t)(h->t.now_ms(h->t.user) - t0) < timeout_ms) {
        int rc = txn(h, 6, NULL, 0, resp, 300);
        if (rc == OSC001_OK && resp[0] > 0) {
            h->ready_seen = 1;
            return 1;
        }
        h->t.sleep_ms(5);   /* 間隔ゼロのポーリングはファームを止めた実績がある */
    }
    return 0;
}

/* arm 後の必要待ち。read は記録時間 192/fs(op6 の完了を見ていれば待たない)。
 * read_deep は後方 19 サンプル/ch の書き込み時刻が未確認なので旧来の 422/fs */
static void wait_acquisition(osc001_handle *h, int n_samples, int skip_if_ready)
{
    double fs = osc001_sample_rate(h);
    long need_ms = (long)(1000.0 * n_samples / fs);
    if (skip_if_ready && h->ready_seen)
        return;
    if (h->armed_at != 0) {
        uint64_t elapsed = h->t.now_ms(h->t.user) - h->armed_at;
        if ((int64_t)elapsed < need_ms)
            h->t.sleep_ms((int)(need_ms - (int64_t)elapsed) + 20);
    }
}

static int read_chunks(osc001_handle *h, int chunks, uint8_t *raw,
                       uint16_t *footer, int check_echo)
{
    int c, rc;
    for (c = 0; c < chunks; c++) {
        uint8_t arg = (uint8_t)c, resp[OSC001_REPORT_LEN];
        rc = txn(h, 8, &arg, 1, resp, h->timeout_ms);
        if (rc != OSC001_OK) {
            set_err(h, "op8 chunk 無応答");
            return rc;
        }
        if (check_echo && resp[0] == OSC001_OP_ECHO) {
            set_err(h, "op8 の byte0=0x08(索引が無効)");
            return OSC001_ERR_PROTO;
        }
        memcpy(raw + c * OSC001_REPORT_LEN, resp, OSC001_REPORT_LEN);
        if (footer && c == chunks - 1) {
            int w;
            for (w = 0; w < OSC001_FOOTER_WORDS; w++)
                footer[w] = (uint16_t)(resp[OSC001_FOOTER_OFFSET + 2 * w] |
                                       (resp[OSC001_FOOTER_OFFSET + 2 * w + 1] << 8));
        }
        if (OSC001_CHUNK_GAP_MS > 0)
            h->t.sleep_ms(OSC001_CHUNK_GAP_MS);
    }
    return OSC001_OK;
}

static void fill_trace(osc001_handle *h, osc001_trace_t *out, int n_bytes)
{
    out->n_bytes = n_bytes;
    out->rate_hz = osc001_sample_rate(h);
    out->v_per_lsb[0] = osc001_v_per_lsb(h, 1);
    out->v_per_lsb[1] = osc001_v_per_lsb(h, 2);
    out->tap[0] = tap_of_byte(h->b[1]);
    out->tap[1] = tap_of_byte(h->b[2]);
    if (osc001_zero_code(h, 1, &out->zero_code[0]) != OSC001_OK) out->zero_code[0] = 0.0;
    if (osc001_zero_code(h, 2, &out->zero_code[1]) != OSC001_OK) out->zero_code[1] = 0.0;
    out->valid = n_bytes / 2;
    out->triggered = -1;
}

int osc001_read(osc001_handle *h, osc001_trace_t *out)
{
    int n = OSC001_SAMPLES_PER_CH * 2;   /* 384 */
    memset(out, 0, sizeof *out);
    wait_acquisition(h, OSC001_SAMPLES_PER_CH, 1);
    int rc = read_chunks(h, OSC001_CHUNKS, out->raw, NULL, 1);
    if (rc != OSC001_OK) return rc;
    fill_trace(h, out, n);
    return OSC001_OK;
}

int osc001_read_partial(osc001_handle *h, osc001_trace_t *out)
{
    int n = OSC001_SAMPLES_PER_CH * 2, rc;
    uint64_t t0;
    long valid;
    memset(out, 0, sizeof *out);
    if (h->armed_at == 0) {
        set_err(h, "arm していない");
        return OSC001_ERR_STATE;
    }
    if (h->b[12]) {
        set_err(h, "ハードトリガ待ちの途中読みは扱わない(op8 は成立まで凍結)");
        return OSC001_ERR_STATE;
    }
    t0 = h->t.now_ms(h->t.user);
    rc = read_chunks(h, OSC001_CHUNKS, out->raw, NULL, 1);
    if (rc != OSC001_OK) return rc;
    fill_trace(h, out, n);
    valid = (long)((double)(t0 - h->armed_at) / 1000.0 * out->rate_hz) - 1;
    out->valid = (int)(valid < 0 ? 0 : valid > OSC001_SAMPLES_PER_CH ? OSC001_SAMPLES_PER_CH : valid);
    return OSC001_OK;
}

/* FNV-1a 64(stale 指紋。md5 の代替 — セッション内一致の判定に使うため) */
static uint64_t fnv1a64(const uint8_t *p, int n)
{
    uint64_t hv = 1469598103934665603ULL;
    int i;
    for (i = 0; i < n; i++) {
        hv ^= p[i];
        hv *= 1099511628211ULL;
    }
    return hv;
}

int osc001_read_deep(osc001_handle *h, osc001_trace_t *out)
{
    int n = OSC001_BUFFER_BYTES;          /* 422(448 B 読み、末尾 26 B は chunk6 残骸) */
    int i, all_ff = 1;
    uint64_t hash;
    memset(out, 0, sizeof *out);
    wait_acquisition(h, n, 0);
    int rc = read_chunks(h, OSC001_CHUNKS_DEEP, out->raw, out->footer, 1);
    if (rc != OSC001_OK) return rc;
    fill_trace(h, out, n);
    out->has_footer = 1;

    /* stale: (パリティ語 w05/w07 + データハッシュ) が前回 read_deep と一致 */
    hash = fnv1a64(out->raw, n);
    out->stale = (h->fp_have_prev &&
                  out->footer[FOOTER_PARITY_IDX[0]] == h->fp_prev_parity[0] &&
                  out->footer[FOOTER_PARITY_IDX[1]] == h->fp_prev_parity[1] &&
                  hash == h->fp_prev_hash);
    h->fp_have_prev = 1;
    h->fp_prev_parity[0] = out->footer[FOOTER_PARITY_IDX[0]];
    h->fp_prev_parity[1] = out->footer[FOOTER_PARITY_IDX[1]];
    h->fp_prev_hash = hash;

    /* power_cycle: 静的語が前回基準と差分(絶対値でなく差分判定。機体差対策) */
    {
        uint16_t now[10];
        for (i = 0; i < 10; i++)
            now[i] = out->footer[FOOTER_FP_IDX[i]];
        out->power_cycle = (h->fp_have_base &&
                            memcmp(now, h->fp_base, sizeof now) != 0);
        memcpy(h->fp_base, now, sizeof now);
        h->fp_have_base = 1;
    }

    for (i = 0; i < n; i++)
        if (out->raw[i] != 0xFF) { all_ff = 0; break; }
    out->filled_ff = all_ff;
    return OSC001_OK;
}

/* arm → 待ち(python _acquire と同じ)。*trig に -1/1/0 を返す。
 * Normal 未成立は ERR_NO_TRIGGER(op8 は凍結バッファなので読まない)。
 * 未成立の arm の直後に op5 を送り直すのは安全(約 600 回で無応答・切断なし) */
static int acquire(osc001_handle *h, int wait_timeout_ms, int *trig)
{
    int rc = osc001_arm(h);
    if (rc != OSC001_OK) return rc;
    if (h->trig.source == OSC001_TRIG_NONE) {
        osc001_wait_ready(h, wait_timeout_ms);
        *trig = -1;
        return OSC001_OK;
    }
    if (h->trig.mode == OSC001_TRIG_NORMAL) {
        if (osc001_wait_ready(h, wait_timeout_ms)) { *trig = 1; return OSC001_OK; }
        *trig = 0;
        set_err(h, "トリガ未成立");
        return OSC001_ERR_NO_TRIGGER;
    }
    {
        int auto_ms = h->trig.auto_timeout_ms;
        if (auto_ms <= 0)
            auto_ms = TRIG_AUTO_MIN_MS +
                      (int)(1000.0 * OSC001_SAMPLES_PER_CH * 2 / osc001_sample_rate(h));
        if (osc001_wait_ready(h, auto_ms)) { *trig = 1; return OSC001_OK; }
    }
    h->b[12] = 0;                      /* 次の arm で apply_trigger が 1 に戻す */
    rc = send_arm(h);
    if (rc != OSC001_OK) return rc;
    osc001_wait_ready(h, wait_timeout_ms);
    *trig = 0;
    return OSC001_OK;
}

int osc001_capture(osc001_handle *h, osc001_trace_t *out, int wait_timeout_ms)
{
    int trig = -1;
    int rc = acquire(h, wait_timeout_ms, &trig);
    if (rc == OSC001_ERR_NO_TRIGGER) {
        memset(out, 0, sizeof *out);
        out->triggered = 0;
    }
    if (rc != OSC001_OK) return rc;
    rc = osc001_read(h, out);
    out->triggered = trig;
    return rc;
}

int osc001_capture_deep(osc001_handle *h, osc001_trace_t *out, int wait_timeout_ms)
{
    int trig = -1;
    int rc = acquire(h, wait_timeout_ms, &trig);
    if (rc == OSC001_ERR_NO_TRIGGER) {
        memset(out, 0, sizeof *out);
        out->triggered = 0;
    }
    if (rc != OSC001_OK) return rc;
    rc = osc001_read_deep(h, out);
    out->triggered = trig;
    return rc;
}

/* ── トレース復号 ─────────────────────────────────────────────────── */

int osc001_trace_samples_per_ch(const osc001_trace_t *t) { return t->n_bytes / 2; }

int osc001_trace_sample(const osc001_trace_t *t, int ch, int i)
{
    int idx;
    if (ch != 1 && ch != 2) return (-2147483647 - 1);
    if (i < 0 || i >= t->n_bytes / 2) return (-2147483647 - 1);
    idx = 2 * i + (ch == 2 ? 1 : 0);
    return (int)t->raw[idx] - 0x7F;
}

int osc001_trace_to_volts(const osc001_trace_t *t, int ch, double *buf, int n)
{
    int i, v;
    if (ch != 1 && ch != 2) return OSC001_ERR_ARG;
    for (i = 0; i < n; i++) {
        v = osc001_trace_sample(t, ch, i);
        if (v == (-2147483647 - 1)) return OSC001_ERR_ARG;
        buf[i] = ((double)v - t->zero_code[ch_index(ch)]) * t->v_per_lsb[ch_index(ch)];
    }
    return OSC001_OK;
}

int osc001_trace_rail_hit(const osc001_trace_t *t, int ch)
{
    int i, n = t->n_bytes / 2, mx = -128, mn = 128;
    for (i = 0; i < n; i++) {
        int v = osc001_trace_sample(t, ch, i);
        if (v > mx) mx = v;
        if (v < mn) mn = v;
    }
    return (mx >= 128 || mn <= -127) ? 1 : 0;
}

int osc001_trace_clipped(const osc001_trace_t *t, int ch)
{
    int i, n = t->n_bytes / 2, mx = -128, mn = 128;
    if (t->tap[ch_index(ch)] != 1)
        return 0;
    for (i = 0; i < n; i++) {
        int v = osc001_trace_sample(t, ch, i);
        if (v > mx) mx = v;
        if (v < mn) mn = v;
    }
    return ((mx - mn) >= 160) ? 1 : 0;   /* 165 LSB 張り付き ≈ クリップ */
}

/* ── op9 モニタ ───────────────────────────────────────────────────── */

int osc001_monitor_raw(osc001_handle *h, int sel1, int sel2, int *m1, int *m2)
{
    uint8_t args[3], resp[OSC001_REPORT_LEN];
    int rc;
    args[0] = (uint8_t)sel1;
    args[1] = (uint8_t)sel2;
    args[2] = 0xAA;
    rc = txn(h, 9, args, 3, resp, h->timeout_ms);
    if (rc != OSC001_OK) {
        set_err(h, "op9 無応答");
        return rc;
    }
    if (m1) *m1 = (resp[0] << 8) | resp[1];
    if (m2) *m2 = (resp[2] << 8) | resp[3];
    return OSC001_OK;
}

int osc001_monitor(osc001_handle *h, int tap, int n, osc001_monitor_t *out)
{
    int k, ti, rc, c;
    double acc[2] = { 0.0, 0.0 }, vc;
    int top;
    if ((tap != 1 && tap != 8) || !out) return OSC001_ERR_ARG;
    if (n < 1) n = 1;
    memset(out, 0, sizeof *out);
    ti = (tap == 8) ? 1 : 0;
    top = (tap == 8) ? OSC001_MON_MAX : OSC001_MON_X1_SAT;
    for (k = 0; k < n; k++) {
        int m[2];
        rc = osc001_monitor_raw(h, MON_SEL[0][ti], MON_SEL[1][ti], &m[0], &m[1]);
        if (rc != OSC001_OK) return rc;
        for (c = 0; c < 2; c++) {
            acc[c] += m[c];
            if (m[c] <= 0 || m[c] >= top) out->sat[c] = 1;
        }
    }
    vc = (tap == 8) ? OSC001_VCOUNT_DIV8 : OSC001_VCOUNT_X1;
    for (c = 0; c < 2; c++) {
        out->m[c] = acc[c] / n;
        out->volts[c] = (out->m[c] - h->m0[c][ti]) * vc;
    }
    return OSC001_OK;
}

int osc001_calibrate_zero(osc001_handle *h, int ch_mask, int duration_ms, int ranges)
{
    int ti, c, rc, r;
    if (!(ch_mask & 3)) return OSC001_ERR_ARG;
    for (ti = 0; ti < 2; ti++) {
        double acc[2] = { 0.0, 0.0 };
        int k = 0;
        uint64_t t0 = h->t.now_ms(h->t.user);
        do {
            int m[2];
            rc = osc001_monitor_raw(h, MON_SEL[0][ti], MON_SEL[1][ti], &m[0], &m[1]);
            if (rc != OSC001_OK) return rc;
            acc[0] += m[0];
            acc[1] += m[1];
            k++;
        } while ((int64_t)(h->t.now_ms(h->t.user) - t0) < duration_ms);
        for (c = 0; c < 2; c++)
            if (ch_mask & (1 << c))
                osc001_set_zero_m0(h, c + 1, ti ? 8 : 1, acc[c] / k);
    }
    if (ranges) {
        uint8_t keep_b[20];
        osc001_trigger_t keep_trig = h->trig;
        osc001_trace_t tr;
        memcpy(keep_b, h->b, sizeof keep_b);
        osc001_trigger_init(&h->trig);
        h->b[12] = 0;
        rc = OSC001_OK;
        for (r = 0; r < 6 && rc == OSC001_OK; r++) {
            for (c = 0; c < 2; c++)
                if (ch_mask & (1 << c))
                    osc001_set_gain(h, c + 1, RANGES[r].gain, RANGES[r].tap, OSC001_POS_AUTO);
            rc = osc001_capture(h, &tr, 2000);
            if (rc != OSC001_OK) break;
            for (c = 0; c < 2; c++) {
                int i, n = OSC001_SAMPLES_PER_CH, mx = -128, mn = 128, g = RANGES[r].gain;
                double sum = 0.0;
                if (!(ch_mask & (1 << c))) continue;
                for (i = 0; i < n; i++) {
                    int v = osc001_trace_sample(&tr, c + 1, i);
                    sum += v;
                    if (v > mx) mx = v;
                    if (v < mn) mn = v;
                }
                if (mx >= 128 || mn <= -127) continue;   /* レールに当たった(0 V が窓の外) */
                ti = (RANGES[r].tap == 8) ? 1 : 0;
                h->m0_gain[c][ti][g] = (sum / n + 127.0 -
                                        POSITION_SLOPE[g] * h->b[(c == 0) ? 10 : 11]) * (double)(1 << g);
                h->m0_gain_set[c][ti][g] = 1;
            }
        }
        memcpy(h->b, keep_b, sizeof keep_b);
        h->trig = keep_trig;
        if (rc != OSC001_OK) return rc;
    }
    return OSC001_OK;
}

/* ── LA────────────────────────────────────────────────────── */

/* byte3=3/2/1 の固定段(2026-10-17 実測)と byte3=0 の可変域(fs = 6e6/(D+29)、D は ≒26.2 で張り付き) */
static const double LA_RATE_FIXED[4] = { 0.0, 200000.0, 490900.0, 979400.0 };
#define LA_D_MIN (OSC001_LA_CLOCK_HZ / 108700.0 - OSC001_RATE_OFFSET)

double osc001_la_set_rate(osc001_handle *h, double hz)
{
    double best_err = -1.0, best_fs = 0.0, target;
    uint8_t best_b3 = 3, best_b47[4] = { 0xFF, 0xBE, 0x01, 0x00 };
    int b3, i6, i7, k;
    if (!(hz > 0.0)) {
        set_err(h, "LA の fs は正の値");
        return (double)OSC001_ERR_ARG;
    }
    for (b3 = 1; b3 <= 3; b3++) {
        double err = fabs(LA_RATE_FIXED[b3] / hz - 1.0);
        if (best_err < 0 || err < best_err) {
            best_err = err; best_fs = LA_RATE_FIXED[b3]; best_b3 = (uint8_t)b3;
        }
    }
    target = OSC001_LA_CLOCK_HZ / hz - OSC001_RATE_OFFSET;
    if (target < LA_D_MIN) target = LA_D_MIN;
    for (i6 = 0; i6 <= 1; i6++) {
        for (i7 = 0; i7 <= 7; i7++) {
            double shift = i6 ? 0.5 : (double)(1 << i7);
            long x = lround(target / shift);
            for (k = -1; k <= 1; k++) {
                long xc = x + k;
                double d, fs, err;
                unsigned v;
                if (xc < 1 || xc > 65280) continue;
                d = shift * (double)xc;
                if (d < LA_D_MIN) d = LA_D_MIN;
                fs = OSC001_LA_CLOCK_HZ / (d + OSC001_RATE_OFFSET);
                err = fabs(fs / hz - 1.0);
                if (err < best_err) {
                    v = (0x10000u - (unsigned)xc) & 0xFFFFu;
                    best_err = err; best_fs = fs; best_b3 = 0;
                    best_b47[0] = (uint8_t)(v >> 8); best_b47[1] = (uint8_t)(v & 0xFF);
                    best_b47[2] = (uint8_t)i6; best_b47[3] = (uint8_t)i7;
                }
            }
        }
    }
    if (best_b3 != 0) {        /* 固定段では byte4..7 は無関係。既定値を載せる */
        best_b47[0] = 0xFF; best_b47[1] = 0xBE; best_b47[2] = 0x01; best_b47[3] = 0x00;
    }
    h->la_b3 = best_b3;
    memcpy(h->la_b47, best_b47, 4);
    h->la_rate = best_fs;
    return best_fs;
}

double osc001_la_rate(const osc001_handle *h) { return h->la_rate; }

int osc001_la_configure(osc001_handle *h, int channel_mask, osc001_edge_t edge)
{
    if (channel_mask & ~(0x80 | 0x40 | 0x20 | 0x10)) {
        set_err(h, "channel_mask は ch1=0x80/ch2=0x40/ch3=0x20/ch4=0x10 の OR");
        return OSC001_ERR_ARG;
    }
    /* ★byte8=2(auto) かつ byte9≠0(単一ビット)はハング領域。
     * byte8=2 かつ byte9=0 は実機で安全を確認済み */
    if ((int)edge == 2 && channel_mask != 0) {
        set_err(h, "edge=AUTO かつ channel_mask≠0 はハング領域"
                   "(op16 byte8=2 かつ byte9≠0)。復旧は物理抜き差しのみ");
        return OSC001_ERR_ARG;
    }
    h->la_frame[0] = 0x00;
    h->la_frame[1] = 0x00;
    h->la_frame[2] = h->la_b3;
    memcpy(&h->la_frame[3], h->la_b47, 4);
    h->la_frame[7] = (uint8_t)edge;
    h->la_frame[8] = (uint8_t)channel_mask;
    h->la_configured = 1;
    return OSC001_OK;
}

int osc001_la_arm(osc001_handle *h)
{
    uint8_t resp[OSC001_REPORT_LEN], zero = 0;
    int rc;
    if (!h->la_configured) {
        rc = osc001_la_configure(h, 0x40, OSC001_EDGE_FALLING); /* 既定: ch2 立ち下がり */
        if (rc != OSC001_OK) return rc;
    }
    rc = txn(h, 3, &zero, 1, resp, h->timeout_ms);
    if (rc != OSC001_OK) { set_err(h, "op3(LA 開始)無応答"); return rc; }
    rc = txn(h, 16, h->la_frame, 9, resp, h->timeout_ms);
    if (rc != OSC001_OK) { set_err(h, "op16(arm)無応答"); return rc; }
    h->la_armed_at = h->t.now_ms(h->t.user);
    h->la_armed = 1;
    h->la_fired = 0;
    h->la_edge_pending = (h->la_frame[7] != (uint8_t)OSC001_EDGE_AUTO);
    return OSC001_OK;
}

int osc001_la_wait(osc001_handle *h, int timeout_ms)
{
    uint64_t t0 = h->t.now_ms(h->t.user);
    uint8_t resp[OSC001_REPORT_LEN];
    if (!h->la_armed) {
        set_err(h, "la_arm の前(または abort/op5 の後)は LA の発火を判定できない");
        return OSC001_ERR_STATE;
    }
    while ((int64_t)(h->t.now_ms(h->t.user) - t0) < timeout_ms) {
        int rc = txn(h, 6, NULL, 0, resp, 300);
        if (rc == OSC001_OK && resp[0] == 1) {
            h->la_fired = 1;
            h->la_edge_pending = 0;
            return 1;
        }
        h->t.sleep_ms(5);
    }
    return 0;
}

/* マスク 0 のエッジ arm で残ったエッジトリガを上書きする(発火はしない) */
static int la_clear(osc001_handle *h)
{
    uint8_t frame[9], resp[OSC001_REPORT_LEN], zero = 0;
    int rc;
    memcpy(frame, h->la_frame, sizeof frame);
    if (!h->la_configured) {
        frame[0] = 0x00; frame[1] = 0x00; frame[2] = 0x03;
        frame[3] = (uint8_t)((LA_B47_DEFAULT >> 0) & 0xFF);
        frame[4] = (uint8_t)((LA_B47_DEFAULT >> 8) & 0xFF);
        frame[5] = (uint8_t)((LA_B47_DEFAULT >> 16) & 0xFF);
        frame[6] = (uint8_t)((LA_B47_DEFAULT >> 24) & 0xFF);
    }
    frame[7] = (uint8_t)OSC001_EDGE_RISING;
    frame[8] = 0x00;
    rc = txn(h, 3, &zero, 1, resp, h->timeout_ms);
    if (rc == OSC001_OK) rc = txn(h, 16, frame, 9, resp, h->timeout_ms);
    if (rc != OSC001_OK) { set_err(h, "LA トリガの解除(op16 マスク 0)無応答"); return rc; }
    h->la_edge_pending = 0;
    return OSC001_OK;
}

int osc001_la_abort(osc001_handle *h)
{
    int rc = la_clear(h);
    h->la_armed = 0;
    h->la_fired = 0;
    return rc;
}

int osc001_la_read(osc001_handle *h, osc001_la_trace_t *out)
{
    if (!h->la_armed || !h->la_fired) {
        set_err(h, "発火前に op8 は読めない(凍結バッファ)");
        return OSC001_ERR_STATE;
    }
    memset(out, 0, sizeof *out);
    out->rate_hz = h->la_rate;
    return read_chunks(h, OSC001_LA_CHUNKS, out->raw, out->footer, 0);
}

int osc001_la_capture(osc001_handle *h, int channel_mask, osc001_edge_t edge,
                      int timeout_ms, osc001_la_trace_t *out)
{
    int rc = osc001_la_configure(h, channel_mask, edge);
    if (rc != OSC001_OK) return rc;
    rc = osc001_la_arm(h);
    if (rc != OSC001_OK) return rc;
    /* op6=1 は窓を記録し終えた時点なので、窓の時間 844/fs を足して待つ */
    if (osc001_la_wait(h, timeout_ms + (int)(1000.0 * OSC001_LA_SAMPLES / h->la_rate) + 1) != 1) {
        osc001_la_abort(h);
        set_err(h, "LA トリガが発火しなかった(arm は解除済み)");
        return OSC001_ERR_TIMEOUT;
    }
    return osc001_la_read(h, out);
}

int osc001_la_bit(const osc001_la_trace_t *t, int ch, int i)
{
    int nib;
    if (ch < 1 || ch > 4) return (-2147483647 - 1);
    if (i < 0 || i >= OSC001_LA_SAMPLES) return (-2147483647 - 1);
    nib = (i & 1) ? ((t->raw[i / 2] >> 4) & 0x0F) : (t->raw[i / 2] & 0x0F);   /* 下位ニブル先 */
    return (nib >> (4 - ch)) & 1;   /* ch1=bit3..ch4=bit0 */
}

int osc001_la_transitions(const osc001_la_trace_t *t, int ch)
{
    int i, cnt = 0, prev = -1;
    for (i = 0; i < OSC001_LA_SAMPLES; i++) {
        int b = osc001_la_bit(t, ch, i);
        if (prev >= 0 && b != prev) cnt++;
        prev = b;
    }
    return cnt;
}
