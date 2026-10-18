/* libosc001 オフラインテスト(モックトランスポート。デバイス不要)。
 * 期待値は実測確定値と python/test_osc001_math.py の数式部と一致させる。
 * ビルド: make check (c/ ディレクトリ) */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../include/osc001.h"
#include "mock_transport.h"

static int fails = 0, checks = 0;

#define CHECK(cond, ...) do {                                        \
    checks++;                                                        \
    if (!(cond)) {                                                   \
        fails++;                                                     \
        printf("FAIL(%d): ", __LINE__);                              \
        printf(__VA_ARGS__);                                         \
        printf("\n");                                                \
    } else {                                                         \
        printf("OK:  ");                                             \
        printf(__VA_ARGS__);                                         \
        printf("\n");                                                \
    }                                                                \
} while (0)

static osc001_handle *open_mock(mock_dev *m)
{
    char err[256];
    osc001_handle *h;
    mock_init(m);
    h = osc001_open(&m->t, err, sizeof err);
    if (!h) { printf("open 失敗: %s\n", err); fails++; }
    return h;
}

int main(void)
{
    mock_dev m;
    osc001_handle *h = open_mock(&m);
    if (!h) return 1;

    /* ── 情報 ─────────────────────────────────────────────────────── */
    {
        char info[64];
        CHECK(osc001_info(h, info, sizeof info) == OSC001_OK &&
              strcmp(info, "DPScope SE") == 0, "op2 情報文字列 = %s", info);
    }

    /* ── fs 統一式──────────────────────── */
    osc001_set_raw(h, 4, 0xFF); osc001_set_raw(h, 5, 0x00);
    osc001_set_raw(h, 6, 0x00); osc001_set_raw(h, 7, 0x00);
    CHECK(fabs(osc001_sample_rate(h) - 3.0e6 / 285.0) < 1.0,
          "X=256 → fs=%.1f(期待 %.1f)", osc001_sample_rate(h), 3.0e6 / 285.0);
    osc001_set_raw(h, 6, 0x01);
    CHECK(fabs(osc001_sample_rate(h) - 3.0e6 / 157.0) < 1.0,
          "byte6 bit0 → fs=%.1f(期待 %.1f)", osc001_sample_rate(h), 3.0e6 / 157.0);
    osc001_set_raw(h, 6, 0x00); osc001_set_raw(h, 7, 0x01);
    CHECK(fabs(osc001_sample_rate(h) - 3.0e6 / 541.0) < 1.0,
          "byte7=1 → fs=%.1f(期待 %.1f)", osc001_sample_rate(h), 3.0e6 / 541.0);
    osc001_set_raw(h, 7, 0x08);   /* 8..15 は下位 3 ビットへ折り返る */
    CHECK(fabs(osc001_sample_rate(h) - 3.0e6 / 285.0) < 1.0,
          "byte7=8 は 0 と同一(折り返し)→ fs=%.1f", osc001_sample_rate(h));

    /* ── rate_settings のラウンドトリップ────────────── */
    {
        static const double targets[] = { 0.36, 1, 10, 100, 1000, 5000,
                                          10000, 20000, 33130 };
        size_t i;
        int worst_ok = 1;
        for (i = 0; i < sizeof targets / sizeof targets[0]; i++) {
            double got = osc001_set_sample_rate(h, targets[i]);
            double rel = fabs(got - targets[i]) / targets[i];
            if (rel > 0.01) {
                worst_ok = 0;
                printf("  hz=%.2f → %.2f(rel %.3f)\n", targets[i], got, rel);
            }
        }
        CHECK(worst_ok, "rate_settings 9 点往復(±1%%)");
    }
    {
        double got = osc001_set_sample_rate(h, 40000.0);
        char wbuf[2048];
        CHECK(got <= OSC001_RATE_MAX + 0.5 && got >= OSC001_RATE_MAX - 0.5,
              "40kHz 指定は %.0f で頭打ち", got);
        /* 除数が実効下限を明示的に下回った場合だけ警告(python と同じ条件) */
        osc001_set_raw(h, 4, 0xFF); osc001_set_raw(h, 5, 0x9C);   /* X=100 */
        osc001_set_raw(h, 6, 0x01); osc001_set_raw(h, 7, 0x00);   /* D=50 */
        osc001_warnings(h, wbuf, sizeof wbuf);
        CHECK(strstr(wbuf, "頭打ち") != NULL, "D=50 < 61.5 → 頭打ち警告");
    }

    /* ── レンジ表──────────────────────────────────── */
    CHECK(osc001_set_range(h, 1, 0.5, 0, OSC001_POS_AUTO) == OSC001_OK &&
          osc001_get_raw(h, 1) == 5 && osc001_get_raw(h, 8) == 0 &&
          osc001_get_raw(h, 10) == 0xC0,
          "CH1 0.5V/div → tap byte=5(÷8) 段=0 Position=0xC0");
    CHECK(fabs(osc001_v_per_lsb(h, 1) - OSC001_VCOUNT_DIV8) < 1e-12,
          "CH1 段0 ÷8 V/LSB=%.6f(期待 %.6f)",
          osc001_v_per_lsb(h, 1), OSC001_VCOUNT_DIV8);
    CHECK(osc001_set_range(h, 2, 0.2, 0, OSC001_POS_AUTO) == OSC001_OK &&
          osc001_get_raw(h, 2) == 9 && osc001_get_raw(h, 9) == 2 &&
          osc001_get_raw(h, 11) == 0x00,
          "CH2 0.2V/div → tap byte=9(×1) 段=2 Position=0x00");
    CHECK(osc001_set_range(h, 1, 0.7, 0, OSC001_POS_AUTO) == OSC001_ERR_ARG,
          "0.7 V/div は 6 ラベル外 → ERR_ARG");
    CHECK(osc001_set_gain(h, 1, 4, 0, OSC001_POS_CENTER) == OSC001_ERR_ARG,
          "段 4 で POS_CENTER → 到達不能エラー");

    /* ── 警告──────────────────────────────────────── */
    {
        char wbuf[2048];
        osc001_set_raw(h, 3, 0x00);
        osc001_warnings(h, wbuf, sizeof wbuf);
        CHECK(strstr(wbuf, "bit7=0") != NULL, "byte3=0x00 → bit7=0 警告");
        osc001_set_raw(h, 3, 0xAA);
        osc001_warnings(h, wbuf, sizeof wbuf);
        CHECK(strstr(wbuf, "bit7=0") == NULL, "byte3=0xAA → 警告消える");
    }

    /* ── 取得(インターリーブ・復号)────────────────────────────── */
    {
        osc001_trace_t tr;
        double volts[8];
        osc001_set_sample_rate(h, 10000.0);
        CHECK(osc001_capture(h, &tr, 1000) == OSC001_OK, "capture 成功");
        CHECK(tr.n_bytes == 384 && osc001_trace_samples_per_ch(&tr) == 192,
              "384 B = 192 サンプル/ch");
        /* モック pattern: raw[i] = i( chunk0 )。偶数番=CH1、値=byte−0x7F */
        CHECK(osc001_trace_sample(&tr, 1, 0) == 0 - 0x7F &&
              osc001_trace_sample(&tr, 2, 0) == 1 - 0x7F &&
              osc001_trace_sample(&tr, 1, 1) == 2 - 0x7F,
              "インターリーブ復号(偶=CH1、byte−0x7F)");
        CHECK(osc001_trace_sample(&tr, 1, 192) == (-2147483647 - 1),
              "範囲外 index は INT_MIN");
        CHECK(osc001_trace_rail_hit(&tr, 1) == 1,
              "全スパン pattern → rail_hit");
        osc001_trace_to_volts(&tr, 1, volts, 8);
        {
            double z1;
            osc001_zero_code(h, 1, &z1);
            CHECK(fabs(tr.zero_code[0] - z1) < 1e-12 &&
                  fabs(volts[0] - (-127.0 - z1) * tr.v_per_lsb[0]) < 1e-12,
                  "to_volts =(生値 − zero_code)× V/LSB");
        }
        CHECK(tr.valid == 192, "read の valid は全数");
        /* op5 ペイロードが実際に送られたこと */
        CHECK(m.op5_count == 1 && m.last_op5[0] == 5 &&
              m.last_op5[3] == 0xAA,
              "op5(20 B)送信・byte3=0xAA");
    }

    /* ── read_deep: 422 B・フッター・stale・power_cycle・0xFF 埋め ── */
    {
        osc001_trace_t tr;
        CHECK(osc001_capture_deep(h, &tr, 1000) == OSC001_OK, "capture_deep 成功");
        CHECK(tr.n_bytes == 422 && tr.has_footer == 1,
              "422 B(211/ch)+フッター");
        CHECK(tr.footer[5] == 0x1005 && tr.footer[12] == 0x100C,
              "フッター 13 語 LE16 解析(chunk6 [38..63])");
        CHECK(tr.stale == 0 && tr.power_cycle == 0,
              "初回: stale/power_cycle なし");
        osc001_read_deep(h, &tr);
        CHECK(tr.stale == 1, "同一データ 2 回目 → stale(ヒューリスティック)");
        m.footer_base = 0x2000;          /* 静的語の差分 = 再電源投入疑い */
        osc001_read_deep(h, &tr);
        CHECK(tr.power_cycle == 1 && tr.stale == 0,
              "静的語差分 → power_cycle(差分判定)");
        m.fill_ff = 1;                   /* 0xFF 埋め */
        osc001_read_deep(h, &tr);
        CHECK(tr.filled_ff == 1, "全バイト 0xFF → filled_ff");
        m.fill_ff = 0;
    }

    /* ── LA──────────────────────────────────────────────── */
    CHECK(osc001_la_configure(h, 0x40, OSC001_EDGE_AUTO) == OSC001_ERR_ARG,
          "ハング領域のガード: AUTO+mask≠0 を拒否");
    CHECK(osc001_la_configure(h, 0, OSC001_EDGE_AUTO) == OSC001_OK,
          "AUTO+mask=0(安全な組合せ)は通す");
    {
        osc001_la_trace_t lat;
        m.la_mode = 1;
        CHECK(osc001_la_capture(h, 0x40, OSC001_EDGE_FALLING, 1000, &lat) == OSC001_OK,
              "la_capture(ch2 falling) 成功");
        CHECK(m.last_op16[0] == 16 && m.last_op16[3] == 0x03 &&
              m.last_op16[4] == 0xFF && m.last_op16[5] == 0xBE &&
              m.last_op16[6] == 0x01 && m.last_op16[7] == 0x00 &&
              m.last_op16[8] == 0x00 && m.last_op16[9] == 0x40,
              "op16 フレーム(byte3=03・b4..7=ff be 01 00・b8=edge・b9=mask)");
        /* chunk 応答の byte0 はエコー(raw[0], raw[64], ...)なのでデータは raw[1] から。
         * 0xA5: 下位ニブル先で 0101,1010。ch1=bit3 → 0,1 / ch4=bit0 → 1,0 */
        CHECK(osc001_la_bit(&lat, 1, 2) == 0 && osc001_la_bit(&lat, 1, 3) == 1,
              "LA ニブル下位優先・ch1=bit3");
        CHECK(osc001_la_bit(&lat, 4, 2) == 1 && osc001_la_bit(&lat, 4, 3) == 0,
              "ch4=bit0");
        CHECK(osc001_la_transitions(&lat, 1) > 0, "遷移カウント > 0");
        CHECK(osc001_la_bit(&lat, 1, OSC001_LA_SAMPLES - 1) != (-2147483647 - 1) &&
              osc001_la_bit(&lat, 1, OSC001_LA_SAMPLES) == (-2147483647 - 1),
              "有効サンプルは 0..843(末尾 26 B のフッターは復号しない)");
        CHECK(lat.footer[0] == m.footer_base && lat.footer[12] == (uint16_t)(m.footer_base + 12),
              "LA フッター(chunk6[38..63])を 13 語に展開");
        CHECK(lat.rate_hz == OSC001_LA_RATE_HZ, "LA rate_hz = 既定 fs(byte3=3)");
        CHECK(m.last_op16[3] == 0x03, "既定は op16 byte3=3");
        {
            double fs = osc001_la_set_rate(h, 2000.0);
            CHECK(fabs(fs / 2000.0 - 1.0) < 2e-3 && osc001_la_rate(h) == fs, "la_set_rate(2 kHz) ≒ 2000 Hz");
            CHECK(osc001_la_configure(h, 0x40, OSC001_EDGE_RISING) == OSC001_OK &&
                  osc001_la_arm(h) == OSC001_OK && m.last_op16[3] == 0x00,
                  "2 kHz は op16 byte3=0(byte4..7 の除数)");
            {
                unsigned x = 65536u - ((unsigned)m.last_op16[4] * 256u + m.last_op16[5]);
                double d = (m.last_op16[6] & 1) ? x / 2.0 : (double)(1u << m.last_op16[7]) * x;
                CHECK(fabs(6e6 / (d + 29) - fs) < 1e-6, "byte4..7 の除数と fs が整合");
            }
            CHECK(osc001_la_set_rate(h, 5e5) == 490900.0 && osc001_la_set_rate(h, 2e5) == 200000.0,
                  "固定段 490.9 k / 200 k");
            CHECK(osc001_la_set_rate(h, 1e6) == OSC001_LA_RATE_HZ, "1 MHz → 979.4 kHz(byte3=3)");
            osc001_la_abort(h);
        }
        /* 待機の中止: op5 で arm を解き、以後 wait/read は ERR_STATE */
        {
            int n5 = m.op5_count;
            CHECK(osc001_la_arm(h) == OSC001_OK, "再 arm");
            CHECK(osc001_la_abort(h) == OSC001_OK && m.op5_count == n5 &&
                  m.last_op16[8] == 0x01 && m.last_op16[9] == 0x00,
                  "la_abort は op5 を送らず、マスク 0 のエッジ arm で上書き");
            CHECK(osc001_la_wait(h, 10) == OSC001_ERR_STATE, "abort 後の la_wait は ERR_STATE");
            CHECK(osc001_la_read(h, &lat) == OSC001_ERR_STATE, "abort 後の la_read は ERR_STATE");
            /* 未発火のエッジ arm を残したまま osc001_arm → 先にマスク 0 で解除してから op5 */
            CHECK(osc001_la_configure(h, 0x40, OSC001_EDGE_FALLING) == OSC001_OK &&
                  osc001_la_arm(h) == OSC001_OK && m.last_op16[9] == 0x40, "ch2 falling arm");
            CHECK(osc001_arm(h) == OSC001_OK && m.op5_count == n5 + 1 &&
                  m.last_op16[8] == 0x01 && m.last_op16[9] == 0x00,
                  "未発火エッジ arm 後の op5 は先に解除");
            memset(m.last_op16, 0, sizeof m.last_op16);
            CHECK(osc001_arm(h) == OSC001_OK && m.last_op16[0] == 0,
                  "解除済みなら op16 を送らない");
        }
        m.la_mode = 0;
    }
    {
        osc001_handle *h2 = open_mock(&m);
        osc001_la_trace_t lat;
        if (h2) {
            CHECK(osc001_la_read(h2, &lat) == OSC001_ERR_STATE,
                  "未発火の la_read は拒否(凍結バッファ)");
            CHECK(osc001_la_wait(h2, 10) == OSC001_ERR_STATE,
                  "la_arm 前の la_wait は拒否(op6 がスコープ状態を返すため)");
            osc001_close(h2);
        }
    }

    /* ── Position の 0 V 切片とハードトリガ ──────────────────────────── */
    {
        osc001_handle *ht = open_mock(&m);
        osc001_trigger_t tc, got;
        osc001_trace_t tr;
        double z, code;
        if (ht) {
            osc001_set_range(ht, 1, 1.0, 0, OSC001_POS_AUTO);   /* ÷8 段 1、Position 0x80 */
            CHECK(osc001_zero_code(ht, 1, &z) == OSC001_OK && fabs(z - (-7.5)) < 1e-9,
                  "zero_code CH1 ÷8 段1 pos 0x80 = %.2f(期待 495/2-127-128 = -7.5)", z);
            CHECK(osc001_volts_to_code(ht, 1, 2.5, &code) == OSC001_OK &&
                  fabs(code - (-7.5 + 2.5 / 0.0958)) < 1e-9,
                  "volts_to_code CH1 2.5V = %.3f", code);
            osc001_set_range(ht, 2, 0.2, 0, OSC001_POS_AUTO);   /* ×1 段 2、Position 0x00 */
            CHECK(osc001_zero_code(ht, 2, &z) == OSC001_OK && fabs(z - (388.0 / 4 - 127)) < 1e-9,
                  "zero_code CH2 ×1 段2 pos 0x00 = %.2f(期待 -30)", z);
            osc001_set_gain(ht, 2, 3, 8, OSC001_POS_KEEP);
            CHECK(osc001_zero_code(ht, 2, &z) == OSC001_OK && fabs(z - (496.5 / 8 - 127)) < 1e-9,
                  "段 3 の zero_code は式からの外挿 = %.3f", z);
            CHECK(osc001_set_zero_m0(ht, 2, 8, 500.0) == OSC001_OK &&
                  osc001_zero_code(ht, 2, &z) == OSC001_OK && fabs(z - (500.0 / 8 - 127)) < 1e-9,
                  "set_zero_m0 が zero_code に効く");
            osc001_set_zero_m0(ht, 2, 8, 496.5);

            osc001_trigger_init(&tc);
            tc.source = OSC001_TRIG_CH1; tc.edge = OSC001_EDGE_RISING; tc.level_v = 2.5;
            CHECK(osc001_set_trigger(ht, &tc) == OSC001_OK &&
                  osc001_get_raw(ht, 12) == 1 && osc001_get_raw(ht, 13) == 1 &&
                  osc001_get_raw(ht, 15) == (int)lround(-7.5 + 2.5 / osc001_v_per_lsb(ht, 1)) + 0x7F &&
                  osc001_get_raw(ht, 19) == 1,
                  "CH1 ÷8 トリガ: byte12/13/15/19 = %d/%d/0x%02x/%d", osc001_get_raw(ht, 12),
                  osc001_get_raw(ht, 13), osc001_get_raw(ht, 15), osc001_get_raw(ht, 19));
            CHECK(osc001_trigger_level_raw(ht) == osc001_get_raw(ht, 15), "trigger_level_raw");
            osc001_set_range(ht, 1, 0.2, 0, OSC001_POS_AUTO);
            CHECK(osc001_arm(ht) == OSC001_OK && m.last_op5[19] == 2,
                  "CH1 ×1 タップにすると arm で経路 byte19=2");
            osc001_set_range(ht, 1, 1.0, 0, OSC001_POS_AUTO);
            {
                int b15 = osc001_get_raw(ht, 15);
                osc001_trigger_t bad = tc;
                bad.level_v = 20.0;
                CHECK(osc001_set_trigger(ht, &bad) == OSC001_ERR_ARG &&
                      osc001_get_trigger(ht, &got) == OSC001_OK && got.level_v == 2.5 &&
                      osc001_get_raw(ht, 15) == b15,
                      "表示域外レベルは ERR_ARG で設定を変えない");
                bad = tc; bad.edge = OSC001_EDGE_AUTO;
                CHECK(osc001_set_trigger(ht, &bad) == OSC001_ERR_ARG, "edge=AUTO は ERR_ARG");
                bad = tc; bad.level_raw = 0xFF;
                CHECK(osc001_set_trigger(ht, &bad) == OSC001_ERR_ARG, "level_raw=0xFF は ERR_ARG");
                bad = tc; bad.source = (osc001_trig_source_t)3;
                CHECK(osc001_set_trigger(ht, &bad) == OSC001_ERR_ARG, "未知の source は ERR_ARG");
            }
            tc.source = OSC001_TRIG_EXT; tc.edge = OSC001_EDGE_FALLING; tc.level_raw = 0x20;
            CHECK(osc001_set_trigger(ht, &tc) == OSC001_OK &&
                  osc001_get_raw(ht, 12) == 1 && osc001_get_raw(ht, 13) == 0 &&
                  osc001_get_raw(ht, 15) == OSC001_TRIG_LEVEL_EXT && osc001_get_raw(ht, 19) == 3,
                  "EXT トリガ: byte15 は固定値、byte19=3");
            {
                char wbuf[2048];
                osc001_warnings(ht, wbuf, sizeof wbuf);
                CHECK(strstr(wbuf, "EXT") != NULL, "EXT の T 浮き警告");
            }

            /* Normal 未成立 → ERR_NO_TRIGGER、op8 は読まない */
            m.op6_fire_after = 1000;
            tc.mode = OSC001_TRIG_NORMAL;
            osc001_set_trigger(ht, &tc);
            {
                int n_send0 = m.n_send;
                int rc = osc001_capture(ht, &tr, 50);
                CHECK(rc == OSC001_ERR_NO_TRIGGER && tr.triggered == 0 && m.last_report[0] != 8,
                      "Normal 未成立 → ERR_NO_TRIGGER(op8 なし、送信 %d)", m.n_send - n_send0);
            }
            /* Auto 未成立 → byte12=0 で取り直し、次の arm で 1 に戻る */
            m.op6_fire_after = m.op5_count + 2;
            tc.mode = OSC001_TRIG_AUTO; tc.auto_timeout_ms = 30;
            osc001_set_trigger(ht, &tc);
            {
                int n5 = m.op5_count;
                CHECK(osc001_capture(ht, &tr, 500) == OSC001_OK && tr.triggered == 0 &&
                      m.op5_count == n5 + 2 && m.last_op5[12] == 0,
                      "Auto 未成立 → フリーランで取り直し(triggered=%d)", tr.triggered);
                CHECK(osc001_arm(ht) == OSC001_OK && m.last_op5[12] == 1,
                      "取り直しの後の arm で byte12=1 に戻る");
            }
            m.op6_fire_after = 0;
            CHECK(osc001_capture(ht, &tr, 500) == OSC001_OK && tr.triggered == 1, "成立 → triggered=1");
            CHECK(osc001_set_trigger(ht, NULL) == OSC001_OK && osc001_get_raw(ht, 12) == 0 &&
                  osc001_trigger_level_raw(ht) < 0, "set_trigger(NULL) でフリーラン");
            CHECK(osc001_capture(ht, &tr, 500) == OSC001_OK && tr.triggered == -1,
                  "トリガ無し → triggered=-1");
            osc001_close(ht);
        }
    }

    /* ── op9 モニタ・0 V 合わせ・途中読み ─────────────────────────────── */
    {
        osc001_handle *hm = open_mock(&m);
        osc001_monitor_t mo;
        osc001_trace_t tr;
        double mm;
        if (hm) {
            m.mon_enable = 1;
            m.mon[5] = 600; m.mon[8] = 497; m.mon[6] = 1019; m.mon[9] = 386;
            m.n_op9 = 0;
            CHECK(osc001_monitor(hm, 8, 3, &mo) == OSC001_OK && m.n_op9 == 3 &&
                  fabs(mo.volts[0] - (600 - 495.0) * 0.0479) < 1e-9 &&
                  fabs(mo.volts[1] - (497 - 496.5) * 0.0479) < 1e-9 && !mo.sat[0] && !mo.sat[1],
                  "monitor ÷8: op9(5,8) を 3 回、CH1 %.3f V / CH2 %.3f V", mo.volts[0], mo.volts[1]);
            CHECK(osc001_monitor(hm, 1, 1, &mo) == OSC001_OK && mo.sat[0] && !mo.sat[1],
                  "monitor ×1: m=1019 を張り付きとして検出");
            CHECK(osc001_monitor(hm, 3, 1, &mo) == OSC001_ERR_ARG, "monitor tap=3 は ERR_ARG");
            m.op8_const = 0x7F;                       /* code 0 */
            osc001_set_trigger(hm, NULL);
            osc001_set_range(hm, 1, 2.0, 0, OSC001_POS_AUTO);
            CHECK(osc001_calibrate_zero(hm, 3, 10, 1) == OSC001_OK, "calibrate_zero 成功");
            CHECK(osc001_zero_m0(hm, 1, 8, 5, &mm) == OSC001_OK && fabs(mm - 600.0) < 1e-9,
                  "calibrate_zero: op9 平均で m0(CH1 ÷8 段 5 は m0 = %.1f)", mm);
            /* code 0 → m0_gain = (0 + 127 − slope·pos)·2^段。÷8 段 1 は Position 0x80 → 510 */
            CHECK(osc001_zero_m0(hm, 1, 8, 1, &mm) == OSC001_OK && fabs(mm - 510.0) < 1e-9,
                  "calibrate_zero: レンジ別 m0(÷8 段 1 = %.1f)", mm);
            CHECK(osc001_get_raw(hm, 8) == 2 && osc001_get_raw(hm, 1) == 5,
                  "calibrate_zero 後に設定(2 V/div)が戻る");
            m.op8_const = 0;
            osc001_set_sample_rate(hm, 100.0);
            osc001_arm(hm);
            m.vtime += 250;                            /* 仮想時計で 250 ms 経過 */
            CHECK(osc001_read_partial(hm, &tr) == OSC001_OK && tr.valid >= 23 && tr.valid <= 25,
                  "read_partial: valid=%d(期待 ≒24)", tr.valid);
            {
                osc001_trigger_t tc;
                osc001_trigger_init(&tc);
                tc.source = OSC001_TRIG_EXT;
                osc001_set_trigger(hm, &tc);
                osc001_arm(hm);
                CHECK(osc001_read_partial(hm, &tr) == OSC001_ERR_STATE,
                      "ハードトリガ待ちの read_partial は ERR_STATE");
            }
            m.mon_enable = 0;
            osc001_close(hm);
        }
    }

    /* ── 脱出口 ───────────────────────────────────────────────────── */
    {
        uint8_t resp[OSC001_REPORT_LEN];
        CHECK(osc001_raw_command(h, 9, NULL, 0, resp) == OSC001_OK &&
              resp[0] == 9, "raw_command(op9)エコー");
    }

    osc001_close(h);
    printf("\n%d/%d 項目 OK\n", checks - fails, checks);
    return fails ? 1 : 0;
}
