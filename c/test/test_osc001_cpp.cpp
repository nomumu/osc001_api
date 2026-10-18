/* osc001.hpp(C++ ラッパ)オフラインテスト。モックは C テストと同じ。
 * C API 側の詳細検証は test_osc001.c(38 項目)が担うので、
 * ここではラッパ固有の挙除(RAII・例外・vector 化・ハング領域のガード)を検証する。
 * ビルド: make check (c/ ディレクトリ) */

#include <climits>
#include <cmath>
#include <cstdio>
#include <string>

#include "../include/osc001.hpp"
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

/* 例外を投げるはずの呼び出しを包む。code が一致すれば OK */
#define CHECK_THROWS(expr, expect_code, ...) do {                    \
    checks++;                                                        \
    bool caught = false;                                             \
    try { expr; }                                                    \
    catch (const osc001::Error &e) {                                 \
        caught = true;                                               \
        if (e.code() != (expect_code)) {                             \
            fails++;                                                 \
            printf("FAIL(%d): code=%d 期待 %d (%s)\n",               \
                   __LINE__, e.code(), (expect_code), e.what());     \
            break;                                                   \
        }                                                            \
    }                                                                \
    if (!caught) { fails++; printf("FAIL(%d): 例外なし: ", __LINE__); \
                   printf(__VA_ARGS__); printf("\n"); }              \
    else { printf("OK:  "); printf(__VA_ARGS__); printf("\n"); }     \
} while (0)

int main()
{
    mock_dev m;
    mock_init(&m);

    /* ── 自前トランスポート注入 + 情報 ── */
    osc001::Scope sc(m.t);
    CHECK(sc.info() == "DPScope SE", "info() 文字列化");
    {
        std::pair<int, int> v = sc.version();
        CHECK(v.first == 0x05 && v.second == 0x01, "version() = (%02X,%02X)",
              v.first, v.second);
    }

    /* ── 設定 + 例外 ── */
    CHECK(std::fabs(sc.setSampleRate(10000.0) - 10000.0) < 100.0,
          "setSampleRate 実効値返却(%.1f)", sc.sampleRate());
    CHECK(sc.setSampleRate(40000.0) <= OSC001_RATE_MAX + 0.5,
          "40kHz は %.0f で頭打ち", sc.sampleRate());
    sc.setRange(1, 0.5);
    CHECK(std::fabs(sc.vPerLsb(1) - OSC001_VCOUNT_DIV8) < 1e-12,
          "CH1 0.5V/div V/LSB=%.6f", sc.vPerLsb(1));
    CHECK_THROWS(sc.setRange(1, 0.7), OSC001_ERR_ARG, "0.7V/div は ERR_ARG 例外");
    CHECK_THROWS(sc.vPerLsb(7), OSC001_ERR_ARG, "vPerLsb(ch 不正)例外");

    /* byte3 不正 → warnings() 文字列に載る */
    sc.setRaw(3, 0x28);
    CHECK(!sc.warnings().empty(), "warnings() 文字列化(byte3 不正)");
    sc.setRaw(3, 0xAA);
    CHECK(sc.warnings().empty(), "warnings() 正常設定で空");

    /* ── 取得(std::vector 化)── */
    {
        osc001::Trace tr = sc.capture();
        CHECK(tr.nBytes() == 384 && tr.samplesPerCh() == 192,
              "capture 384B/192 サンプル");
        /* chunk 応答 byte0 は ACK エコーで raw[0]/raw[64]… を汚染する(C テストと同じ)。
         * chunk0: raw[0]=エコー(0)、raw[1]=1 → sample(1,0)=0−0x7F、sample(2,0)=1−0x7F */
        CHECK(tr.sample(1, 0) == 0 - 0x7F && tr.sample(2, 0) == 1 - 0x7F,
              "インターリーブ復号(偶数番=CH1・エコー汚染込み)");
        CHECK(tr.sample(1, 192) == INT_MIN, "範囲外サンプルは INT_MIN");
        std::vector<double> v = tr.volts(1);
        CHECK(v.size() == 192 &&
              std::fabs(v[0] - (0 - 0x7F - tr.zeroCode(1)) * sc.vPerLsb(1)) < 1e-9 &&
              std::fabs(tr.zeroCode(1) - sc.zeroCode(1)) < 1e-12 && tr.valid() == 192,
              "volts() 192 個・0 V 基準(zeroCode を引く)");
        CHECK(!tr.filledFf() && !tr.hasFooter(), "capture はフッターなし");
    }

    /* ── 完全版(フッター・stale)── */
    {
        osc001::Trace d1 = sc.captureDeep();
        CHECK(d1.hasFooter() && d1.footer().size() == OSC001_FOOTER_WORDS &&
              d1.footer()[5] == 0x1005, "captureDeep フッター語(footer[5]=0x1005)");
        osc001::Trace d2 = sc.captureDeep();
        CHECK(d2.stale() && !d1.stale(), "2 回目同一フッター → stale()");
    }

    /* ── 0xFF 埋め(fail 再現)── */
    {
        mock_dev mf;
        mock_init(&mf);
        mf.fill_ff = 1;
        osc001::Scope scf(mf.t);
        /* filled_ff は read_deep 側で計算される(C テストと同じ経路) */
        osc001::Trace tr = scf.captureDeep();
        CHECK(tr.filledFf(), "fill_ff → filledFf()");
    }

    /* ── LA + ハング領域のガード ── */
    CHECK_THROWS(sc.laCapture(0x80, OSC001_EDGE_AUTO), OSC001_ERR_ARG,
                 "ハング領域のガード: AUTO+mask≠0 は ERR_ARG 例外");
    CHECK_THROWS(sc.laRead(), OSC001_ERR_STATE, "未発火 laRead は ERR_STATE 例外");
    {
        m.la_mode = 1;   /* op8 データを 0xA5 ニブルパターンに(C テストと同じ) */
        osc001::LaTrace la = sc.laCapture(0x80, OSC001_EDGE_RISING);
        /* raw[0]/raw[64]… はエコー汚染。i=2,3 が最初の clean バイト(0xA5)の
         * 下位/上位ニブル(C テストと同じ)。0xA5=1010 0101、ch1=bit3 */
        CHECK(la.bit(1, 2) == 0 && la.bit(1, 3) == 1,
              "LA ニブル復号(0xA5: 下位先で ch1=0,1)");
        CHECK(la.transitions(1) > 0, "LA 遷移数 %d", la.transitions(1));
        CHECK(la.samples() == 844 && la.rateHz() == OSC001_LA_RATE_HZ &&
              la.footer().size() == OSC001_FOOTER_WORDS, "LA 有効長・fs・フッター");
        CHECK(sc.laSetRate(2e5) == 200000.0 && sc.laRate() == 200000.0, "laSetRate 固定段");
        sc.laSetRate(OSC001_LA_RATE_HZ);
        sc.laArm();
        sc.laAbort();
        CHECK_THROWS(sc.laWait(10), OSC001_ERR_STATE, "abort 後 laWait は ERR_STATE 例外");
    }

    /* ── Position の 0 V 切片とハードトリガ ── */
    {
        sc.setRange(1, 1.0);
        CHECK(std::fabs(sc.zeroCode(1) - (-7.5)) < 1e-9, "zeroCode CH1 ÷8 段1 = %.2f", sc.zeroCode(1));
        CHECK(std::fabs(sc.voltsToCode(1, 2.5) - (-7.5 + 2.5 / sc.vPerLsb(1))) < 1e-9, "voltsToCode");
        CHECK(std::fabs(sc.zeroM0(2, 1, 0) - 388.0) < 1e-9, "zeroM0 既定値(CH2 ×1)");
        sc.setZeroM0(1, 8, 497.0);
        CHECK(std::fabs(sc.zeroCode(1) - (497.0 / 2 - 127 - 128)) < 1e-9, "setZeroM0 が zeroCode に効く");
        sc.setZeroM0(1, 8, 495.0);
        m.mon_enable = 1;
        m.mon[5] = 600; m.mon[8] = 497;
        {
            osc001_monitor_t mo = sc.monitor(8, 2);
            std::pair<int, int> r = sc.monitorRaw(5, 8);
            CHECK(std::fabs(mo.volts[0] - (600 - 495.0) * OSC001_VCOUNT_DIV8) < 1e-9 &&
                  r.first == 600 && r.second == 497, "monitor / monitorRaw");
        }
        CHECK_THROWS(sc.monitor(3), OSC001_ERR_ARG, "monitor tap 不正は ERR_ARG 例外");
        m.mon_enable = 0;
        sc.setTriggerCh1(OSC001_EDGE_RISING, 2.5);
        CHECK(sc.getRaw(12) == 1 && sc.getRaw(19) == 1 && sc.triggerLevelRaw() == sc.getRaw(15) &&
              sc.trigger().source == OSC001_TRIG_CH1, "setTriggerCh1 のバイト");
        CHECK_THROWS(sc.setTriggerCh1(OSC001_EDGE_RISING, 20.0), OSC001_ERR_ARG, "表示域外レベルは ERR_ARG 例外");
        sc.setTriggerExt(OSC001_EDGE_FALLING);
        CHECK(sc.getRaw(19) == 3 && sc.getRaw(15) == OSC001_TRIG_LEVEL_EXT, "setTriggerExt のバイト");
        m.op6_fire_after = 1000;
        bool thrown = false;
        try { (void)sc.capture(50); } catch (const osc001::NoTrigger &e) { thrown = (e.code() == OSC001_ERR_NO_TRIGGER); }
        CHECK(thrown, "Normal 未成立 → NoTrigger 例外");
        m.op6_fire_after = m.op5_count + 2;
        sc.setTriggerExt(OSC001_EDGE_RISING, OSC001_TRIG_AUTO, 30);
        CHECK(sc.capture(500).triggered() == 0, "Auto 未成立 → triggered()=0");
        m.op6_fire_after = 0;
        CHECK(sc.capture(500).triggered() == 1, "成立 → triggered()=1");
        sc.setFreeRun();
        CHECK(sc.getRaw(12) == 0 && sc.capture(500).triggered() == -1, "setFreeRun → triggered()=-1");
        sc.setRange(1, 0.5);
    }

    /* ── 脱出口 ── */
    {
        std::vector<uint8_t> r = sc.rawCommand(9);
        CHECK(r[0] == 9, "rawCommand(op9)エコー");
    }

    /* ── RAII: 破棄後にモックへ close が届いている ── */
    {
        mock_dev m2;
        mock_init(&m2);
        int sends_before = m2.n_send;
        {
            osc001::Scope sc2(m2.t);
            (void)sc2.info();
        }   /* デストラクタで osc001_close → mock_close 相当。ここでは
               open 時のドレイン含め send が増えていることで疎通を確認 */
        CHECK(m2.n_send > sends_before, "Scope 生存中に送信発生(n_send=%d)", m2.n_send);
    }

    printf("\n%d/%d 項目 OK\n", checks - fails, checks);
    return fails ? 1 : 0;
}
