/* osc001 C++ ラッパ — ヘッダオンリ(C++11)。C API(osc001.h)の薄い RAII 覆い。
 * プロトコル事実は C 側(osc001.h)と同一。
 *  - Scope は RAII。デストラクタで osc001_close。
 *    Linux 組み込みの hidraw トランスポートを使った場合は ctx 解放も行う。
 *  - 失敗は osc001::Error(C エラーコード付き例外)で投げる。
 *    例外無効ビルド(Arduino 等)ではこのラッパを使わず C API を直接使うこと。
 *  - 復号結果は std::vector で返す。生バッファも raw() で参照できる。
 *
 * 使い方(Linux / Raspberry Pi):
 *     #include "osc001.hpp"
 *     osc001::Scope sc;                    // 04d8:f891 を自動探索
 *     sc.setRange(1, 0.5);                 // CH1 = 0.5 V/div(tap 自動・位置 AUTO)
 *     sc.setSampleRate(10000.0);
 *     osc001::Trace tr = sc.capture();
 *     std::vector<double> v = tr.volts(1); // 校正済み電圧
 *
 * 使い方(自前トランスポート。M5Stack/TinyUSB 等):
 *     osc001_transport_t t = ...;          // コールバックを自分で埋める
 *     osc001::Scope sc(t);                 // t の生存期間は呼び出し側が保証
 */
#ifndef OSC001_HPP
#define OSC001_HPP

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "osc001.h"

namespace osc001 {

/* C API の失敗(rc<0 / NULL)が対応。code() は OSC001_ERR_* */
class Error : public std::runtime_error {
    int code_;
public:
    Error(int code, const std::string &what)
        : std::runtime_error(what), code_(code) {}
    int code() const { return code_; }
};

/* Normal トリガが待ち時間内に成立しなかった(code() == OSC001_ERR_NO_TRIGGER) */
class NoTrigger : public Error {
public:
    explicit NoTrigger(const std::string &what) : Error(OSC001_ERR_NO_TRIGGER, what) {}
};

/* ── 取得結果(スコープ)────────────────────────────────────────── */
class Trace {
    osc001_trace_t t_;
    friend class Scope;
public:
    Trace() : t_() {}                    /* ゼロ埋め(未取得) */

    int    samplesPerCh() const { return osc001_trace_samples_per_ch(&t_); }
    int    sample(int ch, int i) const { return osc001_trace_sample(&t_, ch, i); }
    /* 0 V 基準の電圧(入力端子)=(サンプル − zeroCode)× V/LSB。ch 不正は Error */
    std::vector<double> volts(int ch) const {
        int n = samplesPerCh();
        std::vector<double> v;
        if (n <= 0)
            return v;
        v.resize(static_cast<size_t>(n));
        int rc = osc001_trace_to_volts(&t_, ch, &v[0], n);
        if (rc < 0)
            throw Error(rc, "volts: ch 不正または未取得");
        return v;
    }
    bool railHit(int ch) const  { return osc001_trace_rail_hit(&t_, ch) != 0; }
    bool clipped(int ch) const  { return osc001_trace_clipped(&t_, ch) != 0; }

    double rateHz() const       { return t_.rate_hz; }
    double vPerLsb(int ch) const { return t_.v_per_lsb[(ch == 2) ? 1 : 0]; }
    double zeroCode(int ch) const { return t_.zero_code[(ch == 2) ? 1 : 0]; }   /* 取得時の 0 V コード */
    int    valid() const        { return t_.valid; }   /* readPartial: 書き込み済みの先頭サンプル数/ch */
    int    tap(int ch) const    { return t_.tap[(ch == 2) ? 1 : 0]; }
    int    nBytes() const       { return t_.n_bytes; }

    bool hasFooter() const  { return t_.has_footer != 0; }
    bool stale() const      { return t_.stale != 0; }        /* ヒューリスティック */
    bool powerCycle() const { return t_.power_cycle != 0; }
    bool filledFf() const   { return t_.filled_ff != 0; }    /* 全 0xFF=取得失敗 */
    /* capture のみ: -1=トリガ無し、1=成立、0=Auto で未成立 → フリーランで取り直し */
    int  triggered() const  { return t_.triggered; }
    std::vector<uint16_t> footer() const {
        std::vector<uint16_t> f;
        if (!t_.has_footer)
            return f;
        f.assign(t_.footer, t_.footer + OSC001_FOOTER_WORDS);
        return f;
    }

    const osc001_trace_t &raw() const { return t_; }
};

/* ── 取得結果(LA)──────────────────────────────────────── */
class LaTrace {
    osc001_la_trace_t t_;
    friend class Scope;
public:
    LaTrace() : t_() {}

    /* ニブル復号: 1 バイト=下位ニブル先 2 サンプル。ch1=bit3..ch4=bit0。
     * i は 0..843(OSC001_LA_SAMPLES 未満)、範囲外は INT_MIN */
    int bit(int ch, int i) const { return osc001_la_bit(&t_, ch, i); }
    int transitions(int ch) const { return osc001_la_transitions(&t_, ch); }
    int samples() const { return OSC001_LA_SAMPLES; }
    double rateHz() const { return t_.rate_hz; }
    std::vector<uint16_t> footer() const {
        return std::vector<uint16_t>(t_.footer, t_.footer + OSC001_FOOTER_WORDS);
    }

    const osc001_la_trace_t &raw() const { return t_; }
};

/* ── セッション(RAII)──────────────────────────────────────────── */
class Scope {
    osc001_handle *h_;
    osc001_transport_t t_;
    bool ownsTransport_;                 /* hidraw 組み込み時 true(析出時に ctx 解放) */

    Scope(const Scope &) = delete;
    Scope &operator=(const Scope &) = delete;

    void open_() {
        char err[256];
        h_ = osc001_open(&t_, err, sizeof err);
        if (!h_)
            throw Error(OSC001_ERR_IO, std::string("open 失敗: ") + err);
    }
    void chk(int rc) const {
        if (rc == OSC001_ERR_NO_TRIGGER)
            throw NoTrigger(osc001_errmsg(h_));
        if (rc < 0)
            throw Error(rc, h_ ? osc001_errmsg(h_) : "未オープン");
    }

public:
#ifdef __linux__
    /* 04d8:f891 の hidraw を列挙して最初に接続。見つからなければ Error */
    Scope() {
        char paths[8][64];
        int n = osc001_enumerate_hidraw(paths, 8, 0);
        if (n <= 0)
            throw Error(OSC001_ERR_IO,
                        "04d8:f891 の hidraw が見つからない"
                        "(lsusb にあるか確認。ハング後は物理抜き差し)");
        if (osc001_transport_hidraw(&t_, paths[0]) != 0)
            throw Error(OSC001_ERR_ARG, "hidraw path 不正");
        ownsTransport_ = true;
        open_();
    }
    /* ノード明示(例 "/dev/hidraw2") */
    explicit Scope(const std::string &hidrawPath) {
        if (osc001_transport_hidraw(&t_, hidrawPath.c_str()) != 0)
            throw Error(OSC001_ERR_ARG, "hidraw path 不正(空または長すぎ)");
        ownsTransport_ = true;
        open_();
    }
#endif
    /* 自前トランスポート(TinyUSB 等)。t の生存期間は呼び出し側が保証
     * (Scope の破棄より後に t を壊すこと) */
    explicit Scope(const osc001_transport_t &transport)
        : h_(0), t_(transport), ownsTransport_(false) { open_(); }

    ~Scope() {
        if (h_)
            osc001_close(h_);
        if (ownsTransport_)
            osc001_transport_hidraw_release(&t_);
    }

    Scope(Scope &&o) noexcept
        : h_(o.h_), t_(o.t_), ownsTransport_(o.ownsTransport_) {
        o.h_ = 0;
        o.ownsTransport_ = false;
    }
    Scope &operator=(Scope &&o) noexcept {
        if (this != &o) {
            if (h_)
                osc001_close(h_);
            if (ownsTransport_)
                osc001_transport_hidraw_release(&t_);
            h_ = o.h_;
            t_ = o.t_;
            ownsTransport_ = o.ownsTransport_;
            o.h_ = 0;
            o.ownsTransport_ = false;
        }
        return *this;
    }

    osc001_handle *handle() const { return h_; }   /* C API への脱出口 */

    /* ── 情報・設定 ── */
    void setTimeout(int ms) { osc001_set_timeout(h_, ms); }

    std::string info() const {                       /* op2 製品文字列 */
        char buf[128];
        chk(osc001_info(h_, buf, sizeof buf));
        return std::string(buf);
    }
    std::pair<int, int> version() const {            /* op3 版番号 2 バイト */
        int b1 = 0, b2 = 0;
        chk(osc001_version(h_, &b1, &b2));
        return std::make_pair(b1, b2);
    }

    void setRaw(int index, int byteValue) { chk(osc001_set_raw(h_, index, byteValue)); }
    int  getRaw(int index) const {
        int v = osc001_get_raw(h_, index);
        if (v < 0)
            throw Error(OSC001_ERR_ARG, "getRaw: index 不正");
        return v;
    }

    /* tap: 0=自動 1=×1 8=÷8 / position: POS_AUTO・POS_CENTER・POS_KEEP・byte 値 */
    void setRange(int ch, double vPerDiv, int tap = 0, int position = OSC001_POS_AUTO) {
        chk(osc001_set_range(h_, ch, vPerDiv, tap, position));
    }
    void setGain(int ch, int gain, int tap = 0, int position = OSC001_POS_AUTO) {
        chk(osc001_set_gain(h_, ch, gain, tap, position));
    }
    void setPosition(int ch, int byteValue) { chk(osc001_set_position(h_, ch, byteValue)); }

    double setSampleRate(double hz) {                /* 実効 Hz を返す */
        double r = osc001_set_sample_rate(h_, hz);
        if (r < 0)
            throw Error(OSC001_ERR_ARG, "setSampleRate: 引数不正");
        return r;
    }
    double sampleRate() const { return osc001_sample_rate(h_); }
    double vPerLsb(int ch) const {
        /* C 側は ch 不正時に 0.0 を返す(負にしない)のでここで弾く */
        if (ch != 1 && ch != 2)
            throw Error(OSC001_ERR_ARG, "vPerLsb: ch 不正");
        return osc001_v_per_lsb(h_, ch);
    }
    /* 0 V が来るコード / 電圧 → コード(サンプル値)。m0/2^段 − 127 + slope·Position */
    double zeroCode(int ch) const {
        double c = 0.0;
        int rc = osc001_zero_code(h_, ch, &c);
        if (rc < 0) throw Error(rc, "zeroCode: ch 不正か使用不可のタップ値");
        return c;
    }
    double voltsToCode(int ch, double volts) const {
        double c = 0.0;
        int rc = osc001_volts_to_code(h_, ch, volts, &c);
        if (rc < 0) throw Error(rc, "voltsToCode: ch 不正か使用不可のタップ値");
        return c;
    }
    /* 0 V の m(10 bit 値)。tap: 1=×1 8=÷8、gain: 0..5 */
    double zeroM0(int ch, int tap, int gain) const {
        double m = 0.0;
        chk(osc001_zero_m0(h_, ch, tap, gain, &m));
        return m;
    }
    void setZeroM0(int ch, int tap, double m0) { chk(osc001_set_zero_m0(h_, ch, tap, m0)); }
    /* 0 V 合わせ。入力を GND につないで呼ぶ。chMask: bit0=CH1 bit1=CH2 */
    void calibrateZero(int chMask = 3, int durationMs = 1000, bool ranges = true) {
        chk(osc001_calibrate_zero(h_, chMask, durationMs, ranges ? 1 : 0));
    }

    /* ── op9 モニタ(設定に依らない 10 bit 値の直接読み)── */
    std::pair<int, int> monitorRaw(int sel1, int sel2) {
        int a = 0, b = 0;
        chk(osc001_monitor_raw(h_, sel1, sel2, &a, &b));
        return std::make_pair(a, b);
    }
    osc001_monitor_t monitor(int tap = 8, int n = 1) {
        osc001_monitor_t mo;
        chk(osc001_monitor(h_, tap, n, &mo));
        return mo;
    }

    /* ── ハードトリガ(源は CH1 と EXT だけ。CH2 は無い)── */
    void setTrigger(const osc001_trigger_t &cfg) { chk(osc001_set_trigger(h_, &cfg)); }
    void setTriggerCh1(osc001_edge_t edge, double levelV,
                       osc001_trig_mode_t mode = OSC001_TRIG_NORMAL, int autoTimeoutMs = 0) {
        osc001_trigger_t c;
        osc001_trigger_init(&c);
        c.source = OSC001_TRIG_CH1; c.edge = edge; c.level_v = levelV;
        c.mode = mode; c.auto_timeout_ms = autoTimeoutMs;
        setTrigger(c);
    }
    void setTriggerExt(osc001_edge_t edge,
                       osc001_trig_mode_t mode = OSC001_TRIG_NORMAL, int autoTimeoutMs = 0) {
        osc001_trigger_t c;
        osc001_trigger_init(&c);
        c.source = OSC001_TRIG_EXT; c.edge = edge;
        c.mode = mode; c.auto_timeout_ms = autoTimeoutMs;
        setTrigger(c);
    }
    void setFreeRun() { chk(osc001_set_trigger(h_, 0)); }
    osc001_trigger_t trigger() const {
        osc001_trigger_t c;
        osc001_get_trigger(h_, &c);
        return c;
    }
    int triggerLevelRaw() const { return osc001_trigger_level_raw(h_); }   /* <0 = 無し/換算不能 */

    std::string warnings() const {                   /* 改行区切り。なしは空 */
        char buf[2048];
        size_t n = osc001_warnings(h_, buf, sizeof buf);
        if (n >= sizeof buf) n = sizeof buf - 1;      /* C 側は溢れても欲しかった長さを返す */
        return std::string(buf, n);
    }

    /* ── 取得(スコープ)── */
    void arm() { chk(osc001_arm(h_)); }
    bool waitReady(int timeoutMs = 2000) { return osc001_wait_ready(h_, timeoutMs) == 1; }
    Trace read() { Trace tr; chk(osc001_read(h_, &tr.t_)); return tr; }
    Trace readDeep() { Trace tr; chk(osc001_read_deep(h_, &tr.t_)); return tr; }
    /* 取得の途中読み(フリーランのみ)。Trace::valid() までが書き込み済み */
    Trace readPartial() { Trace tr; chk(osc001_read_partial(h_, &tr.t_)); return tr; }
    /* Normal トリガが未成立なら NoTrigger を投げる */
    Trace capture(int waitTimeoutMs = 5000) {
        Trace tr; chk(osc001_capture(h_, &tr.t_, waitTimeoutMs)); return tr;
    }
    Trace captureDeep(int waitTimeoutMs = 5000) {
        Trace tr; chk(osc001_capture_deep(h_, &tr.t_, waitTimeoutMs)); return tr;
    }

    /* ── LA──
     * 発火点から 844 サンプル。fs は laSetRate(979.4k/490.9k/200k 固定段 + ≦108.7 kHz の可変域)。
     * ★ ハング領域のガード: EDGE_AUTO かつ mask≠0 は Error(ERR_ARG)で拒否。
     * laWait は arm 前/abort 後に呼ぶと Error(ERR_STATE)。 */
    void laConfigure(int channelMask, osc001_edge_t edge) {
        chk(osc001_la_configure(h_, channelMask, edge));
    }
    double laSetRate(double hz) {   /* 実効 fs を返す。次の laConfigure から有効 */
        double fs = osc001_la_set_rate(h_, hz);
        if (fs < 0) chk((int)fs);
        return fs;
    }
    double laRate() const { return osc001_la_rate(h_); }
    void laArm() { chk(osc001_la_arm(h_)); }
    bool laWait(int timeoutMs = 5000) {
        int rc = osc001_la_wait(h_, timeoutMs);
        if (rc < 0) chk(rc);
        return rc == 1;
    }
    void laAbort() { chk(osc001_la_abort(h_)); }   /* マスク 0 のエッジ arm で待機を解く */
    LaTrace laRead() { LaTrace tr; chk(osc001_la_read(h_, &tr.t_)); return tr; }
    LaTrace laCapture(int channelMask, osc001_edge_t edge, int timeoutMs = 5000) {
        LaTrace tr;
        chk(osc001_la_capture(h_, channelMask, edge, timeoutMs, &tr.t_));
        return tr;
    }

    /* ── 脱出口: 生の HID コマンド。★ op16 はハング領域(EDGE_AUTO かつ mask≠0)を含む ── */
    std::vector<uint8_t> rawCommand(int op, const uint8_t *args = 0, int nargs = 0) {
        uint8_t resp[OSC001_REPORT_LEN];
        chk(osc001_raw_command(h_, op, args, nargs, resp));
        return std::vector<uint8_t>(resp, resp + OSC001_REPORT_LEN);
    }
};

} /* namespace osc001 */

#endif /* OSC001_HPP */
