/* osc001(PCB Scope)C API — libosc001
 *
 * プロトコル事実は python/osc001.py(実機検証済みの参照実装)と同一。
 * C99・外部依存なし。コアはトランスポート非依存(コールバック注入)
 *
 * 使い方(最小):
 *     osc001_transport_t t; char err[256];
 *     osc001_transport_hidraw(&t, "/dev/hidraw2");   // Linux の場合
 *     osc001_handle *h = osc001_open(&t, err, sizeof err);
 *     if (!h) { puts(err); return 1; }
 *     osc001_set_range(h, 1, 0.5, 0, OSC001_POS_AUTO);
 *     osc001_set_sample_rate(h, 10000.0);
 *     osc001_trace_t tr;
 *     if (osc001_capture(h, &tr, 5000) == OSC001_OK)
 *         printf("%d\n", osc001_trace_sample(&tr, 1, 0));
 *     osc001_close(h);
 */
#ifndef OSC001_H
#define OSC001_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 定数────────────────────────────────────────── */
#define OSC001_VENDOR          0x04D8
#define OSC001_PRODUCT         0xF891
#define OSC001_REPORT_LEN      64   /* IN/OUT 各 64 B、レポート ID なし */
#define OSC001_CHUNKS          6    /* read(): op8 chunk0..5 = 384 B */
#define OSC001_CHUNKS_DEEP     7    /* read_deep(): chunk0..6 */
#define OSC001_SAMPLES_PER_CH  192  /* 384 B = 2ch × 192(インターリーブ、偶数番=CH1) */
#define OSC001_BUFFER_BYTES    422  /* デバイス実バッファ有効長 */
#define OSC001_SAMPLES_DEEP    211
#define OSC001_OP_ECHO         0x08 /* op8 が無効索引で返す byte0 */
#define OSC001_FOOTER_OFFSET   38   /* chunk6 応答のフッター語列開始バイト */
#define OSC001_FOOTER_WORDS    13   /* 13 語 LE16 = 26 B */
#define OSC001_LA_CHUNKS       7
#define OSC001_LA_BUFFER_BYTES 448  /* LA: op8 chunk0..6 = 448 B(末尾 26 B はフッター) */
#define OSC001_LA_VALID_BYTES  422  /* 有効データ長(スコープの実バッファ長と同じ) */
#define OSC001_LA_SAMPLES      844  /* 有効ニブルサンプル = 422 × 2 */
#define OSC001_LA_RAW_SAMPLES  896  /* フッター込みのニブル数(復号には使わない) */
#define OSC001_LA_RATE_HZ      979400.0 /* 既定 fs(op16 byte3=3)。osc001_la_set_rate で変更 */
#define OSC001_LA_CLOCK_HZ     6.0e6    /* byte3=0 のとき fs = LA_CLOCK/(D+29) */
#define OSC001_CLOCK_HZ        3.0e6
#define OSC001_RATE_OFFSET     29   /* fs = CLOCK/(D+29) */
#define OSC001_RATE_MAX        33130.0 /* 実測の張り付き上限 SPS/ch */
#define OSC001_D_MIN           61.5   /* 実効下限除数(同上) */
/* 縦軸: 8 bit 値は 10 bit ADC 値 m の切り出し(生バイト ≒ m/2^段 − Position·2^(1−段))。
 * 1 カウントの電圧は両 ch 共通。ch ごとに違うのは 0 V の m(可変抵抗 R12/R13)→ osc001_calibrate_zero */
#define OSC001_VCOUNT_DIV8     47.9e-3  /* ÷8 タップ。外部電源 −11.94〜+18.91 V で当てた値 */
#define OSC001_VCOUNT_X1       5.04e-3  /* ×1 タップ(1 点からの見積もり、±2 % 程度) */
#define OSC001_MON_MAX         1023
#define OSC001_MON_X1_SAT      1019     /* ×1 の m はこの付近で張り付く(入力 ≒3.1 V 以上) */
#define OSC001_CHUNK_GAP_MS    0        /* op8 チャンク間の休み(0 でよい。旧版 20) */

/* ── エラーコード(0 = 成功)────────────────────────────────────────── */
#define OSC001_OK          0
#define OSC001_ERR_IO     -1  /* トランスポート失敗 */
#define OSC001_ERR_TIMEOUT -2 /* 応答なし */
#define OSC001_ERR_ARG    -3  /* 引数不正(★ハング領域のガード含む) */
#define OSC001_ERR_STATE  -4  /* 手順誤(未発火での op8 読出など) */
#define OSC001_ERR_PROTO  -5  /* プロトコル不整合(無効索引の応答など) */
#define OSC001_ERR_NO_TRIGGER -6 /* Normal トリガが待ち時間内に成立しない(op8 は読まない) */

/* ── トランスポート(コールバック一式)───────────────────────────────
 * send: 64 B レポートを書き出す(hidraw の 0x00 プレフィックスは実装側で吸収)。
 * recv: 64 B を timeout_ms 待つ。1=受信、0=タイムアウト、<0=失敗。
 * now_ms: 単調増加ミリ秒。sleep_ms: スリープ。
 * open が失敗した場合、open_errmsg が理由文字列を返してよい(NULL 可)。
 * 同時アクセスの排他(例: hidraw の flock)は実装側の責任。 */
typedef struct osc001_transport {
    int  (*open)(void *user);
    void (*close)(void *user);
    int  (*send)(void *user, const uint8_t *report);
    int  (*recv)(void *user, uint8_t *report, int timeout_ms);
    uint64_t (*now_ms)(void *user);
    void (*sleep_ms)(int ms);
    const char *(*open_errmsg)(void *user); /* open 失敗時の説明(任意) */
    void *user;
} osc001_transport_t;

/* ── セッション ────────────────────────────────────────────────────── */
typedef struct osc001_handle osc001_handle;

/* open + ドレイン(残応答の捨て読み)。失敗時は NULL で、errbuf に理由 */
osc001_handle *osc001_open(const osc001_transport_t *transport,
                           char *errbuf, size_t errlen);
void osc001_close(osc001_handle *h);
const char *osc001_errmsg(const osc001_handle *h);
void osc001_set_timeout(osc001_handle *h, int ms); /* 既定 2000 */

/* 情報: op2=製品文字列("DPScope SE")。op3=版番号 2 バイト(LE16) */
int osc001_info(osc001_handle *h, char *out, size_t outlen);
int osc001_version(osc001_handle *h, int *byte1, int *byte2);

/* ── 設定(op5 は arm 兼用。設定変更は arm 前に終えること)──────────── */
/* op5 の byte1..19 を直接指定する脱出口 */
int osc001_set_raw(osc001_handle *h, int index, int byte_value);
int osc001_get_raw(const osc001_handle *h, int index);

/* tap: 0=自動(ラベルが決定)1=×1 8=÷8 */
/* position: OSC001_POS_AUTO=レンジ別の既定補正 / _CENTER=中間を code0 へ
 * (段 0..2 のみ可達)/ _KEEP=触らない / それ以外=byte 値そのもの */
#define OSC001_POS_AUTO   (-1)
#define OSC001_POS_CENTER (-2)
#define OSC001_POS_KEEP   (-3)
/* v_per_div は 6 ラベル(2.0/1.0/0.5/0.2/0.1/0.05)のいずれか */
int osc001_set_range(osc001_handle *h, int ch, double v_per_div,
                     int tap, int position);
/* ゲイン段 0..5(各段ちょうど 1/2)。段 3..5 は Position 中央到達不能 */
int osc001_set_gain(osc001_handle *h, int ch, int gain, int tap, int position);
int osc001_set_position(osc001_handle *h, int ch, int byte_value);

/* per-ch Hz → 最も近い設定を適用し、実効 Hz を返す(<0 でエラー)。
 * 上限超は D を D_MIN に肩代わりさせ、実効値は RATE_MAX で頭打ち */
double osc001_set_sample_rate(osc001_handle *h, double hz);
double osc001_sample_rate(const osc001_handle *h); /* 現在の byte4..7 からの実効値 */
double osc001_v_per_lsb(const osc001_handle *h, int ch); /* VCOUNT[tap]·2^段(両 ch 共通) */

/* 0 V のときの m(10 bit 値)。tap: 1=×1 8=÷8、gain: 0..5。レンジ別の実測(calibrate_zero)があればそれ */
int osc001_zero_m0(const osc001_handle *h, int ch, int tap, int gain, double *m0);
/* 0 V の m を外から与える(保存しておいた calibrate_zero の結果など)。その tap のレンジ別の値は消す */
int osc001_set_zero_m0(osc001_handle *h, int ch, int tap, double m0);
/* 0 V が来るコード(サンプル値、実数)= m0/2^段 − 127 + POSITION_SLOPE[段]·Position。
 * 段 3..5 は式からの外挿(未実測) */
int osc001_zero_code(const osc001_handle *h, int ch, double *code);
/* 入力端子の電圧 → コード(サンプル値、実数)。生バイトはこれ + 0x7F */
int osc001_volts_to_code(const osc001_handle *h, int ch, double volts, double *code);

/* 設定状態の警告(改行区切り。なしなら空文字)。返り値は書いた長さ */
size_t osc001_warnings(const osc001_handle *h, char *buf, size_t buflen);

/* ── 取得(スコープ)────────────────────────────────────────────────── */
typedef struct {
    uint8_t  raw[OSC001_BUFFER_BYTES];
    int      n_bytes;      /* read()=384 / read_deep()=422 */
    double   rate_hz;      /* per-ch */
    double   v_per_lsb[2]; /* [0]=CH1、[1]=CH2(入力端子基準) */
    double   zero_code[2]; /* 取得時の 0 V コード(to_volts が引く) */
    int      valid;        /* 書き込み済みとみなせる先頭サンプル数/ch(read_partial のみ、他は全数) */
    int      tap[2];       /* 1=×1、8=÷8 */
    uint16_t footer[OSC001_FOOTER_WORDS]; /* read_deep のみ(chunk6 応答 [38..63]) */
    int      has_footer;
    int      stale;        /* 前回 read_deep と (パリティ語+ハッシュ) 一致。ヒューリスティック */
    int      power_cycle;  /* 静的語が前回基準と差分=再電源投入疑い(差分判定) */
    int      filled_ff;    /* 全バイト 0xFF=取得失敗(op5 byte3 不正など) */
    int      triggered;    /* -1=トリガ無し、1=成立、0=Auto で未成立 → フリーランで取り直し
                              (capture/capture_deep のみ。read 単体は -1) */
} osc001_trace_t;

/* エッジ。スコープのトリガは FALLING/RISING、LA は AUTO も使う(op16 byte8) */
typedef enum {
    OSC001_EDGE_FALLING = 0,
    OSC001_EDGE_RISING  = 1,
    OSC001_EDGE_AUTO    = 2
} osc001_edge_t;

/* ── ハードトリガ(op5 byte12/13/15/19)────────────────────────────────
 * 源は CH1 と EXT(T 入力)だけ。CH2 を源にする設定は無い(比較器は CH1 端子側に固定)。
 * トリガ点 = サンプル 0(プリトリガなし)。CH1 のレベルは arm のたびに現在のタップ・段・Position
 * から byte15 に換算し、経路 byte19 も CH1 のタップ(÷8 → 1、×1 → 2)に合わせる。
 * EXT のしきい値は固定(実測 立上り ≒1.45 V・立下り ≒1.42 V)で、byte15 は OSC001_TRIG_LEVEL_EXT に置く。
 * T 入力は浮かせると電圧が定まらず誤トリガし得る。 */
typedef enum {
    OSC001_TRIG_NONE = 0,   /* フリーラン(byte12=0) */
    OSC001_TRIG_CH1  = 1,
    OSC001_TRIG_EXT  = 2
} osc001_trig_source_t;
typedef enum {
    OSC001_TRIG_NORMAL = 0, /* 成立まで待つ。capture の待ち時間で ERR_NO_TRIGGER */
    OSC001_TRIG_AUTO   = 1  /* auto_timeout_ms 待って未成立ならフリーランで取り直す */
} osc001_trig_mode_t;
typedef struct {
    osc001_trig_source_t source;
    osc001_edge_t edge;     /* OSC001_EDGE_RISING / _FALLING(EXT でも有効) */
    double level_v;         /* CH1 のレベル(入力端子の電圧) */
    int level_raw;          /* 0x01..0xFE で byte15 を直指定(level_v より優先)。<0 で level_v を換算 */
    osc001_trig_mode_t mode;
    int auto_timeout_ms;    /* 0 = 既定(100 ms + 記録時間 384/fs) */
} osc001_trigger_t;
#define OSC001_TRIG_LEVEL_EXT 0x4D   /* EXT 時の byte15(0x00/0xFF は不安定) */
#define OSC001_TRIG_LEVEL_MIN 0x01   /* 0x00 は T=GND でも成立する */
#define OSC001_TRIG_LEVEL_MAX 0xFE   /* 0xFF はほぼ成立しない */
/* 既定値(NONE、立上り、0 V、level_raw=-1、NORMAL、auto 既定)で埋める */
void osc001_trigger_init(osc001_trigger_t *cfg);
/* 設定を検証して保持する。cfg=NULL または source=NONE でフリーラン。
 * レベルが CH1 の表示域外なら ERR_ARG(設定は変えない) */
int osc001_set_trigger(osc001_handle *h, const osc001_trigger_t *cfg);
int osc001_get_trigger(const osc001_handle *h, osc001_trigger_t *out);
/* 現在の設定で byte15 に入る値。フリーランか換算不能なら <0 */
int osc001_trigger_level_raw(const osc001_handle *h);

/* op5=設定適用兼 arm。トリガ設定があれば byte12/13/15/19 を作り直す(換算不能なら ERR_ARG) */
int osc001_arm(osc001_handle *h);
/* op6 byte0>0 を待つ(5ms 間隔。間隔ゼロのポーリングはファームを止めた実績がある)。記録完了(arm から 192/fs 秒、
 * トリガ時は成立後)で立つ。見たら read は待たない。1=見た 0=見ても応答なし(エラーではない) */
int osc001_wait_ready(osc001_handle *h, int timeout_ms);
/* chunk0..5。op6 の完了を見ていなければ arm から 192/fs 秒まで待つ */
int osc001_read(osc001_handle *h, osc001_trace_t *out);
/* 取得の途中で読む(遅い時間軸の逐次表示。フリーランのみ。トリガ待ち中は ERR_STATE)。
 * out->valid に arm からの経過で書き込み済みとみなせる先頭サンプル数を入れる(以降は前回の残り) */
int osc001_read_partial(osc001_handle *h, osc001_trace_t *out);
int osc001_read_deep(osc001_handle *h, osc001_trace_t *out); /* chunk0..6=422 B+フッター */
/* arm → 待ち → read。トリガ設定に従い out->triggered を埋める。
 * Normal で wait_timeout_ms 内に未成立なら ERR_NO_TRIGGER(out は triggered=0 だけ設定) */
int osc001_capture(osc001_handle *h, osc001_trace_t *out, int wait_timeout_ms);
int osc001_capture_deep(osc001_handle *h, osc001_trace_t *out, int wait_timeout_ms);

/* 復号: 生値(−127..+128)= byte − 0x7F。偶数番=CH1(インターリーブ) */
int osc001_trace_samples_per_ch(const osc001_trace_t *t);
int osc001_trace_sample(const osc001_trace_t *t, int ch, int i); /* 範囲外で INT_MIN */
/* 0 V 基準の電圧 = (サンプル − zero_code) × V/LSB */
int osc001_trace_to_volts(const osc001_trace_t *t, int ch, double *buf, int n);
int osc001_trace_rail_hit(const osc001_trace_t *t, int ch);  /* レール到達 */
int osc001_trace_clipped(const osc001_trace_t *t, int ch);   /* ×1 タップ 3.35V クリップ疑い */

/* ── op9 モニタ(10 bit 値の直接読み)────────────────────
 * セレクタ 5/6/8/9 = CH1 ÷8 / CH1 ×1 / CH2 ÷8 / CH2 ×1。ゲイン段・Position に依らない。
 * 1 回約 1 ms。スコープ取得の最中に読んでも取得を乱さない */
int osc001_monitor_raw(osc001_handle *h, int sel1, int sel2, int *m1, int *m2);
typedef struct {
    double volts[2];   /* [0]=CH1、[1]=CH2(0 V 基準) */
    double m[2];       /* n 回平均の m */
    int    sat[2];     /* 1 回でも張り付いた(÷8 で 0/1023、×1 で 0/1019 付近) */
} osc001_monitor_t;
/* tap=8: ±24 V を 47.9 mV 刻み / tap=1: −1.9〜+3.1 V を ≒5 mV 刻み。n 回の平均 */
int osc001_monitor(osc001_handle *h, int tap, int n, osc001_monitor_t *out);
/* 0 V 合わせ。**入力を GND につないだ状態で**呼ぶ。ch_mask: bit0=CH1 bit1=CH2。
 * op9 を duration_ms 平均して ÷8・×1 の m0 を測り、ranges≠0 なら 6 レンジでスコープも取得して
 * レンジ別の 0 V を実測する。op5 の設定・トリガは元に戻す */
int osc001_calibrate_zero(osc001_handle *h, int ch_mask, int duration_ms, int ranges);

/* ── LA(ロジックアナライザ)────────────────────────────────
 * 記録は発火点から 844 サンプル(プリトリガなし)。fs は osc001_la_set_rate で選ぶ:
 *   op16 byte3=3/2/1 → 979.4 k / 490.9 k / 200.0 kHz 固定、
 *   byte3=0 → byte4..7(スコープと同じ符号化の除数 D)で fs = 6e6/(D+29)、≦108.7 kHz、≧0.72 Sa/s。
 * op6 byte0=1 は窓を記録し終えた時点で立つ(la_wait の timeout は 844/fs を見込む)。
 * ★ ハング領域のガード: edge=AUTO(byte8=2) かつ channel_mask≠0 はハング領域なので
 *   OSC001_ERR_ARG で拒否する(byte8=2 かつ byte9=0 は実機安全確認済み)。
 *   信号動作中の arm は Auto・エッジとも 2026-10-17 の 90 回で再現せず(既知のハングはこの組み合わせのみ)。
 * 未発火の op8 は凍結バッファなので読出を拒否する。
 * 未接続の入力をマスクに含めると隣接 ch のクロストークで誤発火しうる。 */
/* osc001_edge_t はハードトリガの節で定義(LA では AUTO も使う) */
/* channel_mask: ch1=0x80 / ch2=0x40 / ch3=0x20 / ch4=0x10 の OR */
int osc001_la_configure(osc001_handle *h, int channel_mask, osc001_edge_t edge);
/* LA の fs 指定 → 最も近い設定を保持し実効 fs を返す(<0 でエラー)。次の la_configure から有効 */
double osc001_la_set_rate(osc001_handle *h, double hz);
double osc001_la_rate(const osc001_handle *h);
int osc001_la_arm(osc001_handle *h);    /* op3(0) → op16 */
/* op6 byte0==1(発火)。1/0。la_arm 前(または abort 後)は ERR_STATE:
 * op5 の後の op6 byte0 はスコープ側の状態で、LA の発火と区別できないため */
int osc001_la_wait(osc001_handle *h, int timeout_ms);
/* 待機の中止。マスク 0 のエッジ arm(op16 byte8=1, byte9=0)で残ったトリガを上書きする。
 * ★未発火のエッジ arm を残したまま op5(スコープ)へ戻り、その ch にエッジが来ると
 *   ファームが暴走して USB から外れる(2026-10-17 実測)。op5・Auto 再 arm・ゲート閉では
 *   解けない。osc001_arm() は未発火のエッジ arm が残っていれば自動でこの解除を先に行う。
 * 以後 la_wait/la_read は再 arm まで ERR_STATE */
int osc001_la_abort(osc001_handle *h);
typedef struct {
    uint8_t  raw[OSC001_LA_BUFFER_BYTES];   /* [0..421]=データ、[422..447]=フッター */
    uint16_t footer[OSC001_FOOTER_WORDS];   /* raw 末尾 26 B を 13 語 LE16 に展開 */
    double   rate_hz;                       /* 取得時の実効 fs */
} osc001_la_trace_t;
int osc001_la_read(osc001_handle *h, osc001_la_trace_t *out);
int osc001_la_capture(osc001_handle *h, int channel_mask, osc001_edge_t edge,
                      int timeout_ms, osc001_la_trace_t *out);
/* ニブル復号: 1 バイト=下位ニブル先 2 サンプル。ch1=bit3..ch4=bit0 */
int osc001_la_bit(const osc001_la_trace_t *t, int ch, int i); /* 0..843。範囲外 INT_MIN */
int osc001_la_transitions(const osc001_la_trace_t *t, int ch);

/* ── 脱出口: 生の HID コマンド(op と args をそのまま)──────────────
 * ★ op16 はハング領域(edge=AUTO かつ mask≠0)を含む。byte8/byte9 の組合せに注意。 */
int osc001_raw_command(osc001_handle *h, int op,
                       const uint8_t *args, int nargs, uint8_t *resp64);

/* ── Linux hidraw トランスポート(/dev/hidrawN 直叩き。flock 排他)───
 * hidapi には依存しない(Python 実装と同じ経路。SBC で依存を増やさない)。 */
#ifdef __linux__
/* t を hidraw 実装で満たす。path は内部コピー。open は osc001_open が行う */
int osc001_transport_hidraw(osc001_transport_t *t, const char *hidraw_path);
/* 04d8:f891 の hidraw を列挙。/sys/class/hidraw を scan。
 * paths[i] は 64 B バッファに "/dev/hidrawN"。names が NULL でなければ HID_NAME。
 * 見つかった個数を返す(0 でもエラーではない) */
int osc001_enumerate_hidraw(char paths[][64], int max, char names[][64]);
/* osc001_transport_hidraw() が確保する内部 ctx を解放する。
 * osc001_close() の「後」に呼ぶこと。呼んだ t は以後 open に再利用しない
 * (コールバックは NULL 埋めになる)。C++ ラッパ(osc001.hpp)は析出時に自動で呼ぶ */
void osc001_transport_hidraw_release(osc001_transport_t *t);
#endif

#ifdef __cplusplus
}
#endif
#endif /* OSC001_H */
