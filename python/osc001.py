#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""osc001 の Python API プロトタイプ(2026-10-11)。

このモジュールが実装するプロトコル(実機で確定済み):
  * USB HID 04d8:f891、IN/OUT 64 バイト、レポート ID なし。
    hidraw への書込みは 0x00 プレフィックス必須。
  * 取得モデル: op5(SET_REG)= 20 バイト書込みが「設定適用兼 arm」。
    → 記録は 192/fs 秒で終わり、op6 byte0 がその直後に立つ。read() は op6 を見ていれば待たず、
      見ていなければ 192/fs 秒まで待つ(待たないと塗りかけ。途中読みは read_partial)。
    → op8 chunk0..5 = 384 バイト = 2ch × 192 サンプル(インターリーブ、偶数番=CH1)。
  * サンプル = IN[i] − 0x7F。
  * fs:
        X = 65536 − (byte4·256 + byte5)            ; 16 ビット 2 の補数
        D = X/2 (byte6 の bit0 が立っているとき、byte7 は無視)
          = 2^(byte7 & 7) · X (bit0 が 0 のとき)
        fs(per-ch) = 3e6 / (max(D, D_MIN) + 29)
    旧式(byte4=255 → 3e6/(285−byte5)、byte4<255 → 3e6/(255·(256−byte4)))は
    この式の特殊ケース / 近似。到達域は 0.36 SPS 〜 33.13 kSPS/ch で、穴はない。
    max(D, D_MIN) は高レート側の張り付きで、実効下限除数 D_MIN = 61.5。
  * レンジ = tap(byte1/byte2)× 段(byte8/byte9)の積。6 ラベルの対応は RANGES。
  * 縦軸: スコープの 8 bit 値は 10 bit ADC 値 m の切り出し。
        生バイト ≒ m / 2^段 − Position·2^(1−段)
    m は op9 で直接読める(monitor)。1 カウントの電圧は両 ch 共通(÷8 47.9 mV、×1 ≒5.04 mV)で、
    ch ごとに違うのは 0 V の m(m0。基板の可変抵抗 R12/R13 で動く)だけ。calibrate_zero で測り直す。
  * ハードトリガ(op5 byte12/13/15/19)の源は CH1 と EXT(T 入力)だけ。CH2 の経路は無い。
    トリガ点 = サンプル 0(プリトリガなし)。set_trigger() 参照。

byte7 は 0..7 を実機で検証済み。**8..15 は下位 3 ビットへ折り返る**ことが
実測で確定。折り返しを知らないと呼び出し側は誤った除数を計算するので、
0..7 以外は警告して `byte7 & 7` に丸める。

使い方: python/demo_capture.py を参照。stdlib のみ。Linux の hidraw 前提。
"""

import builtins
import fcntl
import hashlib
import os
import re
import select
import struct
import sys
import time

VENDOR = 0x04D8
PRODUCT = 0xF891
REPORT_LEN = 64
SAMPLES_PER_CH = 192          # op8 chunk0..5 = 384 バイト = 2ch × 192
CHUNKS = 6
#   op8 の有効なチャンク索引は 0..6。索引 7 以上は応答 byte0 が 0x08(op エコー)に
#   なって chunk6 の残骸を返すだけ。実バッファの有効長は **422 バイト
#   = 211 サンプル/ch** で、fs を 91〜33130 SPS/ch の範囲で振っても不変。
#   通常読む 384 B はこの一部で、後方に 19 サンプル/ch の余りがある。
CHUNKS_DEEP = 7               # 索引 0..6
BUFFER_BYTES = 422            # 有効データ長
SAMPLES_DEEP = BUFFER_BYTES // 2      # 211 サンプル/ch
OP_ECHO = 0x08                # op8 が索引無効で返す byte0
DEFAULT_DEVICE = "/dev/hidraw2"

# ── ステータスフッター─────────────────────
#   応答末尾 [38..63] の 26 B = 13 語 LE16。**op8 chunk6 応答が本体**で、
#   op6 応答の同じ窓はモード依存の残骸なので当てにしない。
#   w05/w07 はトランザクションパリティ(arm・取得完走・op6 読出で反転)。
#   w11 はセッション内で動きうる(LA 関連候補)。静的語の差分で電源サイクルを検出する。
FOOTER_OFFSET = 38            # 応答内の語列開始バイト
FOOTER_WORDS = 13
INFO_LEN = 10                 # op2 が書く製品文字列は byte1..10
FOOTER_PARITY_IDX = (5, 7)    # w05/w07(A⇔B 対。和 32972 一定)
FOOTER_FP_IDX = (0, 1, 2, 3, 4, 6, 8, 9, 10, 12)   # 電源サイクル指紋に使う語
#   (w05/w07=パリティ、w11=セッション内可変のため除外)

# ── LA(ロジックアナライザ。。2026-10-12 実機確定)─────────────────
#   取得列: op3(0) → op16(arm) → op6 ポーリング(IN byte0: 0=待ち/1=発火)
#            → op8 chunk 0..6。**発火するまで op8 は前回捕捉の凍結バッファ**
#            なので、発火前に読んだデータを信じてはいけない。
#   注意: 信号が動いている最中に op16 で arm するとデバイスがハングする。
#         arm は必ず入力静的のうちに行う。
LA_CHUNKS = 7                 # op8 chunk 0..6
LA_BUFFER_BYTES = LA_CHUNKS * REPORT_LEN          # 448 B(chunk6 末尾 26 B はステータスフッター)
LA_VALID_BYTES = BUFFER_BYTES                     # 422 B = 有効データ(スコープの実バッファ長と同じ)
LA_SAMPLES = LA_VALID_BYTES * 2                   # 1 バイト = ニブル 2 サンプル = 844
LA_RAW_SAMPLES = LA_BUFFER_BYTES * 2              # フッター込み 896(復号には使わない)
#   fs は op16 byte3 の下位 2 ビットで選ぶ(2026-10-17 実測):
#     byte3=3/2/1 → 979.4 k / 490.9 k / 200.0 kHz 固定(byte4..7 は無関係)
#     byte3=0     → byte4..7(スコープと同じ符号化の除数 D)で fs = 6e6/(D+29)。D は実効 ≒26.2 で張り付き
#                   (上限 ≒108.7 kHz)、下限は D=128·65280 で ≒0.72 Sa/s。D≥33 で式と ±0.15%
#   記録は発火点から 844 サンプル。op6 byte0=1 は「窓を記録し終えた」時点で立つ(Auto でも 844/fs 秒後)。
LA_RATE_FIXED = {3: 979400.0, 2: 490900.0, 1: 200000.0}
LA_RATE_HZ = LA_RATE_FIXED[3]                     # 既定(byte3=3)
LA_CLOCK_HZ = 6.0e6           # byte3=0 のときの fs = LA_CLOCK_HZ/(D+29)(スコープ基準クロック 3 MHz の 2 倍)
LA_D_MIN = LA_CLOCK_HZ / 108700.0 - 29            # ≒26.2(実測の張り付き。29 は RATE_OFFSET)
#   ★未発火のエッジ arm は残り続け、op5(スコープ)に戻った後にその ch へエッジが来ると
#   ファームが暴走して USB から外れる(-75 babble → disconnect。2026-10-17 に 3/3 再現)。
#   Auto 再 arm・ゲート閉(byte3=0x04)では解けず、マスク 0 のエッジ arm(byte8=1, byte9=0)で
#   上書きすると解ける(5/5)。la_abort と arm() はこれを使う。
LA_CLEAR_EDGE = 0x01
LA_EDGE = {"falling": 0x00, "rising": 0x01, "auto": 0x02}   # op16 byte8
# ch → op16 byte9 マスク。ch1=0x80..ch4=0x10 = (ニブル bit)<<4
LA_CH_MASK = {1: 0x80, 2: 0x40, 3: 0x20, 4: 0x10}
LA_B47_DEFAULT = 0x0001BEFF   # op16 byte4..7(LE ff be 01 00)。byte3=0 のときだけ効く除数(既定値)
                              # 0..400000 掃引で窓長・遷移統計とも無影響

# ── サンプルレートの統一式─────────────────────
CLOCK_HZ = 3.0e6            # デバイスの基準クロック
RATE_OFFSET = 29            # fs = CLOCK/(D+29)。byte4=255 側なら X=256−byte5 で 285−byte5
RATE_MAX = 33130.0          # 実測の張り付き上限 SPS/ch
D_MIN = 61.5                # 実効下限除数 = 3e6/RATE_MAX − 29
X_MIN = 1                   # byte4=0 は不正値(波形崩壊)なので byte4>=1 → X<=65280
X_MAX = 65280
BYTE7_VERIFIED = 7          # byte7 は 0..7 を実機検証済み。8..15 は下位 3 ビットへ折り返る
BYTE7_MASK = 0x07           # 実機が実際に使うビット

# ── op5 ペイロードの既定値────────────────
#   byte0=op(5) は送信時に付与するので、ここは byte1..byte19 の 19 バイト。
DEFAULT_BYTES = {
    1: 0x05,        # CH1 入力タップ(5=÷8、6=×1)
    2: 0x08,        # CH2 入力タップ(8=÷8、9=×1)
    3: 0xAA,        # 取得イネーブル。**bit7=0 で取得失敗、bits1:0 が 01/10 以外で波形が崩れる**
    4: 0xFF,        # 時間基軸 上位 ┐ byte4·256+byte5 を 16 ビット補数で読んだ
    5: 0x00,        # 時間基軸 下位 ┘ X が除数の主控え。ここは X=256
    6: 0x00,        # bit0=1 で除数 X/2(fs 約 2 倍、byte7 を無視)。bit1 以降は無効果
    7: 0x00,        # 2^byte7 だけ追加分周。**下位 3 ビットのみ有効**
    8: 0x01,        # CH1 ゲイン段 0..5
    9: 0x01,        # CH2 ゲイン段 0..5
    10: 0x80,       # CH1 Position
    11: 0x80,       # CH2 Position
    12: 0x00,       # ハードトリガ 0=フリーラン / 1=トリガ待ち(未成立なら op6 が 0 のまま、op8 は凍結)
    13: 0x00,       # トリガ極性 0=立下り / 1=立上り(byte12=1 のとき)
    14: 0x00,
    15: 0x4D,       # トリガレベル(トリガ経路の生バイト値と比較)。EXT では 1.5/5×256 ≒ 0x4D
    16: 0x00,       # ET モードフラグ
    17: 0x0A,       # 2MHz クロック分周比(同上)
    18: 0x05,       # = byte17 / 2
    19: 0x01,       # トリガ経路(下位 2 ビット): 1=CH1 ÷8 / 2=CH1 ×1 / 3=EXT。CH2 ソースは無い
}

# ── レンジ──────
#   (V/div, tap, ゲイン段, ラベル)
RANGES = [
    (2.0, 8, 2, "2 V/div"),
    (1.0, 8, 1, "1 V/div"),
    (0.5, 8, 0, "0.5 V/div"),
    (0.2, 1, 2, "0.2 V/div"),
    (0.1, 1, 1, "0.1 V/div"),
    (0.05, 1, 0, "50 mV/div"),
]

# ── 縦軸: 10 bit ADC の 1 カウントの電圧(入力端子基準、両 ch 共通)──────
#   ÷8 は外部電源 −11.94〜+18.91 V とテスターで当てた値(残差 ±25 mV)。×1 は 1 点からの見積もり
#   (≒2.8 V で 5.04 mV、±2 % 程度)。スコープの V/LSB = VCOUNT[tap] · 2^段。
VCOUNT = {8: 47.9e-3, 1: 5.04e-3}
#   0 V のときの m(op9 の値)。基板ごと・可変抵抗 R12/R13 の位置ごとに違うので calibrate_zero で測り直す。
#   既定値は 2026-10-18 の実機(CH1 ÷8 495.0 / ×1 379.3、CH2 ÷8 496.5 / ×1 388.0)。
M0_DEFAULT = {1: {8: 495.0, 1: 379.3}, 2: {8: 496.5, 1: 388.0}}
#   op9 のセレクタ = スコープの入力タップ番号(byte1/byte2 の値)。応答 = [BE16 m[s1]][BE16 m[s2]]
MON_SEL = {1: {8: 5, 1: 6}, 2: {8: 8, 1: 9}}
MON_MAX = 1023
MON_X1_SAT = 1019             # ×1 の m はこの付近で張り付く(入力 ≒3.1 V 以上)
#   レンジ別の既定 Position 補正。
#   **これを掛けないと段 0 で信号が上レールに張り付いてスパンが壊れて見える**(実測確認)。
POSITION_CORR = {2: 0x00, 1: 0x80, 0: 0xC0}
#   実測の「中間レベルが code 0(中央)へ来る byte10/11」。
#   傾きは段ごとにちょうど −2^(1−段) codes/byte。
#   **段 3 以上は中央が負 byte になり、0x00..0xFF では中央へ到達不能**。
#   POSITION_CORR はこの実測中央より**段によらず約 18.4 byte 小さい**。
POSITION_CENTER = {0: 209.9, 1: 146.3, 2: 19.0}
POSITION_SLOPE = {0: -2.0, 1: -1.0, 2: -0.5, 3: -0.25, 4: -0.125, 5: -0.0625}
#   0 V のコード(サンプル値)= m0 / 2^段 − 127 + POSITION_SLOPE[段]·Position byte。
#   ×1 タップの入力クランプ電圧
X1_CLAMP_V = 3.35
#   op5 byte3 は取得イネーブル。0xAA でないと
#   0x00/0x01/0x55/0x7F → 全サンプル 0xFF 埋め、0xAB/0xFF → 平坦化、0x80 → 2ch 同一データ。
BYTE3_REQUIRED = 0xAA
#   ビット単位まで決着:
#     bit7 = 0 → **取得失敗**。chunk0..5 が 0xFF で埋まり全サンプルが +128
#     bits1:0 = 01 / 10 → **正常**(0xAA もここ)
#     bits1:0 = 00 → 取得は動くが **DC レベルが −73..−127 へ下がり下レールにクリップ**
#     bits1:0 = 11 → **信号が乗らない**(ほぼ DC)
#     bit2 は bits1:0=00 のときだけスパン 54→96。bit3..bit6 は bits1:0 が正しければ無影響
BYTE3_ENABLE_BIT = 0x80         # bit7。0 で取得そのものが壊れる(致命的)
BYTE3_MODE_MASK = 0x03          # bits1:0 = 入力配線のモード(機構は未同定)
BYTE3_MODE_OK = (1, 2)          # 01 / 10 のとき正常な 2ch 取得になる

# ── スコープのハードトリガ(op5)──────
#   byte12 = 1 でトリガ待ち(未成立なら op6 が 0 のまま、op8 は前回の凍結バッファ)、
#   byte13 = 極性、byte15 = レベル(比較器はゲイン段・Position の前を見る。byte15 = m/2 − 128 と比較)、
#   byte19 = 経路(下位 2 ビット)。CH1 の経路は CH1 のタップと揃える。
#   CH2 を源にする設定は byte1/byte19 のどの組み合わせにも無い(比較器は CH1 端子側に固定)。
TRIG_SOURCES = ("ch1", "ext")
TRIG_EDGE = {"falling": 0, "rising": 1}
TRIG_MODES = ("normal", "auto")
TRIG_PATH = {8: 1, 1: 2}        # CH1 tap → byte19(1 = ÷8 経路、2 = ×1 経路)
TRIG_PATH_EXT = 3
#   EXT のしきい値は固定(実測 立上り ≒1.45 V・立下り ≒1.42 V、ヒステリシス ≒0.03 V)。byte15 は 0x20..0xE0 なら無関係だが 0x00/0xFF は不安定
#   なので中間値(1.5/5·256 ≒ 0x4D)に置く。
TRIG_LEVEL_EXT = 0x4D
#   byte15 の許容域。0x00 は T=GND でも成立し、0xFF はほぼ成立しない(実測)。
TRIG_LEVEL_MIN = 0x01
TRIG_LEVEL_MAX = 0xFE
#   Auto の既定待ち = TRIG_AUTO_MIN_S + 384/fs(記録時間 192/fs の 2 倍の余裕)
TRIG_AUTO_MIN_S = 0.1
#   op8 のチャンク間の休み。0 でも全一致・無応答なし(6 チャンク 12 ms)。旧版は 20 ms。
CHUNK_GAP_S = 0.0


class Osc001Error(Exception):
    pass


class TriggerTimeout(Osc001Error):
    """Normal モードでトリガが成立しなかった(op8 は前回の凍結バッファなので読まない)。"""
    pass


class DeviceInfo(object):
    def __init__(self, path, vendor, product, name):
        self.path = path
        self.vendor = vendor
        self.product = product
        self.name = name

    def __repr__(self):
        return "DeviceInfo(%r, %04x, %04x, %r)" % (self.path, self.vendor,
                                                   self.product, self.name)


def enumerate(devclass="/sys/class/hidraw"):
    """04d8:f891 の hidraw デバイスを列挙する。

    注意: 公開 API(`osc001.enumerate()` / `osc001.open()`)に合わせるため
    内蔵 `enumerate` / `open` を遮蔽している。モジュール内でこれらを使う箇所は
    必ず `builtins.` 経由で呼ぶこと。
    """
    out = []
    try:
        names = sorted(os.listdir(devclass))
    except OSError:
        return out
    for n in names:
        path = os.path.join(devclass, n)
        try:
            # builtins.open: 当モジュール末尾で osc001.open が内蔵 open を遮蔽するため明示する
            with builtins.open(os.path.join(path, "device", "uevent")) as f:
                uv = f.read()
        except OSError:
            continue
        m = re.search(r"HID_ID=0*(\w+):0*(\w+):0*(\w+)", uv)
        if not m:
            continue
        vendor = int(m.group(2), 16)
        product = int(m.group(3), 16)
        if (vendor, product) != (VENDOR, PRODUCT):
            continue
        nm = re.search(r"HID_NAME=(.*)", uv)
        out.append(DeviceInfo(os.path.join("/dev", n), vendor, product,
                              nm.group(1).strip() if nm else ""))
    return out


class Trace(object):
    """op8 から得た 2ch 系列(read() なら 192、read_deep() なら 211 サンプル/ch)。"""

    def __init__(self, raw, rate_hz, calib, taps, armed_at, read_at, zero=None):
        self.raw = raw                    # 生値(0..255)。384 B または 422 B
        self.rate_hz = rate_hz            # per-ch サンプルレート
        self._calib = calib               # {ch: V/LSB}
        self._taps = taps                 # {ch: tap}
        self.zero = zero or {"ch1": 0.0, "ch2": 0.0}   # {ch: 0 V のコード}(取得時の zero_code)
        # read_partial() でのみ設定: 書き込み済みとみなせる先頭サンプル数(/ch)。それ以外は全数。
        self.valid = len(raw) // 2
        self.armed_at = armed_at
        self.read_at = read_at
        # read_deep()/capture_deep() でのみ設定される。
        self.footer = None          # chunk6 応答の 13 語(LE16 タプル)
        self.stale = False          # 前回 read_deep と (パリティ語+md5) が一致
        self.power_cycle = False    # 静的語が前回基準と差分=デバイス再電源投入疑い
        self.filled_ff = False      # 全バイト 0xFF=取得失敗(op5 byte3=0x00 等)
        # capture()/capture_deep() でのみ設定される。None = トリガ無し(フリーラン)、
        # True = トリガ成立、False = Auto で未成立のためフリーランで取り直した。
        self.triggered = None

    @property
    def samples(self):
        """インターリーブを解いた 2 系列(偶数番=CH1、奇数番=CH2)。"""
        s = [b - 0x7F for b in self.raw]
        return {"ch1": s[0::2], "ch2": s[1::2]}

    def to_volts(self, ch=1):
        """入力端子の電圧(0 V 基準)。(サンプル − 取得時の zero_code) × V/LSB。"""
        key = "ch1" if ch == 1 else "ch2"
        vlsb, z = self._calib[key], self.zero[key]
        return [(v - z) * vlsb for v in self.samples[key]]

    def rail_hit(self, ch=1):
        """ADC レール(−127 / +128)に到達したサンプルがあれば True。

        到達しているとスパンから V/LSB を逆算できない(Position が合ってないか、
        レンジが信号に対して小さい)。2026-10-11 の実測で、段 0 で Position 補正を
        掛けないとこの状態になった。
        """
        s = self.samples["ch%d" % ch]
        return max(s) >= 128 or min(s) <= -127

    def clipped(self, ch=1):
        """×1 タップでクリップ(入力 3.35V 付近)の可能性を返す。"""
        key = "ch1" if ch == 1 else "ch2"
        if self._taps[key] != 1:
            return False
        s = self.samples[key]
        return (max(s) - min(s)) >= 160          # 165 LSB 張り付き ≈ クリップ


class LATrace(object):
    """LA 捕捉(op8 chunk0..6 = 448 B。先頭 422 B = 844 サンプル点が有効データ)。

    各サンプル点は 4 デジタル ch のビット状態を収めたニブル。
    ch1=bit3 / ch2=bit2 / ch3=bit1 / ch4=bit0。
    バイト内の順序は下位ニブル先
    末尾 26 B はステータスフッター(footer)。サンプル 0 が発火点(プリトリガなし)。
    """

    def __init__(self, raw, armed_at, fired_at, read_at, rate_hz=LA_RATE_HZ):
        self.raw = raw                    # 448 B(chunk0..6 を連結)
        self.armed_at = armed_at
        self.fired_at = fired_at
        self.read_at = read_at
        self.rate_hz = rate_hz

    @property
    def footer(self):
        """末尾 26 B の 13 語(LE16)。raw が短い(フッター無し)ときは None。"""
        if len(self.raw) < LA_BUFFER_BYTES:
            return None
        return Scope.footer_words(b"\0" * FOOTER_OFFSET + self.raw[LA_VALID_BYTES:LA_BUFFER_BYTES])

    @property
    def nibbles(self):
        out = []
        for byte in self.raw[:LA_VALID_BYTES]:
            out.append(byte & 0x0F)
            out.append((byte >> 4) & 0x0F)
        return out

    def bits(self, ch):
        """ch(1..4)の 0/1 系列(844 サンプル)。"""
        if ch not in LA_CH_MASK:
            raise Osc001Error("LA ch は 1..4: %r" % (ch,))
        bit = 4 - ch                      # ch1=bit3..ch4=bit0
        return [(n >> bit) & 1 for n in self.nibbles]

    def transitions(self, ch):
        s = self.bits(ch)
        return sum(1 for i in range(1, len(s)) if s[i] != s[i - 1])


class Scope(object):
    """1 台分のセッション。C API(osc001_handle)と対応させる。"""

    def __init__(self, path=DEFAULT_DEVICE, timeout=2.0):
        self.path = path
        self.timeout = timeout
        self.b = dict(DEFAULT_BYTES)      # op5 に送る 19 バイト(byte1..19)
        self._fd = None
        self._armed_at = None
        self._ready_at = None           # arm 後に op6 の完了を見た時刻
        self.m0 = {1: dict(M0_DEFAULT[1]), 2: dict(M0_DEFAULT[2])}   # 0 V の m(ch → tap → m)
        self.m0_gain = {}               # (ch, tap, 段) → m。calibrate_zero のレンジ別実測(あれば m0 より優先)
        self._trig = None               # set_trigger() の設定。None = op5 の byte12/13/15/19 に触らない
        self._la_frame = None
        self._la_rate = la_rate_settings(LA_RATE_HZ)   # (fs, byte3, byte4..7)
        self._la_armed_at = None
        self._la_fired_at = None
        self._la_edge_pending = False   # 未発火のエッジ arm が残っている
        self._fp_prev = None        # 前回 read_deep の (パリティ語, md5)
        self._fp_base = None        # 静的語の基準(初 read_deep で設定)

    # ── セッション管理 ────────────────────────────────────────────────
    def open(self):
        # 排他(flock で単一セッション強制)と
        # open 失敗時のエラー区別(権限不足 / ノード無し=未接続・ブートローダ・ハング後 / 占有)。
        if self._fd is not None:
            raise Osc001Error("既に open されている(close してから open)")
        try:
            self._fd = os.open(self.path, os.O_RDWR)
        except PermissionError:
            raise Osc001Error("open %s 失敗: 権限不足(udev ルール適用か確認)"
                              % self.path)
        except FileNotFoundError:
            raise Osc001Error("open %s 失敗: ノード無し(04d8:f891 が lsusb にあるか確認。"
                              "ハング後は物理抜き差しが必要)" % self.path)
        except OSError as e:
            raise Osc001Error("open %s 失敗: %s" % (self.path, e))
        try:
            fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(self._fd)
            self._fd = None
            raise Osc001Error("open %s 失敗: 他プロセスが占有中(flock 排他。"
                              "hidraw は多重オープンできるがコマンドが取り違える)" % self.path)
        self._drain(0.2)
        return self

    def close(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
        return False

    # ── 低レベルトランザクション ──────────────────────────────────────
    def _drain(self, timeout=0.3):
        while True:
            r, _, _ = select.select([self._fd], [], [], timeout)
            if not r:
                return
            try:
                os.read(self._fd, REPORT_LEN)
            except OSError:
                return

    def _txn(self, op, args=(), timeout=None):
        if self._fd is None:
            raise Osc001Error("open されていない")
        buf = bytearray(REPORT_LEN)
        buf[0] = op
        for i, a in builtins.enumerate(args):    # 内蔵 enumerate(当モジュールの遮蔽に注意)
            buf[1 + i] = a & 0xFF
        try:
            os.write(self._fd, b"\x00" + bytes(buf))     # レポート ID なし → 0x00
        except OSError as e:
            raise Osc001Error("write(op%d) 失敗: %s" % (op, e))
        r, _, _ = select.select([self._fd], [], [],
                                timeout if timeout is not None else self.timeout)
        if not r:
            return None
        try:
            return os.read(self._fd, REPORT_LEN)
        except OSError as e:
            raise Osc001Error("read(op%d) 失敗: %s" % (op, e))

    # ── 情報 ─────────────────────────────────────────────────────────
    def info(self):
        """op2(GET_INFO 相当)。"DPScope SE" 文字列を含む 64 バイト。"""
        r = self._txn(2)
        if r is None:
            raise Osc001Error("op2 無応答")
        txt = bytes(r[1:1 + INFO_LEN]).split(b"\x00")[0]   # byte11 以降は前の応答の残骸
        txt = "".join(chr(c) if 32 <= c < 127 else "" for c in txt)
        return {"path": self.path, "info": txt, "raw": bytes(r)}

    # ── 設定(op5 は arm 兼用なので、設定変更は arm 前に終えること)───────
    def set_raw(self, index, value):
        """op5 の byte1..19 を直接指定する脱出口。"""
        if not 1 <= index <= 19:
            raise ValueError("index は 1..19")
        self.b[index] = value & 0xFF

    def set_range(self, ch, v_per_div, tap=None, position="auto"):
        """レンジを V/div で指定。

        position="auto"(既定)だと、レンジ別の既定補正(0x00/0x80/0xc0)を
        Position にも反映する。**段 0 ではこれを掛けないと信号が上レールに張り付き、
        スパンが本来の 1/6 に見える**(2026-10-11 実測。`python/demo_ranges.py`)。
        position=数値 を渡すとその byte をそのまま使う。None は Position を触らない。

        tap を省略するとラベルが一意に決まる(2.0/1.0/0.5 → ÷8、0.2/0.1/0.05 → ×1)。
        tap を明示するとラベルの段だけを使い、タップを独立に選べる
        (バイト上は byte1/2 と byte8/9 は別フィールド)。
        """
        cand = [r for r in RANGES if abs(r[0] - float(v_per_div)) < 1e-9]
        if not cand:
            raise ValueError("range_v %r は 6 ラベル(%s)のいずれか"
                             % (v_per_div, ", ".join(str(r[0]) for r in RANGES)))
        _, tap_auto, gain, label = cand[0]
        t = tap_auto if tap is None else tap
        if t not in (1, 8):
            raise ValueError("tap は 1(×1)か 8(÷8)")
        self.set_gain(ch, gain, tap=t, position=position)
        return label

    def set_gain(self, ch, gain, tap=None, position="auto"):
        """ゲイン段 byte8/9 を直接指定(0..5、各段ちょうど 1/2)。

        6 ラベルは段 0..2 だけを使うが、デバイスには段 3..5 もある
        (×1 で 40.7/80.5/165 mV/LSB)。段 3..5 での Position 補正値は
        未確認なので 0x00 を使う(要実測)。
        **段 3..5 は Position をいくら振っても中央(code 0)へ到達できない**(
        中央へ来る byte が段 3 で −236、段 4 で −748、段 5 で −1768)。`position="center"` は
        段 0..2 でだけ意味を持つ。
        """
        gain = int(gain)
        if not 0 <= gain <= 5:
            raise ValueError("gain は 0..5")
        self.b[8 if ch == 1 else 9] = gain
        if tap is not None:
            TAPBYTE = {1: {1: 6, 8: 5}, 2: {1: 9, 8: 8}}      # ch → (tap → byte 値)
            self.b[1 if ch == 1 else 2] = TAPBYTE[ch][tap]
        if position == "auto":
            self.set_position(ch, POSITION_CORR.get(gain, 0x00))
        elif position == "center":
            c = self.center_position(gain)
            if c is None:
                raise Osc001Error("段 %d は Position で中央(code 0)へ到達不能"
                                  % gain)
            self.set_position(ch, c)
        elif position is not None:
            self.set_position(ch, position)
        return self.v_per_lsb(ch)

    def center_position(self, gain):
        """中間レベルを code 0(中央)へ置く byte 値。段 3..5 は到達不能なので None。"""
        c = POSITION_CENTER.get(int(gain))
        return None if c is None else int(round(c)) & 0xFF

    def zero_m0(self, ch, tap=None, gain=None):
        """0 V のときの m(10 bit 値)。レンジ別の実測(calibrate_zero)があればそれ、無ければ m0。"""
        tap = self._tap(ch) if tap is None else tap
        g = self.b[8 if ch == 1 else 9] if gain is None else gain
        return self.m0_gain.get((ch, tap, g), self.m0[ch][tap])

    def set_zero_m0(self, ch, tap, m0):
        """0 V の m を外から与える(保存しておいた calibrate_zero の結果など)。レンジ別の値は消す。"""
        if tap not in (1, 8):
            raise ValueError("tap は 1(×1)か 8(÷8)")
        self.m0[ch][tap] = float(m0)
        for k in [k for k in self.m0_gain if k[0] == ch and k[1] == tap]:
            del self.m0_gain[k]

    def zero_code(self, ch=1):
        """現在の tap × 段 × Position で 0 V が来るコード(サンプル値、実数)。

        = m0 / 2^段 − 127 + POSITION_SLOPE[段]·Position。段 3..5 は式からの外挿(未実測)。
        """
        g = self.b[8 if ch == 1 else 9]
        return (self.zero_m0(ch) / float(1 << g) - 127.0 +
                POSITION_SLOPE[g] * self.b[10 if ch == 1 else 11])

    def volts_to_code(self, ch, volts):
        """入力端子の電圧 → コード(サンプル値、実数)。生バイトはこれ + 0x7F。"""
        return self.zero_code(ch) + float(volts) / self.v_per_lsb(ch)

    def set_position(self, ch, byte_value):
        """Position を byte 直指定(0x00..0xFF、既定 0x80)。

        set_range(position="auto")はレンジ別の加算補正(0x00/0x80/0xc0)を掛けるが、
        ここでは掛けない。中央点は実測で ×1 タップ 0x62〜0x64、÷8 段 2 で 0x0B 等。
        """
        self.b[10 if ch == 1 else 11] = int(byte_value) & 0xFF

    def x_value(self):
        """byte4/byte5 を 16 ビット 2 の補数と見たときの大きさ X。"""
        return 0x10000 - (self.b[4] * 256 + self.b[5])

    def divisor(self):
        """fs の除数 D(設定そのまま)。byte6 bit0 が立っていれば X/2(byte7 無視)、
        そうでなければ 2^(byte7 & 7)·X。byte7 は下位 3 ビットだけ効く。"""
        x = self.x_value()
        return x / 2.0 if (self.b[6] & 1) else (1 << (self.b[7] & BYTE7_MASK)) * x

    def effective_divisor(self):
        """fs 式に実際に代入される除数。低 D 側は D_MIN で頭打ち。"""
        return max(self.divisor(), D_MIN)

    def sample_rate(self):
        """現在の byte4..byte7 からの per-ch サンプルレート(Hz、上限でクランプ)。"""
        return min(CLOCK_HZ / (self.effective_divisor() + RATE_OFFSET), RATE_MAX)

    def rate_settings(self, hz):
        """per-ch Hz 指定 → 最も近い (fs, byte4, byte5, byte6, byte7)。

        除数を逆算する: D = 3e6/hz − 29。同じ D を
          byte6=0/byte7=n → X = D/2^n、byte6=1 → X = 2D
        で実現できる。X は整数なので、丸め誤差が最小の組を選ぶ(同率なら X が大きい方、
        つまり相対目目の細かい組を選ぶ)。。
        hz が上限を超える場合は **D を D_MIN に肩代わり**させてから byte を選ぶ
        (実機は D を下げても張付きで上がらない)。
        """
        hz = float(hz)
        target = max(CLOCK_HZ / hz - RATE_OFFSET, D_MIN)
        best = None
        for b6 in (0, 1):
            for b7 in range(0, BYTE7_VERIFIED + 1):
                shift = 0.5 if b6 else float(1 << b7)
                x = max(X_MIN, min(X_MAX, int(round(target / shift))))
                for xc in (x - 1, x, x + 1):
                    if not X_MIN <= xc <= X_MAX:
                        continue
                    fs = min(CLOCK_HZ / (max(shift * xc, D_MIN) + RATE_OFFSET),
                             RATE_MAX)
                    v = (0x10000 - xc) & 0xFFFF
                    key = (abs(fs - hz), -xc)
                    if best is None or key < best[0]:
                        best = (key, (fs, v >> 8, v & 0xFF, b6, b7))
        return best[1]

    def set_sample_rate(self, hz):
        """per-ch Hz 指定で byte4..byte7 を設定し、実効値(Hz)を返す。"""
        fs, b4, b5, b6, b7 = self.rate_settings(hz)
        self.b[4], self.b[5], self.b[6], self.b[7] = b4, b5, b6, b7
        return fs

    def v_per_lsb(self, ch=1):
        """現在の tap×段での入力端子での V/LSB = VCOUNT[tap] · 2^段(両 ch 共通)。"""
        return VCOUNT[self._tap(ch)] * (1 << self.b[8 if ch == 1 else 9])

    def _tap(self, ch):
        """byte1/byte2 から tap(1=×1、8=÷8)を引く。使用不可の値は例外。"""
        tapbyte = self.b[1 if ch == 1 else 2]
        if tapbyte in (6, 9):
            return 1
        if tapbyte in (5, 8):
            return 8
        raise Osc001Error("byte%d=%d は使用不可のタップ値"
                          % (1 if ch == 1 else 2, tapbyte))

    def byte3_status(self):
        """op5 byte3 のビットから得る取得状態。

        "disabled"    = bit7=0。取得が失敗し chunk0..5 が 0xFF 埋め(致命的)
        "level-shift" = bit7=1, bits1:0=00。DC レベルが −73..−127 へ下がり下レールにクリップ
        "no-signal"   = bit7=1, bits1:0=11。ほぼ DC で信号が乗らない
        "ok"          = bit7=1, bits1:0=01 / 10(既定 0xAA もここ)
        """
        if not (self.b[3] & BYTE3_ENABLE_BIT):
            return "disabled"
        mode = self.b[3] & BYTE3_MODE_MASK
        if mode == 0:
            return "level-shift"
        if mode == 3:
            return "no-signal"
        return "ok"

    def warnings(self):
        """設定状態に対する警告文(空なら問題なし)。"""
        w = []
        for ch in (1, 2):
            tapbyte = self.b[1 if ch == 1 else 2]
            if tapbyte not in (5, 6, 8, 9):
                w.append("byte%d=%d: 使用不可のタップ値"
                         % (1 if ch == 1 else 2, tapbyte))
                continue
            tap = 1 if tapbyte in (6, 9) else 8
            span = 256 * self.v_per_lsb(ch)
            if tap == 1 and span > X1_CLAMP_V * 1.5:
                w.append("CH%d: ×1 タップの全スパン %.2gV は入力 %.2gV でクランプ"
                         % (ch, span, X1_CLAMP_V))
        st3 = self.byte3_status()
        if st3 == "disabled":
            w.append("byte3=%02x: bit7=0 で**取得が失敗**し chunk0..5 が 0xFF 埋め"
                     "(全サンプル +128)。致命的" % self.b[3])
        elif st3 == "level-shift":
            w.append("byte3=%02x: bits1:0=00 で DC レベルが -73..-127 へ下がり下レールに"
                     "クリップする" % self.b[3])
        elif st3 == "no-signal":
            w.append("byte3=%02x: bits1:0=11 で信号が乗らない(ほぼ DC)。"
                     % self.b[3])
        elif self.b[3] != BYTE3_REQUIRED:
            w.append("byte3=%02x: 正常に動くが既定の %02x と異なる。bit3/bit5 の意味は"
                     "未解明" % (self.b[3], BYTE3_REQUIRED))
        for ch in (1, 2):
            g = self.b[8 if ch == 1 else 9]
            if g not in POSITION_CENTER:
                w.append("CH%d: ゲイン段 %d は Position をいくら振っても中央(code 0)に届かない"
                         "" % (ch, g))
        if self.b[4] == 0:
            w.append("byte4=0: 不正値(波形崩壊)。X=%d は範囲外"
                     % self.x_value())
        if self.b[7] > BYTE7_VERIFIED:
            w.append("byte7=%d: 実機は下位 3 ビットだけ使い %d と同一の動作になる"
                     "" % (self.b[7], self.b[7] & BYTE7_MASK))
        if self.divisor() < D_MIN:
            w.append("fs は実測上限 ≒%.0f SPS/ch で頭打ち(除数 D=%.1f < 実効下限 %.1f。"
                     ")" % (RATE_MAX, self.divisor(), D_MIN))
        if self._trig is not None:
            if self._trig["source"] == "ext":
                w.append("トリガ EXT: T 入力は浮かせると電圧が定まらず誤トリガし得る(T を信号か GND に繋ぐ)")
            else:
                try:
                    self._trigger_bytes()
                except (ValueError, Osc001Error) as e:
                    w.append("トリガ CH1: %s" % e)
        return w

    # ── トリガ ───────────────────────────────────────────────────────
    def set_trigger(self, source=None, edge="rising", level_v=0.0, level_raw=None,
                    mode="normal", auto_timeout=None):
        """ハードトリガを設定する。source=None でフリーラン(byte12=0)。

        source: "ch1" か "ext"(T 入力)。CH2 のハードトリガは無い(呼び出し側でソフト処理する)。
        edge:   "rising" / "falling"(EXT でも有効)。
        level_v: CH1 のレベル(入力端子の電圧)。比較器はゲイン段・Position の前(op9 の m と
                 同じ所)を見るので、byte15 = m/2 − 128(m = m0 + level_v / VCOUNT)。
                 段・Position に依らず、タップと m0 だけで決まる。arm のたびに換算する。
        level_raw: byte15 を直接指定(level_v より優先)。EXT では無視して TRIG_LEVEL_EXT を使う。
        mode:   "normal" = 成立まで待ち、capture の timeout で TriggerTimeout。
                "auto"   = auto_timeout(既定 TRIG_AUTO_MIN_S + 384/fs 秒)待って未成立なら
                           フリーランで取り直す(Trace.triggered = False)。
        設定はこの場で検証する(レベルが比較器の範囲外なら ValueError)。
        """
        if source is None:
            self._trig = None
            self.b[12] = 0
            return
        if source not in TRIG_SOURCES:
            raise ValueError("source は None / %s(CH2 のハードトリガは無い)"
                             % " / ".join(TRIG_SOURCES))
        if edge not in TRIG_EDGE:
            raise ValueError("edge は rising / falling")
        if mode not in TRIG_MODES:
            raise ValueError("mode は normal / auto")
        if auto_timeout is not None and not auto_timeout > 0:
            raise ValueError("auto_timeout は正の秒数")
        if level_raw is not None and not TRIG_LEVEL_MIN <= int(level_raw) <= TRIG_LEVEL_MAX:
            raise ValueError("level_raw は 0x%02x..0x%02x" % (TRIG_LEVEL_MIN, TRIG_LEVEL_MAX))
        cfg = {"source": source, "edge": edge, "level_v": float(level_v),
               "level_raw": None if level_raw is None else int(level_raw),
               "mode": mode, "auto_timeout": auto_timeout}
        keep = (self._trig, self.b[12], self.b[13], self.b[15], self.b[19])
        self._trig = cfg
        try:
            self._apply_trigger()
        except Exception:
            self._trig, self.b[12], self.b[13], self.b[15], self.b[19] = keep
            raise

    def trigger(self):
        """現在のトリガ設定(dict の写し)。フリーランなら None。"""
        return None if self._trig is None else dict(self._trig)

    def trigger_level_raw(self):
        """現在の設定で byte15 に入る値。フリーランなら None。"""
        if self._trig is None:
            return None
        return self._trigger_bytes()[2]

    def _trigger_bytes(self):
        """_trig → (byte12, byte13, byte15, byte19)。レベルが比較器の範囲外なら ValueError。"""
        t = self._trig
        b13 = TRIG_EDGE[t["edge"]]
        if t["source"] == "ext":
            return 1, b13, TRIG_LEVEL_EXT, TRIG_PATH_EXT
        b19 = TRIG_PATH[self._tap(1)]
        if t["level_raw"] is not None:
            return 1, b13, t["level_raw"], b19
        tap = self._tap(1)
        raw = int(round((self.m0[1][tap] + t["level_v"] / VCOUNT[tap]) / 2.0)) - 128
        if not TRIG_LEVEL_MIN <= raw <= TRIG_LEVEL_MAX:
            raise ValueError("トリガレベル %.3g V は比較器の範囲外(生バイト %d)"
                             % (t["level_v"], raw))
        return 1, b13, raw, b19

    def _apply_trigger(self):
        if self._trig is not None:
            self.b[12], self.b[13], self.b[15], self.b[19] = self._trigger_bytes()

    def _auto_timeout(self):
        t = self._trig["auto_timeout"]
        return t if t is not None else TRIG_AUTO_MIN_S + SAMPLES_PER_CH * 2 / self.sample_rate()

    # ── 取得 ─────────────────────────────────────────────────────────
    def arm(self):
        """op5(SET_REG)= 設定適用兼 arm。この時点でデバイスが取得を始める。

        set_trigger() の設定があれば、現在のタップで byte12/13/15/19 を作り直す。
        未発火の LA エッジ arm が残っていれば、先に解除する(LA_CLEAR_EDGE の注記)。
        """
        self._apply_trigger()
        return self._send_arm()

    def _send_arm(self):
        if self._la_edge_pending:
            self._la_clear()
        args = [self.b[i] for i in range(1, 20)]
        r = self._txn(5, args)
        if r is None:
            raise Osc001Error("op5 無応答")
        self._armed_at = time.time()
        self._ready_at = None
        self._la_armed_at = None          # 以後 op6 はスコープ側の状態を返す
        return r

    def _acquire(self, timeout):
        """arm → 待ち。トリガ設定に従って成否を返す(None / True / False、Trace.triggered と同じ)。

        Normal で未成立なら TriggerTimeout(op8 は凍結バッファなので読まない)。
        未成立の arm の直後に op5 を送り直すのは安全(約 600 回で無応答・切断なし)。
        """
        self.arm()
        if self._trig is None:
            self.wait_ready(timeout)
            return None
        if self._trig["mode"] == "normal":
            if self.wait_ready(timeout):
                return True
            raise TriggerTimeout("トリガ未成立(%.3g s)" % timeout)
        if self.wait_ready(self._auto_timeout()):
            return True
        self.b[12] = 0                    # 次の arm() で _apply_trigger が 1 に戻す
        self._send_arm()
        self.wait_ready(timeout)
        return False

    def wait_ready(self, timeout=5.0):
        """op6(GET_STATUS 相当)を 5ms 間隔で読み、byte0>0 を待つ。

        記録完了(arm から 192/fs 秒、トリガ時は成立後)で立つ。見たら read() は待たずに読む。
        見つからなくても True/False で返す。間隔ゼロの過剰ポーリングはファームを停止させた実績がある。
        """
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = self._txn(6, (), timeout=0.3)
            if r is not None and r[0] > 0:
                self._ready_at = time.time()
                return True
            time.sleep(0.005)
        return False

    def _read_chunks(self, n):
        raw = bytearray()
        resp = None
        for c in range(n):
            resp = self._txn(8, (c,))
            if resp is None:
                raise Osc001Error("op8 chunk%d 無応答" % c)
            if n == CHUNKS_DEEP and resp[0] == OP_ECHO:
                raise Osc001Error("op8 chunk%d の byte0=0x08(索引が無効)" % c)
            raw.extend(resp)
            if CHUNK_GAP_S:
                time.sleep(CHUNK_GAP_S)
        return raw, resp

    def _trace(self, raw, fs):
        calib, taps, zero = {}, {}, {}
        for ch in (1, 2):
            key = "ch%d" % ch
            calib[key] = self.v_per_lsb(ch)
            try:
                taps[key], zero[key] = self._tap(ch), self.zero_code(ch)
            except Osc001Error:               # 使用不可のタップ値(試験用の set_raw)
                taps[key], zero[key] = None, 0.0
        return Trace(bytes(raw), fs, calib, taps, self._armed_at, time.time(), zero)

    def read(self, wait_extra=0.02):
        """op8 chunk0..5 を読む。op6 の完了を見ていなければ、arm から 192/fs 秒まで待つ。"""
        fs = self.sample_rate()
        need = SAMPLES_PER_CH / fs
        if self._armed_at is not None and self._ready_at is None:
            elapsed = time.time() - self._armed_at
            if elapsed < need:
                time.sleep(need - elapsed + wait_extra)
        raw, _ = self._read_chunks(CHUNKS)
        return self._trace(raw, fs)

    def read_partial(self):
        """取得の途中で op8 を読む(遅い時間軸の逐次表示用。フリーランのときだけ)。

        バッファは時系列順に書かれていく。Trace.valid に、arm からの経過時間で
        書き込み済みとみなせる先頭サンプル数(/ch、1 サンプルの余裕を引く)を入れる。
        valid 以降は前回の取得の残り。ハードトリガ待ちの途中は op8 が凍結なので扱わない。
        """
        if self._armed_at is None:
            raise Osc001Error("arm していない")
        if self.b[12]:
            raise Osc001Error("ハードトリガ待ちの途中読みは扱わない(op8 は成立まで凍結)")
        fs = self.sample_rate()
        t0 = time.time()
        raw, _ = self._read_chunks(CHUNKS)
        t = self._trace(raw, fs)
        t.valid = max(0, min(SAMPLES_PER_CH, int((t0 - self._armed_at) * fs) - 1))
        return t

    def capture(self, timeout=5.0):
        """arm → 待ち → read のまとまった操作。トリガ設定があれば Trace.triggered に成否が入る。"""
        trig = self._acquire(timeout)
        t = self.read()
        t.triggered = trig
        return t

    @staticmethod
    def footer_words(resp):
        """応答 [38..63] を 13 語 LE16 として返す。

        読むなら op8 chunk6 応答から。op6 応答の同窓はモード依存の残骸。
        """
        return struct.unpack("<%dH" % FOOTER_WORDS,
                             bytes(resp[FOOTER_OFFSET:
                                        FOOTER_OFFSET + 2 * FOOTER_WORDS]))

    def _footer_check(self, footer, raw):
        """read_deep 用の整合性判定。Trace の stale/power_cycle/filled_ff を返す。

        stale: (w05/w07 パリティ語 + raw md5) が前回 read_deep と一致。
          **ヒューリスティック**。w05/w07 は HID トランザクション数の
          パリティで、あらゆるコマンド(op2/op3/op5/op6/op8/op9/op16)が
          1 回反転させ、取得完走・タイマは反転しない。
          静的入力では新取得でも指紋が一致して誤検出し得る(out_footer_api)。
          データ信頼性の主チェックは filled_ff と Trace.rail_hit()/md5。
        power_cycle: 静的語(FOOTER_FP_IDX)が前回基準と差分。検出時は基準を更新
          するので同一サイクルで一度だけ True。絶対値でなく差分判定(機体差対策)。
        """
        fp = (tuple(footer[i] for i in FOOTER_PARITY_IDX),
              hashlib.md5(bytes(raw)).hexdigest())
        stale = (fp == self._fp_prev)
        self._fp_prev = fp
        static = tuple(footer[i] for i in FOOTER_FP_IDX)
        if self._fp_base is None:
            power_cycle = False
        else:
            power_cycle = (static != self._fp_base)
        self._fp_base = static
        return stale, power_cycle, all(x == 0xFF for x in bytes(raw))

    def read_deep(self, wait_extra=0.02):
        """chunk0..6 を読み、実バッファ 422 B(211 サンプル/ch)を得る。

        read() の 192 サンプル/ch に対し、後方に 19 サンプル/ch 余っている。
        索引 7 以上は無効なので read_deep は 0..6 しか読まない。
        応答 byte0 が 0x08(op エコー)なら索引が無効な応答なのでエラーにする。
        """
        fs = self.sample_rate()
        need = BUFFER_BYTES / fs               # 後方 19 サンプル/ch の書き込み時刻は未確認なので旧来の待ち
        if self._armed_at is not None:
            elapsed = time.time() - self._armed_at
            if elapsed < need:
                time.sleep(need - elapsed + wait_extra)
        raw, last = self._read_chunks(CHUNKS_DEEP)
        t = self._trace(raw[:BUFFER_BYTES], fs)
        t.footer = self.footer_words(last)     # chunk6 がフッター本体
        t.stale, t.power_cycle, t.filled_ff = self._footer_check(t.footer, t.raw)
        return t

    def capture_deep(self, timeout=5.0):
        """arm → 待ち → read_deep のまとまった操作(211 サンプル/ch)。"""
        trig = self._acquire(timeout)
        t = self.read_deep()
        t.triggered = trig
        return t

    # ── op9 モニタ(10 bit 値の直接読み)────────────────────
    def monitor_raw(self, sel1, sel2):
        """op9(sel1, sel2) → (m[sel1], m[sel2])。セレクタ 5/6/8/9 = CH1 ÷8 / CH1 ×1 / CH2 ÷8 / CH2 ×1。

        ゲイン段・Position に依らない。1 回約 1 ms。スコープ取得の最中に読んでも取得を乱さない。
        """
        r = self._txn(9, (sel1 & 0xFF, sel2 & 0xFF, 0xAA))
        if r is None:
            raise Osc001Error("op9 無応答")
        return (r[0] << 8 | r[1]), (r[2] << 8 | r[3])

    def monitor(self, tap=8, n=1):
        """CH1・CH2 の入力電圧を op9 で 1 トランザクションずつ n 回読み、平均を返す。

        tap=8: ±24 V を 47.9 mV 刻み。tap=1: −1.9〜+3.1 V を ≒5 mV 刻み(上限で張り付く)。
        返り値 {"ch1": V, "ch2": V, "m": (m1, m2), "sat": (bool, bool)}。
        """
        if tap not in (1, 8):
            raise ValueError("tap は 1(×1)か 8(÷8)")
        a = b = 0.0
        sat = [False, False]
        for _ in range(max(1, int(n))):
            m1, m2 = self.monitor_raw(MON_SEL[1][tap], MON_SEL[2][tap])
            a += m1
            b += m2
            for k, m in ((0, m1), (1, m2)):
                if m <= 0 or m >= (MON_X1_SAT if tap == 1 else MON_MAX):
                    sat[k] = True
        n = max(1, int(n))
        a, b = a / n, b / n
        return {"ch1": (a - self.m0[1][tap]) * VCOUNT[tap], "ch2": (b - self.m0[2][tap]) * VCOUNT[tap],
                "m": (a, b), "sat": tuple(sat)}

    def calibrate_zero(self, channels=(1, 2), duration=1.0, ranges=True):
        """0 V 合わせ。**入力を GND につないだ状態で**呼ぶ。

        op9 を duration 秒平均して ÷8・×1 の m0 を測る。ranges=True なら 6 レンジでスコープも取得し、
        レンジごとの 0 V(m0_gain)を実測する(スコープと op9 の間の ±2.5 コードの差も吸収する)。
        設定(op5 の 19 バイト・トリガ)は元に戻す。返り値 {"m0": ..., "m0_gain": ...}。
        """
        out = {"m0": {}, "m0_gain": {}}
        for tap in (8, 1):
            acc, k = [0.0, 0.0], 0
            t0 = time.time()
            while time.time() - t0 < duration or k == 0:
                m1, m2 = self.monitor_raw(MON_SEL[1][tap], MON_SEL[2][tap])
                acc[0] += m1
                acc[1] += m2
                k += 1
            for ch in channels:
                self.set_zero_m0(ch, tap, acc[ch - 1] / k)
                out["m0"][(ch, tap)] = self.m0[ch][tap]
        if ranges:
            keep_b, keep_trig = dict(self.b), self._trig
            self._trig = None
            self.b[12] = 0
            try:
                for _vd, tap, g, _label in RANGES:
                    for ch in channels:
                        self.set_gain(ch, g, tap=tap, position="auto")
                    t = self.capture()
                    for ch in channels:
                        s = t.samples["ch%d" % ch]
                        if max(s) >= 128 or min(s) <= -127:
                            continue                  # レールに当たった(0 V が窓の外)
                        mean = sum(s) / float(len(s))
                        pos = self.b[10 if ch == 1 else 11]
                        m = (mean + 127.0 - POSITION_SLOPE[g] * pos) * (1 << g)
                        self.m0_gain[(ch, tap, g)] = m
                        out["m0_gain"][(ch, tap, g)] = m
            finally:
                self.b, self._trig = keep_b, keep_trig
        return out

    # ── LA────────────────────────────────────────────────────
    def la_set_rate(self, hz):
        """LA の fs を選ぶ(次の la_configure から有効)。実効 fs を返す。la_rate_settings 参照。"""
        self._la_rate = la_rate_settings(hz)
        return self._la_rate[0]

    def la_rate(self):
        return self._la_rate[0]

    def la_configure(self, channels=(2,), edge="falling"):
        """op16 トリガフレーム(byte8=極性、byte9=チャネルマスクの OR)を構築。

        edge: falling/rising/auto。auto は arm 直後に即発火する(channels は空にすること)。
        ★auto かつ channels 非空はハング領域なので拒否する(C 版と同じガード)。
        未接続の入力をマスクに含めると隣接 ch のクロストークで誤発火しうる。
        """
        if edge not in LA_EDGE:
            raise Osc001Error("edge は %s のいずれか: %r"
                              % (sorted(LA_EDGE), edge))
        mask = 0
        for ch in channels:
            if ch not in LA_CH_MASK:
                raise Osc001Error("LA ch は 1..4: %r" % (ch,))
            mask |= LA_CH_MASK[ch]
        if edge == "auto" and mask:
            raise Osc001Error("edge=auto かつ channels 指定はハング領域"
                              "(op16 byte8=2 かつ byte9≠0)。復旧は物理抜き差しのみ")
        # args は wire byte1.. に載る(op エコーが byte0)。よって
        # args[2]=byte3=0x03、args[3..6]=byte4..7、args[7]=byte8、args[8]=byte9。
        _, b3, b47 = self._la_rate
        self._la_frame = ((0, 0, b3) + b47 + (LA_EDGE[edge], mask))
        return self._la_frame

    def la_arm(self):
        """op3(0) → op16 で arm。"""
        if self._la_frame is None:
            self.la_configure()
        r = self._txn(3, (0,))
        if r is None:
            raise Osc001Error("op3(LA 開始)無応答")
        r = self._txn(16, self._la_frame)
        if r is None:
            raise Osc001Error("op16(arm)無応答")
        self._la_armed_at = time.time()
        self._la_fired_at = None
        self._la_edge_pending = self._la_frame[7] != LA_EDGE["auto"]
        return r

    def la_wait(self, timeout=5.0):
        """op6 IN byte0==1(発火して窓を記録し終えた)を待つ。True / timeout で False(arm は残る)。

        timeout は発火待ちに加えて窓の時間 844/fs を見込むこと(低速 fs では秒単位)。

        la_arm 前・la_abort/arm(op5)後は拒否する。op5 の後の op6 byte0 は
        スコープ側の状態で、LA の発火と区別できないため。
        """
        if self._la_armed_at is None:
            raise Osc001Error("la_arm の前(または abort/op5 の後)は LA の発火を判定できない")
        t0 = time.time()
        while time.time() - t0 < timeout:
            r = self._txn(6, (), timeout=0.3)
            if r is not None and r[0] == 1:
                self._la_fired_at = time.time()
                self._la_edge_pending = False
                return True
            time.sleep(0.005)
        return False

    def la_read(self):
        """op8 chunk0..6 を読んで LATrace を返す。**発火後**でないとエラー。

        未発火の op8 は前回発火時の凍結バッファを返すだけなので、
        ライブラリ側で読み取りを拒否する。
        """
        if self._la_armed_at is None or self._la_fired_at is None:
            raise Osc001Error("発火前に op8 は読めない(凍結バッファ)")
        raw = bytearray()
        for c in range(LA_CHUNKS):
            r = self._txn(8, (c,))
            if r is None:
                raise Osc001Error("op8 chunk%d 無応答" % c)
            raw.extend(r)
            if CHUNK_GAP_S:
                time.sleep(CHUNK_GAP_S)
        return LATrace(bytes(raw[:LA_BUFFER_BYTES]), self._la_armed_at,
                       self._la_fired_at, time.time(), self._la_rate[0])

    def _la_clear(self):
        """マスク 0 のエッジ arm で、残っているエッジトリガを上書きする。発火はしない。"""
        frame = self._la_frame[:7] + (LA_CLEAR_EDGE, 0x00) if self._la_frame else \
            self.la_configure(channels=(), edge="rising")
        if self._txn(3, (0,)) is None or self._txn(16, frame) is None:
            raise Osc001Error("LA トリガの解除(op16 マスク 0)無応答")
        self._la_edge_pending = False

    def la_abort(self):
        """待機の中止。マスク 0 のエッジ arm で残ったトリガを解く(op5 では解けない)。

        以後 la_wait/la_read は再 arm まで拒否する。スコープに戻ってよい状態になる。
        """
        self._la_clear()
        self._la_armed_at = None
        self._la_fired_at = None

    def la_capture(self, channels=(2,), edge="falling", timeout=5.0):
        """configure → arm → wait → read。未発火で timeout したら abort して例外。

        timeout は発火待ちの時間。窓の記録時間 844/fs は自動で足す。
        """
        self.la_configure(channels, edge)
        self.la_arm()
        if not self.la_wait(timeout + LA_SAMPLES / self._la_rate[0]):
            self.la_abort()
            raise Osc001Error("LA トリガが %ss で発火しなかった(arm は解除済み)" % timeout)
        return self.la_read()


def la_rate_settings(hz):
    """LA の fs 指定 → 最も近い (実効 fs, byte3, (byte4, byte5, byte6, byte7))。

    候補は byte3=3/2/1 の固定 3 段と、byte3=0 の可変域(fs = 6e6/(D+29)、≦108.7 kHz)。
    比の誤差 |fs/hz − 1| が最小のものを選ぶ。byte4..7 の符号化はスコープと同じ
    (X = 65536 − (b4·256 + b5)、b6 bit0 で D = X/2、それ以外 D = 2^b7·X)。
    """
    hz = float(hz)
    if hz <= 0:
        raise Osc001Error("LA の fs は正の値: %r" % hz)
    dflt = tuple((LA_B47_DEFAULT >> (8 * i)) & 0xFF for i in range(4))
    cands = [(fs, b3, dflt) for b3, fs in LA_RATE_FIXED.items()]
    target = max(LA_CLOCK_HZ / hz - RATE_OFFSET, LA_D_MIN)
    for b6 in (0, 1):
        for b7 in range(0, BYTE7_VERIFIED + 1):
            shift = 0.5 if b6 else float(1 << b7)
            x = max(X_MIN, min(X_MAX, int(round(target / shift))))
            for xc in (x - 1, x, x + 1):
                if not X_MIN <= xc <= X_MAX:
                    continue
                fs = LA_CLOCK_HZ / (max(shift * xc, LA_D_MIN) + RATE_OFFSET)
                v = (0x10000 - xc) & 0xFFFF
                cands.append((fs, 0, (v >> 8, v & 0xFF, b6, b7)))
    return min(cands, key=lambda c: abs(c[0] / hz - 1.0))


def open(path=DEFAULT_DEVICE, timeout=2.0):        # noqa: A001 (builtins.open を意図的に遮蔽)
    """Scope を返す(まだ open していない。with で使うこと)。"""
    return Scope(path, timeout)


def main(argv):
    dev = argv[0] if argv else DEFAULT_DEVICE
    devs = enumerate()
    if not devs:
        print("04d8:%04x が見つからない(%s)" % (PRODUCT, dev))
        return 1
    print("見つかったデバイス: %r" % (devs,))
    with Scope(devs[0].path) as sc:
        try:
            print("info: %r" % sc.info()["info"])
        except Osc001Error as e:
            print("info 失敗: %s" % e)
        print("range CH1 = %s" % sc.set_range(1, 1.0))
        print("range CH2 = %s" % sc.set_range(2, 1.0))
        fs = sc.set_sample_rate(10000.0)
        print("sample_rate = %.1f SPS/ch(byte4=%d byte5=%d)"
              % (fs, sc.b[4], sc.b[5]))
        for w in sc.warnings():
            print("警告: %s" % w)
        t = sc.capture()
        print("read: %d バイト、%.1f SPS/ch" % (len(t.raw), t.rate_hz))
        for ch in (1, 2):
            v = t.to_volts(ch)
            print("  CH%d: min=%+.3fV max=%+.3fV mean=%+.3fV クリップ=%s"
                  % (ch, min(v), max(v), sum(v) / len(v), t.clipped(ch)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
