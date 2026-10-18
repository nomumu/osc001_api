#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""osc001.py の「デバイス不要な部分」を検証する(実測値との突き合わせ)。

検証する式(期待値はいずれも実機で確定した値):
  * fs 式 `fs = 3e6/(D+29)`、D = X/2 / 2^byte7·X(実機 20 点 ±1%)
  * レンジ表と V/div = 10 × V/LSB
  * 縦軸: V/LSB = VCOUNT[tap]·2^段(両 ch 共通)、0 V コード = m0/2^段 − 127 + slope·Position
  * op9 モニタ・0 V 合わせ・途中読み・待ち時間(偽トランスポート)

使い方: python3 -I python/test_osc001_math.py
"""
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import osc001                                                    # noqa: E402

fails = []


def check(name, got, want, tol=1e-3):
    ok = abs(got - want) <= tol * max(1.0, abs(want))
    print("%-46s got=%-12.5g want=%-12.5g %s"
          % (name, got, want, "OK" if ok else "NG"))
    if not ok:
        fails.append(name)


sc = osc001.Scope()

print("=== 1) byte4..byte7 → fs ===")
# want は実測値。式は ±1% で追う。
for b4, b5, b6, b7, want in (
        (255, 0, 0, 0, 10524.4), (255, 188, 0, 0, 30904.0),
        (255, 255, 0, 0, 33129.7),                    # 上限で頭打ち
        (254, 44, 0, 0, 6045.9), (254, 0, 0, 0, 5529.3),
        (254, 255, 0, 0, 10468.7),                    # byte5 が byte4<255 でも効く
        (254, 44, 0, 1, 3108.8), (254, 44, 0, 3, 797.9),
        (254, 44, 0, 6, 100.3), (254, 44, 0, 7, 50.0),
        (252, 87, 0, 6, 50.3),
        (254, 44, 1, 0, 11383.9), (254, 44, 1, 6, 11446.6),
        (254, 44, 1, 3, 11390.8),                     # byte6 bit0 → byte7 無視
        (254, 44, 2, 0, 6046.2), (254, 44, 3, 0, 11385.8),
        (139, 6, 1, 0, 200.6), (139, 6, 0, 0, 100.3),  # 実機で確かめたペイロード
        (253, 222, 1, 0, 9970.0), (255, 10, 1, 0, 19800.8)):
    sc.b[4], sc.b[5], sc.b[6], sc.b[7] = b4, b5, b6, b7
    check("b4=%3d b5=%3d b6=%d b7=%d" % (b4, b5, b6, b7),
          sc.sample_rate(), want, tol=2e-2)
# 実機の張り付き上限 ≒33.1 kSPS/chを式側も再現していること。
sc.set_sample_rate(1e9)
print("  指定超過(1e9)の実効 = %.1f SPS(byte4=%d byte5=%d byte6=%d byte7=%d)"
      % (sc.sample_rate(), sc.b[4], sc.b[5], sc.b[6], sc.b[7]))
if not (32000.0 <= sc.sample_rate() <= 34000.0):
    fails.append("上限が実機の張り付き(≒33.1k)に寄せられていない")
if sc.b[4] == 0:
    fails.append("上限丸めで byte4=0(不正値)になっている")

print("\n=== 1b) byte7 は下位 3 ビットだけ ===")
# want は実測値。
# 折り返さず 2^byte7 と計算すると 24000〜25000% 外れる領域。
for b4, b5, b6, b7, want in (
        (254, 44, 0, 8, 6036.2),                     # = byte7=0 の 6045.9
        (254, 44, 0, 9, 3109.3),                     # = byte7=1 の 3108.8
        (254, 44, 0, 10, 1586.6),                    # = byte7=2 の 1578.6
        (254, 44, 0, 14, 100.1),                     # = byte7=6 の 100.3
        (255, 255, 0, 15, 19214.1),                  # = byte7=7 の 19112.4
        (255, 255, 0, 12, 33146.0)):                 # = byte7=4(張り付き)
    sc.b[4], sc.b[5], sc.b[6], sc.b[7] = b4, b5, b6, b7
    check("b7=%2d → 折り返し %d" % (b7, b7 & osc001.BYTE7_MASK),
          sc.sample_rate(), want, tol=2e-2)
for b7 in range(8, 16):                              # 除数が折り返して一致すること
    sc.b[7] = b7
    d_hi = sc.divisor()
    sc.b[7] = b7 & osc001.BYTE7_MASK
    if abs(d_hi - sc.divisor()) > 1e-9:
        fails.append("divisor() が byte7=%d で折り返していない" % b7)
sc.b[7] = 0

print("\n=== 1c) 高レート側の張り付き = 定数 ===")
# X=64 で統一式がまだ成立し、X<=63 で張り付きへ遷移する境界の実測。
for b4, b5, b6, b7, want in (
        (255, 192, 0, 0, 32256.0),                   # X=64 → 則 32258.1
        (255, 194, 0, 0, 32769.9),                   # X=62 → 則 32967.0
        (255, 196, 0, 0, 33122.4),                   # X=60 → 張り付き
        (255, 255, 0, 0, 33129.7)):                  # X=1   → 張り付き
    sc.b[4], sc.b[5], sc.b[6], sc.b[7] = b4, b5, b6, b7
    check("X=%-4d(b4=%d b5=%d)" % (sc.x_value(), b4, b5),
          sc.sample_rate(), want, tol=2e-2)
if not (61.0 <= osc001.D_MIN <= 62.0):
    fails.append("D_MIN が実測の張り付き(3e6/33127.5−29=61.56)から離れている")

print("\n=== 2) set_sample_rate の丸め(指定 → 実効 → 再計算が一致) ===")
for hz in (30000.0, 20000.0, 10526.0, 10000.0, 8000.0, 5000.0, 1000.0, 200.0,
           100.0, 50.0, 10.0, 1.0, 0.5, 1e9):
    got = sc.set_sample_rate(hz)
    again = sc.sample_rate()
    ok = (abs(got - again) < 1e-9 and 1 <= sc.b[4] <= 255
          and sc.b[7] <= osc001.BYTE7_VERIFIED)
    if hz <= osc001.RATE_MAX:                 # X が整数なので 2% 以内(実際は 0.1% 未満)
        ok = ok and abs(got - hz) <= 0.02 * hz
    print("%-10.4g → b4=%3d b5=%3d b6=%d b7=%d X=%5d 実効 %9.2f (誤差 %+8.4f%%) %s"
          % (hz, sc.b[4], sc.b[5], sc.b[6], sc.b[7], sc.x_value(), got,
             100.0 * (got - hz) / hz, ok))
    if not ok:
        fails.append("set_sample_rate %g" % hz)

print("\n=== 3) レンジ表 ===")
for vdiv, tap, gain, label in osc001.RANGES:
    sc.set_range(1, vdiv)
    vlsb = sc.v_per_lsb(1)
    print("%-12s tap=%-3s 段=%d  byte1=%d byte8=%d  V/LSB=%8.4g  "
          "10×V/LSB=%8.4g  全スパン=%7.3gV"
          % (label, "÷8" if tap == 8 else "×1", gain, sc.b[1], sc.b[8],
             vlsb, 10 * vlsb, 256 * vlsb))
    if (sc.b[1], sc.b[8]) != ({8: 5, 1: 6}[tap], gain):
        fails.append("レンジ表のバイト値不一致: %s" % label)
    if tap == 1:                                  # ×1 は 6 ラベルの目盛と 0.5% 以内
        check("%s: 10×V/LSB == V/div" % label, 10 * vlsb, vdiv, tol=5e-3)

print("\n=== 4) CH2 は byte2/byte9 を使う ===")
sc.set_range(2, 0.05)
print("byte2=%d(9=×1) byte9=%d(0=50mV) V/LSB=%.4g"
      % (sc.b[2], sc.b[9], sc.v_per_lsb(2)))
if (sc.b[2], sc.b[9]) != (9, 0):
    fails.append("CH2 のレンジバイト値")

print("\n=== 5) tap を独立指定できる(range は段だけ変える) ===")
sc.set_range(1, 0.2, tap=8)                       # ×1 ラベルの段を ÷8 タップで
print("byte1=%d(5=÷8) byte8=%d V/LSB=%.4g" % (sc.b[1], sc.b[8], sc.v_per_lsb(1)))
check("÷8 段2 = VCOUNT[8] × 4", sc.v_per_lsb(1), osc001.VCOUNT[8] * 4, tol=1e-9)

print("\n=== 5b) Position のレンジ別加算補正===")
for vdiv, tap, gain, label in osc001.RANGES:
    sc.set_range(1, vdiv)
    want = osc001.POSITION_CORR[gain]
    print("%-12s 段%d → byte10=%02x(期待 %02x)" % (label, gain, sc.b[10], want))
    if sc.b[10] != want:
        fails.append("Position 補正: %s" % label)
sc.set_range(1, 0.5, position=0x40)
if sc.b[10] != 0x40:
    fails.append("position 直指定")
print("position 直指定: byte10=%02x" % sc.b[10])

print("\n=== 5c) set_gain で段 3..5(6 ラベルの外)=== ")
for g in range(6):
    sc.set_gain(1, g, tap=1)                     # ×1 タップで段だけ振る
    print("段%d V/LSB=%8.4g 全スパン=%8.4gV byte10=%02x"
          % (g, sc.v_per_lsb(1), 256 * sc.v_per_lsb(1), sc.b[10]))
    if abs(sc.v_per_lsb(1) - osc001.VCOUNT[1] * 2 ** g) > 1e-15:
        fails.append("set_gain 段%d" % g)

print("\n=== 5d) Position の中央 byte と段依存則 ===")
# 傾きは段ごとにちょうど −2^(1−段) codes/byte(実測 −1.9958/−0.9999/−0.5000/−0.2500/−0.1250/−0.0625)
for g in range(6):
    if abs(osc001.POSITION_SLOPE[g] - (-(2.0 ** (1 - g)))) > 1e-12:
        fails.append("POSITION_SLOPE 段%d" % g)
for g, want in ((0, 210), (1, 146), (2, 19)):
    c = sc.center_position(g)
    print("段%d 中央 byte=%s(実測中央 %.1f)"
          % (g, "None" if c is None else "%02x" % c, osc001.POSITION_CENTER[g]))
    if c != want:
        fails.append("center_position 段%d" % g)
if sc.center_position(3) is not None:
    fails.append("段3 の中央は到達不能のはずが byte を返した")
try:
    sc.set_gain(1, 3, tap=1, position="center")
    fails.append('段3 で position="center" が通ってしまった')
except osc001.Osc001Error:
    print('段3 + position="center" → 例外(中央到達不能)')
# POSITION_CORR は実測中央より**段によらず**約 18.4 byte 小さい
for g in (0, 1, 2):
    d = osc001.POSITION_CENTER[g] - osc001.POSITION_CORR[g]
    print("段%d: 実測中央 − 既定補正 = %+0.1f byte" % (g, d))
    if not 15.0 <= d <= 21.0:
        fails.append("POSITION_CORR と実測中央の差が段依存になっている(段%d: %+0.1f)" % (g, d))
sc.set_gain(1, 2, tap=1)

print("\n=== 6) 警告(clamp / 使用不可タップ / byte7 / fs 上限) ===")
sc.set_sample_rate(10526.0)                       # fs 上限警告を消してからタップを見る
sc.set_range(1, 0.05)                             # ×1 段0 → 全スパン 1.29V:警告なし
print("×1 段0:", sc.warnings())
if sc.warnings():
    fails.append("×1 段0 で警告が出てはいけない")
sc.set_range(1, 0.1)                              # ×1 段1 → 2.57V:警告なし
sc.set_range(1, 1.0)                              # ÷8 段1 → 22V:警告なし
if sc.warnings():
    fails.append("÷8 で ×1 クランプ警告が出てはいけない")
print("÷8 段1:", sc.warnings())
sc.b[2] = 7                                       # 使用不可タップ
print("byte2=7:", sc.warnings())
if not any("使用不可" in w for w in sc.warnings()):
    fails.append("使用不可タップが警告されない")
sc.b[2] = 8
sc.b[3] = 0x00
print("byte3=00(取得イネーブル):", sc.warnings())
if not any("byte3" in w for w in sc.warnings()):
    fails.append("byte3=0x00(0xFF 埋めで取得失敗)が警告されない")
sc.b[3] = osc001.BYTE3_REQUIRED
if any("byte3" in w for w in sc.warnings()):
    fails.append("byte3=0xAA(既定・正常)で警告が出てはいけない")
# byte3 のビット単位解読。
# bit7=取得イネーブル / bits1:0=01,10 が正常 / 00 でレベル崩れ / 11 で信号が乗らない
if (osc001.BYTE3_ENABLE_BIT, osc001.BYTE3_MODE_MASK, osc001.BYTE3_MODE_OK) != \
        (0x80, 0x03, (1, 2)):
    fails.append("byte3 の定数が bit7 / bits1:0 / (1,2) でない")
CASES3 = []
CASES3 += [(v, "disabled", "bit7") for v in
           (0x00, 0x02, 0x08, 0x20, 0x0A, 0x2A, 0x4A, 0x6A, 0x55, 0x7F)]
CASES3 += [(v, "level-shift", "bits1:0=00") for v in
           (0x80, 0x84, 0x88, 0x8C, 0x90, 0x94, 0x98, 0x9C,
            0xA0, 0xA4, 0xA8, 0xAC, 0xB0, 0xB4, 0xB8, 0xBC)]
CASES3 += [(v, "no-signal", "bits1:0=11") for v in
           (0x83, 0x87, 0x8B, 0x93, 0x97, 0x9B, 0x9F, 0xF7, 0xEF, 0xDF)]
CASES3 += [(v, "ok", None) for v in
           (0x81, 0x82, 0x85, 0x86, 0x89, 0x8A, 0x8D, 0x8E, 0x91, 0x92,
            0x95, 0x96, 0x99, 0x9A, 0x9D, 0x9E, 0xA1, 0xA2, 0xAA, 0xAE,
            0xBA, 0xEA, 0xFD, 0xFE)]
for v, want, key in CASES3:
    sc.b[3] = v
    got = sc.byte3_status()
    ws = sc.warnings()
    if got != want:
        fails.append("byte3=%02x の byte3_status が %s(期待 %s)" % (v, got, want))
    if key and not any(key in w for w in ws):
        fails.append("byte3=%02x が %r で警告されない" % (v, key))
    if want == "ok" and v != osc001.BYTE3_REQUIRED and not ws:
        fails.append("byte3=%02x(0xAA 以外)が既定と違う警告を出さない" % v)
    if v == osc001.BYTE3_REQUIRED and ws:
        fails.append("byte3=0xAA(既定・正常)で警告が出てはいけない")
print("byte3 49 値の分類: %s" % ", ".join(
    "%s %d 件" % (s, sum(1 for _, w, _ in CASES3 if w == s))
    for s in ("disabled", "level-shift", "no-signal", "ok")))
sc.b[3] = osc001.BYTE3_REQUIRED
sc.b[7] = 6
print("byte7=6(検証済み域):", sc.warnings())
if sc.warnings():
    fails.append("byte7=6 は検証済みなので警告してはいけない")
sc.b[7] = 8
print("byte7=8(未検証域):", sc.warnings())
if not any("byte7" in w for w in sc.warnings()):
    fails.append("byte7=8 が警告されない")
sc.b[7] = 0
sc.b[4] = 0
print("byte4=0(不正値):", sc.warnings())
if not any("byte4" in w for w in sc.warnings()):
    fails.append("byte4=0 が警告されない")
sc.b[4], sc.b[5], sc.b[6], sc.b[7] = 255, 200, 0, 0     # X=56 → 式は 35294 → 頭打ち
print("fs 上限超((255,200,0,0) → %.0f SPS):" % sc.sample_rate(), sc.warnings())
if not any("上限" in w for w in sc.warnings()):
    fails.append("fs 上限での頭打ちが警告されない")

print("\n=== 7) Trace のインターリーブ解釈(偶数番=CH1、奇数番=CH2) ===")
raw = bytes([0x7F + (i % 100) for i in range(384)])
t = osc001.Trace(raw, 1000.0, {"ch1": 1e-3, "ch2": 2e-3},
                 {"ch1": 8, "ch2": 8}, 0.0, 0.0)
s = t.samples
print("ch1 先頭 5: %s" % s["ch1"][:5])
print("ch2 先頭 5: %s" % s["ch2"][:5])
if s["ch1"][:5] != [0, 2, 4, 6, 8] or s["ch2"][:5] != [1, 3, 5, 7, 9]:
    fails.append("インターリーブ")
if len(s["ch1"]) != 192 or len(s["ch2"]) != 192:
    fails.append("サンプル数")
check("to_volts(ch2) の係数", t.to_volts(2)[10], s["ch2"][10] * 2e-3, tol=1e-9)
check("to_volts(ch1) の係数", t.to_volts(1)[10], s["ch1"][10] * 1e-3, tol=1e-9)
tz = osc001.Trace(raw, 1000.0, {"ch1": 1e-3, "ch2": 2e-3}, {"ch1": 8, "ch2": 8}, 0.0, 0.0,
                  {"ch1": -7.5, "ch2": 3.0})
check("to_volts は 0 V 基準(zero_code を引く)", tz.to_volts(1)[10], (s["ch1"][10] + 7.5) * 1e-3, tol=1e-9)
check("to_volts ch2 の 0 V 基準", tz.to_volts(2)[10], (s["ch2"][10] - 3.0) * 2e-3, tol=1e-9)
t2 = osc001.Trace(bytes([0xFF] + [0x7F] * 383), 1000.0, {"ch1": 1e-3, "ch2": 1e-3},
                  {"ch1": 8, "ch2": 8}, 0.0, 0.0)
print("rail_hit: 正常=%s / +128 到達=%s" % (t.rail_hit(1), t2.rail_hit(1)))
if t.rail_hit(1) or not t2.rail_hit(1):
    fails.append("rail_hit")

print("\n=== 7b) read_deep のバッファ定数 ===")
print("CHUNKS_DEEP=%d BUFFER_BYTES=%d SAMPLES_DEEP=%d OP_ECHO=%02x"
      % (osc001.CHUNKS_DEEP, osc001.BUFFER_BYTES, osc001.SAMPLES_DEEP, osc001.OP_ECHO))
if (osc001.CHUNKS_DEEP, osc001.BUFFER_BYTES, osc001.SAMPLES_DEEP) != (7, 422, 211):
    fails.append("バッファ定数が 7 / 422 / 211 でない")
deep = osc001.Trace(bytes([0x7F] * osc001.BUFFER_BYTES), 10526.0,
                    {"ch1": 0.02, "ch2": 0.02}, {"ch1": 8, "ch2": 8}, 0.0, 0.0)
n1, n2 = len(deep.samples["ch1"]), len(deep.samples["ch2"])
print("422 B の Trace → ch1 %d / ch2 %d サンプル" % (n1, n2))
if (n1, n2) != (osc001.SAMPLES_DEEP, osc001.SAMPLES_DEEP):
    fails.append("422 B の Trace から 211 サンプル/ch にならない")

print("\n=== 8) 記録時間 192/fs(op6 を見ていないときの read の待ち)===")
for b4, b5 in ((255, 0), (139, 6)):
    sc.b[4], sc.b[5], sc.b[6], sc.b[7] = b4, b5, 0, 0
    fs = sc.sample_rate()
    print("fs=%9.2f → 記録時間 %8.4f 秒" % (fs, osc001.SAMPLES_PER_CH / fs))
if osc001.CHUNK_GAP_S != 0.0:
    fails.append("チャンク間の休みは 0")

print("\n=== 9) LA:ニブル展開・ch↔bit 対応・凍結ガード ===")


def pack_nibs(nibs):
    return bytes(((nibs[i + 1] << 4) | nibs[i]) & 0xFF      # 下位ニブル先
                 for i in range(0, len(nibs), 2))


nibs = [0x6] * osc001.LA_SAMPLES          # 0b0110: ch2=1, ch3=1, ch1=ch4=0
tr = osc001.LATrace(pack_nibs(nibs), 0, 0, 0)
if len(tr.nibbles) != osc001.LA_SAMPLES or osc001.LA_SAMPLES != 844:
    fails.append("LA サンプル点数")
if tr.footer is not None:
    fails.append("フッター無し raw の footer は None")
# 448 B(フッター込み)の raw: 末尾 26 B は復号せず footer へ
W_LA = (544, 0, 552, 256, 552, 16520, 640, 16452, 640, 16420, 704, 16484, 704)
raw448 = pack_nibs([0x4] * osc001.LA_SAMPLES) + b"".join(bytes((w & 0xFF, w >> 8)) for w in W_LA)
tr3 = osc001.LATrace(raw448, 0, 0, 0)
if len(tr3.bits(2)) != 844 or set(tr3.bits(2)) != {1} or tr3.footer != W_LA:
    fails.append("LA フッター分離(844 + 26 B)")
if tr3.rate_hz != osc001.LA_RATE_HZ:
    fails.append("LA rate_hz")
for ch, want in ((1, 0), (2, 1), (3, 1), (4, 0)):
    got = set(tr.bits(ch))
    if got != {want}:
        fails.append("LA ch%d のニブル bit 対応" % ch)
if tr.transitions(2) != 0:
    fails.append("LA 一定列の遷移数")

nibs2 = [0x0] * 400 + [0x4] * (osc001.LA_SAMPLES - 400)   # ch2 のみ 0→1
tr2 = osc001.LATrace(pack_nibs(nibs2), 0, 0, 0)
s2 = tr2.bits(2)
edge_pos = [i for i in range(1, len(s2)) if s2[i] != s2[i - 1]]
if edge_pos != [400]:
    fails.append("LA エッジ位置(400 期待: %s)" % edge_pos)
# 奇数位置のエッジ: 0x40 の 1 バイト(下位=0、上位=4)は「0 → 1」。上位先で読むと 1 サンプルの往復になる
raw_odd = bytes([0x00] * 200 + [0x40] + [0x44] * (osc001.LA_VALID_BYTES - 201))
s3 = osc001.LATrace(raw_odd, 0, 0, 0).bits(2)
edge_pos = [i for i in range(1, len(s3)) if s3[i] != s3[i - 1]]
if edge_pos != [401]:
    fails.append("LA 下位ニブル先(奇数エッジ 401 期待: %s)" % edge_pos)

sc_la = osc001.Scope()
try:
    sc_la.la_read()                        # 未発火 → 凍結ガードで例外
    fails.append("la_read の未発火ガードが働かない")
except osc001.Osc001Error:
    pass
fr = sc_la.la_configure(channels=(1, 3), edge="rising")
# args は wire byte1.. に載る。args[7]=byte8(極性)、args[8]=byte9(マスク)
if len(fr) != 9 or fr[7] != 0x01 or fr[8] != (0x80 | 0x20):
    fails.append("la_configure フレーム(byte8/9)")
for bad in ((lambda: sc_la.la_configure(edge="both")),
            (lambda: sc_la.la_configure(channels=(5,))),
            (lambda: sc_la.la_configure(channels=(2,), edge="auto")),   # ハング領域のガード
            (lambda: sc_la.la_wait(0.01))):                             # arm 前
    try:
        bad()
        fails.append("la_configure 引数検証 / la_wait 状態検証")
    except osc001.Osc001Error:
        pass
# 偽トランスポートで arm → abort の状態遷移(op5 を送り、以後 wait/read を拒否)
sent = []
sc_la._txn = lambda op, args=(), timeout=None: (sent.append(op), bytes([1]) + bytes(63))[1]
sc_la.la_configure(channels=(2,), edge="rising")
sc_la.la_arm()
if not sc_la.la_wait(0.1):
    fails.append("偽 op6=1 で発火判定")
sc_la.la_abort()
if sent[-2:] != [3, 16]:
    fails.append("la_abort は op3 → op16(マスク 0)を送る")
# 未発火のエッジ arm が残ったまま arm()(op5)すると、先にマスク 0 の op16 で解除する
frames = []
sc_la._txn = lambda op, args=(), timeout=None: (frames.append((op, tuple(args))), bytes([0]) + bytes(63))[1]
sc_la.la_configure(channels=(2,), edge="falling")
sc_la.la_arm()
if sc_la.la_wait(0.02):
    fails.append("偽 op6=0 で未発火")
frames.clear()
sc_la.arm()
if [f[0] for f in frames] != [3, 16, 5] or frames[1][1][7:9] != (0x01, 0x00):
    fails.append("未発火エッジ arm 後の op5 は先にマスク 0 で解除(%s)" % frames)
frames.clear()
sc_la.arm()
if [f[0] for f in frames] != [5]:
    fails.append("解除済みなら op5 だけ")
for bad in ((lambda: sc_la.la_wait(0.01)), (lambda: sc_la.la_read())):
    try:
        bad()
        fails.append("la_configure 引数検証")
    except osc001.Osc001Error:
        pass

# LA の fs 選択(byte3=3/2/1 固定段と byte3=0 の可変域、out_la_rate_fine の実測値と照合)
for hz, want_b3, want_fs, tol in ((1e6, 3, 979400.0, 1e-9), (5e5, 2, 490900.0, 1e-9),
                                  (2e5, 1, 200000.0, 1e-9), (2e4, 0, 20000.0, 2e-3),
                                  (2000.0, 0, 2000.0, 2e-3), (10.0, 0, 10.0, 2e-3),
                                  (1e5, 0, 1e5, 5e-3)):
    fs, b3, b47 = osc001.la_rate_settings(hz)
    if b3 != want_b3 or abs(fs / want_fs - 1) > tol:
        fails.append("la_rate_settings(%g) → %g Hz byte3=%d" % (hz, fs, b3))
# 実測の byte4..7 → fs(D=271: fe f1 00 00 で 19982 Hz、D=2971: f4 65 00 00 で 2000 Hz)
fs, b3, b47 = osc001.la_rate_settings(20000.0)
x = 65536 - (b47[0] * 256 + b47[1])
d = x / 2.0 if b47[2] & 1 else (1 << b47[3]) * x
if abs(6e6 / (d + 29) - fs) > 1e-6:
    fails.append("la_rate_settings の byte4..7 と fs の整合")
sc_r = osc001.Scope()
sc_r.la_set_rate(2000.0)
fr = sc_r.la_configure(channels=(2,), edge="rising")
if fr[2] != 0 or fr[7] != 1 or fr[8] != 0x40:
    fails.append("la_set_rate 後の op16 フレーム(byte3=0)")

print("\n=== 10) ステータスフッター:抽出・stale・電源サイクル ===")
# 実測基準(out_footer_toggle2 の 13 語)。w05/w07 の A⇔B と w02/w10 の電源間変動を使う
W_A = (544, 0, 552, 256, 552, 16388, 640, 16584, 640, 16420, 704, 16484, 704)
W_B = (544, 0, 552, 256, 552, 16520, 640, 16452, 640, 16420, 704, 16484, 704)
W_PC = (544, 0, 568, 256, 568, 16388, 640, 16584, 640, 16420, 708, 16484, 708)

resp = bytearray(38) + b"".join(struct.pack("<H", w) for w in W_A)
if osc001.Scope.footer_words(bytes(resp)) != W_A:
    fails.append("footer_words の 13 語抽出")

sc_f = osc001.Scope()
raw = bytes([0x7F] * osc001.BUFFER_BYTES)
st, pc, ff = sc_f._footer_check(W_A, raw)          # 初回:基準設定のみ
if st or pc or ff:
    fails.append("初回フッター判定(全て False 期待)")
st, _, _ = sc_f._footer_check(W_A, raw)            # 同一フッター+同一データ
if not st:
    fails.append("stale 検出(一致で True 期待)")
st, _, _ = sc_f._footer_check(W_B, raw)            # パリティ反転=別トランザクション
if st:
    fails.append("stale 検出(不一致で False 期待)")
st, pc, _ = sc_f._footer_check(W_PC, raw)          # 静的語差分=電源サイクル
if not pc:
    fails.append("電源サイクル検出(差分で True 期待)")
st, pc, _ = sc_f._footer_check(W_PC, raw)          # 基準更新済みでもう一度
if pc:
    fails.append("電源サイクルは同一サイクルで 1 回だけ True")
_, _, ff = sc_f._footer_check(W_A, bytes([0xFF] * osc001.BUFFER_BYTES))
if not ff:
    fails.append("0xFF 埋め(取得失敗)検出")

print("\n=== 11) Position の 0 V 切片とハードトリガ ===")
sc_t = osc001.Scope()
sc_t.set_range(1, 1.0)                     # ÷8 段 1、Position 0x80(既定補正)
check("zero_code CH1 ÷8 段1 pos 0x80", sc_t.zero_code(1), 495.0 / 2 - 127 - 128)
check("volts_to_code CH1 2.5V", sc_t.volts_to_code(1, 2.5), -7.5 + 2.5 / 0.0958)
sc_t.set_range(2, 0.2)                     # ×1 段 2、Position 0x00
check("zero_code CH2 ×1 段2 pos 0x00", sc_t.zero_code(2), 388.0 / 4 - 127)
sc_t.set_gain(2, 3, tap=8, position=0x10)
check("zero_code 段 3 は式からの外挿", sc_t.zero_code(2), 496.5 / 8 - 127 - 0.25 * 0x10)
sc_t.set_zero_m0(2, 8, 500.0)
check("set_zero_m0 が zero_code に効く", sc_t.zero_code(2), 500.0 / 8 - 127 - 0.25 * 0x10)
sc_t.set_zero_m0(2, 8, osc001.M0_DEFAULT[2][8])
sc_t.set_trigger("ch1", "rising", level_v=2.5)
want15 = int(round(sc_t.volts_to_code(1, 2.5))) + 0x7F
if (sc_t.b[12], sc_t.b[13], sc_t.b[15], sc_t.b[19]) != (1, 1, want15, 1):
    fails.append("CH1 ÷8 トリガのバイト(%s)" % [sc_t.b[i] for i in (12, 13, 15, 19)])
if sc_t.trigger_level_raw() != want15:
    fails.append("trigger_level_raw")
sc_t.set_range(1, 0.2)                     # ×1 にすると次の arm で経路 2 へ
sc_t._apply_trigger()
if sc_t.b[19] != 2:
    fails.append("CH1 ×1 タップでトリガ経路 byte19=2")
sc_t.set_range(1, 1.0)
keep = (sc_t.trigger(), [sc_t.b[i] for i in (12, 13, 15, 19)])
for bad in ((lambda: sc_t.set_trigger("ch2")),
            (lambda: sc_t.set_trigger("ch1", edge="both")),
            (lambda: sc_t.set_trigger("ch1", mode="single")),
            (lambda: sc_t.set_trigger("ch1", level_v=20.0)),       # 1 V/div の表示域外
            (lambda: sc_t.set_trigger("ch1", level_raw=0x00)),
            (lambda: sc_t.set_trigger("ch1", level_raw=0xFF))):
    try:
        bad()
        fails.append("set_trigger 引数検証")
    except ValueError:
        pass
if (sc_t.trigger(), [sc_t.b[i] for i in (12, 13, 15, 19)]) != keep:
    fails.append("set_trigger の失敗で設定が巻き戻らない")
sc_t.set_trigger("ext", "falling", level_raw=0x20)
if (sc_t.b[12], sc_t.b[13], sc_t.b[15], sc_t.b[19]) != (1, 0, osc001.TRIG_LEVEL_EXT, 3):
    fails.append("EXT トリガのバイト")
if not any("EXT" in w for w in sc_t.warnings()):
    fails.append("EXT の T 浮き警告")
sc_t.set_trigger(None)
if sc_t.trigger() is not None or sc_t.b[12] != 0:
    fails.append("set_trigger(None) でフリーラン")


def fake_scope(op6_seq):
    """op6 の応答 byte0 を op5 ごとに切り替える偽トランスポート。op6_seq[k] = k 回目の arm 後の値。"""
    sc_ = osc001.Scope()
    sc_.set_sample_rate(20000.0)
    st = {"arms": 0, "ops": []}

    def txn(op, args=(), timeout=None):
        st["ops"].append((op, tuple(args)))
        if op == 5:
            st["arms"] += 1
        v = op6_seq[min(st["arms"], len(op6_seq)) - 1] if op == 6 else 0x7F
        return bytes([v]) + bytes(63)
    sc_._txn = txn
    return sc_, st


sc_n, st_n = fake_scope([0])
sc_n.set_trigger("ch1", "rising", level_v=1.0, mode="normal")
try:
    sc_n.capture(timeout=0.05)
    fails.append("Normal 未成立で TriggerTimeout")
except osc001.TriggerTimeout:
    pass
if any(o[0] == 8 for o in st_n["ops"]):
    fails.append("Normal 未成立で op8(凍結バッファ)を読んだ")
sc_a, st_a = fake_scope([0, 1])
sc_a.set_trigger("ch1", "rising", level_v=1.0, mode="auto", auto_timeout=0.05)
tr = sc_a.capture(timeout=0.5)
op5s = [o[1] for o in st_a["ops"] if o[0] == 5]
if tr.triggered is not False or len(op5s) != 2 or op5s[0][11] != 1 or op5s[1][11] != 0:
    fails.append("Auto 未成立 → byte12=0 で取り直し(triggered=%r, op5=%d)" % (tr.triggered, len(op5s)))
sc_a.arm()
if [o[1] for o in st_a["ops"] if o[0] == 5][-1][11] != 1:
    fails.append("Auto 取り直しの後の arm で byte12=1 に戻る")
sc_ok, _ = fake_scope([1])
sc_ok.set_trigger("ext", "rising")
if sc_ok.capture(timeout=0.5).triggered is not True:
    fails.append("成立で triggered=True")
sc_fr, _ = fake_scope([1])
if sc_fr.capture(timeout=0.5).triggered is not None:
    fails.append("トリガ無しで triggered=None")


print("\n=== 12) op9 モニタ・0 V 合わせ・途中読み・待ち(偽トランスポート)===")


def fake_mon(m=(600, 497, 1019, 386), op6=1):
    """op9 は sel 5/8/6/9 に m を返し、op8 は 0x7F(code 0)、op6 は op6 を返す。"""
    sc_ = osc001.Scope()
    sc_.set_sample_rate(20000.0)
    st = {"ops": []}
    tbl = {5: m[0], 8: m[1], 6: m[2], 9: m[3]}

    def txn(op, args=(), timeout=None):
        st["ops"].append((op, tuple(args), time.time()))
        if op == 9:
            a, b = tbl.get(args[0], 0), tbl.get(args[1], 0)
            return bytes([a >> 8, a & 0xFF, b >> 8, b & 0xFF]) + bytes(60)
        if op == 6:
            return bytes([op6]) + bytes(63)
        return bytes([0x7F]) * 64
    sc_._txn = txn
    return sc_, st


sc_m, st_m = fake_mon()
mo = sc_m.monitor(tap=8, n=3)
check("monitor ÷8 CH1(m=600)", mo["ch1"], (600 - 495.0) * 0.0479, tol=1e-9)
check("monitor ÷8 CH2(m=497)", mo["ch2"], (497 - 496.5) * 0.0479, tol=1e-9)
if [o[1][:2] for o in st_m["ops"]] != [(5, 8)] * 3 or mo["sat"] != (False, False):
    fails.append("monitor ÷8 は op9(5,8) を n 回")
mo1 = sc_m.monitor(tap=1)
if mo1["sat"] != (True, False):
    fails.append("monitor ×1 の張り付き検出(m=1019)")
cz = sc_m.calibrate_zero(duration=0.01)
if abs(sc_m.m0[1][8] - 600) > 1e-9 or abs(sc_m.m0[2][1] - 386) > 1e-9:
    fails.append("calibrate_zero の op9 平均(%s)" % cz["m0"])
# 偽 op8 は code 0 → m0_gain = (0 + 127 − slope·pos)·2^段。÷8 段 1 は Position 0x80 → (127 + 128)·2
if abs(sc_m.m0_gain.get((1, 8, 1), 0) - 510.0) > 1e-9 or abs(sc_m.m0_gain.get((2, 8, 2), 0) - 508.0) > 1e-9:
    fails.append("calibrate_zero のレンジ別 m0(%s)" % sorted(cz["m0_gain"].items()))
sc_m.set_range(1, 1.0)
check("m0_gain が zero_code に優先", sc_m.zero_code(1), 0.0, tol=1e-9)
if sc_m.b[12] != 0 or sc_m.trigger() is not None:
    fails.append("calibrate_zero 後に設定が戻らない")

sc_w, st_w = fake_mon(op6=1)
sc_w.set_sample_rate(5.0)                      # 192/fs = 38 s。op6 を見ていれば待たない
t0 = time.time()
tr_w = sc_w.capture(timeout=1.0)
dt = time.time() - t0
print("op6 を見た後の read: %.3f s" % dt)
if dt > 0.5:
    fails.append("op6 を見た後の read が待っている(%.2f s)" % dt)
sc_d, _ = fake_mon(op6=1)
td = sc_d.capture_deep(timeout=0.5)
td2 = sc_d.capture_deep(timeout=0.5)
if len(td.samples["ch1"]) != osc001.SAMPLES_DEEP or td.footer is None or td.stale or not td2.stale:
    fails.append("capture_deep(211 点・フッター・stale 判定)")
sc_p, _ = fake_mon(op6=0)
sc_p.set_sample_rate(100.0)
sc_p.arm()
time.sleep(0.25)
tp = sc_p.read_partial()
print("read_partial: valid=%d(期待 ≒24)" % tp.valid)
if not 20 <= tp.valid <= 26:
    fails.append("read_partial の valid(%d)" % tp.valid)
sc_p.set_trigger("ch1", level_v=1.0)
sc_p.arm()
try:
    sc_p.read_partial()
    fails.append("ハードトリガ待ちの read_partial は拒否")
except osc001.Osc001Error:
    pass

print()
if fails:
    print("NG %d 件: %s" % (len(fails), fails))
    sys.exit(1)
print("全項目 OK")
