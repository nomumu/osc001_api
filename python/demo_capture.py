#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""osc001.py の実機デモ

信号源は Mega2560(pin11 OC1A→CH1、pin12 OC1B→CH2、GND 共通、0〜5V 矩形 1kHz)。
既定設定(÷8 タップ・段 1・byte4=255/byte5=0 = 10526 SPS)は  の
既知良好ペイロードそのものなので、このスクリプトはデバイスを既定状態に置き、
最後に取得が生きていることを確認して終わる。

使い方: python3 -I python/demo_capture.py [/dev/hidraw2]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import osc001                                                   # noqa: E402

BARS = " .:-=+*#%@"


def edges(v, min_edges=6):
    """矩形波のしきい交差位置(サンプル単位)を返す。"""
    lo, hi = min(v), max(v)
    if hi - lo < 8:
        return None, lo, hi
    thr = (lo + hi) / 2.0
    prev = v[0] > thr
    e = []
    for i in range(1, len(v)):
        cur = v[i] > thr
        if cur != prev:
            d = v[i] - v[i - 1]
            e.append(i - 1 + ((thr - v[i - 1]) / d if d else 0.0))
        prev = cur
    if len(e) < min_edges:
        return None, lo, hi
    return (e[-1] - e[0]) / (len(e) - 1), lo, hi


def plot(v, width=64):
    lo, hi = min(v), max(v)
    out = []
    step = max(1, len(v) // width)
    for k in range(0, len(v), step):
        pos = int((v[k] - lo) * (len(BARS) - 1) / max(1, hi - lo))
        out.append(BARS[pos])
    return "".join(out)


def main(argv):
    dev = argv[0] if argv else osc001.DEFAULT_DEVICE
    devs = osc001.enumerate()
    if not devs:
        print("04d8:f891 が見つからない(udev ルールと接続を確認)")
        return 1
    print("列挙: %r" % (devs,))
    path = dev if dev != osc001.DEFAULT_DEVICE else devs[0].path

    with osc001.Scope(path) as sc:
        try:
            print("op2 info: %r" % sc.info()["info"])
        except osc001.Osc001Error as e:
            print("op2 失敗: %s" % e)

        print("range CH1 = %s / CH2 = %s"
              % (sc.set_range(1, 1.0), sc.set_range(2, 1.0)))
        fs = sc.set_sample_rate(10526.0)
        print("sample_rate 指定 10526 → 実効 %.1f SPS/ch"
              "(byte4=%d byte5=%d byte6=%d byte7=%d X=%d D=%.0f)"
              % (fs, sc.b[4], sc.b[5], sc.b[6], sc.b[7],
                 sc.x_value(), sc.divisor()))
        print("V/LSB: CH1 %.4g  CH2 %.4g" % (sc.v_per_lsb(1), sc.v_per_lsb(2)))
        for w in sc.warnings():
            print("警告: %s" % w)

        t = sc.capture()
        print("op8 chunk0..5 = %d バイト、arm→read %.3f 秒"
              % (len(t.raw), t.read_at - t.armed_at))
        for ch in (1, 2):
            s = t.samples["ch%d" % ch]
            v = t.to_volts(ch)
            print("CH%d LSB: min=%+4d max=%+4d span=%3d mean=%+.1f"
                  % (ch, min(s), max(s), max(s) - min(s), sum(s) / len(s)))
            print("     V : min=%+.3f max=%+.3f span=%.3f mean=%+.3f  クリップ=%s"
                  % (min(v), max(v), max(v) - min(v), sum(v) / len(v),
                     t.clipped(ch)))
            h, lo, hi = edges(s)
            if h:
                # 半周期サンプル数 → 実測 fs。1kHz 矩形なら fs ≒ 2 × 1000 × 半周期
                print("     半周期 %.2f サンプル → fs 推定 %.0f SPS(設定 %.0f)"
                      % (h, 2 * h * 1000.0 if h else 0, t.rate_hz))
            else:
                print("     矩形波のエッジが検出できない(信号なし?)lo/hi=%d/%d"
                      % (lo, hi))
            print("     %s" % plot(s))

        # 期待値チェック: Mega 5V ÷8 段1(V/LSB 86.4 mV)なら span ≒ 58 LSB
        s1 = t.samples["ch1"]
        span = max(s1) - min(s1)
        want = 5.0 / sc.v_per_lsb(1)
        print("CH1 スパン %.0f LSB(5V ÷8 段1 の期待 ≒ %.0f LSB、比 %.2f)"
              % (span, want, span / want))

        # 実測後は既定へ戻して生存確認
        for i, v in osc001.DEFAULT_BYTES.items():
            sc.b[i] = v
        t = sc.capture()
        s = t.samples["ch1"]
        print("既定復帰: byte4=%d byte5=%d byte6=%d byte7=%d → CH1 min=%+d max=%+d "
              "span=%d(生存確認)"
              % (sc.b[4], sc.b[5], sc.b[6], sc.b[7], min(s), max(s), max(s) - min(s)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
