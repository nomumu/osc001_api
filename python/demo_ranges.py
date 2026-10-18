#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""6 レンジを総なめして、Mega2560 の 5V 矩形(0〜5V)の実測スパンから V/LSB を逆算する。

信号源: Mega2560 pin11(OC1A)→CH1、pin12(OC1B)→CH2、GND 共通、1kHz 矩形 0〜5V。
期待値は osc001.VCOUNT[tap] × 2^段。
終了時に既定設定(÷8・段 1・10526 SPS)へ戻す。

使い方: python3 -I python/demo_ranges.py [/dev/hidraw2]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import osc001                                                   # noqa: E402

INPUT_V = 5.0                                                   # Mega 確認済み
BARS = " .:-=+*#%@"


def plot(v, width=48):
    lo, hi = min(v), max(v)
    step = max(1, len(v) // width)
    return "".join(BARS[int((v[k] - lo) * (len(BARS) - 1) / max(1, hi - lo))]
                   for k in range(0, len(v), step))


def main(argv):
    dev = argv[0] if argv else osc001.DEFAULT_DEVICE
    devs = osc001.enumerate()
    if not devs:
        print("04d8:f891 が見つからない")
        return 1
    with osc001.Scope(dev if dev != osc001.DEFAULT_DEVICE else devs[0].path) as sc:
        fs = sc.set_sample_rate(10526.0)
        print("fs = %.1f SPS/ch(byte4=%d byte5=%d byte6=%d byte7=%d X=%d)、"
              "入力 = %.2fV 矩形 1kHz"
              % (fs, sc.b[4], sc.b[5], sc.b[6], sc.b[7], sc.x_value(), INPUT_V))
        print("%-11s %-4s %-3s %-7s | %-22s | %-22s"
              % ("range", "tap", "段", "byte1/2", "CH1 実測(期待)"
                 "                    ", "CH2 実測(期待)"))
        for vdiv, tap, gain, label in osc001.RANGES:
            sc.set_range(1, vdiv)
            sc.set_range(2, vdiv)
            t = sc.capture()
            row = []
            for ch in (1, 2):
                s = t.samples["ch%d" % ch]
                span = max(s) - min(s)
                meas = INPUT_V / span if span else float("inf")
                row.append("min=%+4d max=%+4d span=%3d V/LSB=%7.4g(期待比 %5.2f)%s"
                           % (min(s), max(s), span, meas, meas / sc.v_per_lsb(ch),
                              (" ★レール到達" if t.rail_hit(ch)
                               else " ★クリップ" if t.clipped(ch) else "")))
                print("     CH%d %s" % (ch, plot(s, 48)))
            print("%-11s %-4s %-3d %-7s | %s | %s"
                  % (label, "÷8" if tap == 8 else "×1", gain,
                     "%d/%d" % (sc.b[1], sc.b[2]), row[0], row[1]))
        print("\n=== Position 掃引")
        CORR = {2: 0x00, 1: 0x80, 0: 0xc0}
        for vdiv, tap, gain, label in osc001.RANGES:
            sc.set_range(1, vdiv)
            sc.set_range(2, vdiv)
            outs = []
            for pos in (0x00, 0x40, 0x80, 0xc0, (0x80 + CORR[gain]) & 0xFF):
                sc.set_position(1, pos)
                sc.set_position(2, pos)
                t = sc.capture()
                s = t.samples["ch1"]
                outs.append("pos=%02x:%+d/%+d(%3d)"
                            % (pos, min(s), max(s), max(s) - min(s)))
            print("%-11s 段%d %s" % (label, gain, "  ".join(outs)))
        # 既定へ戻す
        for i, v in osc001.DEFAULT_BYTES.items():
            sc.b[i] = v
        t = sc.capture()
        s1 = t.samples["ch1"]
        print("既定復帰: byte1=%d byte2=%d byte8=%d byte9=%d byte4=%d byte5=%d "
              "→ CH1 span=%d(生存確認)"
              % (sc.b[1], sc.b[2], sc.b[8], sc.b[9], sc.b[4], sc.b[5],
                 max(s1) - min(s1)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
