/* C++ ラッパの実機デモ(Linux のみ。hidraw 自動探索)。
 * C 版 demo_capture.c と同じ手順を例外と vector で書く。
 * 使い方: ./demo_capture_cpp [dev]   (既定は自動探索) */

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "../include/osc001.hpp"

int main(int argc, char **argv)
{
#ifdef __linux__
    try {
        osc001::Scope sc = (argc > 1)
            ? osc001::Scope(std::string(argv[1]))
            : osc001::Scope();

        std::printf("info: %s\n", sc.info().c_str());
        sc.setRange(1, 0.5);
        std::printf("fs 実効: %.1f Hz/ch\n", sc.setSampleRate(10000.0));
        const std::string w = sc.warnings();
        if (!w.empty())
            std::printf("警告: %s\n", w.c_str());

        osc001::Trace tr = sc.capture();
        std::printf("V/LSB CH1=%.5f CH2=%.5f tap=%d/%d\n",
                    tr.vPerLsb(1), tr.vPerLsb(2), tr.tap(1), tr.tap(2));
        std::printf("rail_hit CH1=%d CH2=%d clipped CH1=%d CH2=%d\n",
                    tr.railHit(1) ? 1 : 0, tr.railHit(2) ? 1 : 0,
                    tr.clipped(1) ? 1 : 0, tr.clipped(2) ? 1 : 0);

        std::vector<double> v = tr.volts(1);
        std::printf("CH1 先頭 8 サンプル(V):");
        for (size_t i = 0; i < 8 && i < v.size(); i++)
            std::printf(" %+.3f", v[i]);
        std::printf("\nCH1 電圧範囲: %.3f 〜 %.3f V\n",
                    *std::min_element(v.begin(), v.end()),
                    *std::max_element(v.begin(), v.end()));
        return 0;
    } catch (const osc001::Error &e) {
        std::printf("エラー(code=%d): %s\n", e.code(), e.what());
        return 1;
    }
#else
    (void)argc;
    (void)argv;
    std::printf("このデモは Linux の hidraw 自動探索を使います。"
                "自前トランスポートで Scope(osc001_transport_t) を作ってください\n");
    return 1;
#endif
}
