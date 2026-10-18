/* デモ: 1 発取得して要約を表示(Linux hidraw)。python/demo_capture.py に対応。
 * 使い方: ./demo_capture [/dev/hidrawN]
 * 信号源(Arduino Mega)が無くても静的入力で動作する。 */

#include <stdio.h>
#include <string.h>

#include "../include/osc001.h"

int main(int argc, char **argv)
{
    char paths[8][64], names[8][64];
    char err[256], warn[2048];
    const char *path = NULL;
    osc001_transport_t t;
    osc001_handle *h;
    osc001_trace_t tr;
    int n, i, rc;

    if (argc > 1) {
        path = argv[1];
    } else {
        n = osc001_enumerate_hidraw(paths, 8, names);
        if (n <= 0) {
            printf("04d8:%04x が見つからない(udev ルール/接続を確認)\n", OSC001_PRODUCT);
            return 1;
        }
        for (i = 0; i < n && i < 8; i++)
            printf("発見: %s %s\n", paths[i], names[i]);
        path = paths[0];
    }

    if (osc001_transport_hidraw(&t, path) != 0) {
        printf("トランスポート初期化失敗\n");
        return 1;
    }
    h = osc001_open(&t, err, sizeof err);
    if (!h) {
        printf("open 失敗: %s\n", err);
        return 1;
    }

    {
        char info[64];
        if (osc001_info(h, info, sizeof info) == OSC001_OK)
            printf("info: %s\n", info);
    }

    osc001_set_range(h, 1, 0.5, 0, OSC001_POS_AUTO);
    osc001_set_range(h, 2, 0.5, 0, OSC001_POS_AUTO);
    printf("fs 指定 10000 → 実効 %.1f SPS/ch\n", osc001_set_sample_rate(h, 10000.0));
    osc001_warnings(h, warn, sizeof warn);
    if (warn[0])
        printf("警告:\n%s", warn);

    rc = osc001_capture(h, &tr, 5000);
    if (rc != OSC001_OK) {
        printf("取得失敗(%d): %s\n", rc, osc001_errmsg(h));
        osc001_close(h);
        return 1;
    }
    printf("取得: %d サンプル/ch @ %.1f SPS/ch、V/LSB CH1=%.5f CH2=%.5f\n",
           osc001_trace_samples_per_ch(&tr), tr.rate_hz,
           tr.v_per_lsb[0], tr.v_per_lsb[1]);
    printf("CH1 先頭 8:");
    for (i = 0; i < 8; i++)
        printf(" %d", osc001_trace_sample(&tr, 1, i));
    printf("\nCH2 先頭 8:");
    for (i = 0; i < 8; i++)
        printf(" %d", osc001_trace_sample(&tr, 2, i));
    printf("\nレール到達: CH1=%s CH2=%s / ×1 クリップ疑い: CH1=%s CH2=%s\n",
           osc001_trace_rail_hit(&tr, 1) ? "あり" : "なし",
           osc001_trace_rail_hit(&tr, 2) ? "あり" : "なし",
           osc001_trace_clipped(&tr, 1) ? "あり" : "なし",
           osc001_trace_clipped(&tr, 2) ? "あり" : "なし");

    /* 電圧変換の例(CH1、0 V 基準 = サンプル − zero_code) */
    {
        double v[OSC001_SAMPLES_DEEP], vmin = 1e9, vmax = -1e9;
        int nch = osc001_trace_samples_per_ch(&tr);
        osc001_trace_to_volts(&tr, 1, v, nch);
        for (i = 0; i < nch; i++) {
            if (v[i] < vmin) vmin = v[i];
            if (v[i] > vmax) vmax = v[i];
        }
        printf("CH1 電圧範囲(入力端子基準): %.3f 〜 %.3f V\n", vmin, vmax);
    }

    osc001_close(h);
    return 0;
}
