/* open エラー区別と排他(flock)の実機確認ツール。
 * 想定出力(実機検証済み 2026-10-13):
 *   ENOENT 経路: open 失敗: ノード無し(...)
 *   1回目: OK
 *   2回目: open 失敗: 他プロセスが占有中(flock 排他)...
 *   close 後再オープン: OK
 * 使い方: ./lock_check [dev]   (既定 /dev/hidraw2) */

#include <stdio.h>

#include "../include/osc001.h"

int main(int argc, char **argv)
{
    const char *dev = (argc > 1) ? argv[1] : "/dev/hidraw2";
    char err[256];
    osc001_transport_t t1, t2, t3;
    osc001_handle *h1, *h2, *h3;

    /* 1) 存在しないノード */
    osc001_transport_hidraw(&t3, "/dev/hidraw99");
    h3 = osc001_open(&t3, err, sizeof err);
    printf("ENOENT 経路: %s\n", h3 ? "開いてしまった(想定外)" : err);
    if (h3) osc001_close(h3);

    /* 2) 同一ノードの二重オープン(flock で拒否されるはず) */
    osc001_transport_hidraw(&t1, dev);
    h1 = osc001_open(&t1, err, sizeof err);
    printf("1回目: %s\n", h1 ? "OK" : err);
    osc001_transport_hidraw(&t2, dev);
    h2 = osc001_open(&t2, err, sizeof err);
    printf("2回目: %s\n", h2 ? "通ってしまった(想定外)" : err);
    if (h2) osc001_close(h2);
    if (h1) osc001_close(h1);

    /* 3) close 後の再オープン */
    osc001_transport_hidraw(&t1, dev);
    h1 = osc001_open(&t1, err, sizeof err);
    printf("close 後再オープン: %s\n", h1 ? "OK" : err);
    if (h1) osc001_close(h1);
    return 0;
}
