/* Linux hidraw トランスポート(/dev/hidrawN 直叩き)。
 * python/osc001.py の open() と同じ方針:
 *   - 排他: flock(LOCK_EX|LOCK_NB)。hidraw は多重オープンできるが
 *     コマンドが取り違えるため単一セッションを強制。
 *   - open 失敗の区別: 権限不足(udev)/ ノード無し(未接続・ハング後)/ 占有。
 *   - 書込みは 0x00 プレフィックス付き 65 バイト(レポート ID なし)。
 * hidapi 非依存。必要依存は glibc のみ(Raspberry Pi 等で追加インストール不要)。 */

#define _DEFAULT_SOURCE
#include "osc001.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <time.h>
#include <unistd.h>

#define HR_PATH_MAX 128

typedef struct {
    char path[HR_PATH_MAX];
    int fd;
    const char *why;   /* open 失敗理由(静的文字列) */
} hidraw_ctx;

static int hr_open(void *u)
{
    hidraw_ctx *c = (hidraw_ctx *)u;
    c->fd = open(c->path, O_RDWR | O_CLOEXEC);
    if (c->fd < 0) {
        if (errno == EACCES || errno == EPERM)
            c->why = "権限不足(udev ルール適用か確認。README の udev 例/03 の Linux 運用メモ)";
        else if (errno == ENOENT)
            c->why = "ノード無し(04d8:f891 が lsusb にあるか確認。"
                     "ハング後は物理抜き差しが必要)";
        else
            c->why = strerror(errno);
        return -1;
    }
    if (flock(c->fd, LOCK_EX | LOCK_NB) != 0) {
        close(c->fd);
        c->fd = -1;
        c->why = "他プロセスが占有中(flock 排他)。hidraw は多重オープンできるが"
                 "コマンドが取り違える";
        return -1;
    }
    return 0;
}

static void hr_close(void *u)
{
    hidraw_ctx *c = (hidraw_ctx *)u;
    if (c->fd >= 0) {
        close(c->fd);
        c->fd = -1;
    }
}

static int hr_send(void *u, const uint8_t *report)
{
    hidraw_ctx *c = (hidraw_ctx *)u;
    uint8_t buf[OSC001_REPORT_LEN + 1];
    ssize_t n;
    buf[0] = 0x00;   /* レポート ID なし → 0x00 プレフィックス必須 */
    memcpy(buf + 1, report, OSC001_REPORT_LEN);
    for (;;) {
        n = write(c->fd, buf, sizeof buf);
        if (n == (ssize_t)sizeof buf)
            return 0;
        if (n < 0 && errno == EINTR)
            continue;
        return -1;
    }
}

static int hr_recv(void *u, uint8_t *report, int timeout_ms)
{
    hidraw_ctx *c = (hidraw_ctx *)u;
    for (;;) {
        struct pollfd pfd;
        int r;
        pfd.fd = c->fd;
        pfd.events = POLLIN;
        r = poll(&pfd, 1, timeout_ms);
        if (r == 0)
            return 0;   /* タイムアウト */
        if (r < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        ssize_t n = read(c->fd, report, OSC001_REPORT_LEN);
        if (n == OSC001_REPORT_LEN)
            return 1;
        if (n < 0 && errno == EINTR)
            continue;
        return -1;
    }
}

static uint64_t hr_now_ms(void *u)
{
    (void)u;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

static void hr_sleep_ms(int ms)
{
    struct timespec req;
    req.tv_sec = ms / 1000;
    req.tv_nsec = (long)(ms % 1000) * 1000000L;
    while (nanosleep(&req, &req) == -1 && errno == EINTR)
        ;
}

static const char *hr_open_errmsg(void *u)
{
    return ((hidraw_ctx *)u)->why;
}

int osc001_transport_hidraw(osc001_transport_t *t, const char *hidraw_path)
{
    hidraw_ctx *c;
    if (!t || !hidraw_path || strlen(hidraw_path) >= HR_PATH_MAX)
        return -1;
    c = (hidraw_ctx *)calloc(1, sizeof *c);
    if (!c)
        return -1;
    strncpy(c->path, hidraw_path, HR_PATH_MAX - 1);
    c->fd = -1;
    t->open = hr_open;
    t->close = hr_close;
    t->send = hr_send;
    t->recv = hr_recv;
    t->now_ms = hr_now_ms;
    t->sleep_ms = hr_sleep_ms;
    t->open_errmsg = hr_open_errmsg;
    t->user = c;
    return 0;
}

void osc001_transport_hidraw_release(osc001_transport_t *t)
{
    if (!t || !t->user)
        return;
    free(t->user);
    memset(t, 0, sizeof *t);
}

/* ── 列挙: /sys/class/hidraw/<name>/device/uevent を scan ────────── */

int osc001_enumerate_hidraw(char paths[][64], int max, char names[][64])
{
    static const char SYSDIR[] = "/sys/class/hidraw";
    DIR *d = opendir(SYSDIR);
    int count = 0;
    struct dirent *ent;
    if (!d)
        return 0;
    while ((ent = readdir(d)) != NULL) {
        char uev[256], buf[4096];
        FILE *f;
        char *p;
        unsigned bus, ven, prod;
        if (strncmp(ent->d_name, "hidraw", 6) != 0)
            continue;
        snprintf(uev, sizeof uev, "%s/%.128s/device/uevent", SYSDIR, ent->d_name);
        f = fopen(uev, "r");
        if (!f)
            continue;
        size_t rd = fread(buf, 1, sizeof buf - 1, f);
        fclose(f);
        if (rd == 0)
            continue;
        buf[rd] = '\0';
        p = strstr(buf, "HID_ID=");
        if (!p || sscanf(p, "HID_ID=%x:%x:%x", &bus, &ven, &prod) != 3)
            continue;
        if (ven != OSC001_VENDOR || prod != OSC001_PRODUCT)
            continue;
        if (count < max) {
            snprintf(paths[count], 64, "/dev/%.57s", ent->d_name);
            if (names) {
                names[count][0] = '\0';
                p = strstr(buf, "HID_NAME=");
                if (p) {
                    p += 9;
                    size_t k = 0;
                    while (p[k] && p[k] != '\n' && k < 62) {
                        names[count][k] = p[k];
                        k++;
                    }
                    names[count][k] = '\0';
                }
            }
        }
        count++;
    }
    closedir(d);
    return count;
}
