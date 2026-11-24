#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
72 小时长稳观察器（2026-11-24 建）

★ 它盯什么
    · /data 的 rf（F407 收到的 CAN 帧数）—— 不涨了说明 CAN 或采集链断了
    · Q 的 drop / bad / to / cfd —— 这四个任何一个涨了都是真问题
    · HTTP 响应耗时 —— 主循环被拖住时会变长（P0-1 那个 334 ms 上限的哨兵）

★ 为什么不用串口
    F103 每 100 ms 打一行 REC（约 220 字节），一天 190 MB。
    而且串口一开就独占 COM11，期间想烧写或敲 W 都不行。
    长稳要的是"三天不碰它"，所以走网络。

★ 为什么只看增量
    今天压测把 drop / cfd 的累计值顶上去过（drop=84、cfd=328），
    绝对值没有意义，只有"这三天里有没有新增"才算数。

★ 判据（硬失败 = 直接退出并记 [FAIL]）
    · 连续 3 次 HTTP 打不通
    · rf 连续 3 次不增长（= 3 分钟没有任何 CAN 帧进来）
    · bad / drop 的增量 > 0
  软告警（记 [WARN] 但继续跑）
    · to 增量 > 0（历史上出现过一夜涨一次）
    · cfd 增量 > 0（只该在 OVFLW 用例里出现，长稳里不该有）
    · HTTP 耗时 > 500 ms

★ 用法
    python soak72.py                      # 默认 72 小时，60 秒一采
    python soak72.py --hours 24 --interval 30
    python soak72.py --hours 0.1          # 冒烟用
"""

import argparse
import json
import os
import socket
import sys
import time
import urllib.request

HOST = "192.168.10.50"
HTTP_URL = "http://%s/data" % HOST
TCP_ADDR = (HOST, 5000)

HARD_HTTP_FAIL = 3          # 连续几次 HTTP 失败算死
HARD_RF_STALL = 3           # 连续几次 rf 不动算死
WARN_HTTP_MS = 500.0


def http_data(timeout=6.0):
    t0 = time.time()
    with urllib.request.urlopen(HTTP_URL, timeout=timeout) as r:
        body = r.read().decode("utf-8", "replace")
    return json.loads(body), (time.time() - t0) * 1000.0


def q_line(timeout=6.0):
    s = socket.create_connection(TCP_ADDR, timeout=timeout)
    s.settimeout(timeout)
    try:
        s.sendall(b"Q\r\n")
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(256)
            if not chunk:
                break
            buf += chunk
        return buf.decode("ascii", "replace").strip()
    finally:
        s.close()


def parse_q(line):
    out = {}
    for kv in line.split()[1:]:
        if "=" not in kv:
            continue
        k, v = kv.split("=", 1)
        try:
            out[k] = int(v)
        except ValueError:
            pass
    return out


def stamp():
    return time.strftime("%Y-%m-%dT%H:%M:%S")


class Watch:
    def __init__(self):
        self.base = None          # 上一次采样
        self.first = None         # 起始基线
        self.http_fail = 0
        self.rf_stall = 0
        self.n = 0
        self.t0 = time.time()

    def snap(self):
        d, ms = http_data()
        q = parse_q(q_line())
        return {
            "rf": q.get("rf", -1),
            "ack": q.get("ack", -1),
            "ok": q.get("ok", -1),
            "drop": q.get("drop", -1),
            "bad": q.get("bad", -1),
            "to": q.get("to", -1),
            "cfd": q.get("cfd", -1),
            "ff": q.get("ff", -1),
            "cf": q.get("cf", -1),
            "wms": q.get("wms", -1),
            "wmax": q.get("wmax", -1),
            "mq": d.get("mq", -1),
            "er": d.get("er", -1),
            "t": d.get("t", None),
            "p": d.get("p", None),
            "vb": d.get("vb", None),
            "http_ms": ms,
        }

    @staticmethod
    def delta(cur, old, key):
        a, b = cur.get(key), old.get(key)
        if isinstance(a, int) and isinstance(b, int) and a >= 0 and b >= 0:
            return a - b
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description="72 小时长稳观察器")
    ap.add_argument("--hours", type=float, default=72.0)
    ap.add_argument("--interval", type=float, default=60.0, help="采样间隔（秒）")
    ap.add_argument("--hour-sec", type=float, default=3600.0,
                    help="多久算一\"小时\"（只是冒烟时调小，正式跑别动）")
    ap.add_argument("--log", default=None)
    ap.add_argument("--pid", default=None)
    args = ap.parse_args()

    log_path = args.log or os.path.join(
        r"C:\Users\acnkx\Desktop\stm32\杂项文件\证据日志",
        "soak72_%s.log" % time.strftime("%Y%m%d_%H%M"))
    pid_path = args.pid or (os.path.splitext(log_path)[0] + ".pid")

    logf = open(log_path, "a", encoding="utf-8", buffering=1)
    with open(pid_path, "w", encoding="utf-8") as f:
        f.write(str(os.getpid()))

    def emit(tag, text):
        line = "%s %s %s" % (stamp(), tag, text)
        print(line, flush=True)
        logf.write(line + "\n")

    emit("[START]", "hours=%.2f interval=%.0fs log=%s pid=%d"
         % (args.hours, args.interval, os.path.basename(log_path), os.getpid()))

    w = Watch()
    end_at = time.time() + args.hours * 3600.0
    hour_idx = 0
    hard = None

    while time.time() < end_at:
        loop_t0 = time.time()
        try:
            cur = w.snap()
            w.http_fail = 0
        except Exception as e:                                    # noqa: BLE001
            w.http_fail += 1
            emit("[WARN]", "采样失败 %d/%d：%s" % (w.http_fail, HARD_HTTP_FAIL, e))
            if w.http_fail >= HARD_HTTP_FAIL:
                hard = "连续 %d 次采样失败（HTTP 或 Q 打不通）" % w.http_fail
                emit("[FAIL]", hard)
                break
            time.sleep(args.interval)
            continue

        w.n += 1

        if w.first is None:
            w.first = cur
            emit("[BASE]", "rf=%d drop=%d bad=%d to=%d cfd=%d mq=%d er=%d"
                 % (cur["rf"], cur["drop"], cur["bad"], cur["to"],
                    cur["cfd"], cur["mq"], cur["er"]))
            w.base = cur
            hour_idx = 0
            next_hour = time.time() + args.hour_sec
        else:
            d_rf = Watch.delta(cur, w.base, "rf")
            if d_rf == 0:
                w.rf_stall += 1
                if w.rf_stall >= HARD_RF_STALL:
                    hard = "rf 连续 %d 次不增长（%.0f 秒没有 CAN 帧）" % (
                        w.rf_stall, w.rf_stall * args.interval)
                    emit("[FAIL]", hard)
                    break
            else:
                w.rf_stall = 0

            d_bad = Watch.delta(cur, w.base, "bad")
            d_drop = Watch.delta(cur, w.base, "drop")
            d_to = Watch.delta(cur, w.base, "to")
            d_cfd = Watch.delta(cur, w.base, "cfd")

            if (d_bad or 0) > 0:
                hard = "bad 涨了 %d（F407 收到无法解析的命令）" % d_bad
                emit("[FAIL]", hard)
                break
            if (d_drop or 0) > 0:
                hard = "drop 涨了 %d（转发被丢弃：邮箱满或对端没收）" % d_drop
                emit("[FAIL]", hard)
                break
            if (d_to or 0) > 0:
                emit("[WARN]", "to 涨了 %d（流控等待超时）" % d_to)
            if (d_cfd or 0) > 0:
                emit("[WARN]", "cfd 涨了 %d（连续帧被丢，长稳里不该出现）" % d_cfd)
            if cur["http_ms"] > WARN_HTTP_MS:
                emit("[WARN]", "HTTP 耗时 %.0f ms（> %.0f）"
                     % (cur["http_ms"], WARN_HTTP_MS))

            if time.time() >= next_hour:
                hour_idx += 1
                secs = time.time() - w.t0
                d_ack = Watch.delta(cur, w.base, "ack") or 0
                d_mq = Watch.delta(cur, w.base, "mq") or 0
                emit("[HOUR %02d]" % hour_idx,
                     "rf=%d (+%d, %.2f/s) ack=+%d mq=+%d | drop=+%d bad=+%d to=+%d cfd=+%d"
                     " | wms=%d wmax=%d http=%.0fms  t=%s p=%s vb=%s  elapsed=%.1fh"
                     % (cur["rf"], d_rf, (d_rf or 0) / secs,
                        d_ack, d_mq,
                        d_drop or 0, d_bad or 0, d_to or 0, d_cfd or 0,
                        cur["wms"], cur["wmax"], cur["http_ms"],
                        cur["t"], cur["p"], cur["vb"],
                        secs / 3600.0))
                # 每小时把基线推到当前值，这样 [HOUR] 那一行就是"这一小时的增量"
                w.base = cur
                next_hour += args.hour_sec

        # 补睡眠，保证采样间隔不被 HTTP 往返拉长
        spent = time.time() - loop_t0
        time.sleep(max(1.0, args.interval - spent))

    if hard is None:
        total_h = (time.time() - w.t0) / 3600.0
        d_all = {k: Watch.delta(cur, w.first, k) for k in ("rf", "drop", "bad", "to", "cfd", "mq")}
        emit("[DONE]", "跑满 %.2f 小时，采样 %d 次，全程增量 %s"
             % (total_h, w.n, json.dumps(d_all)))
    logf.close()
    try:
        os.remove(pid_path)
    except OSError:
        pass
    return 0 if hard is None else 2


if __name__ == "__main__":
    sys.exit(main())
