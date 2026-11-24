#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
UDS 0x28 通信控制测试：验证"能不能真的让总线安静下来"

★ 这个用例的判据和前面不一样
    前面的用例看"响应对不对"，这个要看"总线上的行为变没变"。
    判据是 F407 的接收帧计数 rf：
        · 关掉之后 rf 必须停止增长
        · 打开之后 rf 必须恢复增长
    响应对了但那两件事没发生，等于没关。

★ 顺带验一张安全网
    Tester 关掉总线后如果不发 3E，5 秒后设备应当自己退回默认会话并
    恢复发送。这条不验的话，一次异常升级就能让设备"永远静默"。

★ 用法
    python comm_test.py
"""

import json
import sys
import time
import urllib.request

from sec_test import Uds, fmt

HTTP = "http://192.168.10.50/data"


def rf() -> int:
    """读 F407 的接收帧计数：它就是"总线有没有在跑"的唯一客观指标"""
    with urllib.request.urlopen(HTTP, timeout=4) as r:
        return json.loads(r.read().decode())["rf"]


def rf_delta(seconds=2.0) -> int:
    a = rf()
    time.sleep(seconds)
    b = rf()
    return b - a


def main() -> int:
    try:
        u = Uds()
    except (OSError, RuntimeError) as e:
        print(f"连不上或握手失败：{e}", file=sys.stderr)
        return 2

    fails = []

    def check(name, req, expect, got):
        ok = (got == expect)
        if not ok:
            fails.append(name)
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")
        print(f"       请求 {fmt(req)}  期望 {fmt(expect)}  实际 {fmt(got)}")

    try:
        # ---- ① 先确认总线是活的 ----
        # ★ 要给它时间：上一组测试（sec_test）结尾会复位设备，
        #   刚复位完 COP 还在启动，直接测会得到 rf 不动，那是假失败
        d0 = 0
        for _ in range(12):
            d0 = rf_delta(2.0)
            if d0 > 0:
                break
            time.sleep(0.5)
        print(f"[步骤] 默认状态下 2 秒内 rf 增长 {d0}")
        if d0 <= 0:
            print("★ 等了 24 秒总线仍没在跑，后面的判据没意义", file=sys.stderr)
            return 3

        # ---- ② 默认会话里不许关总线 ----
        u.request(bytes.fromhex("1001"))
        r = u.request(bytes.fromhex("280301"))
        check("默认会话关总线被拒（NRC 0x7F）", bytes.fromhex("280301"),
              bytes.fromhex("7F287F"), r)

        # ---- ③ 切扩展会话 ----
        r = u.request(bytes.fromhex("1003"))
        check("切扩展会话", bytes.fromhex("1003"),
              bytes.fromhex("50030032 1388".replace(" ", "")), r)

        # ---- ④ 参数校验 ----
        r = u.request(bytes.fromhex("280302"))
        check("不支持的通信类型（NRC 0x31）", bytes.fromhex("280302"),
              bytes.fromhex("7F2831"), r)

        r = u.request(bytes.fromhex("2803"))
        check("长度不足（NRC 0x13）", bytes.fromhex("2803"),
              bytes.fromhex("7F2813"), r)

        # ---- ⑤ 关总线，看 rf 停不停 ----
        r = u.request(bytes.fromhex("280301"))
        check("关总线", bytes.fromhex("280301"), bytes.fromhex("6803"), r)
        d1 = rf_delta(2.0)
        ok = (d1 == 0)
        if not ok:
            fails.append("关总线后 rf 仍增长")
        print(f"[{'PASS' if ok else 'FAIL'}] 关总线后 2 秒内 rf 增长 {d1}（应为 0）")

        # ---- ⑥ 开回总线，看 rf 恢复不恢复 ----
        r = u.request(bytes.fromhex("280001"))
        check("开回总线", bytes.fromhex("280001"), bytes.fromhex("6800"), r)
        d2 = rf_delta(2.0)
        ok = (d2 > 0)
        if not ok:
            fails.append("开回总线后 rf 没恢复")
        print(f"[{'PASS' if ok else 'FAIL'}] 开回总线后 2 秒内 rf 增长 {d2}（应大于 0）")

        # ---- ⑦ 安全网：关掉后不再发 3E，S3 超时应自动恢复 ----
        r = u.request(bytes.fromhex("280301"))
        check("再关一次总线", bytes.fromhex("280301"), bytes.fromhex("6803"), r)
        print("[步骤] 现在不发任何请求，等 7 秒让 S3 超时 ...")
        time.sleep(7.0)
        d3 = rf_delta(2.0)
        ok = (d3 > 0)
        if not ok:
            fails.append("S3 超时后总线没自动恢复")
        print(f"[{'PASS' if ok else 'FAIL'}] S3 超时后 2 秒内 rf 增长 {d3}"
              "（应大于 0，说明安全网生效）")

    finally:
        u.close()

    print()
    if fails:
        print("失败项：" + "、".join(fails))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
