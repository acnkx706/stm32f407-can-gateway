#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
UDS 0x31 例程控制测试（擦除暂存区 / 校验暂存镜像）

★ 这一组和前几组最大的不同
    它会**真的擦掉 W25Q64 上的 27 个扇区**，耗时 1.2~10.8 秒。
    所以多了两个前几组没有的判据：
        ① 擦除期间必须多次收到 0x78（Tester 每收到一次只放宽 5 秒）
        ② 擦完之后"校验"必须报"没有暂存镜像"

★ 怎么数 0x78 的个数
    抓帧槽只能留住最后一帧，所以要靠 F407 的 CAPN 命令读"抓到过几帧"。
    这个计数器就是为这条判据加的。

★ 用法
    python routine_test.py
"""

import sys
import time

from isotp_test import ID_RSP
from sec_test import Uds, fmt, calc_key


def request_final(u, req, timeout=30.0):
    """发一条请求，一直读到"不是 0x78 的那条"

    返回 (最终响应, 期间抓到几帧, 耗时秒数)
    """
    u.ch.cmd(f"CAP {ID_RSP:03X}")        # 布置抓帧，同时清 hit 和计数
    u.send_only(req)

    t0 = time.time()
    final = None

    while (time.time() - t0) < timeout:
        r = u.ch.cmd("CAPQ")
        if r.startswith("RX "):
            data = bytes(int(x, 16) for x in r[6:].split())
            n = data[0] & 0x0F
            if n == 0 or n > 7:
                continue
            p = data[1:1 + n]
            if not (len(p) >= 3 and p[0] == 0x7F and p[2] == 0x78):
                final = p
                break
        time.sleep(0.02)

    try:
        cnt = int(u.ch.cmd("CAPN").split()[1])
    except Exception:                    # noqa: BLE001
        cnt = -1

    return final, cnt, time.time() - t0


def main() -> int:
    try:
        u = Uds()
    except (OSError, RuntimeError) as e:
        print(f"连不上或握手失败：{e}", file=sys.stderr)
        return 2

    fails = []

    def check(name, expect, got):
        ok = (got == expect)
        if not ok:
            fails.append(name)
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")
        print(f"       期望 {fmt(expect)}   实际 {fmt(got)}")

    try:
        # ---- ① 回到干净起点 ----
        u.request(bytes.fromhex("1001"))

        # ---- ② 没解锁就擦 Flash，必须被拒 ----
        r = u.request(bytes.fromhex("3101FF00"))
        check("未解锁擦除被拒（NRC 0x33）", bytes.fromhex("7F3133"), r)

        # ---- ③ 切编程会话并解锁 ----
        u.request(bytes.fromhex("1002"))
        r = u.request(bytes.fromhex("2701"))
        if not (r and len(r) == 6 and r[0] == 0x67):
            print(f"[FAIL] 求种子失败：{fmt(r)}", file=sys.stderr)
            return 1
        key = calc_key(r[2:6])
        r = u.request(bytes([0x27, 0x02]) + key)
        check("安全访问解锁", bytes.fromhex("6702"), r)

        # ---- ④ 擦除暂存区 ----
        final, cnt, dt = request_final(u, bytes.fromhex("3101FF00"), timeout=30.0)
        check("擦除暂存区最终响应", bytes.fromhex("7101FF0000"), final)
        print(f"       擦除耗时 {dt:.1f}s，期间抓到 {cnt} 帧")

        # 超过 5 秒就必须有多于一次 0x78：一次 0x78 只放宽到 P2* = 5 秒
        need = 1 if dt <= 5.0 else 2
        ok = (cnt >= need)
        if not ok:
            fails.append("0x78 次数不够")
        print(f"[{'PASS' if ok else 'FAIL'}] 0x78 次数 {cnt} >= {need}"
              f"（耗时 {dt:.1f}s，单次 0x78 只放宽 5 秒）")

        # ---- ⑤ 擦完再校验：应当报"没有暂存镜像" ----
        final, cnt2, dt2 = request_final(u, bytes.fromhex("31010202"), timeout=30.0)
        check("擦完后校验报无镜像", bytes.fromhex("7101020201"), final)
        print(f"       校验耗时 {dt2:.1f}s")

        # ---- ⑥ 参数校验 ----
        r = u.request(bytes.fromhex("31011234"))
        check("不存在的例程（NRC 0x31）", bytes.fromhex("7F3131"), r)

        r = u.request(bytes.fromhex("3102FF00"))
        check("不支持的子功能（NRC 0x12）", bytes.fromhex("7F3112"), r)

        r = u.request(bytes.fromhex("3101FF"))
        check("长度不足（NRC 0x13）", bytes.fromhex("7F3113"), r)

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
