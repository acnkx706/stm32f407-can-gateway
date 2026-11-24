#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
UDS 0x34/0x36/0x37 完整下载测试：把真的 app.bin 通过 CAN 灌进 F103 的暂存区

★ 这一组和前面所有组的区别
    前面都是几十个字节的请求-响应，这一组要传 **7 万多字节**，
    而且是逐块确认。所以它同时测了三件事：
        ① 34/36/37 的状态机对不对
        ② ISO-TP 在 1 万多帧的量级上稳不稳
        ③ 整包 CRC 能不能对上（PC 算的 vs F103 累加的）

★ 判据
    · 34 回 74 20 01 02（块长 258）
    · 每块 36 回 76 <序号>
    · 37 回 77（PC 给的 CRC 和 F103 累加的一致）
    · 31 01 0202 回 CRC 一致（再从 W25 读一遍算 CRC-B）

★ 用法
    python download_test.py
    python download_test.py --bin 别的.bin
"""

import argparse
import os
import sys
import time

from isotp_test import crc16_kermit
from routine_test import request_final
from sec_test import Uds, fmt, calc_key

APP_BIN = (r"C:\Users\acnkx\Desktop\stm32\项目1：工业记录仪\工业记录仪"
           r"\Objects\app.bin")
CHUNK = 256          # 和 maxNumberOfBlockLength - 2 一致


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=APP_BIN)
    ap.add_argument("--chunk", type=int, default=CHUNK)
    ap.add_argument("--install", action="store_true",
                    help="最后发 11 01 复位，让 Boot 把暂存镜像装进内部 Flash")
    args = ap.parse_args()

    if not os.path.exists(args.bin):
        print(f"找不到镜像：{args.bin}", file=sys.stderr)
        return 2

    with open(args.bin, "rb") as f:
        data = f.read()

    size = len(data)
    nblk = (size + args.chunk - 1) // args.chunk
    crc = crc16_kermit(data)

    print(f"镜像   {args.bin}")
    print(f"大小   {size} 字节，{nblk} 块 × {args.chunk} 字节")
    print(f"整包 CRC16/KERMIT = 0x{crc:04X}")
    print()

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
        if not ok:
            print(f"       期望 {fmt(expect)}   实际 {fmt(got)}")
        return ok

    try:
        # ---- ① 会话 + 解锁 ----
        u.request(bytes.fromhex("1001"))
        u.request(bytes.fromhex("1002"))
        r = u.request(bytes.fromhex("2701"))
        if not (r and len(r) == 6 and r[0] == 0x67):
            print(f"[FAIL] 求种子：{fmt(r)}", file=sys.stderr)
            return 1
        u.request(bytes([0x27, 0x02]) + calc_key(r[2:6]))
        print("[步骤] 已进编程会话并解锁")

        # ---- ② 擦除暂存区 ----
        final, _cnt, dt = request_final(u, bytes.fromhex("3101FF00"), timeout=40.0)
        if not check("擦除暂存区", bytes.fromhex("7101FF0000"), final):
            return 1
        print(f"       擦除耗时 {dt:.1f}s")

        # ---- ③ 34 请求下载 ----
        req = (bytes([0x34, 0x00, 0x44]) + (0).to_bytes(4, "big")
               + size.to_bytes(4, "big"))
        r = u.request(req)
        if not check("请求下载（块长 258）", bytes.fromhex("74200102"), r):
            return 1

        # ---- ④ 36 逐块传 ----
        t0 = time.time()
        for i in range(nblk):
            blk = data[i * args.chunk:(i + 1) * args.chunk]
            seq = (i + 1) & 0xFF
            # ★ 单块重试：一次往返卡住不该让整包重来
            #   真实 Tester 也这么干（ISO 14229 允许重发同一个 blockSeq）
            r = None
            for _try in range(3):
                r = u.request(bytes([0x36, seq]) + blk)
                if r == bytes([0x76, seq]):
                    break
                print(f"       第 {i + 1} 块第 {_try + 1} 次失败：{fmt(r)}，重试")
            if r != bytes([0x76, seq]):
                fails.append(f"第 {i + 1} 块")
                print(f"[FAIL] 第 {i + 1} 块（序号 {seq:02X}）：{fmt(r)}")
                break
            if ((i + 1) % 50) == 0 or (i + 1) == nblk:
                el = time.time() - t0
                rate = (i + 1) * args.chunk / el
                print(f"       {i + 1}/{nblk} 块  {el:.1f}s  {rate / 1024:.1f} KB/s")
        else:
            print(f"[PASS] {nblk} 块全部确认")

        el = time.time() - t0
        done = (i + 1) * args.chunk if nblk > 0 else 0
        print(f"       传输耗时 {el:.1f}s，{done / el / 1024:.1f} KB/s（按已发字节算）")

        # ---- ⑤ 37 退出（带 PC 算的 CRC）----
        r = u.request(bytes([0x37, (crc >> 8) & 0xFF, crc & 0xFF]))
        check("退出传输（CRC 一致）", bytes.fromhex("77"), r)

        # ---- ⑥ 从 W25 读回来再算一遍（CRC-B）----
        final, _c, dt2 = request_final(u, bytes.fromhex("31010202"), timeout=40.0)
        check("暂存镜像校验通过", bytes.fromhex("7101020200"), final)
        print(f"       校验耗时 {dt2:.1f}s")

        # ---- ⑦ 故意用错 CRC 再退一次，应当被拒 ----
        # 37 已经把状态清了，这里只验"没下载就 37"这条路
        r = u.request(bytes([0x37, 0x12, 0x34]))
        check("没有下载就退出被拒（NRC 0x24）", bytes.fromhex("7F3724"), r)

        # ---- ⑧ 复位，交给 Boot 装（可选）----
        if args.install:
            print()
            print("[步骤] 发 11 01 复位，Boot 应当发现'升级标志 + 就绪暂存'并开始安装")
            r = u.request(bytes.fromhex("1101"), timeout=3.0)
            check("复位请求", bytes.fromhex("5101"), r)

    finally:
        u.close()

    print()
    if fails:
        print("失败项：" + "、".join(fails))
        return 1
    print("全部通过")
    # ★ 2026-11-24 加：把"暂存区还留着东西"这件事说清楚
    #   · 本脚本只传到"校验通过"就收工，暂存区里是一份完整镜像 + READY 头
    #   · 下次请求升级（发 U）时，Boot 会先把这份镜像装进去再重启，
    #     于是紧接着的那次 flash_app.py 会收不到 E 的回应（退出码 6）
    #   · 这不是故障：重跑一次就好。两次实测都是这样（2026-11-24）
    print("")
    print("注意：暂存区还留着一份完整镜像（READY 头）。")
    print("      紧接着做串口升级时，Boot 会先把它装完再理你的 E，")
    print("      所以那一次会失败（退出码 6）—— 重跑一次即可，不是故障。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
