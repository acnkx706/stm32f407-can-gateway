#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
UDS 会话与保活测试（0x10 / 0x3E / 0x11 / S3 定时器）

★ 为什么单独一个文件
    这一组是最"无害"的一组：全在内存里改状态，不碰 Flash。
    单拎出来就能在烧录后第一秒跑，作为"设备是不是活的"的冒烟测试。

★ 判据说明
    · 正向：响应的字节要完全等于期望
    · 抑制位：子功能最高位置 1 时**必须没有响应**，收到任何东西都算错
    · S3：切会话后不理它，6 秒后复位应当被拒（说明已退回默认会话）

★ 用法
    python session_test.py
"""

import sys
import time

from sec_test import Uds, fmt


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
        # ---- 会话切换 ----
        r = u.request(bytes.fromhex("1001"))
        check("回到默认会话", bytes.fromhex("1001"),
              bytes.fromhex("500100321388"), r)

        r = u.request(bytes.fromhex("1002"))
        check("切编程会话", bytes.fromhex("1002"),
              bytes.fromhex("500200321388"), r)

        r = u.request(bytes.fromhex("1003"))
        check("切扩展会话", bytes.fromhex("1003"),
              bytes.fromhex("500300321388"), r)

        # ---- 保活 ----
        r = u.request(bytes.fromhex("3E00"))
        check("保活回显子功能", bytes.fromhex("3E00"), bytes.fromhex("7E00"), r)

        r = u.request(bytes.fromhex("3E80"), timeout=0.8)
        check("抑制位生效（应无响应）", bytes.fromhex("3E80"), None, r)

        # ---- 参数校验 ----
        r = u.request(bytes.fromhex("1009"))
        check("不存在的子功能（NRC 0x12）", bytes.fromhex("1009"),
              bytes.fromhex("7F1012"), r)

        r = u.request(bytes.fromhex("10"))
        check("长度不足（NRC 0x13）", bytes.fromhex("10"),
              bytes.fromhex("7F1013"), r)

        # ★ 2026-11-24 改：原来用 22 当"未实现的服务"，现在 0x22 真实现了
        #   （DID F1A0 读 ISO-TP 统计），所以换 0x2E 来测 NRC 0x11
        r = u.request(bytes.fromhex("2EF190"))
        check("未实现的服务（NRC 0x11）", bytes.fromhex("2EF190"),
              bytes.fromhex("7F2E11"), r)

        # 0x22 已实现，那就顺手验一条"认识服务、不认 DID"
        r = u.request(bytes.fromhex("22F190"))
        check("0x22 不认识的 DID（NRC 0x31）", bytes.fromhex("22F190"),
              bytes.fromhex("7F2231"), r)

        # ---- 复位受会话限制 ----
        r = u.request(bytes.fromhex("1001"))
        time.sleep(0.1)
        r = u.request(bytes.fromhex("1101"))
        check("默认会话复位被拒（NRC 0x7F）", bytes.fromhex("1101"),
              bytes.fromhex("7F117F"), r)

        # ---- S3 定时器：切会话后不理它，应当自动退回默认 ----
        r = u.request(bytes.fromhex("1003"))
        print("[步骤] 切到扩展会话，然后不发任何请求，等 6 秒（S3 = 5 秒）...")
        time.sleep(6.0)
        r = u.request(bytes.fromhex("1101"))
        check("S3 超时后已退回默认会话", bytes.fromhex("1101"),
              bytes.fromhex("7F117F"), r)

    finally:
        u.close()

    print()
    if fails:
        print(f"失败 {len(fails)} 项：" + "、".join(fails))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
