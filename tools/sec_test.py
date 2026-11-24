#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
UDS 安全访问（0x27）全流程测试：PC → F407 → CAN → F103

★ 为什么这个测试比前面几个复杂
    0x27 是"挑战-响应"：设备回一个种子，PC 算密钥再送回去。
    所以 PC 必须能**读到**设备的响应。F407 的命令通道原本只发不收，
    这次加了 CAP/CAPQ 两个命令（布置抓帧 / 读回抓帧槽）才有回程。

★ 用法
    python sec_test.py
"""

import os
import sys
import time

from isotp_test import Chan, ID_REQ, ID_RSP, iso_tp_frames

# ISO-TP 的 CAN ID 要和 F103 的 iso_tp.h 一致
# ID_REQ / ID_RSP 已从 isotp_test 导入


def calc_key(seed: bytes) -> bytes:
    """和 uds.c 的 Uds_CalcKey 逐字一致。"""
    x = seed[0] | (seed[1] << 8)
    y = seed[2] | (seed[3] << 8)
    for _ in range(2):
        x ^= 0x5A3C
        y ^= 0xA5C3
        a = ((x << 5) | (x >> 11)) & 0xFFFF
        b = ((y << 5) | (y >> 11)) & 0xFFFF
        x = (a + y) & 0xFFFF
        y = (b ^ x) & 0xFFFF
    return bytes([x & 0xFF, (x >> 8) & 0xFF, y & 0xFF, (y >> 8) & 0xFF])


class Uds:
    def __init__(self, host="192.168.10.50", port=5000):
        self.ch = Chan(host, port)
        if self.ch.cmd("PING") != "PONG":
            raise RuntimeError("F407 命令通道不响应 PING")
        self._wait_alive()

    def _wait_alive(self, timeout=40.0):
        """等 F103 的业务跑起来

        ★ 为什么 PING 通了还要等
          PING 只证明 F407 活着。而 UDS 的处理挂在 F103 的 PROC 任务里，
          刚烧完固件时 F103 还在做 SD/传感器初始化，PROC 根本没跑，
          这时候发 27 会"无响应"——看起来像协议坏了，其实设备还没起床。

        ★ 判据用 rf（F407 收到的帧数）
          它在涨就说明 F103 已经在周期发 0x101/0x102，PROC 一定在跑。
        """
        t0 = time.time()
        last_rf = None
        first_ack = None
        last_q = ""
        while (time.time() - t0) < timeout:
            try:
                q = self.ch.cmd("Q")
                last_q = q
                rf = int(q.split("rf=")[1].split()[0])
                ack = int(q.split("ack=")[1].split()[0])
                if first_ack is None:
                    first_ack = ack
                if (last_rf is not None) and (rf > last_rf):
                    return
                last_rf = rf
            except Exception:                  # noqa: BLE001
                pass
            time.sleep(0.5)

        # ★ 失败时要把"为什么"直接说出来，别只报一句"没数据"
        #   ack 是硬件应答计数：它在涨 = 有人在发帧但没人应答 = 线断/没电/接反
        dack = (ack - first_ack) if (first_ack is not None) else 0
        raise RuntimeError(
            f"等了 {timeout:.0f} 秒 F103 还没开始发数据\n"
            f"       最后一行 Q: {last_q}\n"
            f"       ack 增长 {dack} -> "
            + ("总线没人应答：查 CANH/CANL/GND 接线和两块板供电"
               if dack > 1000 else "总线是安静的：查 F103 是否卡在 Boot"))

    def close(self):
        self.ch.close()

    def send_only(self, req: bytes):
        """只发，不等

        ★ 2026-11-24 改：不再每 3 帧发一个 Q
          · 原来需要 Q：F407 无脑转发，一次推几十帧会把 F103 的 3 级接收 FIFO
            冲爆，只能靠 Q 的往返强行降速
          · 现在 F407 自己会等对端流控帧，节流由协议保证，Q 变成了多余的往返
          · 37 帧约 814 字节，W5500 每个 socket 的接收缓冲 2 KB，放得下不会溢出
          · 收益：每块省掉 12 到 13 次 TCP 往返，实测约 30 ms
        """
        for f in iso_tp_frames(req):
            self.ch.send_line(f"CAN {ID_REQ:03X} {f.hex()}")

    def request(self, req: bytes, timeout=1.5):
        """发一条 UDS 请求，读回一条响应（单帧）

        返回 bytes；超时返回 None
        """
        self.ch.cmd(f"CAP {ID_RSP:03X}")     # 布置抓帧，同时清掉上一次的
        self.send_only(req)

        t0 = time.time()
        while (time.time() - t0) < timeout:
            r = self.ch.cmd("CAPQ")
            if r.startswith("RX "):
                # 形如 "RX 3E0 02 7E 00 00 00 00 00 00"
                body = r[6:]
                data = bytes(int(x, 16) for x in body.split())
                n = data[0] & 0x0F          # ISO-TP 单帧：首字节低 4 位是长度
                if n == 0 or n > 7:
                    continue
                return data[1:1 + n]
            time.sleep(0.02)

        # ★ 2026-11-24 加：超时时把现场打出来（UDS_DIAG=1 才生效）
        #   · 抓帧计数能区分两种完全不同的情况：
        #     计数=0 说明 F407 一帧 0x3E0 都没收到，是链路/设备没回
        #     计数>0 而内容不是 76 xx，说明抓到了流控帧却把响应弄丢了
        #   · 平时不打，免得污染正常测试的输出
        if os.environ.get("UDS_DIAG"):
            print(f"       [DIAG] 超时{timeout}s 抓帧={self.ch.cmd('CAPN')}"
                  f" 槽={self.ch.cmd('CAPQ')}", file=sys.stderr)
        return None


def fmt(b):
    return "无响应" if b is None else " ".join(f"{x:02X}" for x in b)


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
        print(f"       请求 {fmt(req)}")
        print(f"       期望 {fmt(expect)}")
        print(f"       实际 {fmt(got)}")

    try:
        # ---- ⓪ 先退回默认会话，把上一轮残留的种子/解锁状态清掉 ----
        # 固件里切会话会清安全状态（ISO 14229 的要求），所以这一步就是"回到干净起点"
        r = u.request(bytes.fromhex("1001"))
        print(f"[步骤] 退回默认会话 -> {fmt(r)}")

        # ---- ① 切编程会话 ----
        r = u.request(bytes.fromhex("1002"))
        check("切编程会话", bytes.fromhex("1002"),
              bytes.fromhex("50020032 1388".replace(" ", "")), r)

        # ---- ② 未解锁就复位，应当被安全访问挡住 ----
        r = u.request(bytes.fromhex("1101"))
        check("未解锁复位被拒（NRC 0x33）", bytes.fromhex("1101"),
              bytes.fromhex("7F1133"), r)

        # ---- ③ 没求种子就送密钥，应当报顺序错 ----
        r = u.request(bytes.fromhex("2702 00000000".replace(" ", "")))
        check("未求种子先送密钥（NRC 0x24）", bytes.fromhex("270200000000"),
              bytes.fromhex("7F2724"), r)

        # ---- ④ 正流程：求种子 ----
        r = u.request(bytes.fromhex("2701"))
        seed = None
        # 响应是 6 字节：67 01 + 4 字节种子（不是 5，我一次写错过）
        if r is not None and len(r) == 6 and r[0] == 0x67 and r[1] == 0x01:
            seed = r[2:6]
            print(f"[PASS] 求种子")
            print(f"       种子 {seed.hex().upper()}")
        else:
            print(f"[FAIL] 求种子  实际 {fmt(r)}")
            fails.append("求种子")

        if seed is not None:
            # ---- ⑤ 送正确密钥 ----
            key = calc_key(seed)
            req = bytes([0x27, 0x02]) + key
            r = u.request(req)
            check("送正确密钥", req, bytes.fromhex("6702"), r)

            # ---- ⑥ 已解锁，再求种子应回全 0 ----
            r = u.request(bytes.fromhex("2701"))
            check("已解锁再求种子回全 0", bytes.fromhex("2701"),
                  bytes.fromhex("670100000000"), r)

            # ---- ⑦ 解锁后复位应当通过（会重启设备）----
            r = u.request(bytes.fromhex("1101"), timeout=2.0)
            check("解锁后复位", bytes.fromhex("1101"), bytes.fromhex("5101"), r)

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
