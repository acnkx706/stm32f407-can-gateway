#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
ISO-TP 一致性测试：电脑 → F407 网关 → CAN → F103

★ 为什么要有它
    电脑上没有 CAN 硬件（只有 CH340 和 ST-Link），所以只能借 F407 的命令通道
    把帧打到总线上。F407 那边收的是文本命令：CAN <3位十六进制ID> <十六进制数据>。

★ 它怎么判对错
    ① 先 PING / Q 确认通道通
    ② 把一条 len 字节的报文按 ISO 15765-2 拆成 首帧 + 连续帧
    ③ 逐帧下发给 F407，每发 4 帧插一个 Q 做同步（Q 有回应就说明前面 4 帧都处理过了）
    ④ 最后打印本脚本自己算的 CRC16/KERMIT 和头 16 字节
    设备把"上一条收完的包"的长度和 CRC 存着，用 UDS 22 F1 A0 读回来
    （62 F1 A0 <len> <len> <crc> <crc>），和本脚本算的 CRC 一致 = 传输层通过。
    这条路不占串口，所以 dump 可以关（2026-11-24 起默认关）。

★ 用法
    python isotp_test.py                  # 256 字节，默认地址
    python isotp_test.py --len 8          # 单帧路径（首帧/连续帧都不走）
    python isotp_test.py --len 300        # 超过 F103 缓冲 288，应当被 OVFLW 拒掉
    python isotp_test.py --crc-only       # 只算 CRC，不连网
"""

import argparse
import socket
import sys
import time

HOST = "192.168.10.50"
PORT = 5000

# ISO-TP 的 CAN ID，必须和 iso_tp.h 里的一致
ID_REQ = 0x2E0
ID_RSP = 0x3E0

# ISO-TP 协议控制信息
PCI_SF = 0x00
PCI_FF = 0x10
PCI_CF = 0x20


def crc16_kermit(data: bytes) -> int:
    """和 Hardware\\Crc16.c 的 Crc16_Kermit 逐字一致：
    初值 0、多项式 0x8408（反射）、无输出异或。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def make_pattern(n: int) -> bytes:
    """固定的可复现图案：每字节 = (i*7+3) & 0xFF
    用乘加而不是随机数，是为了两边都能独立算出来，出问题能对着差在哪一位。"""
    return bytes(((i * 7 + 3) & 0xFF) for i in range(n))


def iso_tp_frames(payload: bytes):
    """把一条报文拆成 ISO-TP 帧（每帧都是 8 字节，本工程约定不裁短帧）"""
    n = len(payload)
    frames = []

    if n <= 7:
        f = bytearray(8)
        f[0] = PCI_SF | n
        f[1:1 + n] = payload
        frames.append(bytes(f))
        return frames

    # 首帧：12 位长度，自带 6 个数据字节
    f = bytearray(8)
    f[0] = PCI_FF | ((n >> 8) & 0x0F)
    f[1] = n & 0xFF
    f[2:8] = payload[0:6]
    frames.append(bytes(f))

    off = 6
    sn = 1
    while off < n:
        f = bytearray(8)
        f[0] = PCI_CF | (sn & 0x0F)
        chunk = payload[off:off + 7]
        f[1:1 + len(chunk)] = chunk
        frames.append(bytes(f))
        off += len(chunk)
        sn = (sn + 1) & 0x0F

    return frames


class Chan:
    """F407 命令通道的最小封装：一行一条命令，回包也是一行"""

    # ★ 2026-11-24 加：和固件 net_tcp.c 的 NET_TCP_TOKEN 必须一致
    TOKEN = "can-gw-2026"

    def __init__(self, host: str, port: int, timeout: float = 3.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = b""
        self._auth()

    def _auth(self):
        """建链后先认证

        ★ 为什么放在 __init__ 里
            固件对 CAN 命令加了 s_authed 检查，读命令不设门槛。
            放在构造里，所有用 Chan 的脚本（isotp/sec/download/run_all）自动带上，
            不用每个脚本各写一遍，也不会有人忘了写。

        ★ 为什么失败要抛异常
            认证失败时后面每条 CAN 命令都会收到 AUTH REQUIRED，
            会在几十行之后才以"超时"的形式暴露出来，极难定位。这里直接报。
        """
        r = self.cmd(f"AUTH {self.TOKEN}")
        if r != "AUTH OK":
            raise RuntimeError(f"命令通道认证失败：{r}")

    def send_line(self, text: str):
        self.sock.sendall((text + "\n").encode("ascii"))

    def read_line(self) -> str:
        while b"\n" not in self.buf:
            chunk = self.sock.recv(512)
            if not chunk:
                raise RuntimeError("对端关闭了连接")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode("ascii", errors="replace").strip()

    def cmd(self, text: str) -> str:
        self.send_line(text)
        return self.read_line()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def parse_q(line: str) -> dict:
    """把 'Q rf=1 tx=2 ok=3 drop=4 bad=5 ack=6' 解析成字典"""
    out = {}
    for kv in line.split()[1:]:
        if "=" in kv:
            k, v = kv.split("=", 1)
            out[k] = int(v)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--len", type=int, default=256,
                    help="报文长度，默认 256（= UDS 一个块的净数据）")
    ap.add_argument("--crc-only", action="store_true",
                    help="只算 CRC 和头字节，不连网")
    args = ap.parse_args()

    payload = make_pattern(args.len)
    crc = crc16_kermit(payload)
    frames = iso_tp_frames(payload)

    print(f"报文长度 = {len(payload)}")
    print(f"拆成 {len(frames)} 帧")
    print(f"期望 CRC16/KERMIT = 0x{crc:04X}")
    print("期望 head = " + " ".join(f"{b:02X}" for b in payload[:16]))

    if args.crc_only:
        for i, f in enumerate(frames[:4]):
            print(f"  帧{i}: " + " ".join(f"{b:02X}" for b in f))
        print(f"  ...（共 {len(frames)} 帧）")
        return 0

    try:
        ch = Chan(args.host, args.port)
    except OSError as e:
        print(f"连不上 {args.host}:{args.port} -> {e}", file=sys.stderr)
        return 2

    try:
        # ---- ① 通道冒烟：PING ----
        pong = ch.cmd("PING")
        print(f"PING -> {pong}")
        if pong != "PONG":
            print("通道不通，先查 F407 的 TCP 5000", file=sys.stderr)
            return 3

        # ---- ② 记下发之前的基线计数 ----
        base = parse_q(ch.cmd("Q"))
        print("基线: " + str(base))

        # ---- ③ 发帧，每 4 帧用 Q 同步一次 ----
        t0 = time.time()
        # ★ 2026-11-24：循环里的 Q 同步去掉了，理由见 sec_test.py 的 send_only
        #   · F407 现在自己会等对端流控帧，靠 Q 的往返强行降速已经不需要
        #   · 最后这一个 Q 保留：下面要读它的 drop / bad 计数当判据
        for f in frames:
            ch.send_line(f"CAN {ID_REQ:03X} {f.hex()}")
        ch.cmd("Q")
        dt = time.time() - t0

        # ---- ④ 看有没有被 F407 丢掉 ----
        end = parse_q(ch.cmd("Q"))
        print("发完: " + str(end))
        print(f"耗时 {dt * 1000:.0f} ms，平均 {len(frames) / dt:.0f} 帧/秒")

        dropped = end.get("drop", 0) - base.get("drop", 0)
        bad = end.get("bad", 0) - base.get("bad", 0)
        if dropped != 0 or bad != 0:
            print(f"★ 警告：F407 侧丢帧 {dropped}，坏命令 {bad}"
                  "（发太快邮箱满了，脚本要减批次）", file=sys.stderr)
            return 4

        print("")
        print("F407 侧已全部转发。设备侧不用盯串口了，读 UDS 0x22 即可：")
        print(f"  期望 62 F1 A0 {len(payload) >> 8:02X} {len(payload) & 0xFF:02X} "
              f"{crc >> 8:02X} {crc & 0xFF:02X}")
        print("（run_all.py 的 isotp 组会自动做这个比对）")
        return 0
    finally:
        ch.close()


if __name__ == "__main__":
    sys.exit(main())
