#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
UDS 请求发送器：PC → F407 网关 → CAN → F103

★ 它只负责"发"
    回程这条链还没有（F407 的命令通道目前只发不收），所以响应要看 F103 的串口。
    uds.c 在 TEST_ISO_TP_DUMP = 1 时会把每条收发打成
        [UDS] RX  3E 00
        [UDS] TX+ 7E 00
    校验方式就是拿这两行对照。

★ 用法
    python uds_test.py --req 3E00
    python uds_test.py --req 1003 --req 3E00
    参数就是 UDS 请求的十六进制字节串，长度不限（超过 7 字节会自动走多帧）。

★ 为什么不直接连 F103
    电脑上没有 CAN 硬件，只能借 F407 的命令通道（端口 5000）。
"""

import argparse
import sys

from isotp_test import Chan, ID_REQ, iso_tp_frames


def send_request(ch, req: bytes, sync_every: int = 4) -> None:
    """把一条 UDS 请求按 ISO-TP 拆帧后逐帧下发，每 4 帧用 Q 同步一次"""
    frames = iso_tp_frames(req)

    for i, f in enumerate(frames):
        ch.send_line(f"CAN {ID_REQ:03X} {f.hex()}")
        if (i % sync_every) == (sync_every - 1):
            ch.cmd("Q")

    # 最后一定补一个 Q：保证最后一帧之后 F407 已经处理完，
    # 否则脚本退出太快，F103 还没收齐就开始读串口了
    ch.cmd("Q")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--req", action="append", default=[],
                    help="UDS 请求的十六进制，例如 3E00；可以给多次")
    ap.add_argument("--host", default="192.168.10.50")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--quiet", action="store_true", help="只发不打印")
    args = ap.parse_args()

    if not args.req:
        print("至少要给一个 --req", file=sys.stderr)
        return 2

    reqs = [bytes.fromhex(r.replace(" ", "")) for r in args.req]

    try:
        ch = Chan(args.host, args.port)
    except OSError as e:
        print(f"连不上 {args.host}:{args.port} -> {e}", file=sys.stderr)
        return 3

    try:
        if ch.cmd("PING") != "PONG":
            print("F407 命令通道不响应 PING", file=sys.stderr)
            return 3

        base = ch.cmd("Q")
        for req in reqs:
            if not args.quiet:
                print("发 " + " ".join(f"{b:02X}" for b in req)
                      + f"  （{len(iso_tp_frames(req))} 帧）")
            send_request(ch, req)
        if not args.quiet:
            print("F407 计数: " + ch.cmd("Q") + f"   （发之前: {base}）")
        return 0
    finally:
        ch.close()


if __name__ == "__main__":
    sys.exit(main())
