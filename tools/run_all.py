#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
CAN-OTA 回归测试统一入口

★ 它解决什么问题
    之前四个测试脚本要挨个手敲，改了一行代码之后"该跑哪几个"全靠记忆。
    记忆一断就会漏跑，漏跑就会把回归问题带到下一轮。
    这个入口把"全部跑一遍"变成一条命令，二十来秒出结论。

★ 为什么用子进程包着跑、而不是改成函数调用
    那几个脚本都已经是"自己打印 PASS/FAIL、自己返回退出码"的独立程序了。
    改成函数要动四个已经验证过的文件，改动量大于收益。
    包一层是常规做法：被测的东西不动，加一个外壳。

★ 为什么 ISO-TP 那一组不用子进程
    它要"发一批帧 → 在同一个连接上立刻读 Q / 读 UDS"交替进行，
    子进程只能整段跑完再给结果，插不进这些步骤。
    判据原来依赖设备的串口打印，2026-11-24 改成读 UDS 0x22（DID F1A0），
    所以这一组现在不占 COM11，串口工具开着也能跑。

★ 用法
    python run_all.py             # 跑全部
    python run_all.py --list      # 看有哪些组
    python run_all.py --only sec  # 只跑一组（可给多次）
    python run_all.py --stop      # 第一个失败就停
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable

HOST = "192.168.10.50"
HTTP = f"http://{HOST}/data"
SERIAL_PORT = "COM11"          # F103 的调试串口，换 USB 口时要改
# ★ 同上：去掉写死的家目录。环境变量优先，兜底按仓库相对位置找烧录工具
TOOL = os.environ.get(
    "CAN_GW_FLASH_TOOL",
    os.path.join(HERE, "..", "..", "..", "杂项文件", "PC工具", "flash_app.py"))


# ---------------------------------------------------------------- 小工具

def rf() -> int:
    """F407 的接收帧计数：总线有没有在跑，看它就够了"""
    with urllib.request.urlopen(HTTP, timeout=4) as r:
        return json.loads(r.read().decode())["rf"]


def wait_ready(timeout=25.0, name=""):
    """等设备恢复发数据

    ★ 为什么必须有这一步
      有些用例会主动复位设备（比如 0x11 复位、sec_test 的最后一步）。
      紧接着跑的用例如果假设"总线正在跑"，就会把"设备还在启动"误判成
      "总线挂了"。这类假失败在回归里最浪费时间——因为它不是代码坏了。
    """
    t0 = time.time()
    while (time.time() - t0) < timeout:
        try:
            a = rf()
            time.sleep(1.0)
            if rf() > a:
                return True
        except Exception:                  # noqa: BLE001
            time.sleep(0.5)
    print(f"  [警告] 等了 {timeout:.0f}s 设备还没开始发数据（{name}）")
    return False


def firmware_version() -> str:
    """从编译产物里读版本号，不碰串口也不复位设备"""
    if not os.path.exists(TOOL):
        return "（找不到 flash_app.py）"
    try:
        # ★ 2026-11-23 修：子进程用 -X utf8 输出，父进程必须也按 utf-8 解码
        #   · 原来写 text=True，等于让 subprocess 用系统 locale（中文 Windows 是 cp936）解码
        #   · 两边编码不一致，任何非 UTF-8 locale 的机器上都会抛 UnicodeDecodeError
        #   · errors="replace" 是兜底：万一哪天真有坏字节，也宁可显示乱码不要崩
        out = subprocess.run([PY, "-X", "utf8", TOOL, "--check"],
                             capture_output=True, encoding="utf-8",
                             errors="replace", timeout=30).stdout
    except Exception as e:                 # noqa: BLE001
        return f"（读取失败：{type(e).__name__}）"

    m = re.search(r"版本\s*：\s*(.+)", out)
    if m:
        return m.group(1).strip()
    m = re.search(r"img ver=(\d+) build=(\d+)", out)
    if m:
        return f"img ver={m.group(1)} build={m.group(2)}"
    return "（版本串没找到）"


def run_script(script, timeout=180):
    """跑一个会打印 [PASS]/[FAIL] 的脚本，数成败"""
    cmd = [PY, "-X", "utf8", os.path.join(HERE, script)]
    try:
        # ★ 2026-11-23 修：同上，cmd 里带 -X utf8，解码也必须用 utf-8
        p = subprocess.run(cmd, capture_output=True, encoding="utf-8",
                           errors="replace", timeout=timeout)
    except subprocess.TimeoutExpired:
        return 0, 1, [f"超时（>{timeout}s）：{script}"]

    out = (p.stdout or "") + (p.stderr or "")
    ok = len(re.findall(r"^\[PASS\]", out, re.M))
    bad = len(re.findall(r"^\[FAIL\]", out, re.M))

    detail = [l for l in out.splitlines() if l.startswith("[FAIL]")]

    # ★ 2026-11-24 审计修：退出码是主判据，不能只看打印行
    # · 原来只有"退出码非 0 且没有 [FAIL] 行"才补一次失败，
    #   于是"脚本 exit(1) 但打了一堆 [PASS]"会被判全绿（假绿）
    # · 现在退出码非 0 一律记失败，详情里带上 PASS 行数方便对照
    if p.returncode not in (0, None):
        bad += 1
        detail.append(f"退出码 {p.returncode}（PASS 行 {ok} 条）")

    return ok, bad, detail


# ---------------------------------------------------------------- 测试组

def uds_read(ch, req, want_sid, timeout=1.5):
    """发一条 UDS 请求，读回一条单帧响应（不含 ISO-TP 的 PCI 字节）

    ★ 为什么要过滤 want_sid
        被测报文本身不是合法的 UDS 请求（第一个字节是图案），设备会先回一条
        7F xx 11。那条否定响应如果正好落在抓帧窗口里，会被误当成这条请求的答复。
    """
    from isotp_test import ID_REQ, ID_RSP, iso_tp_frames

    ch.cmd(f"CAP {ID_RSP:03X}")
    for f in iso_tp_frames(req):
        ch.send_line(f"CAN {ID_REQ:03X} {f.hex()}")

    t0 = time.time()
    while (time.time() - t0) < timeout:
        r = ch.cmd("CAPQ")
        if r.startswith("RX "):
            data = bytes(int(x, 16) for x in r[6:].split())
            n = data[0] & 0x0F              # ISO-TP 单帧：首字节低 4 位是长度
            if n == 0 or n > 7:
                continue
            body = data[1:1 + n]
            if body and body[0] == want_sid:
                return body
        time.sleep(0.02)
    return None


def suite_isotp():
    """ISO-TP 传输层：两套独立实现算出的 CRC 必须相等

    ★ 2026-11-24 改：判据从"设备串口打了什么字"换成 UDS 0x22（DID F1A0）
        · 串口那一行要在 task_KEY 里忙等约 8 ms，想关 dump 就关不掉
        · 现在设备把"上一条包的长度 + CRC"存着，Tester 用 22 F1 A0 读回来
        · 附带好处：这一组不再占用 COM11，串口工具开着也能跑回归
    """
    from isotp_test import (Chan, ID_REQ, ID_RSP, iso_tp_frames,
                            make_pattern, crc16_kermit, parse_q)

    ok = 0
    bad = 0
    detail = []

    ch = None
    try:
        ch = Chan(HOST, 5000)
        if ch.cmd("PING") != "PONG":
            return 0, 1, ["F407 命令通道不响应 PING"]

        for n in (7, 8, 256, 300):
            payload = make_pattern(n)
            crc = crc16_kermit(payload)
            frames = iso_tp_frames(payload)

            # ★ 2026-11-24 审计修：发之前记基线，发完算增量
            # · 原来只看"设备有没有打出 [ISOTP]" —— 那只能证明包收齐了，
            #   证明不了流控对不对。to（等待超时）和 cfd（被丢弃的 CF）
            #   原来一条断言都没有
            base = parse_q(ch.cmd("Q"))

            for f in frames:
                ch.send_line(f"CAN {ID_REQ:03X} {f.hex()}")
            end = parse_q(ch.cmd("Q"))

            to_d = end.get("to", 0) - base.get("to", 0)
            fc_d = end.get("fc", 0) - base.get("fc", 0)
            cfd_d = end.get("cfd", 0) - base.get("cfd", 0)

            # ★ 2026-11-24 复核：断言要按用例性质分，不能一刀切
            # · len=300 是"对端必须拒收"的用例，它回的是 OVFLW 而不是 CTS，
            #   而 WaitFc 只在拿到 CTS 时才 fc++，所以这条不该要求 fc>0
            # · len=300 出现一次 20 ms 超时也是正常的：CF 可能先到、OVFLW 后到，
            #   那一下就走了超时分支。它是竞态不是缺陷
            # · 真正该盯的是 cfd（OVFLW 之后那些 CF 有没有被拦住），见下面
            if n != 300:
                if to_d != 0:
                    bad += 1
                    detail.append(f"len={n} 出现 {to_d} 次流控等待超时（to）")

                if (n >= 8) and (fc_d <= 0):
                    bad += 1
                    detail.append(f"len={n} 一个流控帧都没等到（fc={fc_d}）")

            # 读"上一条收完的包"，也就是刚发出去的那一条
            r = uds_read(ch, bytes((0x22, 0xF1, 0xA0)), 0x62)
            if (r is None) or (len(r) != 7) or (r[0] != 0x62):
                bad += 1
                shown = "无响应" if r is None else r.hex(" ").upper()
                detail.append(f"len={n} 读不回 0x22 统计（{shown}）")
                continue

            got_len = (r[3] << 8) | r[4]
            got_crc = (r[5] << 8) | r[6]

            if n == 300:
                # 判据一：超过 288 缓冲，设备必须拒收 —— "上一条"不能变成 300
                if got_len == 300:
                    bad += 1
                    detail.append("len=300 本该拒收却收到了整包")
                else:
                    ok += 1

                # 判据二（2026-11-24 审计补）：F407 侧必须真的走进 dropping 分支
                # · 原来只验证"F103 没收到整包"，F407 的 dropping 路径一次都没走到
                # · cfd = 被丢弃的连续帧数，它涨了才证明 OVFLW 之后那些 CF 被拦住了
                if cfd_d <= 0:
                    bad += 1
                    detail.append(
                        f"len=300 OVFLW 后 cfd 没涨（={cfd_d}），dropping 分支可能没走到")
                else:
                    ok += 1
                continue

            if (got_len == n) and (got_crc == crc):
                ok += 1
            else:
                bad += 1
                detail.append(
                    f"len={n} 不一致：设备 n={got_len} crc={got_crc:04X}，"
                    f"期望 n={n} crc={crc:04X}")
    finally:
        if ch is not None:
            ch.close()

    return ok, bad, detail


def suite_script(script):
    return lambda: run_script(script)


SUITES = [
    ("isotp",   "ISO-TP 传输层",   suite_isotp),
    ("session", "UDS 会话与保活",  suite_script("session_test.py")),
    ("sec",     "UDS 安全访问",    suite_script("sec_test.py")),
    ("comm",    "UDS 通信控制",    suite_script("comm_test.py")),
    ("routine", "UDS 例程控制",    suite_script("routine_test.py")),
]


# ---------------------------------------------------------------- 主流程

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="列出所有测试组")
    ap.add_argument("--only", action="append", default=[], help="只跑指定组")
    ap.add_argument("--stop", action="store_true", help="第一个失败就停")
    args = ap.parse_args()

    if args.list:
        for key, name, _ in SUITES:
            print(f"{key:<10} {name}")
        return 0

    picked = [s for s in SUITES if (not args.only) or (s[0] in args.only)]
    if not picked:
        print("没有匹配的测试组，用 --list 看有哪些", file=sys.stderr)
        return 2

    print("=" * 54)
    print(f"固件版本：{firmware_version()}")
    print(f"设备地址：{HOST}    串口：{SERIAL_PORT}")
    print("=" * 54)

    total_ok = 0
    total_bad = 0
    t_all = time.time()

    for idx, (key, name, fn) in enumerate(picked):
        # 第一个组之前不必等；后面每组之前都可能刚被上一组复位过
        if idx > 0:
            wait_ready(name=name)

        t0 = time.time()
        ok, bad, detail = fn()
        dt = time.time() - t0
        total_ok += ok
        total_bad += bad

        flag = "OK  " if bad == 0 else "FAIL"
        print(f"[{flag}] {name:<16} {ok}/{ok + bad} 通过  {dt:.1f}s")
        for d in detail:
            print(f"         {d}")

        if args.stop and bad > 0:
            print("（--stop：遇到失败提前结束）")
            break

    print("-" * 54)
    n = total_ok + total_bad
    print(f"合计 {total_ok}/{n} 通过，失败 {total_bad}，总耗时 {time.time() - t_all:.1f}s")

    return 0 if total_bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
