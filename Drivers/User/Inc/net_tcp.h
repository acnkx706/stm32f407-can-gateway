#ifndef __NET_TCP_H
#define __NET_TCP_H

/*==============================================================================
 *  网关命令通道（W5500 socket 0，端口 5000）
 *
 *  ★ 做什么
 *    给电脑开一条"能直接往 CAN 总线上打帧"的命令通道，文本协议，一行一条命令。
 *    原来这里是个回显服务器，只用来验证 socket 层通不通；现在它承担转发职责，
 *    PING 命令保留了原来那个冒烟测试的作用。
 *
 *  ★ 为什么需要它
 *    电脑上没有任何 CAN 硬件（只有 CH340 和 ST-Link），想把 ISO-TP 报文送进
 *    F103 只有一条路：电脑走 TCP 到这里，这里再发 CAN 帧。没有这条通道，
 *    F103 那边写的 ISO-TP 只能靠自环验证，证明不了它符合协议。
 *
 *  ★ 影响什么
 *    占用 socket 0 和 2 KB 收发缓冲，主循环每 2 ms 轮询一次。
 *    这条通道会往总线上发帧，正式产品里要么关掉，要么加访问控制。
 *
 *  ★ 怎么验证
 *    电脑 telnet 192.168.10.50 5000，敲 PING 应回 PONG，敲 Q 应回一行计数。
 *    逐条命令与判据见 77 号文档 §7。
 *============================================================================*/

#include <stdint.h>

/* ★ 命令通道端口。原来叫回显端口，改成命令通道后端口号不变，
 *   免得所有已经写好的测试脚本和笔记都要跟着改 */
#define NET_TCP_CMD_PORT   5000u

/* ★ 网关口令（2026-11-24 从 net_tcp.c 挪过来）
 * · 命令通道的 AUTH 和网页的 POST /set 共用同一把
 * · 放头文件是为了让 web_srv.c 也能看到 —— 两处各写一份一定会走差，
 *   到时候改了一边忘了另一边，现象是"命令通道能过、网页改不了"
 */
#define NET_TCP_TOKEN   "can-gw-2026"

/* ★ 打开 socket 0 监听
 *   返回 0 = 成功，非 0 = 失败 */
int8_t Net_TcpInit(void);

/* ★ 状态机轮询：必须由主循环定期调用
 *   内部处理 连接 / 收命令 / 发 CAN 帧 / 回包 / 断开重连 */
void Net_TcpPoll(void);

/* ★ 统计量，供屏幕或串口显示 */
extern volatile uint32_t g_tcp_conn_cnt;      /* 累计连接次数 */
extern volatile uint32_t g_tcp_rx_bytes;      /* 累计收到字节 */
extern volatile uint32_t g_tcp_tx_bytes;      /* 累计发出字节 */
extern volatile uint8_t  g_tcp_connected;     /* 1 = 当前有客户端连接 */

/* ★ 转发通道自己的计数：脚本用 Q 命令读，判断是不是自己发太快丢了帧
 *   · can_ok   成功装进发送邮箱的帧数
 *   · can_drop 邮箱满导致丢掉的帧数（说明电脑发太快了）
 *   · bad_cmd  解析失败或超长的行数 */
extern volatile uint32_t g_fwd_cmd_cnt;
extern volatile uint32_t g_fwd_can_ok;
extern volatile uint32_t g_fwd_can_drop;
extern volatile uint32_t g_fwd_bad_cmd;

void Net_TcpSelfTest(void);

#endif /* __NET_TCP_H */
