/**
  ******************************************************************************
  * @file    net_tcp.c
  * @brief   网关命令通道：TCP 5000 收文本命令，往 CAN 总线发帧
  ******************************************************************************
  * ★ 做什么
  *   给电脑开一条可以直接往 CAN 总线打帧的通道。原来这里是个回显服务器，
  *   现在改成一行一条命令的文本协议，PING 保留下来当 socket 层的冒烟测试。
  *
  * ★ 为什么改
  *   电脑上没有 CAN 硬件，要把 ISO-TP 报文送进 F103 只能走这条路。
  *   不改的话 F103 那边的传输层只能自环验证，证明不了它符合协议。
  *
  * ★ 影响什么
  *   占 socket 0 和 2 KB 缓冲；会往总线上发帧，正式产品要么关掉要么加鉴权。
  *   回显功能没有了，原来那条"发什么回什么"的验证改由 PING 顶替。
  *
  * ★ 怎么验证
  *   telnet 192.168.10.50 5000，PING 回 PONG，Q 回一行计数，
  *   CAN 201 0102 之后 F103 侧应看到对应的帧。命令表见 77 号文档 §7。
  ******************************************************************************
  */

#include "net_tcp.h"
#include "w5500_port.h"
#include "usart.h"
#include "main.h"
#include "can_gw.h"

#include <stdio.h>

#include "wizchip_conf.h"
#include "socket.h"

/* ★ socket 0：W5500 一共 8 个 socket，0 号原来做回显，现在做命令通道 */
#define TCP_SN          0u
#define TCP_BUF_SZ      1024u

/* ★ 保活时间：Sn_KPALVTR 单位是 5 秒，写 4 = 20 秒
 *   · 拔网线、电脑休眠这类异常断开，20 秒内会自己回收 socket
 *   · 不设的话对端不见了这边也不知道，socket 会一直占着 */
#define TCP_KEEPALIVE   4u

/* ★ 一行命令最长多少字符，以及一次回包最长多少字符
 *   · 最长的命令是 CAN 加 3 位 ID 加 8 字节数据的十六进制，约 28 个字符
 *   · 回包最长的是 Q：2026-11-23 加四个流控计数后按最长取值约 98 个字符
 *   · 留 160 是为了给以后加参数留余量，同时不浪费 RAM */
#define FWD_LINE_MAX    96u
#define FWD_TX_MAX      160u

/* ★ 命令通道口令（2026-11-24 加）
 * · 做什么：往总线发帧（CAN 命令）之前必须先发一行 AUTH，读操作不设门槛
 * · 为什么：5000 端口原来完全裸奔，同一网段任何人 telnet 进来就能往 CAN 发任意帧
 * · 影响什么：PC 侧每个新连接都要先认证，否则 CAN 命令回 AUTH REQUIRED
 * · 怎么验证：不认证发 CAN 被拒，认证之后正常发出
 * · 局限：口令写死在固件里，属于演示级防护；量产应换成每台设备唯一的密钥
 * · 2026-11-24：定义已挪到 net_tcp.h，因为 web_srv.c 的 POST 也要用同一把
 */

volatile uint32_t g_tcp_conn_cnt  = 0u;
volatile uint32_t g_tcp_rx_bytes  = 0u;
volatile uint32_t g_tcp_tx_bytes  = 0u;
volatile uint8_t  g_tcp_connected = 0u;

volatile uint32_t g_fwd_cmd_cnt  = 0u;
volatile uint32_t g_fwd_can_ok   = 0u;
volatile uint32_t g_fwd_can_drop = 0u;
volatile uint32_t g_fwd_bad_cmd   = 0u;
volatile uint32_t g_fwd_auth_fail = 0u;

/* 1 = 本次连接已通过 AUTH。新连接必须重新认证，所以建链时清零 */
static uint8_t s_authed = 0u;

static uint8_t  s_buf[TCP_BUF_SZ];
static uint8_t  s_tx[FWD_TX_MAX];
static char     s_line[FWD_LINE_MAX];
static uint16_t s_line_len;

/*==============================================================================
 *  ★ 等 PHY 链路起来（最长 5 秒）
 *
 *  ★ 为什么必须先等：
 *    W5500 在 PHY 链路 DOWN 时会静默忽略 socket 命令，
 *    PHYCFGR 的 LNK 位必须是 1 才能碰任何 socket 寄存器。
 *
 *  ★ 2026-11-22 修：这个函数以前写了但没人调用，
 *    导致"等了链路还是失败"那条历史结论其实没被测过。
 *============================================================================*/
static void Net_WaitLink(void)
{
    uint8_t i;

    for (i = 0u; i < 50u; i++)
    {
        if (W5500_LinkUp() != 0u)
        {
            printf("[NET] link up after %u ms\r\n", (unsigned)(i * 100u));
            return;
        }
        HAL_Delay(100u);
    }

    printf("[NET] link still down after 5s\r\n");
}

int8_t Net_TcpInit(void)
{
    Net_WaitLink();

    close(TCP_SN);

    if (socket(TCP_SN, Sn_MR_TCP, NET_TCP_CMD_PORT, 0u) != TCP_SN)
    {
        return -1;
    }

    if (listen(TCP_SN) != SOCK_OK)
    {
        return -2;
    }

    /* ★ 打上保活，异常断开的 socket 才收得回来 */
    {
        uint8_t ka = (uint8_t)TCP_KEEPALIVE;
        setSn_KPALVTR(TCP_SN, ka);
    }

    s_line_len = 0u;

    return 0;
}

/*==============================================================================
 *  ★ 读 socket 状态，三次采样取多数
 *
 *  ★ 为什么不能只读一次：
 *    SPI 跑在 42 MHz 时，单次读 Sn_SR 偶尔会返回 SOCK_CLOSED，
 *    哪怕 socket 其实是 LISTEN 或 ESTABLISHED。
 *    这让状态机跳过 ESTABLISHED 分支，连上的客户端收不到任何回包。
 *============================================================================*/
static uint8_t Net_ReadSr(void)
{
    uint8_t a = getSn_SR(TCP_SN);
    uint8_t b = getSn_SR(TCP_SN);
    uint8_t c = getSn_SR(TCP_SN);

    if (a == b) { return a; }
    if (a == c) { return a; }
    if (b == c) { return b; }

    /* 三次全不一样（很少见）：优先返回任何非 CLOSED 的值 */
    if (a != SOCK_CLOSED) { return a; }
    if (b != SOCK_CLOSED) { return b; }
    return c;
}

/*==============================================================================
 *  ★ 十六进制字符转数值，非法返回 -1
 *============================================================================*/
static int8_t Fwd_HexVal(char c)
{
    if ((c >= '0') && (c <= '9')) { return (int8_t)(c - '0'); }
    if ((c >= 'A') && (c <= 'F')) { return (int8_t)(c - 'A' + 10); }
    if ((c >= 'a') && (c <= 'f')) { return (int8_t)(c - 'a' + 10); }
    return -1;
}

/*==============================================================================
 *  ★ 执行一条命令
 *
 *  ★ 返回 回包长度，0 表示这条命令不需要回包
 *
 *  ★ 为什么 CAN 命令成功时不回包：
 *    一次 256 字节的 ISO-TP 报文要发 37 帧。如果每帧都等一个回包，
 *    37 次往返就把传输时间从 10 ms 拖到 100 ms 以上，测不出真实吞吐。
 *    所以成功时不回话，出错靠 Q 命令里的 drop 计数看。
 *
 *  ★ 命令表（一行一条，\n 结尾，\r 会被忽略）
 *    PING               回 PONG
 *    Q                  回一行计数
 *    CAN <ID> <DATA>    ID 是 3 位十六进制，DATA 是每字节 2 位十六进制
 *============================================================================*/
static uint16_t Fwd_Exec(const char *s, uint16_t n, uint8_t *out)
{
    uint16_t i;
    uint16_t k;

    /* 去掉行尾的 \r 和空格 */
    while ((n > 0u) && ((s[n - 1u] == '\r') || (s[n - 1u] == ' ')))
    {
        n--;
    }
    if (n == 0u)
    {
        return 0u;
    }

    g_fwd_cmd_cnt++;

    /* ---- PING：socket 层通不通的冒烟测试 ---- */
    if ((n == 4u) && (s[0] == 'P') && (s[1] == 'I') && (s[2] == 'N') && (s[3] == 'G'))
    {
        out[0] = 'P'; out[1] = 'O'; out[2] = 'N'; out[3] = 'G';
        out[4] = '\r'; out[5] = '\n';
        return 6u;
    }
    /* ---- AUTH <token>：解锁写操作 ---- */
    if ((n > 5u) && (s[0] == 'A') && (s[1] == 'U') && (s[2] == 'T')
        && (s[3] == 'H') && (s[4] == ' '))
    {
        static const char tok[] = NET_TCP_TOKEN;
        uint8_t ok = 1u;
        int     mm;

        k = 5u;
        while ((k < n) && (s[k] == ' '))
        {
            k++;
        }

        for (i = 0u; tok[i] != '\0'; i++)
        {
            if ((k >= n) || (s[k] != tok[i]))
            {
                ok = 0u;
                break;
            }
            k++;
        }

        /* 后面多给了字符也算错，免得 "can-gw-2026XYZ" 被当成通过 */
        if ((ok != 0u) && (k != n))
        {
            ok = 0u;
        }

        if (ok != 0u)
        {
            s_authed = 1u;
            mm = snprintf((char *)out, (size_t)FWD_TX_MAX, "AUTH OK\r\n");
        }
        else
        {
            s_authed = 0u;
            g_fwd_auth_fail++;
            mm = snprintf((char *)out, (size_t)FWD_TX_MAX, "AUTH FAIL\r\n");
        }

        return (mm > 0) ? (uint16_t)mm : 0u;
    }

    /* ---- Q：一行状态，脚本靠它判断有没有发太快 ---- */
    if ((n == 1u) && (s[0] == 'Q'))
    {
        /* ★ 2026-11-23 加 ff/cf/fc/to 四项：没有这四个数就没法证明流控在工作
         *   · ff 是首帧数，cf 是连续帧数，两者之比就是平均每包几帧
         *   · fc 是等到的流控帧数，to 是等待失败数 —— 健康时 to 恒为 0 */
        int m = snprintf((char *)out, (size_t)FWD_TX_MAX,
                         "Q rf=%lu tx=%lu ok=%lu drop=%lu bad=%lu ack=%lu "
                         "ff=%lu cf=%lu fc=%lu to=%lu wms=%lu wmax=%lu "
                         "cfd=%lu\r\n",
                         (unsigned long)g_can_rx_frames,
                         (unsigned long)g_can_tx_frames,
                         (unsigned long)g_fwd_can_ok,
                         (unsigned long)g_fwd_can_drop,
                         (unsigned long)g_fwd_bad_cmd,
                         (unsigned long)g_can_ack_err,
                         (unsigned long)g_fwd_isotp_ff,
                         (unsigned long)g_fwd_isotp_cf,
                         (unsigned long)g_fwd_isotp_fc,
                         (unsigned long)g_fwd_isotp_fc_to,
                         (unsigned long)g_fwd_isotp_wait_ms,
                         (unsigned long)g_fwd_isotp_wait_max,
                         (unsigned long)g_fwd_isotp_cf_dropped);
        if (m <= 0)
        {
            return 0u;
        }
        return (uint16_t)m;
    }

    /* ---- CAPN：抓到了几帧 ---- */
    if ((n == 4u) && (s[0] == 'C') && (s[1] == 'A') && (s[2] == 'P') && (s[3] == 'N'))
    {
        int m = snprintf((char *)out, (size_t)FWD_TX_MAX, "N %lu\r\n",
                         (unsigned long)g_can_cap_cnt);

        return (m > 0) ? (uint16_t)m : 0u;
    }

    /* ---- CAPQ：读回抓帧槽 ---- */
    if ((n == 4u) && (s[0] == 'C') && (s[1] == 'A') && (s[2] == 'P') && (s[3] == 'Q'))
    {
        int m;

        if (g_can_cap_hit == 0u)
        {
            out[0] = 'N'; out[1] = 'O'; out[2] = 'N'; out[3] = 'E';
            out[4] = '\r'; out[5] = '\n';
            return 6u;
        }

        m = snprintf((char *)out, (size_t)FWD_TX_MAX, "RX %03X ", (unsigned)g_can_cap_id);
        if (m <= 0)
        {
            return 0u;
        }

        for (k = 0u; (k < (uint16_t)g_can_cap_dlc) && (k < 8u) && ((m + 3) < (int)FWD_TX_MAX);
             k++)
        {
            m += snprintf((char *)out + m, (size_t)(FWD_TX_MAX - (uint16_t)m),
                          "%02X ", (unsigned)g_can_cap_data[k]);
        }

        out[m] = '\r';
        out[m + 1] = '\n';

        return (uint16_t)(m + 2);
    }

    /* ---- CAP <ID>：布置抓帧 ---- */
    if ((n > 4u) && (s[0] == 'C') && (s[1] == 'A') && (s[2] == 'P') && (s[3] == ' '))
    {
        uint16_t cid = 0u;
        int      m;

        i = 4u;
        while ((i < n) && (s[i] == ' '))
        {
            i++;
        }

        for (k = 0u; k < 3u; k++)
        {
            int8_t v = (i < n) ? Fwd_HexVal(s[i]) : (int8_t)-1;

            if (v < 0)
            {
                g_fwd_bad_cmd++;
                return 0u;
            }
            cid = (uint16_t)((cid << 4) | (uint16_t)v);
            i++;
        }

        /* ★ 先清 hit 再改 ID：反过来会有一个瞬间"新 ID + 旧 hit"被别人读到 */
        g_can_cap_hit = 0u;
        g_can_cap_cnt = 0u;
        g_can_cap_id  = cid;

        m = snprintf((char *)out, (size_t)FWD_TX_MAX, "CAP %03X\r\n", (unsigned)cid);

        return (m > 0) ? (uint16_t)m : 0u;
    }

    /* ---- CAN <ID> <DATA> ---- */
    if ((n > 6u) && (s[0] == 'C') && (s[1] == 'A') && (s[2] == 'N') && (s[3] == ' '))
    {
        uint16_t id = 0u;
        uint8_t  d[8];
        uint8_t  dl = 0u;
        int8_t   hi;
        int8_t   lo;
        int      mm;

        /* ★ 2026-11-24 加：发帧是写操作，必须先认证
         * · 读命令（PING / Q / CAP / CAPQ / CAPN）不设门槛，方便只做监控的客户端
         * · 拒绝时回一行而不是静默丢弃：让 PC 立刻分清"没认证"和"设备没响应" */
        if (s_authed == 0u)
        {
            g_fwd_auth_fail++;
            mm = snprintf((char *)out, (size_t)FWD_TX_MAX, "AUTH REQUIRED\r\n");
            return (mm > 0) ? (uint16_t)mm : 0u;
        }

        i = 4u;
        while ((i < n) && (s[i] == ' '))
        {
            i++;
        }

        /* ID 固定 3 位十六进制，正好覆盖 11 位标准帧的全部取值 */
        for (k = 0u; k < 3u; k++)
        {
            int8_t v = (i < n) ? Fwd_HexVal(s[i]) : (int8_t)-1;

            if (v < 0)
            {
                g_fwd_bad_cmd++;
                return 0u;
            }
            id = (uint16_t)((id << 4) | (uint16_t)v);
            i++;
        }

        while ((i < n) && (s[i] == ' '))
        {
            i++;
        }

        /* 数据每两个字符一个字节，最多 8 字节；不够偶数就按解析失败算 */
        while ((i + 1u < n) && (dl < 8u))
        {
            hi = Fwd_HexVal(s[i]);
            lo = Fwd_HexVal(s[i + 1u]);
            if ((hi < 0) || (lo < 0))
            {
                break;
            }
            d[dl] = (uint8_t)(((uint8_t)hi << 4) | (uint8_t)lo);
            dl++;
            i += 2u;
        }

        if (dl == 0u)
        {
            g_fwd_bad_cmd++;
            return 0u;
        }

        /* ★ 2026-11-23：改走 CanGw_FwdIsoTp，多帧报文内部会等对端流控帧
         *   · 首帧和连续帧不再一口气推下去，F103 的 3 级接收 FIFO 才不会被冲爆
         *   · 单帧没有流控，行为和原来完全一样
         *
         * ★ 邮箱满就等一下再试，最多 3 次
         *   · F407 只有 3 个发送邮箱，而总线上一帧要约 250 us 才发完
         *   · 电脑一次 TCP 段塞来 4 条命令时，第 4 条会撞上"邮箱还没排空"
         *   · 每等 1 ms 足够排空一个邮箱，3 次之后还算失败就是真堵了
         *   · 代价：最坏阻塞主循环 3 ms，只影响这个测试通道，不影响周期任务 */
        {
            uint8_t try_n;

            for (try_n = 0u; try_n < 3u; try_n++)
            {
                if (CanGw_FwdIsoTp(id, d, dl) == HAL_OK)
                {
                    g_fwd_can_ok++;
                    break;
                }
                HAL_Delay(1u);
            }

            if (try_n >= 3u)
            {
                g_fwd_can_drop++;
            }
        }

        return 0u;
    }

    /* ---- AQ：故障告警状态（2026-11-24 加） ---- */
    if ((n == 2u) && (s[0] == 'A') && (s[1] == 'Q'))
    {
        /* AQ 也快照一份：ch/reason/count 三个字段由中断分开更新，同理 */
        CanGwAlarm_t a;
        CanGwData_t  cd;
        int m;

        __disable_irq();
        a  = *CanGw_GetAlarm();
        cd = *CanGw_GetData();
        __enable_irq();

        m = snprintf((char *)out, (size_t)FWD_TX_MAX,
                     "AQ ch=%u rsn=%u cnt=%u total=%lu lv=%u ok=%u bad=%lu\r\n",
                     (unsigned)a.ch, (unsigned)a.reason,
                     (unsigned)a.count, (unsigned long)a.total,
                     (unsigned)cd.proto_ver, (unsigned)cd.layout_ok,
                     (unsigned long)cd.layout_bad);

        return (m > 0) ? (uint16_t)m : 0u;
    }
    /* ---- CQ：命令通道状态（2026-11-24 加） ---- */
    if ((n == 2u) && (s[0] == 'C') && (s[1] == 'Q'))
    {
        /* ★ 2026-11-24 审计修：先关中断拷一份快照，再格式化
         * · ack_cnt 是 32 位、读它原子；但 ack_code/ack_result 是两个独立 8 位，
         *   中断在几次读之间更新，就会出现"ack_cnt 已是新的、res 还是上一条的"
         * · 脚本靠这个组合判断执行结果，读到错配的 res 会误判成功/失败
         * · 拷一份只有十几个字节、几条指令，代价可忽略 */
        CanGwCmd_t snap;
        int m;

        __disable_irq();
        snap = *CanGw_GetCmd();
        __enable_irq();

        m = snprintf((char *)out, (size_t)FWD_TX_MAX,
                     "CQ cmd=%u ack=%u res=%u n=%lu m=%lu\r\n",
                     (unsigned)snap.last_cmd, (unsigned)snap.ack_code,
                     (unsigned)snap.ack_result,
                     (unsigned long)snap.cmd_cnt,
                     (unsigned long)snap.ack_cnt);

        return (m > 0) ? (uint16_t)m : 0u;
    }

    /* ---- CMD <2位hex>：往 0x201 下发一条命令 ---- */
    if ((n >= 6u) && (s[0] == 'C') && (s[1] == 'M') && (s[2] == 'D') && (s[3] == ' '))
    {
        uint8_t cmd;
        int8_t  vh;
        int8_t  vl;
        int     m;

        /* 下发命令会动到现场设备，和 CAN 命令一样要求先认证 */
        if (s_authed == 0u)
        {
            g_fwd_auth_fail++;
            m = snprintf((char *)out, (size_t)FWD_TX_MAX, "AUTH REQUIRED\r\n");
            return (m > 0) ? (uint16_t)m : 0u;
        }

        vh = Fwd_HexVal(s[4]);
        vl = Fwd_HexVal(s[5]);

        if ((vh < 0) || (vl < 0))
        {
            g_fwd_bad_cmd++;
            return 0u;
        }

        cmd = (uint8_t)(((uint8_t)vh << 4) | (uint8_t)vl);

        if (CanGw_SendCmd(cmd, 0u, 0u) != HAL_OK)
        {
            g_fwd_can_drop++;
            return 0u;
        }

        /* 成功不回包，和 CAN 命令保持一致；要结果就去读 CQ */
        return 0u;
    }

    g_fwd_bad_cmd++;
    return 0u;
}

/*==============================================================================
 *  ★ 把收到的字节喂进行缓冲，遇到 \n 就执行
 *
 *  ★ 为什么按字节走而不是按行切：
 *    TCP 是字节流，一条命令可能被拆成两次 recv，也可能一次来了好几条。
 *    按字节攒到 \n 才执行，两种情况都对。
 *============================================================================*/
static void Fwd_Feed(const uint8_t *p, int32_t n)
{
    int32_t i;

    for (i = 0; i < n; i++)
    {
        char c = (char)p[i];

        if (c == '\n')
        {
            uint16_t olen = Fwd_Exec(s_line, s_line_len, s_tx);

            s_line_len = 0u;

            if (olen > 0u)
            {
                if (send(TCP_SN, s_tx, olen) > 0)
                {
                    g_tcp_tx_bytes += (uint32_t)olen;
                }
            }
        }
        else if (s_line_len < (FWD_LINE_MAX - 1u))
        {
            s_line[s_line_len] = c;
            s_line_len++;
        }
        else
        {
            /* 太长：丢掉整行，等下一条 \n 重新开始
             * 不清空的话后面所有字节都会被当成这一行的一部分，越错越远 */
            s_line_len = 0u;
            g_fwd_bad_cmd++;
        }
    }
}

void Net_TcpPoll(void)
{
    static uint32_t s_last_poll   = 0u;
    static uint8_t  s_closed_cnt  = 0u;
    static uint32_t s_closed_tick = 0u;
    static uint32_t s_listen_tick = 0u;

    uint8_t  sr;
    uint16_t len;
    int32_t  ret;

    /* ★ 限速到 2 ms 一次
     *   · 全速轮询时 SPI 事务约 45k 次/秒，芯片命令仲裁被压得很紧
     *   · 收紧之后单次读 Sn_SR 返回 SOCK_CLOSED 的假读明显减少 */
    if ((HAL_GetTick() - s_last_poll) < 2u)
    {
        return;
    }
    s_last_poll = HAL_GetTick();

    sr = Net_ReadSr();

    switch (sr)
    {
    case SOCK_ESTABLISHED:
        s_closed_cnt = 0u;

        if ((getSn_IR(TCP_SN) & Sn_IR_CON) != 0u)
        {
            setSn_IR(TCP_SN, Sn_IR_CON);
            g_tcp_conn_cnt++;
            g_tcp_connected = 1u;
            s_authed = 0u;              /* 新连接必须重新认证 */
            s_line_len = 0u;
            printf("[TCP] connect #%lu\r\n", (unsigned long)g_tcp_conn_cnt);
        }

        len = getSn_RX_RSR(TCP_SN);
        if (len > 0u)
        {
            if (len > TCP_BUF_SZ)
            {
                len = TCP_BUF_SZ;
            }

            ret = recv(TCP_SN, s_buf, (uint16_t)len);
            if (ret > 0)
            {
                g_tcp_rx_bytes += (uint32_t)ret;
                Fwd_Feed(s_buf, ret);
            }
        }
        break;

    case SOCK_CLOSE_WAIT:
        s_closed_cnt = 0u;
        g_tcp_connected = 0u;
        s_line_len = 0u;

        /* ★ 用有界的 close()，不用 disconnect()
         *   · ioLibrary 的 disconnect() 内部是无超时的死等循环
         *   · 客户端异常断开时会全速刷 SPI，把整片 W5500 一起压垮
         *   · 现象是 MCU 还活着但两个 socket 同时不响应，只能复位 */
        (void)close(TCP_SN);
        break;

    case SOCK_CLOSED:
        /* ★ 要求连续 5 次读到 CLOSED 且跨度不少于 20 ms 才重开
         *   · 单次假读会让 socket() 内部的 close() 关掉一条活着的 LISTEN */
        if (s_closed_cnt == 0u)
        {
            s_closed_tick = HAL_GetTick();
        }
        s_closed_cnt++;

        if ((s_closed_cnt >= 5u) && ((HAL_GetTick() - s_closed_tick) >= 20u))
        {
            s_closed_cnt = 0u;
            g_tcp_connected = 0u;
            s_line_len = 0u;

            if (socket(TCP_SN, Sn_MR_TCP, NET_TCP_CMD_PORT, 0u) == (int8_t)TCP_SN)
            {
                (void)listen(TCP_SN);
                printf("[TCP] reopened SR=0x%02X\r\n", (unsigned)Net_ReadSr());
            }
        }
        break;

    case SOCK_INIT:
        s_closed_cnt = 0u;
        /* 已经打开但还没进监听：最多每 100 ms 重试一次 LISTEN */
        if ((HAL_GetTick() - s_listen_tick) >= 100u)
        {
            s_listen_tick = HAL_GetTick();
            (void)listen(TCP_SN);
        }
        break;

    default:
        /* SOCK_LISTEN 和过渡态：什么都不做
         * 在这里碰命令寄存器正是当初把 LISTEN 搞坏的原因 */
        s_closed_cnt = 0u;
        break;
    }
}

/*==============================================================================
 *  ★ 2026-11-22 的临时自测：手工走一遍 close / socket / listen，逐次打状态
 *  ★ 目的：确认 socket() / listen() 本身有没有问题
 *  ★ 现在没有调用者，保留是为了下次怀疑 socket 层时能直接拿来用
 *============================================================================*/
void Net_TcpSelfTest(void)
{
    int8_t  r;
    uint8_t i;

    printf("[ST] ---- start ----\r\n");

    close(0u);
    HAL_Delay(20u);

    r = socket(0u, Sn_MR_TCP, NET_TCP_CMD_PORT, 0u);
    printf("[ST] socket() = %d\r\n", (int)r);

    for (i = 0u; i < 8u; i++)
    {
        HAL_Delay(10u);
        printf("[ST] SR=0x%02X MR=0x%02X CR=0x%02X\r\n",
               (unsigned)getSn_SR(0u), (unsigned)getSn_MR(0u), (unsigned)getSn_CR(0u));
    }

    r = listen(0u);
    printf("[ST] listen() = %d\r\n", (int)r);

    for (i = 0u; i < 8u; i++)
    {
        HAL_Delay(10u);
        printf("[ST] SR=0x%02X\r\n", (unsigned)getSn_SR(0u));
    }

    printf("[ST] ---- end ----\r\n");
}
