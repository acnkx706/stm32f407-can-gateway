/**
  ******************************************************************************
  * @file    web_srv.c
  * @brief   HTTP 服务器（W5500 socket 2~7，端口 80，页面存在 Flash 里）
  ******************************************************************************
  * ★ 做什么
  *   六个 socket 同时监听 80 端口，可以同时服务六个浏览器连接。
  *   HTML 页面是编译进 Flash 的常量字符串，请求来了直接拷进发送缓冲，
  *   不再每次现场拼字符串。实时数值由页面脚本每秒到 /data 取一次。
  *
  * ★ 为什么页面放 Flash 而不是每次拼
  *   拼一次要占 1.6 KB RAM 和几毫秒 CPU；页面本身一个字都不会变。
  *   存 Flash 后不占 RAM、发送还快，改页面只要改这个字符串再烧一次。
  *
  * ★ 影响什么
  *   占满 socket 2~7（6 个），每个 2 KB 收发缓冲。
  *   一个 socket 只能扛一条连接，所以并发上限就是 6。
  *
  * ★ 怎么验证
  *   浏览器打开 http://192.168.10.50/ 能看到数据；
  *   同时开多个标签页都能刷出来；串口打印 [WEB] sX GET /
  ******************************************************************************
  */

#include <string.h>

#include "web_srv.h"
#include "w5500_port.h"
#include "usart.h"
#include "main.h"
#include "modbus_tcp.h"
#include "net_tcp.h"        /* 2026-11-24：POST 认证要用 NET_TCP_TOKEN */

#include "wizchip_conf.h"
#include "socket.h"

/* ★ 从 2 号 socket 开始，一共用 6 个（2~7）
 *    0 号回显、1 号 Modbus，剩下的全给网页 */
#define WEB_SN_FIRST    2u
#define WEB_SN_COUNT    6u

/* ★ 轮询间隔，单位毫秒
 *    每毫秒只轮询一个 socket，六个轮流来，每个约 6 毫秒访问一次。
 *    这样 SPI 负载和 socket 数量无关，加 socket 不会加重总线压力 */
#define WEB_POLL_MS     1u

/* ★ 每个 socket 的请求缓冲
 *    Chrome 的 GET 请求带 User-Agent 和 Accept，大约 500 字节，
 *    留 512 勉强够，这里放到 640 保险 */
#define WEB_RX_BUF      640u

/* ★ 整帧发送缓冲，必须小于 socket 的 2 KB 发送缓冲 */
#define WEB_TX_BUF      2000u

/* ★ JSON 缓冲，一百来字节够用 */
/* ★ 2026-11-24 审计修：192 -> 320
 * · 今天补了 8 个 JSON 键（vb/lx/ax/ay/az/gx/gy/gz），按典型值已经 161 字节，
 *   只剩 31 字节余量；参数区写成 5 位数、计数再涨几位就顶爆
 * · 320 给到一倍余量，代价只有 128 字节 RAM
 */
#define WEB_JSON_BUF    320u

/* ★ HTTP 状态行 */
#define WEB_HDR_OK      "HTTP/1.1 200 OK\r\n"
#define WEB_HDR_400     "HTTP/1.1 400 Bad Request\r\n"
#define WEB_HDR_404     "HTTP/1.1 404 Not Found\r\n"
#define WEB_HDR_500     "HTTP/1.1 500 Internal Server Error\r\n"
#define WEB_HDR_204     "HTTP/1.1 204 No Content\r\n"

/* ★ 网页正文，编译进 Flash
 *    · 数字位置全部写 --，由页面脚本每秒填一次
 *    · 属性值不加引号，省掉 C 字符串里的转义
 *    · 整篇约 1.1 KB，加上响应头仍然一帧能发完
 */
static const char S_PAGE[] =
"<!doctype html><html><head><meta charset=utf-8><title>F407 Gateway</title>"
"<style>"
"body{background:#111;color:#ddd;font:14px/1.7 monospace;margin:18px}"
"h1{color:#7f7;font-size:18px;margin:0 0 12px}"
"table{border-collapse:collapse}"
"td{padding:3px 14px;border-bottom:1px solid #333}"
".k{color:#888}.v{color:#ff0;text-align:right}"
"</style></head><body>"
"<h1>STM32F407 CAN Gateway</h1><table>"
"<tr><td class=k>Temperature<td class=v id=t>--"
"<tr><td class=k>Humidity<td class=v id=h>--"
"<tr><td class=k>Pressure<td class=v id=p>--"
"<tr><td class=k>CAN frames<td class=v id=rf>--"
"<tr><td class=k>I2C errors<td class=v id=er>--"
"<tr><td class=k>Modbus requests<td class=v id=mq>--"
"<tr><td class=k>Modbus exceptions<td class=v id=me>--"
"</table><p class=k>Param regs 4..9 = <span id=pr>--</span></p>"
"<script>"
"function u(){fetch('/data').then(function(r){return r.json()})"
".then(function(d){"
"var g=function(i){return document.getElementById(i)};"
"g('t').textContent=d.t+' C';g('h').textContent=d.h+' %';"
"g('p').textContent=d.p+' hPa';g('rf').textContent=d.rf;"
"g('er').textContent=d.er;g('mq').textContent=d.mq;"
"g('me').textContent=d.me;g('pr').textContent=d.pr;})}"
"u();setInterval(u,1000);"
"</script></body></html>";

volatile uint32_t g_web_req_cnt  = 0u;
volatile uint32_t g_web_rx_bytes = 0u;
volatile uint32_t g_web_tx_bytes = 0u;
volatile uint8_t  g_web_busy     = 0u;

/* ★ 每个 socket 各自一份状态
 *    · 请求缓冲必须分开，不然两个连接同时收到数据会互相踩
 *    · drain 表示响应已交给芯片、等发送缓冲排空后再关连接 */
typedef struct
{
    uint8_t  rx[WEB_RX_BUF];
    uint16_t rx_len;
    uint8_t  drain;
    uint8_t  closed_cnt;
    uint32_t closed_tick;
    uint32_t reopen_tick;
} Web_Chan_t;

static Web_Chan_t s_ch[WEB_SN_COUNT];

/* 发送缓冲和 JSON 缓冲共用一份
 * 每次只处理一个 socket 的请求，处理完立刻 send，不会互相覆盖 */
static uint8_t s_tx[WEB_TX_BUF];
static uint8_t s_json[WEB_JSON_BUF];

/* 拼字符串的当前输出目标 */
static uint8_t *s_out;
static uint16_t s_out_max;
static uint16_t s_len;

/* ─────────────────── 拼字符串的工具函数 ─────────────────── */

/* ★ 2026-11-24 审计修：溢出要能被发现
 * · 原来悄悄截断，JSON 少一个右花括号就变成非法，网页和上位机同时停止刷新，
 *   设备侧却什么都不会报 —— 这种"安静的错误"最难查
 * · 加一个标志，Web_BuildJson 之后由调用方检查并回 500
 */
static uint8_t s_json_overflow;

static void CatCh(char c)
{
    if (s_len < (uint16_t)(s_out_max - 1u))
    {
        s_out[s_len++] = (uint8_t)c;
        s_out[s_len] = 0u;
    }
    else
    {
        s_json_overflow = 1u;
    }
}

static void Cat(const char *s)
{
    while (*s != '\0')
    {
        CatCh(*s);
        s++;
    }
}

static void CatDigit(uint8_t d)
{
    CatCh((char)('0' + (d % 10u)));
}

/* ★ 无符号整数转十进制，先取个位倒存再倒着输出 */
static void CatU32(uint32_t v)
{
    char tmp[11];
    uint8_t n = 0u;

    if (v == 0u)
    {
        CatDigit(0u);
        return;
    }

    while ((v > 0u) && (n < 10u))
    {
        tmp[n] = (char)('0' + (v % 10u));
        v /= 10u;
        n++;
    }

    while (n > 0u)
    {
        n--;
        CatCh(tmp[n]);
    }
}

/* ★ 放大 100 倍的定点数，1893 输出成 18.93 */
static void CatFix2(int32_t v)
{
    uint32_t a;

    if (v < 0)
    {
        Cat("-");
        a = (uint32_t)(-v);
    }
    else
    {
        a = (uint32_t)v;
    }

    CatU32(a / 100u);
    Cat(".");
    CatDigit((uint8_t)((a / 10u) % 10u));
    CatDigit((uint8_t)(a % 10u));
}

/* ★ 放大 10 倍的定点数，426 输出成 42.6 */
static void CatFix1(int32_t v)
{
    uint32_t a;

    if (v < 0)
    {
        Cat("-");
        a = (uint32_t)(-v);
    }
    else
    {
        a = (uint32_t)v;
    }

    CatU32(a / 10u);
    Cat(".");
    CatDigit((uint8_t)(a % 10u));
}

/* ★ 气压是 32 位，拆在寄存器 2 和 3 里，高字在前 */
/* ★ 输出一个有符号整数（2026-11-24 加）
 * · CatU32 只认无符号，加速度是 int16，直接传进去会变成 65xxx
 * · 只是给负号开个口，正数仍走 CatU32
 */
static void CatS32(int32_t v)
{
    if (v < 0)
    {
        CatCh('-');
        v = -v;
    }

    CatU32((uint32_t)v);
}

static uint32_t Web_GetPress(void)
{
    uint32_t hi = (uint32_t)Modbus_GetReg(2u);
    uint32_t lo = (uint32_t)Modbus_GetReg(3u);

    return ((hi << 16) | lo);
}

/* ★ 给页面脚本用的 JSON，字段名尽量短，省字节 */
static uint16_t Web_BuildJson(void)
{
    uint16_t i;

    s_out = s_json;
    s_out_max = (uint16_t)sizeof(s_json);
    s_len = 0u;
    s_json_overflow = 0u;       /* 2026-11-24：每次重新计溢出 */

    Cat("{\"t\":");
    CatFix2((int32_t)Modbus_GetReg(0u));
    Cat(",\"h\":");
    CatFix1((int32_t)Modbus_GetReg(1u));
    Cat(",\"p\":");
    /* ★ 2026-11-24 复核：这里本来就是对的，不要除 10
     * · 审计报告建议"再除 10"，按它的改法实测 /data 的 p 从 1028.28 掉到 102.84，
     *   说明原代码已经是对的
     * · 单位链：rec.press(0.1 Pa) → 0x101 帧 p_pa = rec.press/10 (Pa)
     *   → Web_GetPress() 返回 Pa → CatFix2 除以 100 = 百Pa 的十进制表示，
     *   而 1 hPa = 100 Pa，所以 CatFix2(103035) 给出的 1030.35 正好是 hPa
     * · 审计那条自己标了"未上板实测"，属于纸面推导的误报 */
    CatFix2((int32_t)Web_GetPress());
    Cat(",\"rf\":");
    CatU32((uint32_t)Modbus_GetReg(10u));
    Cat(",\"er\":");
    CatU32((uint32_t)Modbus_GetReg(12u));
    Cat(",\"mq\":");
    CatU32((uint32_t)Modbus_GetReg(13u));
    Cat(",\"me\":");
    CatU32((uint32_t)Modbus_GetReg(14u));
    /* ★ 2026-11-24 加：0x103 帧的四个量
     * · 单位和 Modbus 一致，全是原始值，换算留给前端
     * · 加速度按有符号输出，直接给负值 */
    Cat(",\"vb\":");
    CatU32((uint32_t)Modbus_GetReg(16u));
    Cat(",\"lx\":");
    CatU32((uint32_t)Modbus_GetReg(17u));
    Cat(",\"ax\":");
    CatS32((int32_t)(int16_t)Modbus_GetReg(18u));
    Cat(",\"ay\":");
    CatS32((int32_t)(int16_t)Modbus_GetReg(19u));
    Cat(",\"az\":");
    CatS32((int32_t)(int16_t)Modbus_GetReg(20u));
    Cat(",\"gx\":");
    CatS32((int32_t)(int16_t)Modbus_GetReg(21u));
    Cat(",\"gy\":");
    CatS32((int32_t)(int16_t)Modbus_GetReg(22u));
    Cat(",\"gz\":");
    CatS32((int32_t)(int16_t)Modbus_GetReg(23u));
    Cat(",\"pr\":\"");

    for (i = 0u; i < 6u; i++)
    {
        if (i != 0u)
        {
            Cat(", ");
        }
        CatU32((uint32_t)Modbus_GetReg((uint16_t)(4u + i)));
    }

    Cat("\"}");

    return s_len;
}

/* ★ 拼整帧：状态行 + 头 + 空行 + 正文
 * Content-Length 必须和正文一模一样，少了浏览器一直等，多了会截断 */
static uint16_t Web_Frame(const char *status, const char *ctype,
                          const uint8_t *body, uint16_t body_len)
{
    uint16_t i;

    s_out = s_tx;
    s_out_max = (uint16_t)sizeof(s_tx);
    s_len = 0u;

    Cat(status);
    Cat("Content-Type: ");
    Cat(ctype);
    Cat("\r\nContent-Length: ");
    CatU32((uint32_t)body_len);
    Cat("\r\nConnection: close\r\nCache-Control: no-store\r\n\r\n");

    for (i = 0u; (i < body_len) && (s_len < (uint16_t)(sizeof(s_tx) - 1u)); i++)
    {
        s_tx[s_len++] = body[i];
    }
    s_tx[s_len] = 0u;

    return s_len;
}

/* ─────────────────── socket 状态机 ─────────────────── */

static uint8_t Web_ReadSr(uint8_t sn)
{
    uint8_t a = getSn_SR(sn);
    uint8_t b = getSn_SR(sn);
    uint8_t c = getSn_SR(sn);

    if (a == b)
    {
        return a;
    }
    if (a == c)
    {
        return a;
    }
    if (b == c)
    {
        return b;
    }
    if (a != SOCK_CLOSED)
    {
        return a;
    }
    if (b != SOCK_CLOSED)
    {
        return b;
    }

    return c;
}

/* ★ 请求头收全了没有：HTTP 请求头以空行结束 */
static uint8_t Web_ReqComplete(Web_Chan_t *c)
{
    uint16_t i;

    if (c->rx_len < 4u)
    {
        return 0u;
    }

    uint16_t hdr_end = 0u;

    for (i = 0u; i < (uint16_t)(c->rx_len - 3u); i++)
    {
        if ((c->rx[i] == '\r') && (c->rx[i + 1u] == '\n') &&
            (c->rx[i + 2u] == '\r') && (c->rx[i + 3u] == '\n'))
        {
            hdr_end = (uint16_t)(i + 4u);
            break;
        }
    }

    if (hdr_end == 0u)
    {
        return 0u;
    }

    /* ★ 2026-11-24 审计修：POST 必须等 body 收齐才算完整请求
     * · 原来只看请求头有没有结束。TCP 把 header 和 body 拆成两段时
     *   （网络稍慢、包稍大、中间有代理都会），第一次处理时 body_len=0，
     *   于是回 400 no body 并把连接关掉，合法的写参数请求被丢掉
     * · 这里从请求头里找 Content-Length，收到那么多字节才算完整
     * · 找不到 Content-Length 就当 0，行为退回原来的"头齐就算完整" */
    if ((c->rx_len >= 5u) && (memcmp(c->rx, "POST ", 5u) == 0))
    {
        static const char key[] = "Content-Length:";
        uint16_t klen = (uint16_t)(sizeof(key) - 1u);
        uint16_t k;
        uint32_t clen = 0u;
        uint8_t  got_cl = 0u;

        for (k = 0u; (uint16_t)(k + klen) <= hdr_end; k++)
        {
            if (memcmp(&c->rx[k], key, klen) == 0)
            {
                uint16_t m = (uint16_t)(k + klen);

                while ((m < hdr_end) && ((c->rx[m] == ' ') || (c->rx[m] == '\t')))
                {
                    m++;
                }
                while ((m < hdr_end) && (c->rx[m] >= '0') && (c->rx[m] <= '9'))
                {
                    clen = clen * 10u + (uint32_t)(c->rx[m] - '0');
                    m++;
                }
                got_cl = 1u;
                break;
            }
        }

        if (got_cl != 0u)
        {
            if ((uint32_t)(c->rx_len - hdr_end) < clen)
            {
                return 0u;          /* body 还没收齐，让调用方继续 recv */
            }
        }
    }

    return 1u;
}

/* ★ 把一个请求变成响应并交给芯片
 *    send 只是把数据拷进芯片的发送缓冲，真正上网线还在后面，
 *    所以发完不能马上关连接，要等缓冲排空 */
static void Web_SendResponse(uint8_t sn, Web_Chan_t *c, uint16_t len)
{
    if ((len == 0u) || (len > WEB_TX_BUF))
    {
        return;
    }

    if (send(sn, s_tx, len) > 0)
    {
        g_web_tx_bytes += len;
        g_web_busy = 1u;
        c->drain = 1u;
        printf("[WEB] s%u sent %u bytes\r\n", (unsigned)sn, (unsigned)len);
    }
}

/* ★ 解析请求并生成响应 */
/* ★ 处理 POST /set（2026-11-24 加）
 * · 做什么：把表单里的 reg/value 写进 Modbus 参数寄存器 4~9
 * · 为什么用 POST 不用 GET：GET 会被浏览器和中间代理缓存，改参数的事不能缓存
 * · 请求体格式 r=<寄存器号>&v=<值>，十进制；不认十六进制，表单里本来也是十进制
 * · 怎么验证：curl -d "r=4&v=100" http://192.168.10.50/set，再读 Modbus 寄存器 4
 */
static void Web_HandlePost(uint8_t sn, Web_Chan_t *c)
{
    const uint8_t *p;
    const uint8_t *body = 0;
    uint16_t body_len = 0u;
    uint16_t i;
    uint16_t len;
    uint16_t reg = 0u;
    uint16_t val = 0u;
    uint8_t  got_r = 0u;
    uint8_t  got_v = 0u;
    uint8_t  ok;

    /* 路径必须是 /set 或 /set?t=xxx */
    p = &c->rx[5];

    if ((c->rx_len < 11u) || (memcmp(p, "/set", 4u) != 0))
    {
        len = Web_Frame(WEB_HDR_404, "text/plain", (const uint8_t *)"path", 4u);
        Web_SendResponse(sn, c, len);
        return;
    }

    /* ★ 2026-11-24 加：改参数必须带口令
     * · 网页这条路原来完全没有认证，任何人 curl 一下就能改 Modbus 参数
     * · 口令和命令通道的 AUTH 共用一把（NET_TCP_TOKEN）
     * · 放 query（/set?t=xxx）而不是 header：少解析二十行，演示级防护够用
     * · 局限：口令会出现在 URL 里，中间代理的日志能看到；量产应换 header + TLS
     * · 在整个请求里搜而不是只搜 request line：浏览器把 query 放在请求行，
     *   但 curl -d 有时把参数塞进 body，两个位置都认更省事 */
    {
        static const char key[] = "t=" NET_TCP_TOKEN;
        uint16_t klen = (uint16_t)(sizeof(key) - 1u);
        uint16_t k;
        uint8_t  ok_tok = 0u;

        for (k = 0u; (uint16_t)(k + klen) <= c->rx_len; k++)
        {
            if (memcmp(&c->rx[k], key, klen) == 0)
            {
                ok_tok = 1u;
                break;
            }
        }

        if (ok_tok == 0u)
        {
            printf("[WEB] s%u POST /set 无口令，拒绝\r\n", (unsigned)sn);
            len = Web_Frame(WEB_HDR_400, "text/plain",
                            (const uint8_t *)"need token", 10u);
            Web_SendResponse(sn, c, len);
            return;
        }
    }

    /* 找头结束位置，body 从它后面开始 */
    for (i = 0u; i < (uint16_t)(c->rx_len - 3u); i++)
    {
        if ((c->rx[i] == '\r') && (c->rx[i + 1u] == '\n') &&
            (c->rx[i + 2u] == '\r') && (c->rx[i + 3u] == '\n'))
        {
            body = &c->rx[i + 4u];
            body_len = (uint16_t)(c->rx_len - (i + 4u));
            break;
        }
    }

    if (body_len == 0u)
    {
        len = Web_Frame(WEB_HDR_400, "text/plain", (const uint8_t *)"no body", 7u);
        Web_SendResponse(sn, c, len);
        return;
    }

    /* 解析 r= 和 v=，两个都必须出现 */
    i = 0u;
    while (i < body_len)
    {
        if ((body[i] == 'r') && ((uint16_t)(i + 1u) < body_len) && (body[i + 1u] == '='))
        {
            i = (uint16_t)(i + 2u);
            while ((i < body_len) && (body[i] >= '0') && (body[i] <= '9'))
            {
                reg = (uint16_t)(reg * 10u + (uint16_t)(body[i] - '0'));
                i++;
                got_r = 1u;
            }
            continue;
        }

        if ((body[i] == 'v') && ((uint16_t)(i + 1u) < body_len) && (body[i + 1u] == '='))
        {
            i = (uint16_t)(i + 2u);
            while ((i < body_len) && (body[i] >= '0') && (body[i] <= '9'))
            {
                val = (uint16_t)(val * 10u + (uint16_t)(body[i] - '0'));
                i++;
                got_v = 1u;
            }
            continue;
        }

        i++;
    }

    if ((got_r == 0u) || (got_v == 0u))
    {
        len = Web_Frame(WEB_HDR_400, "text/plain", (const uint8_t *)"need r and v", 12u);
        Web_SendResponse(sn, c, len);
        return;
    }

    ok = Modbus_SetReg(reg, val);

    printf("[WEB] s%u POST set r=%u v=%u -> %s\r\n", (unsigned)sn,
           (unsigned)reg, (unsigned)val, (ok != 0u) ? "ok" : "reject");

    len = Web_Frame((ok != 0u) ? WEB_HDR_OK : WEB_HDR_400, "text/plain",
                    (const uint8_t *)((ok != 0u) ? "ok" : "bad"), 3u);
    Web_SendResponse(sn, c, len);
}

static void Web_Handle(uint8_t sn, Web_Chan_t *c)
{
    const uint8_t *path;
    uint16_t path_len = 0u;
    uint16_t body_len;
    uint16_t len;

    /* 缓冲塞满还没见到空行，说明这是畸形请求，直接回 400 关掉 */
    if ((Web_ReqComplete(c) == 0u) && (c->rx_len < WEB_RX_BUF))
    {
        return;
    }

    g_web_req_cnt++;

    if (c->rx_len < 5u)
    {
        len = Web_Frame(WEB_HDR_400, "text/plain", (const uint8_t *)"bad", 3u);
        Web_SendResponse(sn, c, len);
        return;
    }

    /* ★ 2026-11-24 改：POST 也放行，交给下面的路径分支去处理
     * · 原来只认 GET，浏览器一张表单提交过来直接吃 404，看不出哪里错 */
    if ((memcmp(c->rx, "GET ", 4u) != 0) && (memcmp(c->rx, "POST ", 5u) != 0))
    {
        len = Web_Frame(WEB_HDR_404, "text/plain", (const uint8_t *)"method", 6u);
        Web_SendResponse(sn, c, len);
        return;
    }

    if (memcmp(c->rx, "POST ", 5u) == 0)
    {
        Web_HandlePost(sn, c);
        return;
    }

    path = &c->rx[4];
    while (((uint16_t)(4u + path_len) < c->rx_len) && (path[path_len] != ' '))
    {
        path_len++;
    }

    /* 只打印路径本身
     * 注意：rx 缓冲是 recv 直接填的，末尾没有字符串结束符，
     * 用百分号 s 直接打印会一路打到内存里碰巧遇到的 0，
     * 把整段请求头（含浏览器 UA）都刷出来。115200 下要几十毫秒，
     * 主循环会被拖慢，所以先拷到带结束符的临时串再打印 */
    {
        char     line[48];
        uint16_t k = path_len;

        if (k > (uint16_t)(sizeof(line) - 1u))
        {
            k = (uint16_t)(sizeof(line) - 1u);
        }

        (void)memcpy(line, path, k);
        line[k] = '\0';
        printf("[WEB] s%u GET %s\r\n", (unsigned)sn, line);
    }

    if ((path_len == 1u) && (path[0] == '/'))
    {
        /* 首页：正文直接从 Flash 拷，不占 RAM、不重复拼 */
        body_len = (uint16_t)(sizeof(S_PAGE) - 1u);
        len = Web_Frame(WEB_HDR_OK, "text/html; charset=utf-8",
                        (const uint8_t *)S_PAGE, body_len);
        Web_SendResponse(sn, c, len);
    }
    else if ((path_len == 5u) && (memcmp(path, "/data", 5u) == 0))
    {
        body_len = Web_BuildJson();

        /* ★ 2026-11-24 审计修：截断就回 500，别发半截 JSON
         * · 半截 JSON 让浏览器和上位机同时解析失败，
         *   现象是"两边同时不刷新"，而设备侧完全看不出来
         * · 宁可明确报错，也不要让上层拿到解析不了的响应还以为是网络问题 */
        if (s_json_overflow != 0u)
        {
            printf("[WEB] s%u /data JSON 超长（%u 字节）已截断\r\n",
                   (unsigned)sn, (unsigned)body_len);
            len = Web_Frame(WEB_HDR_500, "text/plain",
                            (const uint8_t *)"json too long", 13u);
            Web_SendResponse(sn, c, len);
            return;
        }

        len = Web_Frame(WEB_HDR_OK, "application/json", s_json, body_len);
        Web_SendResponse(sn, c, len);
    }
    else if ((path_len >= 12u) && (memcmp(path, "/favicon.ico", 12u) == 0))
    {
        len = Web_Frame(WEB_HDR_204, "text/plain", NULL, 0u);
        Web_SendResponse(sn, c, len);
    }
    else
    {
        len = Web_Frame(WEB_HDR_404, "text/plain", (const uint8_t *)"notfound",
                        8u);
        Web_SendResponse(sn, c, len);
    }
}

/* ★ 轮询一个 socket */
static void Web_ChanPoll(uint8_t idx)
{
    uint8_t     sn = (uint8_t)(WEB_SN_FIRST + idx);
    Web_Chan_t *c  = &s_ch[idx];
    uint8_t     sr;
    uint16_t    n;
    int32_t     ret;

    /* 上一个响应还没发完，先等发送缓冲排空 */
    if (c->drain != 0u)
    {
        if (getSn_TX_FSR(sn) == getSn_TxMAX(sn))
        {
            c->drain = 0u;
            c->rx_len = 0u;
            (void)close(sn);
            g_web_busy = 0u;
        }
        return;
    }

    sr = Web_ReadSr(sn);

    switch (sr)
    {
    case SOCK_ESTABLISHED:
        c->closed_cnt = 0u;

        n = getSn_RX_RSR(sn);
        if (n > 0u)
        {
            if (n > (uint16_t)(WEB_RX_BUF - c->rx_len))
            {
                n = (uint16_t)(WEB_RX_BUF - c->rx_len);
            }

            if (n > 0u)
            {
                ret = recv(sn, &c->rx[c->rx_len], n);
                if (ret > 0)
                {
                    c->rx_len = (uint16_t)(c->rx_len + (uint16_t)ret);
                    g_web_rx_bytes += (uint32_t)ret;
                }
            }

            Web_Handle(sn, c);
        }
        break;

    case SOCK_CLOSE_WAIT:
        /* disconnect() 内部有无超时死等，一律用有界的 close() */
        c->closed_cnt = 0u;
        c->rx_len = 0u;
        (void)close(sn);
        break;

    case SOCK_CLOSED:
        if (c->closed_cnt == 0u)
        {
            c->closed_tick = HAL_GetTick();
        }
        c->closed_cnt++;

        if ((c->closed_cnt >= 5u) &&
            ((HAL_GetTick() - c->closed_tick) >= 20u) &&
            ((HAL_GetTick() - c->reopen_tick) >= 200u))
        {
            c->closed_cnt = 0u;
            c->reopen_tick = HAL_GetTick();
            c->rx_len = 0u;

            if (socket(sn, Sn_MR_TCP, WEB_SRV_PORT, 0u) == (int8_t)sn)
            {
                (void)listen(sn);
            }
        }
        break;

    case SOCK_INIT:
        c->closed_cnt = 0u;
        (void)listen(sn);
        break;

    default:
        /* SOCK_LISTEN 及过渡状态，什么都不碰 */
        c->closed_cnt = 0u;
        break;
    }
}

int8_t Web_SrvInit(void)
{
    uint8_t i;
    uint8_t ok = 0u;

    (void)memset(s_ch, 0, sizeof(s_ch));
    g_web_busy = 0u;

    /* 六个 socket 全部监听 80 端口
     * 一个 socket 只能接一条连接，所以并发上限就是个数 */
    for (i = 0u; i < WEB_SN_COUNT; i++)
    {
        uint8_t sn = (uint8_t)(WEB_SN_FIRST + i);

        close(sn);

        if (socket(sn, Sn_MR_TCP, WEB_SRV_PORT, 0u) != (int8_t)sn)
        {
            continue;
        }

        if (listen(sn) == SOCK_OK)
        {
            ok++;
        }
    }

    return (ok == 0u) ? -1 : 0;
}

void Web_SrvPoll(void)
{
    static uint32_t s_last = 0u;
    static uint8_t  s_rr   = 0u;

    if ((HAL_GetTick() - s_last) < WEB_POLL_MS)
    {
        return;
    }
    s_last = HAL_GetTick();

    /* 每次只查一个 socket，六个轮流
     * 这样 SPI 访问频率固定，和 socket 数量无关 */
    Web_ChanPoll(s_rr);
    s_rr = (uint8_t)((s_rr + 1u) % WEB_SN_COUNT);
}
