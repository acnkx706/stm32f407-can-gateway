/**
  ******************************************************************************
  * @file    modbus_tcp.c
  * @brief   Modbus TCP 从站（W5500 socket 1，端口 502）
  ******************************************************************************
  * ★ 做什么
  *   在已建立的 TCP 连接上按 Modbus 格式收发：解析 MBAP 头加 PDU，
  *   处理 0x01 / 0x03 / 0x05 / 0x06 / 0x10 五个功能码，拼出响应帧。
  *
  * ★ 为什么先做这五个
  *   现场绝大多数读写都落在它们身上：读数据（0x03）、读开关（0x01）、
  *   写参数（0x06 / 0x10）、写开关（0x05）。其余功能码先一律回异常码。
  *
  * ★ 影响什么
  *   只占 socket 1，socket 0 的回显代码一行没动。
  *   线圈 0 驱动板上 LED1，写线圈能看见灯亮灭。
  *
  * ★ 怎么验证
  *   modbus_test.ps1 全部 PASS；写线圈 0 时板上的 LED 跟着亮灭。
  ******************************************************************************
  */

#include <string.h>

#include "modbus_tcp.h"
#include "w5500_port.h"
#include "usart.h"
#include "main.h"
#include "led.h"

#include "wizchip_conf.h"
#include "socket.h"

/* ★ socket 编号
 *    W5500 共 8 个 socket，0 号已经被回显占用，这里用 1 号 */
#define MB_SN           1u

/* ★ 轮询最小间隔，单位毫秒
 *    高速 SPI 下无节制轮询会压垮芯片的命令仲裁，
 *    回显服务器就是吃了这个亏才加限速，这里直接沿用同一做法 */
#define MB_RATE_MS      2u

/* ★ 发送缓冲
 *    最大响应 = MBAP 7 字节 + 功能码 1 + 字节数 1 + 125 个寄存器 250 = 259
 *    读线圈最大 250 字节位图，比这个还小，取 300 够用 */
#define MB_TX_BUF       300u

/* ★ 累积接收缓冲
 *    TCP 是字节流，一次 recv 可能拿到半个请求，也可能两个请求粘连，
 *    所以必须先把字节攒够一整帧再解析 */
#define MB_RX_BUF       320u

/* ★ 长度字段上限
 *    从站合法请求里最长的是写多个寄存器，PDU 不超过 253 字节，
 *    超过这个数说明帧结构已经错了，直接丢弃这个连接上的残留数据 */
#define MB_LEN_MAX      254u

/* ★ MBAP 报文头各字段相对帧首的字节偏移 */
#define MB_O_TRANS      0u
#define MB_O_PROTO      2u
#define MB_O_LEN        4u
#define MB_O_UNIT       6u
#define MB_MBAP_LEN     7u

/* ★ Modbus 异常码 */
#define MB_EXC_ILLEGAL_FUNC   0x01u     /* 功能码不支持 */
#define MB_EXC_ILLEGAL_ADDR   0x02u     /* 寄存器或线圈地址越界 */
#define MB_EXC_ILLEGAL_VALUE  0x03u     /* 数量、字节数、线圈值非法 */

/* ★ 写单个线圈的合法取值，只有这两个，其它一律算参数错误 */
#define MB_COIL_ON      0xFF00u
#define MB_COIL_OFF     0x0000u

volatile uint32_t g_mb_req_cnt   = 0u;
volatile uint32_t g_mb_exc_cnt   = 0u;
volatile uint32_t g_mb_rx_bytes  = 0u;
volatile uint32_t g_mb_tx_bytes  = 0u;
volatile uint8_t  g_mb_connected = 0u;

static Modbus_Data_t s_data;

/* ★ 参数区存储：只有地址 4 ~ 9 会被写进来
 *    · 掉电不保存，重启回 0，掉电保存是下一步的事
 *    · 用 16 个而不是 6 个，是为了让地址索引直接当数组下标，不用减偏移 */
static uint16_t s_hold[MODBUS_REG_COUNT];

/* ★ 线圈位图：8 个线圈占 1 字节，bit0 就是线圈 0 */
static uint8_t s_coil;

static uint8_t  s_rx[MB_RX_BUF];
static uint16_t s_rx_len = 0u;
static uint8_t  s_tx[MB_TX_BUF];

void Modbus_SetData(const Modbus_Data_t *d)
{
    if (d == NULL)
    {
        return;
    }

    s_data = *d;
}

/* ★ 状态字：让主站一次读出来就知道链路健不健康
 *    · bit0 = 网线已连接
 *    · bit1 = Modbus 主站已连接
 *    · bit2 = 收到过 I2C 数据
 *    · bit3 = I2C 没有错误
 *    位定义写在这里，主站那边按位解析，改位序要两边一起改
 */
static uint16_t Mb_Status(void)
{
    uint16_t v = 0u;

    if (s_data.link_up != 0u)
    {
        v |= 0x0001u;
    }
    if (g_mb_connected != 0u)
    {
        v |= 0x0002u;
    }
    if (s_data.i2c_rx != 0u)
    {
        v |= 0x0004u;
    }
    if (s_data.i2c_err == 0u)
    {
        v |= 0x0008u;
    }

    return v;
}


/* ★ 写一个保持寄存器（2026-11-24 加）
 * · 给网页的 POST 用：Modbus 自己写是在协议解析里直接改 s_hold，
 *   网页走的是另一条路，需要一个正规入口
 * · 只允许写参数区（和 0x06 功能码同一套规则），数据区仍归采集任务独占
 * · 返回 1 = 写入成功，0 = 地址不可写
 */
static uint8_t Mb_IsWritable(uint16_t addr);   /* 定义在后面，这里先声明 */

uint8_t Modbus_SetReg(uint16_t addr, uint16_t val)
{
    if (Mb_IsWritable(addr) == 0u)
    {
        return 0u;
    }

    s_hold[addr] = val;

    return 1u;
}
uint16_t Modbus_GetReg(uint16_t addr)
{
    /* 地址 4 ~ 9 是参数区，存在 s_hold 里，先于其它分支处理 */
    if ((addr >= MODBUS_WR_FIRST) && (addr <= MODBUS_WR_LAST))
    {
        return s_hold[addr];
    }

    switch (addr)
    {
    case 0u:
        return (uint16_t)s_data.temp_x100;

    case 1u:
        return s_data.humi_x10;

    /* 32 位数据要拆成两个 16 位寄存器，高字在前，主站按大端拼回 */
    case 2u:
        return (uint16_t)(s_data.press_pa >> 16);

    case 3u:
        return (uint16_t)(s_data.press_pa & 0xFFFFu);

    case 10u:
        return (uint16_t)(s_data.i2c_rx & 0xFFFFu);

    case 11u:
        return Mb_Status();

    case 12u:
        return (uint16_t)(s_data.i2c_err & 0xFFFFu);

    case 13u:
        return (uint16_t)(g_mb_req_cnt & 0xFFFFu);

    case 14u:
        return (uint16_t)(g_mb_exc_cnt & 0xFFFFu);

    case 15u:
        return (uint16_t)(g_mb_rx_bytes & 0xFFFFu);

    /* ★ 2026-11-24 加：0x103 帧带来的四个量
     * · 加速度是 int16 有符号，直接强转成 uint16 让主站按有符号解
     * · 单位都是原始值：mV、光敏 ADC 码、加速度 LSB，不做换算，
     *   换算规则写在代码里一旦改了主站就跟着崩，不如把原值给出去 */
    case 16u:
        return s_data.vbat_mv;

    case 17u:
        return s_data.light;

    case 18u:
        return (uint16_t)s_data.ax;

    case 19u:
        return (uint16_t)s_data.ay;

    case 20u:
        return (uint16_t)s_data.az;

    case 21u:
        return (uint16_t)s_data.gx;

    case 22u:
        return (uint16_t)s_data.gy;

    case 23u:
        return (uint16_t)s_data.gz;

    default:
        return 0u;
    }
}

/* ★ 判断一个寄存器地址能不能写
 *    · 只有 4 ~ 9 参数区可写
 *    · 传感器值 0 ~ 3 是实测数据，不允许主站覆盖
 *    · 状态计数 10 ~ 15 是板子自己维护的，也不允许写
 */
static uint8_t Mb_IsWritable(uint16_t addr)
{
    return ((addr >= MODBUS_WR_FIRST) && (addr <= MODBUS_WR_LAST)) ? 1u : 0u;
}

uint8_t Modbus_GetCoil(uint16_t addr)
{
    if (addr >= MODBUS_COIL_COUNT)
    {
        return 0u;
    }

    return (uint8_t)((s_coil >> addr) & 0x01u);
}

/* ★ 改一个线圈的值，并同步板上 LED
 *    · 线圈 0 直接控制 LED1，写它眼睛就能看见，不用示波器
 *    · LED 是低电平点亮，所以线圈为 1 时要输出低
 *    · 其余线圈只存位图，没有接执行器
 */
static void Mb_SetCoil(uint16_t addr, uint8_t on)
{
    if (addr >= MODBUS_COIL_COUNT)
    {
        return;
    }

    if (on != 0u)
    {
        s_coil = (uint8_t)(s_coil | (uint8_t)(1u << addr));
    }
    else
    {
        s_coil = (uint8_t)(s_coil & (uint8_t)(~(1u << addr)));
    }

    if (addr == 0u)
    {
        /* 注意：led.h 里的 LED1_ON 宏写的是 BRR 寄存器，那是 F1 的写法
         * F4 的 GPIO 没有 BRR，直接用会编译报 no field "BRR"
         * 所以这里直接调 HAL 函数，不碰那两个宏 */
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN,
                          (on != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

/* ★ 读 socket 状态，三次取多数
 *    · 高速 SPI 下单次读 Sn_SR 偶尔会返回 SOCK_CLOSED
 *    · 单次读错会让状态机误判，把正在监听的 socket 亲手关掉
 *    · 三取二能滤掉这种偶发误读，代价只是多两次寄存器读
 */
static uint8_t Mb_ReadSr(void)
{
    uint8_t a = getSn_SR(MB_SN);
    uint8_t b = getSn_SR(MB_SN);
    uint8_t c = getSn_SR(MB_SN);

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

/* ★ 拼一个异常响应
 *    · 功能码最高位置 1 表示这是异常，主站据此区分正常响应
 *    · 长度字段固定 3：单元标识 1 + 功能码 1 + 异常码 1
 *    · 返回整帧长度，固定 9 字节
 */
static uint16_t Mb_BuildException(const uint8_t *req, uint8_t fc,
                                  uint8_t code, uint8_t *resp)
{
    resp[0] = req[MB_O_TRANS];
    resp[1] = req[MB_O_TRANS + 1u];
    resp[MB_O_PROTO] = 0u;
    resp[MB_O_PROTO + 1u] = 0u;
    resp[MB_O_LEN] = 0u;
    resp[MB_O_LEN + 1u] = 3u;
    resp[MB_O_UNIT] = req[MB_O_UNIT];
    resp[MB_MBAP_LEN] = (uint8_t)(fc | 0x80u);
    resp[MB_MBAP_LEN + 1u] = code;

    g_mb_exc_cnt++;
    printf("[MB] exception 0x%02X\r\n", (unsigned)code);

    return 9u;
}

/* ★ 拼读保持寄存器的响应
 *    · 事务标识和单元标识原样抄回，主站靠事务标识配对请求与响应
 *    · 长度字段 = 1 + 1 + 1 + 2 * 数量，算错主站会认为帧不完整
 *    · 每个寄存器先发高字节再发低字节，这是 Modbus 规定的大端顺序
 */
/* ★ 2026-11-24 改：多一个 fc 参数
 * · 为什么：0x03（读保持寄存器）和 0x04（读输入寄存器）的数据源是同一张表，
 *   但响应里的功能码必须回显请求的那个，写死 0x03 会让 0x04 的响应被判成非法
 * · 影响什么：两个调用点都要传 fc
 */
static uint16_t Mb_BuildRead(const uint8_t *req, uint8_t fc, uint16_t start,
                             uint16_t qty, uint8_t *resp)
{
    uint16_t len;
    uint16_t i;
    uint16_t n;

    len = (uint16_t)(3u + 2u * qty);

    resp[0] = req[MB_O_TRANS];
    resp[1] = req[MB_O_TRANS + 1u];
    resp[MB_O_PROTO] = 0u;
    resp[MB_O_PROTO + 1u] = 0u;
    resp[MB_O_LEN] = (uint8_t)(len >> 8);
    resp[MB_O_LEN + 1u] = (uint8_t)(len & 0xFFu);
    resp[MB_O_UNIT] = req[MB_O_UNIT];
    resp[MB_MBAP_LEN] = fc;
    resp[MB_MBAP_LEN + 1u] = (uint8_t)(2u * qty);

    n = (uint16_t)(MB_MBAP_LEN + 2u);

    for (i = 0u; i < qty; i++)
    {
        uint16_t v = Modbus_GetReg((uint16_t)(start + i));

        resp[n] = (uint8_t)(v >> 8);
        resp[n + 1u] = (uint8_t)(v & 0xFFu);
        n = (uint16_t)(n + 2u);
    }

    return n;
}

/* ★ 拼读线圈的响应
 *    · 位图按低位在前打包：第一个线圈放第一个字节的 bit0
 *    · 顺序反了主站读到的开关状态会整体错位，而且不报错
 *    · 字节数 = (数量 + 7) / 8，向上取整
 */
static uint16_t Mb_BuildReadCoils(const uint8_t *req, uint16_t start,
                                  uint16_t qty, uint8_t *resp)
{
    uint16_t bytes = (uint16_t)((qty + 7u) / 8u);
    uint16_t len = (uint16_t)(3u + bytes);
    uint16_t i;

    resp[0] = req[MB_O_TRANS];
    resp[1] = req[MB_O_TRANS + 1u];
    resp[MB_O_PROTO] = 0u;
    resp[MB_O_PROTO + 1u] = 0u;
    resp[MB_O_LEN] = (uint8_t)(len >> 8);
    resp[MB_O_LEN + 1u] = (uint8_t)(len & 0xFFu);
    resp[MB_O_UNIT] = req[MB_O_UNIT];
    resp[MB_MBAP_LEN] = 0x01u;
    resp[MB_MBAP_LEN + 1u] = (uint8_t)bytes;

    for (i = 0u; i < bytes; i++)
    {
        resp[MB_MBAP_LEN + 2u + i] = 0u;
    }

    for (i = 0u; i < qty; i++)
    {
        if (Modbus_GetCoil((uint16_t)(start + i)) != 0u)
        {
            resp[MB_MBAP_LEN + 2u + (i / 8u)] |=
                (uint8_t)(1u << (i % 8u));
        }
    }

    return (uint16_t)(MB_MBAP_LEN + 2u + bytes);
}

/* ★ 拼写操作的响应
 *    · 0x06 和 0x05 的响应就是请求回送：功能码加地址加值
 *    · 0x10 只回起始地址和数量，不回数据
 *    · 三种情况的帧长都是 12 字节，长度字段都是 6，所以共用一个函数
 *    · 长度字段必须重写成 6，0x10 的请求长度字段是 7 + 2 * 数量
 */
static uint16_t Mb_BuildWriteResp(const uint8_t *req, uint8_t *resp)
{
    resp[0] = req[MB_O_TRANS];
    resp[1] = req[MB_O_TRANS + 1u];
    resp[MB_O_PROTO] = 0u;
    resp[MB_O_PROTO + 1u] = 0u;
    resp[MB_O_LEN] = 0u;
    resp[MB_O_LEN + 1u] = 6u;
    resp[MB_O_UNIT] = req[MB_O_UNIT];
    resp[MB_MBAP_LEN] = req[MB_MBAP_LEN];
    resp[MB_MBAP_LEN + 1u] = req[MB_MBAP_LEN + 1u];
    resp[MB_MBAP_LEN + 2u] = req[MB_MBAP_LEN + 2u];
    resp[MB_MBAP_LEN + 3u] = req[MB_MBAP_LEN + 3u];
    resp[MB_MBAP_LEN + 4u] = req[MB_MBAP_LEN + 4u];

    return 12u;
}

/* ★ 解析一帧请求并生成响应
 *    · 返回 0 表示这一帧不该回东西，比如协议标识不对
 *    · 协议标识必须是 0，否则收到的根本不是 Modbus 报文
 *    · 校验顺序按协议来：先数量、再地址，这样异常码不会乱
 */
static uint16_t Mb_Process(const uint8_t *req, uint16_t req_len,
                           uint8_t *resp, uint16_t resp_size)
{
    uint8_t  fc;
    uint16_t start;
    uint16_t qty;
    uint16_t val;
    uint16_t bytes;
    uint16_t i;

    /* 连功能码都没有，不回任何东西 */
    if (req_len < (uint16_t)(MB_MBAP_LEN + 1u))
    {
        return 0u;
    }

    if ((((uint16_t)req[MB_O_PROTO] << 8) | req[MB_O_PROTO + 1u]) != 0u)
    {
        return 0u;
    }

    fc = req[MB_MBAP_LEN];

    switch (fc)
    {
    case 0x01u:
        /* 读线圈请求固定 12 字节 */
        if (req_len < 12u)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        start = (uint16_t)(((uint16_t)req[8] << 8) | req[9]);
        qty = (uint16_t)(((uint16_t)req[10] << 8) | req[11]);

        if ((qty == 0u) || (qty > MODBUS_COIL_READ_MAX))
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        if (((uint32_t)start + (uint32_t)qty) > MODBUS_COIL_COUNT)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_ADDR, resp);
        }

        g_mb_req_cnt++;
        printf("[MB] 01 addr=%u qty=%u\r\n", (unsigned)start, (unsigned)qty);

        return Mb_BuildReadCoils(req, start, qty, resp);

    case 0x03u:
        /* 读保持寄存器请求固定 12 字节，短了说明帧被截断
         * 这种情况不能静默丢弃，否则主站会一直等不到回应而超时
         * 回异常码 0x03 让主站立刻知道参数有问题 */
        if (req_len < 12u)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        start = (uint16_t)(((uint16_t)req[8] << 8) | req[9]);
        qty = (uint16_t)(((uint16_t)req[10] << 8) | req[11]);

        if ((qty == 0u) || (qty > MODBUS_READ_MAX))
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        if (((uint32_t)start + (uint32_t)qty) > MODBUS_REG_COUNT)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_ADDR, resp);
        }

        if (resp_size < (uint16_t)(MB_MBAP_LEN + 2u + 2u * qty))
        {
            return 0u;
        }

        g_mb_req_cnt++;
        printf("[MB] 03 addr=%u qty=%u\r\n", (unsigned)start, (unsigned)qty);

        return Mb_BuildRead(req, fc, start, qty, resp);

    case 0x04u:
        /* ★ 2026-11-24 加：读输入寄存器
         * · 做什么：和 0x03 读同一张寄存器表，只是响应回显的功能码是 04
         * · 为什么：主站习惯把实时量放输入寄存器、参数放保持寄存器；
         *   本从站两类数据本来就是同一张表，硬拆成两套反而让人对不上号
         * · 影响什么：纯读，不碰 s_hold，没有副作用
         * · 怎么验证：发 01 04 00 00 00 10，回 01 04 20 加 32 字节，值应与 0x03 一致
         */
        if (req_len < 12u)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        start = (uint16_t)(((uint16_t)req[8] << 8) | req[9]);
        qty = (uint16_t)(((uint16_t)req[10] << 8) | req[11]);

        if ((qty == 0u) || (qty > MODBUS_READ_MAX))
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        if (((uint32_t)start + (uint32_t)qty) > MODBUS_REG_COUNT)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_ADDR, resp);
        }

        if (resp_size < (uint16_t)(MB_MBAP_LEN + 2u + 2u * qty))
        {
            return 0u;
        }

        g_mb_req_cnt++;
        printf("[MB] 04 addr=%u qty=%u\r\n", (unsigned)start, (unsigned)qty);

        return Mb_BuildRead(req, fc, start, qty, resp);
    case 0x05u:
        /* 写单个线圈：值只能是 0xFF00 开 或 0x0000 关 */
        if (req_len < 12u)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        start = (uint16_t)(((uint16_t)req[8] << 8) | req[9]);
        val = (uint16_t)(((uint16_t)req[10] << 8) | req[11]);

        if ((val != MB_COIL_ON) && (val != MB_COIL_OFF))
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        if (start >= MODBUS_COIL_COUNT)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_ADDR, resp);
        }

        Mb_SetCoil(start, (uint8_t)((val != MB_COIL_OFF) ? 1u : 0u));

        g_mb_req_cnt++;
        printf("[MB] 05 coil[%u]=%u\r\n", (unsigned)start,
               (unsigned)Modbus_GetCoil(start));

        return Mb_BuildWriteResp(req, resp);

    case 0x06u:
        /* 写单个保持寄存器：只有参数区可写 */
        if (req_len < 12u)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        start = (uint16_t)(((uint16_t)req[8] << 8) | req[9]);
        val = (uint16_t)(((uint16_t)req[10] << 8) | req[11]);

        if (Mb_IsWritable(start) == 0u)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_ADDR, resp);
        }

        s_hold[start] = val;

        g_mb_req_cnt++;
        printf("[MB] 06 reg[%u]=%u\r\n", (unsigned)start, (unsigned)val);

        return Mb_BuildWriteResp(req, resp);

    case 0x10u:
        /* 写多个保持寄存器：帧长 = 13 + 2 * 数量 */
        if (req_len < 13u)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        start = (uint16_t)(((uint16_t)req[8] << 8) | req[9]);
        qty = (uint16_t)(((uint16_t)req[10] << 8) | req[11]);
        bytes = req[12];

        if ((qty == 0u) || (qty > MODBUS_WRITE_MAX))
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        /* 字节数必须严格等于 2 乘数量，不然数据区怎么切都说不清 */
        if (bytes != (uint16_t)(2u * qty))
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        if (req_len < (uint16_t)(13u + 2u * qty))
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_VALUE, resp);
        }

        if (((uint32_t)start + (uint32_t)qty) > MODBUS_REG_COUNT)
        {
            return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_ADDR, resp);
        }

        /* 先全部检查再写：有一个地址不可写就整帧拒绝
         * 写一半再报错会让主站以为全部成功，参数出现半新半旧的状态 */
        for (i = 0u; i < qty; i++)
        {
            if (Mb_IsWritable((uint16_t)(start + i)) == 0u)
            {
                return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_ADDR, resp);
            }
        }

        for (i = 0u; i < qty; i++)
        {
            s_hold[start + i] =
                (uint16_t)(((uint16_t)req[13u + 2u * i] << 8) |
                           req[14u + 2u * i]);
        }

        g_mb_req_cnt++;
        printf("[MB] 10 addr=%u qty=%u\r\n", (unsigned)start, (unsigned)qty);

        return Mb_BuildWriteResp(req, resp);

    default:
        return Mb_BuildException(req, fc, MB_EXC_ILLEGAL_FUNC, resp);
    }
}

/* ★ 从累积缓冲里切出完整帧并逐帧响应
 *    · 长度字段决定这一帧有多长，帧长 = 6 + 长度字段
 *    · 剩余字节不足一帧时先退出，等下一次 recv 补齐，这就是粘包处理
 *    · 一帧处理完把后面的字节前移，避免缓冲无限增长
 */
static void Mb_Serve(void)
{
    uint16_t len_field;
    uint16_t total;
    uint16_t n;

    while (s_rx_len >= MB_MBAP_LEN)
    {
        len_field = (uint16_t)(((uint16_t)s_rx[MB_O_LEN] << 8) |
                               s_rx[MB_O_LEN + 1u]);

        if ((len_field < 2u) || (len_field > MB_LEN_MAX))
        {
            printf("[MB] bad len=%u, drop\r\n", (unsigned)len_field);
            s_rx_len = 0u;
            return;
        }

        total = (uint16_t)(6u + len_field);

        if (s_rx_len < total)
        {
            break;
        }

        n = Mb_Process(s_rx, total, s_tx, (uint16_t)sizeof(s_tx));

        if (n > 0u)
        {
            if (send(MB_SN, s_tx, n) > 0)
            {
                g_mb_tx_bytes += n;
            }
        }

        s_rx_len = (uint16_t)(s_rx_len - total);

        if (s_rx_len > 0u)
        {
            (void)memmove(s_rx, &s_rx[total], s_rx_len);
        }
    }
}

int8_t Modbus_TcpInit(void)
{
    uint16_t i;

    s_rx_len = 0u;
    g_mb_connected = 0u;

    for (i = 0u; i < MODBUS_REG_COUNT; i++)
    {
        s_hold[i] = 0u;
    }

    /* 板上 LED 上电就是亮的，线圈 0 跟着置 1，两边状态才一致 */
    s_coil = 0x01u;

    close(MB_SN);

    if (socket(MB_SN, Sn_MR_TCP, MODBUS_TCP_PORT, 0u) != (int8_t)MB_SN)
    {
        return -1;
    }

    if (listen(MB_SN) != SOCK_OK)
    {
        return -2;
    }

    return 0;
}

void Modbus_TcpPoll(void)
{
    static uint32_t s_last_poll   = 0u;
    static uint8_t  s_closed_cnt  = 0u;
    static uint32_t s_closed_tick = 0u;
    static uint32_t s_listen_tick = 0u;
    static uint32_t s_reopen_tick = 0u;

    uint8_t  sr;
    uint16_t len;
    int32_t  ret;

    if ((HAL_GetTick() - s_last_poll) < MB_RATE_MS)
    {
        return;
    }
    s_last_poll = HAL_GetTick();

    sr = Mb_ReadSr();

    switch (sr)
    {
    case SOCK_ESTABLISHED:
        s_closed_cnt = 0u;

        if ((getSn_IR(MB_SN) & Sn_IR_CON) != 0u)
        {
            setSn_IR(MB_SN, Sn_IR_CON);
            g_mb_connected = 1u;
            s_rx_len = 0u;
            printf("[MB] master connected\r\n");
        }

        len = getSn_RX_RSR(MB_SN);
        if (len > 0u)
        {
            if (len > (uint16_t)(MB_RX_BUF - s_rx_len))
            {
                len = (uint16_t)(MB_RX_BUF - s_rx_len);
            }

            if (len > 0u)
            {
                ret = recv(MB_SN, &s_rx[s_rx_len], len);

                if (ret > 0)
                {
                    s_rx_len = (uint16_t)(s_rx_len + (uint16_t)ret);
                    g_mb_rx_bytes += (uint32_t)ret;
                }
            }

            Mb_Serve();
        }
        break;

    case SOCK_CLOSE_WAIT:
        s_closed_cnt = 0u;
        g_mb_connected = 0u;
        s_rx_len = 0u;
        printf("[MB] master closed\r\n");

        /* 这里绝对不能用 disconnect()，原因是它内部有个死等循环
         * ioLibrary 的 disconnect 里有 while(getSn_SR != CLOSED)，没有超时
         * 状态不跳变时它会用全速 SPI 读 Sn_SR，把 W5500 命令仲裁压垮
         * 实测后果：socket 1 和回显一起失联，只能复位板子恢复
         * close() 里对 W5500 已经加了 64 次上限，是有界的，这里用它 */
        (void)close(MB_SN);
        break;

    case SOCK_CLOSED:
        /* 要连续 5 次读到 CLOSED，且跨度超过 20 ms，才认为真的断了。
         * 单次误读就去 reopen，会把正在监听的 socket 亲手关掉 */
        if (s_closed_cnt == 0u)
        {
            s_closed_tick = HAL_GetTick();
        }
        s_closed_cnt++;

        /* 重开也要限速：socket() 内部会先 close()，两个命令连发
         * 如果重开一直失败还每 20 ms 试一次,等于持续给芯片灌命令
         * 限到 200 ms 一次，失败时给芯片留出喘息时间 */
        if ((s_closed_cnt >= 5u) &&
            ((HAL_GetTick() - s_closed_tick) >= 20u) &&
            ((HAL_GetTick() - s_reopen_tick) >= 200u))
        {
            s_closed_cnt = 0u;
            s_reopen_tick = HAL_GetTick();
            g_mb_connected = 0u;
            s_rx_len = 0u;

            if (socket(MB_SN, Sn_MR_TCP, MODBUS_TCP_PORT, 0u) == (int8_t)MB_SN)
            {
                (void)listen(MB_SN);
            }
        }
        break;

    case SOCK_INIT:
        s_closed_cnt = 0u;

        /* 已经开好但还没进监听，最多每 100 ms 补一次 listen */
        if ((HAL_GetTick() - s_listen_tick) >= 100u)
        {
            s_listen_tick = HAL_GetTick();
            (void)listen(MB_SN);
        }
        break;

    default:
        /* SOCK_LISTEN 以及过渡状态：什么都不碰，碰命令寄存器会坏事 */
        s_closed_cnt = 0u;
        break;
    }
}
