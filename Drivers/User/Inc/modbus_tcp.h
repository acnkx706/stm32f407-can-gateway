#ifndef __MODBUS_TCP_H
#define __MODBUS_TCP_H

/*==============================================================================
 *  Modbus TCP 从站（W5500 socket 1，端口 502）
 *
 *  ★ 做什么
 *    在 TCP 之上实现 Modbus TCP 从站，对外提供 16 个保持寄存器和 8 个线圈。
 *    功能码：0x01 读线圈、0x03 读保持寄存器、0x04 读输入寄存器（2026-11-24 加）、
 *    0x05 写单个线圈、0x06 写单个寄存器、0x10 写多个寄存器。
 *
 *  ★ 为什么单独占一个 socket
 *    socket 0 保留给 TCP 回显（端口 5000）。
 *    网络出问题时先连回显确认链路，再怀疑协议层，排错路径更短。
 *
 *  ★ 影响什么
 *    占用 socket 1 和它的 2 KB 收发缓冲，其余 socket 不受影响。
 *    Poll 函数非阻塞，必须由主循环定期调用。
 *
 *  ★ 怎么验证
 *    见同目录上级的 modbus_test.ps1，跑一遍应全部 PASS：
 *    读 10 个寄存器、写 4~9 参数区、越界回 0x02、写只读区回 0x02、
 *    线圈 0 控制 LED1。
 *============================================================================*/

#include <stdint.h>

/* ★ Modbus TCP 标准端口，主站默认连 502，不要改 */
#define MODBUS_TCP_PORT     502u

/* ★ 保持寄存器总数，合法地址范围 0 ~ 15 */
/* ★ 2026-11-24：16 -> 24（分两次扩的，这里记总数）
 * · 第一次 16->20 装 0x103 的 vbat/light/ax/ay
 * · 第二次 20->24 装 0x104 的 az/gx/gy/gz
 * · 地址表：0-3 测量值、4-9 参数区（可写）、10-15 统计、16-23 扩展量
 * · 扩到 24 只多占 16 字节 RAM，主站照旧能只读前 16 个 */
#define MODBUS_REG_COUNT    24u

/* ★ 一次请求最多读几个寄存器，这是 Modbus 协议规定的上限
 *    实际数量还要受 MODBUS_REG_COUNT 限制，超了回异常码 0x02 */
#define MODBUS_READ_MAX     125u

/* ★ 线圈总数，合法地址范围 0 ~ 7
 *    地址 0 直接映射板上的 LED1，写它能看见灯亮灭，是最直观的验证手段 */
#define MODBUS_COIL_COUNT   8u

/* ★ 一次请求最多读几个线圈，协议上限 2000 */
#define MODBUS_COIL_READ_MAX 2000u

/* ★ 可写寄存器的地址范围：4 ~ 9 是参数区
 *    0 ~ 3 是传感器实测值，只读；主站写它们要回异常码 0x02
 *    10 ~ 15 是状态和计数，也只读 */
#define MODBUS_WR_FIRST     4u
#define MODBUS_WR_LAST      9u

/* ★ 一次请求最多写几个寄存器，协议上限 123
 *    为什么不是 125：写请求还要带字节数和数据，123 是协议给的上限 */
#define MODBUS_WRITE_MAX    123u

/* ★ 主循环推给协议层的数据
 *    · 单位要和串口打印保持一致，否则主站读出来的数对不上
 *    · 协议层不关心数据来自 I2C 还是以后的 CAN，只认这个结构体
 *    · 字段类型决定了寄存器怎么映射，改这里要同步改 Modbus_GetReg()
 */
typedef struct
{
    int16_t  temp_x100;     /* 温度，0.01 摄氏度 */
    uint16_t humi_x10;      /* 湿度，0.1 %RH */
    uint32_t press_pa;      /* 气压，单位 Pa */
    uint32_t i2c_rx;        /* 已收到的 I2C 帧数 */
    uint32_t i2c_err;       /* I2C 错误次数 */
    uint8_t  link_up;       /* 1 = 网线已连接 */

    /* 2026-11-24 加：来自 F103 的 0x103 帧 */
    uint16_t vbat_mv;       /* 电池电压，mV */
    uint16_t light;         /* 光照原始值 */
    int16_t  ax;            /* 加速度 X 原始值 */
    int16_t  ay;            /* 加速度 Y 原始值 */
    int16_t  az;            /* 加速度 Z 原始值 */
    int16_t  gx;            /* 角速度 X 原始值 */
    int16_t  gy;            /* 角速度 Y 原始值 */
    int16_t  gz;            /* 角速度 Z 原始值 */
} Modbus_Data_t;

/* ★ 打开 socket 1 并在 502 端口监听
 *    返回 0 = 成功，-1 = socket 打开失败，-2 = listen 失败 */
int8_t Modbus_TcpInit(void);

/* ★ 状态机轮询，必须由主循环定期调用
 *    内部自限速 2 ms，调用频率再高也不会刷崩 SPI */
void Modbus_TcpPoll(void);

/* ★ 主循环把最新采集值推给协议层
 *    每 5 ms 推一次即可，Modbus 主站一般是每秒读一次 */
void Modbus_SetData(const Modbus_Data_t *d);

/* ★ 读一个保持寄存器的当前值
 *    · 地址越界时返回 0，不报错
 *    · 屏幕显示和功能码 0x03 用的是同一份映射，不会出现两套口径 */
uint16_t Modbus_GetReg(uint16_t addr);

/* ★ 写一个保持寄存器（2026-11-24 加）：只允许写参数区，返回 1 成功 */
uint8_t Modbus_SetReg(uint16_t addr, uint16_t val);

/* ★ 读一个线圈的当前值，返回 0 或 1
 *    · 地址越界时返回 0 */
uint8_t Modbus_GetCoil(uint16_t addr);

extern volatile uint32_t g_mb_req_cnt;      /* 成功处理的请求数 */
extern volatile uint32_t g_mb_exc_cnt;      /* 回出去的异常响应数 */
extern volatile uint32_t g_mb_rx_bytes;     /* 累计收到字节 */
extern volatile uint32_t g_mb_tx_bytes;     /* 累计发出字节 */
extern volatile uint8_t  g_mb_connected;    /* 1 = 主站已连接 */

#endif /* __MODBUS_TCP_H */
