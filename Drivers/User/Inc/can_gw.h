/**
  ******************************************************************************
  * @file    can_gw.h
  * @brief   CAN1 收发驱动（F407ZGT6，PB8/PB9 复用，500 kbps）
  ******************************************************************************
  * ★ 做了什么
  * 把 CAN1 的引脚从默认的 PA11/PA12 换到 PB8/PB9，封装初始化、发帧和统计计数。
  *
  * ★ 为什么不用 PA11/PA12
  * PA11/PA12 是 USB 的 D-/D+，板上引到 USB 座，复用成 CAN 有干扰风险。
  * PB8/PB9 同样支持 CAN1 的 AF9 复用，而且都是 5 V 容忍引脚。
  *
  * ★ 影响什么
  * 初始化之后 PB8 和 PB9 不能再当普通 GPIO 用。
  * 这两个脚接反，现象是发送计数在涨、接收计数不动、错误计数很快变大。
  *
  * ★ 怎么验证
  * loopback 模式不接线，TX 与 RX 计数同步增长，错误计数保持 0。
  * 正常模式接总线并挂上第二个节点，双方接收计数同步增长。
  ******************************************************************************
  */

#ifndef __CAN_GW_H
#define __CAN_GW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ★ 一帧接收结果的快照
 * · 中断里只填这一份，主循环读来输出，避免在中断里做耗时操作
 */
typedef struct
{
    uint32_t id;        /* 收到的帧 ID，标准帧取 11 位 */
    uint8_t  dlc;       /* 数据长度 0 到 8 */
    uint8_t  data[8];   /* 数据内容 */
    uint32_t tick_ms;   /* 收到时刻，单位 ms */
} CanGwFrame_t;

/* ★ 网关从 CAN 解出来的最新遥测值
 * · 只有收到 0x101 帧才更新，解码在接收中断里做，主循环直接读
 * · valid 由 CanGw_Poll() 维护，超过 500 毫秒没有新帧就置 0
 */
typedef struct
{
    int16_t  temp_x100;     /* 温度，0.01 摄氏度 */
    uint16_t humi_x10;      /* 湿度，0.1 %RH */
    uint32_t press_pa;      /* 气压，单位 Pa */
    uint32_t frames;        /* 收到的 0x101 帧数 */
    uint32_t last_tick;     /* 最近一帧的时刻 */
    uint8_t  valid;         /* 1 = 链路正常 */

    /* STAT 帧（0x102）解出来的记录仪状态 */
    uint16_t seq;           /* 记录仪帧序号，用来判断丢帧 */
    uint8_t  status;        /* 状态位，定义见协议契约 4.4 */
    uint8_t  proto_ver;     /* 协议版本 */
    uint32_t rtc;           /* 记录仪采集时刻，unix 秒 */
    uint32_t stat_frames;   /* 收到的 0x102 帧数 */
    uint32_t drop;          /* 按 seq 连续性算出的丢帧数 */

/* 2026-11-24 加：0x103 扩展帧（三轴加速度同帧 + 电池）
 * · 三轴必须同帧：姿态解算要的是同一时刻的重力向量，
 *   分开两帧算出来的倾角会偏（实测差 11 度） */
int16_t  ax;            /* 加速度 X */
int16_t  ay;            /* 加速度 Y */
int16_t  az;            /* 加速度 Z */
uint16_t vbat_mv;       /* 电池电压 mV */
uint32_t extra_frames;  /* 收到的 0x103 帧数 */

/* 2026-11-24 加：0x104 扩展帧（三轴角速度同帧 + 光照）
 * · 同上：三轴角速度也必须同帧，否则互补滤波的陀螺积分用错 dt */
int16_t  gx;            /* 角速度 X */
int16_t  gy;            /* 角速度 Y */
int16_t  gz;            /* 角速度 Z */
uint16_t light;         /* 光照原始值 */
uint32_t extra2_frames; /* 收到的 0x104 帧数 */

    /* 2026-11-24 加：布局版本检查 */
    uint8_t  layout_ok;     /* 1 = 收到的版本和本机一致 */
    uint32_t layout_bad;    /* 累计收到几个不匹配的 0x102 */
} CanGwData_t;

/* ★ 统计计数，全部在中断里累加，主循环只读 */
extern volatile uint32_t g_can_tx_frames;   /* 成功装进发送邮箱的帧数 */
extern volatile uint32_t g_can_rx_frames;   /* 从 FIFO0 取出的帧数 */
extern volatile uint32_t g_can_tx_err;      /* 装邮箱失败次数 */
extern volatile uint32_t g_can_err_cnt;     /* CAN 错误中断触发次数 */
extern volatile uint32_t g_can_ack_err;     /* 其中应答错误次数 */
extern volatile uint32_t g_can_busoff_cnt;  /* 其中进入 bus-off 的次数 */
extern volatile uint32_t g_can_last_err;    /* 最近一次的 HAL 错误码 */

extern CanGwFrame_t g_can_last_rx;          /* 最近一帧的内容 */

/* ★ 抓帧槽（2026-11-23 加）：命令通道用 CAP <id> 布置、CAPQ 读取
 * · 写 g_can_cap_id 时必须同时把 g_can_cap_hit 清 0，否则会读到上一轮的旧帧
 * · 0xFFFF = 不抓 */
extern volatile uint16_t g_can_cap_id;
extern volatile uint8_t  g_can_cap_hit;
extern volatile uint8_t  g_can_cap_dlc;
extern volatile uint8_t  g_can_cap_data[8];
extern volatile uint32_t g_can_cap_cnt;     /* 抓到过几帧，CAP 布置时清零 */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;   /* 2026-11-24 加：第二路总线 */
extern volatile uint8_t  g_can_init_step;
extern volatile uint8_t  g_can2_self_test;   /* 2026-11-24 加：CAN2 回环自检结果 */

/* ★ 双通道与路由计数（2026-11-24 加），都从 TCP 的 Q 观察 */
extern volatile uint32_t g_can2_rx_frames;   /* CAN2 收到的帧数 */
extern volatile uint32_t g_can2_tx_frames;   /* CAN2 发出去的帧数 */
extern volatile uint32_t g_gw_route_hit;     /* 命中路由表并转发的帧数 */
extern volatile uint32_t g_gw_route_miss;    /* 没命中路由表（只计数不转发） */
extern volatile uint32_t g_gw_txq_drop;      /* 后备队列也满、真丢掉的帧数（应恒为 0） */

/* ★ 在指定通道上发一帧：ch = 0 走 CAN1，ch = 1 走 CAN2 */
HAL_StatusTypeDef CanGw_SendStdCh(uint8_t ch, uint16_t std_id, const uint8_t *data, uint8_t len);   /* 初始化挂在第几步，定位用 */

/* ★ 初始化
 * · loopback 为 1 是内部回环，不接总线也能收到自己发的帧
 * · 为 0 是正常总线模式，必须有第二个节点应答才算发送成功
 */
HAL_StatusTypeDef CanGw_Init(uint8_t loopback);

/* ★ CAN2（第二路总线）初始化，2026-11-24 加
 * · loopback = 1 走内部回环，不接收发器也能自检
 * · 滤波器组从 14 起，和 CAN1 的 0 组不冲突 */
HAL_StatusTypeDef CanGw_Init2(uint8_t loopback);

/* ★ CAN2 回环自检：返回 0 = 通过，非 0 = 出错步骤号 */
uint8_t CanGw_SelfTest2(void);

/* ★ 发一帧标准数据帧
 * · 返回 HAL_OK 只表示成功装进邮箱，不代表总线上已经发完
 */
HAL_StatusTypeDef CanGw_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len);
/* ★ ISO-TP 发送侧流控（2026-11-23 加）
 * · 做什么：转发 0x2E0 时按 ISO 15765-2 发送侧的规矩走，首帧之后等对端流控帧再发连续帧
 * · 为什么：原来是无脑转发，F103 的 CAN 接收 FIFO 硬件深度只有 3 级，
 *   一次推 8 帧必然溢出；丢一帧整包作废，剩下的连续帧全被对端判成孤儿帧
 * · 影响什么：一条多帧报文会让主循环阻塞到拿到流控帧，最长 CAN_GW_ISOTP_FC_TIMEOUT
 * · 怎么验证：命令通道 Q 里 fc 在涨而 to 为 0，同时 F103 串口不再刷 why=6
 */
#define CAN_GW_ISOTP_ID_REQ     0x2E0u
#define CAN_GW_ISOTP_ID_RESP    0x3E0u
/* ★ 2026-11-24 审计修：100 -> 20 ms
 * · 这个超时是"等对端流控帧"的上限，而等待跑在裸机主循环里
 * · F103 的 FC 实测 1~2 ms 就到，20 ms 足够；原来 100 ms 让最坏阻塞放大到 3.7 s
 * · 放到 20 ms 后，一个 37 帧报文最坏阻塞 = 37×5(STmin) + 5×20(FC超时) ≈ 285 ms */
#define CAN_GW_ISOTP_FC_TIMEOUT 20u

extern volatile uint32_t g_fwd_isotp_ff;
extern volatile uint32_t g_fwd_isotp_cf;
extern volatile uint32_t g_fwd_isotp_fc;
extern volatile uint32_t g_fwd_isotp_fc_to;
extern volatile uint32_t g_fwd_isotp_wait_ms;
extern volatile uint32_t g_fwd_isotp_wait_max;
extern volatile uint32_t g_fwd_isotp_cf_dropped;   /* 2026-11-24：被丢弃的连续帧数 */

HAL_StatusTypeDef CanGw_FwdIsoTp(uint16_t id, const uint8_t *data, uint8_t len);
/* ★ 命令通道（2026-11-24 加）
 * · 做什么：F407 往 0x201 下发命令，F103 执行后往 0x301 回 ACK
 * · 为什么：原来网关只会收，想控制 F103 只能靠 PC 从 TCP 转发，
 *   那就不是网关而是个转接头。有了这条通路，网关才能自己按条件下命令。
 * · 帧格式见 76-通信协议契约：0x201 的 data[0] 是命令码，data[1..2] 是参数
 * · 怎么验证：命令通道发 CMD 02，F103 停止记录；CQ 能看到 ack 计数在涨且 res=0
 */
#define CAN_GW_ID_CMD   0x201u
#define CAN_GW_ID_ACK   0x301u

/* 命令码：和 F103 侧 uds.c / freertos_demo.c 的处理分支一一对应 */
#define CAN_CMD_REC_START   0x01u   /* 开始记录 */
#define CAN_CMD_REC_STOP    0x02u   /* 停止记录 */
#define CAN_CMD_PAGE_NEXT   0x03u   /* 翻页 */
#define CAN_CMD_BUZZ        0x04u   /* 蜂鸣一声 */
#define CAN_CMD_REPORT_NOW  0x10u   /* 立即上报一次状态 */

typedef struct
{
    uint8_t  last_cmd;      /* 最近下发的命令码，0 = 还没发过 */
    uint8_t  ack_code;      /* 最近收到的 ACK 回显的命令码 */
    uint8_t  ack_result;    /* ACK 结果：0 = 成功，其它是对端定义的错误码 */
    uint32_t cmd_cnt;       /* 累计下发的命令数 */
    uint32_t ack_cnt;       /* 累计收到的 ACK 数 */
    uint32_t ack_tick;      /* 最近一次收到 ACK 的时刻 */
} CanGwCmd_t;

/* ★ 下发一条命令
 * · 返回 HAL_OK 只表示装进邮箱成功，不代表 F103 收到了
 * · 有没有执行要看 CanGw_GetCmd()->ack_cnt 有没有涨
 */
HAL_StatusTypeDef CanGw_SendCmd(uint8_t cmd, uint8_t p1, uint8_t p2);

/* ★ 取命令状态快照，指针常驻 */
const CanGwCmd_t *CanGw_GetCmd(void);
/* ★ 故障告警（2026-11-24 加）
 * · 做什么：收 F103 发来的 0x080 告警帧，记住最近一条和总数
 * · 为什么：网关要能回答"现场有没有在出事"，而不是只显示温度曲线好看
 * · 帧格式：ch(1) + reason(1) + 该通道累计次数(2) + 保留(4)
 * · 怎么验证：拔掉 F103 的 SD 卡，AQ 命令看到 total 在涨、ch 和 rsn 有值
 */
#define CAN_GW_ID_ALARM   0x080u
#define CAN_GW_ID_EXTRA   0x103u   /* ax(2) + ay(2) + az(2) + vbat(2) */
#define CAN_GW_ID_EXTRA2  0x104u   /* gx(2) + gy(2) + gz(2) + light(2) */

/* ★ 帧布局版本（2026-11-24 加）
 * · 必须和 F103 的 0x102 帧 data[3] 一致
 * · 改了任何一帧的字节含义，两边一起 +1，否则收到就拒绝解析
 */
#define GW_LAYOUT_VER   2u

typedef struct
{
    uint8_t  ch;        /* 告警通道，含义见 task_sd.c 的 EV_CH_* */
    uint8_t  reason;    /* 原因码，含义见 task_sd.c 的 EV_RSN_* */
    uint16_t count;     /* 该通道在 F103 侧累计的次数（快照） */
    uint32_t total;     /* F407 累计收到的告警帧数 */
    uint32_t tick;      /* 最近一条的到达时刻 */
} CanGwAlarm_t;

/* ★ 取告警快照，指针常驻 */
const CanGwAlarm_t *CanGw_GetAlarm(void);

/* ★ 取最新遥测值
 * · 返回的指针常驻，不需要判空
 * · frames 为 0 表示开机到现在一个遥测帧都没收到
 */
const CanGwData_t *CanGw_GetData(void);

/* ★ 主循环定期调用，维护链路有效标志
 * · 只读时间戳，不碰硬件，开销可以忽略
 */
void CanGw_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_GW_H */
