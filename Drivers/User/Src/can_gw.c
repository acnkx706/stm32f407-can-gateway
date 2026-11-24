/**
  ******************************************************************************
  * @file    can_gw.c
  * @brief   CAN1 收发驱动（F407ZGT6，PB8/PB9 复用，500 kbps）
  ******************************************************************************
  * ★ 位时序怎么算
  * · PCLK1 是 42 MHz，预分频 4 得到 10.5 MHz 的 tq 时钟
  * · 一位 21 个 tq，10.5 MHz 除以 21 就是 500 kbps
  * · 采样点在第 17 个 tq，占比 81%，落在推荐的 75% 到 87.5% 区间内
  *
  * ★ 为什么一位必须是 21 个 tq
  * · bxCAN 规定 BS1 最多 16 tq、BS2 最多 8 tq，一位最多 25 tq
  * · 42 MHz 下要凑出 500 kbps，21 tq 是唯一解：1 同步 + 16 BS1 + 4 BS2
  *
  * ★ 影响什么
  * · 两块板的波特率必须对得上，差一点就会持续出错误帧
  * · PB8 和 PB9 初始化之后不能再当普通 GPIO 用
  *
  * ★ 怎么验证
  * · 回环模式：TX 与 RX 计数同步增长，错误计数恒为 0
  * · 正常模式：拔掉对端后应答错误会快速上涨，插回后停止上涨
  ******************************************************************************
  */

#include "can_gw.h"
#include <string.h>

/* ★ 句柄和统计量
 * · hcan1 放文件作用域，因为中断回调和本文件的函数都要访问它
 * · 六个计数在头文件里 extern 出去，主循环直接读，不需要加锁
 */
CAN_HandleTypeDef hcan1;

volatile uint32_t g_can_tx_frames  = 0u;
volatile uint32_t g_can_rx_frames  = 0u;
volatile uint32_t g_can_tx_err     = 0u;
volatile uint32_t g_can_err_cnt    = 0u;
volatile uint32_t g_can_ack_err    = 0u;
volatile uint32_t g_can_busoff_cnt = 0u;

/* ★ 超过这个时间没有新遥测帧，就认为链路断了 */
#define CAN_GW_TIMEOUT_MS   500u

/* ★ 解码后的最新遥测值，接收中断写，主循环读 */
static CanGwData_t s_data;
volatile uint32_t g_can_last_err   = 0u;

CanGwFrame_t g_can_last_rx;

/* ★ 抓帧槽（2026-11-23 加）：让命令通道能读回设备发出的帧
 * · 为什么需要：PC 只能往总线上发，读不到回程。UDS 的 27 服务必须先拿到种子，
 *   没有回程这条路就做不了安全访问，也就验证不了任何肯定响应
 * · 为什么只抓一个 ID：一次只等一条响应，一个槽比一个队列省 RAM 也够用
 * · 0xFFFF 表示不抓 */
volatile uint16_t g_can_cap_id   = 0xFFFFu;
volatile uint8_t  g_can_cap_hit  = 0u;
volatile uint8_t  g_can_cap_dlc  = 0u;
volatile uint8_t  g_can_cap_data[8];

/* ★ 抓到过几帧（2026-11-23 加）
 * · 一个槽只能留住最后一帧，但"到底回了几次"要靠计数
 * · 典型用途：擦除要 10 秒，期间应当多次回 0x78；不数就证明不了这一点 */
volatile uint32_t g_can_cap_cnt  = 0u;
/* ★ ISO-TP 转发流控的计数（2026-11-23 加）
 * · ff/cf 是转发出去的帧型分类，fc 是等到的流控帧，to 是等待失败的次数
 * · wait_ms 是从开始等到拿到流控帧的累计毫秒，用来算每块的平均等待
 * · 为什么要单独计数：F103 那边的 why 环只有 4 格，一次事件就刷满，看不出分布
 */
volatile uint32_t g_fwd_isotp_ff      = 0u;
volatile uint32_t g_fwd_isotp_cf      = 0u;
volatile uint32_t g_fwd_isotp_fc      = 0u;
volatile uint32_t g_fwd_isotp_fc_to   = 0u;
volatile uint32_t g_fwd_isotp_wait_ms  = 0u;
volatile uint32_t g_fwd_isotp_wait_max = 0u;
volatile uint32_t g_fwd_isotp_cf_dropped = 0u;   /* 2026-11-24：被丢弃的连续帧数 */

volatile uint8_t g_can_init_step = 0u;

/* ★ 时钟、引脚、中断优先级
 * · HAL_CAN_Init 内部会调用这个弱函数，在这里配 GPIOB 和 NVIC
 * · 放在本文件而不是 stm32f4xx_hal_msp.c，是为了让 CAN 的代码集中在一处
 * · PB8 是 CAN1_RX、PB9 是 CAN1_TX，两个都是 AF9 复用
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    GPIO_InitTypeDef gpio = {0};

    if (hcan->Instance != CAN1)
    {
        return;
    }

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;   /* CAN_RX 悬空时读到低电平会让 bxCAN 退不出初始化 */
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* ★ 三个中断入口都要开
     * · CAN1_RX0 收报文，CAN1_TX 发完一帧，CAN1_SCE 报错误状态变化
     * · 注意错误中断挂在 SCE 上，不是 RX0，这是 bxCAN 的固定映射
     * · 优先级 5 高于 SysTick 的 15，保证收帧不被刷屏拖住
     */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn,   5u, 0u);
    HAL_NVIC_EnableIRQ  (CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn,  5u, 0u);
    HAL_NVIC_EnableIRQ  (CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn,  5u, 0u);
    HAL_NVIC_EnableIRQ  (CAN1_SCE_IRQn);
}

/* ★ 初始化
 * · loopback 为 1 是内部回环，不接总线也能收到自己发的帧
 * · 为 0 是正常总线模式，必须有第二个节点应答才算发送成功
 * · AutoBusOff 打开，总线被干扰进 bus-off 后硬件能自动恢复
 */
HAL_StatusTypeDef CanGw_Init(uint8_t loopback)
{
    CAN_FilterTypeDef filter = {0};

    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 4u;
    hcan1.Init.Mode                 = (loopback != 0u) ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_16TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_4TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    g_can_init_step = 1u;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* ★ 过滤器全通过
     * · 调试阶段不筛 ID，先确认链路通，正式版再按 ID 收窄
     * · 32 位掩码模式，ID 和掩码全填 0 就是"什么都收"
     */
    filter.FilterBank           = 0u;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000u;
    filter.FilterIdLow          = 0x0000u;
    filter.FilterMaskIdHigh     = 0x0000u;
    filter.FilterMaskIdLow      = 0x0000u;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14u;

    g_can_init_step = 2u;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    g_can_init_step = 3u;
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    g_can_init_step = 4u;
    if (HAL_CAN_ActivateNotification(&hcan1,
                                     CAN_IT_RX_FIFO0_MSG_PENDING
                                   | CAN_IT_ERROR
                                   | CAN_IT_ERROR_WARNING
                                   | CAN_IT_ERROR_PASSIVE
                                   | CAN_IT_BUSOFF
                                   | CAN_IT_LAST_ERROR_CODE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ★ 发一帧标准数据帧
 * · 返回 HAL_OK 只表示成功装进发送邮箱，不代表总线上已经发完
 * · 到底有没有人收到，看接收计数和应答错误计数的变化
 * · data 允许传 NULL 表示只发包头，但要保证 len 为 0
 */
HAL_StatusTypeDef CanGw_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox = 0u;
    uint8_t  buf[8]  = {0};

    if (len > 8u)
    {
        return HAL_ERROR;
    }
    if ((data == NULL) && (len > 0u))
    {
        return HAL_ERROR;
    }
    if (len > 0u)
    {
        memcpy(buf, data, (size_t)len);
    }

    header.StdId              = (uint32_t)std_id & 0x7FFu;
    header.ExtId              = 0u;
    header.IDE                = CAN_ID_STD;
    header.RTR                = CAN_RTR_DATA;
    header.DLC                = len;
    header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &header, buf, &mailbox) != HAL_OK)
    {
        g_can_tx_err++;
        return HAL_ERROR;
    }

    g_can_tx_frames++;
    return HAL_OK;
}
/*==============================================================================
 *  ★ ISO-TP 发送侧流控（2026-11-23 加）
 *
 *  ★ 做什么
 *    转发 0x2E0 的报文时按 ISO 15765-2 发送侧的规矩走：发完首帧等对端流控帧，
 *    拿到 CTS 才发连续帧，每发满一个块再等下一个流控帧。
 *
 *  ★ 为什么必须这么做
 *    F103 的 CAN 接收 FIFO 硬件深度只有 3 级（STM32F1 参考手册 CAN 章节）。
 *    原来无脑推 8 帧，第 4 帧就溢出；丢帧后序号对不上，整包作废，
 *    对端把剩下的连续帧判成孤儿帧，串口日志里就是连着几个 why=6。
 *
 *  ★ 影响什么
 *    一条多帧报文会让主循环阻塞到拿到流控帧，最长 CAN_GW_ISOTP_FC_TIMEOUT。
 *    等超时就降级为继续发（不把整包卡死），并计入 g_fwd_isotp_fc_to。
 *
 *  ★ 怎么验证
 *    命令通道 Q 里 fc 在涨、to 为 0；同时 F103 串口不再出现连续的 why=6。
 *============================================================================*/

/* ★ 流控状态：active/sent/bs_left 由发送侧维护，fc_ 那四个由接收中断写 */
static struct
{
    uint8_t  active;        /* 1 = 正在转发一个多帧报文 */
    uint16_t total;         /* 首帧声明的总长度 */
    uint16_t sent;          /* 已经发出去的字节数 */
    uint8_t  bs;            /* 对端给的块大小，0 表示不再要流控帧 */
    uint8_t  bs_left;       /* 本块还能发几帧，0xFF 表示不限 */
    uint8_t  stmin;         /* 对端要求的最小帧间隔，单位毫秒 */
    uint8_t  need_fc;       /* 1 = 下一帧之前必须等到流控帧 */
    uint8_t  dropping;      /* 1 = 上一包被对端拒收，后续连续帧静默丢弃 */
    uint32_t drop_tick;     /* dropping 置位的时刻，用于 1 秒兜底复位 */

    volatile uint8_t fc_ready;  /* 中断置 1，发送侧取走时清 0 */
    volatile uint8_t fc_fs;     /* 0 = CTS, 1 = WAIT, 2 = OVFLW */
    volatile uint8_t fc_bs;
    volatile uint8_t fc_stmin;
} s_isotp_fwd;

/* ★ 等一个流控帧
 * · HAL_OK 拿到 CTS，HAL_TIMEOUT 等太久，HAL_ERROR 对端回 OVFLW 拒收
 * · 为什么在发送侧等而不是中断里等：中断里不许阻塞，等待只能在任务上下文做
 * · WAIT 帧会把超时重新计时，标准里它的语义就是"我还要时间"
 */
static HAL_StatusTypeDef CanGw_WaitFc(void)
{
    uint32_t t0 = HAL_GetTick();

    for (;;)
    {
        if (s_isotp_fwd.fc_ready != 0u)
        {
            uint8_t fs = s_isotp_fwd.fc_fs;

            s_isotp_fwd.fc_ready = 0u;

            if (fs == 0u)                   /* CTS：拿到底数，可以发 */
            {
                uint8_t st = s_isotp_fwd.fc_stmin;

                s_isotp_fwd.bs      = s_isotp_fwd.fc_bs;
                s_isotp_fwd.bs_left = (s_isotp_fwd.fc_bs == 0u)
                                      ? 0xFFu : s_isotp_fwd.fc_bs;

                /* ★ 2026-11-24 审计修：STmin 上限钳到 5 ms
                 * · 原来 0x00~0x7F 原样当毫秒用，最坏 127 ms/帧 × 37 帧 = 4.7 s，
                 *   而这 4.7 s 全跑在裸机主循环里 —— LVGL/Modbus/HTTP/5000 一起停摆
                 * · 我们的对端是 F103，实测 STmin 恒为 0；钳到 5 ms 是给"以后接别的
                 *   设备"留余地，同时保证最坏阻塞有上界（37 × 5 = 185 ms）
                 * · STmin 两段编码：0~127 毫秒，0xF1~0xF9 是 100~900 微秒；
                 *   微秒段和保留值一律按 1 ms 处理，只会更慢不会更快 */
                s_isotp_fwd.stmin = (st <= 127u) ? ((st > 5u) ? 5u : st) : 1u;

                {
                    uint32_t w = HAL_GetTick() - t0;

                    g_fwd_isotp_wait_ms += w;
                    if (w > g_fwd_isotp_wait_max)
                    {
                        g_fwd_isotp_wait_max = w;
                    }
                }

                g_fwd_isotp_fc++;
                return HAL_OK;
            }

            if (fs == 1u)                   /* WAIT：对端忙，继续等 */
            {
                t0 = HAL_GetTick();
                continue;
            }

            s_isotp_fwd.active = 0u;        /* OVFLW：对端收不下，这包作废 */
            return HAL_ERROR;
        }

        if ((HAL_GetTick() - t0) >= CAN_GW_ISOTP_FC_TIMEOUT)
        {
            return HAL_TIMEOUT;
        }

        HAL_Delay(1u);
    }
}

/* ★ 转发一帧，内部处理 ISO-TP 发送侧流控
 * · 单帧没有流控可言，直接发
 * · 首帧发完就置 need_fc，第一个连续帧之前必须拿到 CTS
 * · 返回 HAL_OK 只表示这一帧装进了邮箱，不代表对端一定收到
 */
HAL_StatusTypeDef CanGw_FwdIsoTp(uint16_t id, const uint8_t *data, uint8_t len)
{
    uint8_t pci;

    if ((data == NULL) || (len == 0u))
    {
        return HAL_ERROR;
    }

    pci = (uint8_t)(data[0] & 0xF0u);

    /* ---- 单帧：一帧装得下，没有流控 ---- */
    if (pci == 0x00u)
    {
        s_isotp_fwd.active   = 0u;
        s_isotp_fwd.need_fc  = 0u;
        s_isotp_fwd.dropping = 0u;

        return CanGw_SendStd(id, data, len);
    }

    /* ---- 首帧：记下总长度，发出去之后必须等第一个流控帧 ---- */
    if (pci == 0x10u)
    {
        uint16_t total;

        if (len < 8u)
        {
            return HAL_ERROR;
        }

        total = (uint16_t)(((uint16_t)(data[0] & 0x0Fu) << 8) | (uint16_t)data[1]);

        /* 声明长度 7 以内本该用单帧，对端造错了：照原样发过去，让它自己按单帧解 */
        if (total <= 7u)
        {
            s_isotp_fwd.active  = 0u;
            s_isotp_fwd.need_fc = 0u;

            return CanGw_SendStd(id, data, len);
        }

        s_isotp_fwd.active   = 1u;
        s_isotp_fwd.total    = total;
        s_isotp_fwd.sent     = 6u;      /* 首帧自带 6 个数据字节 */
        s_isotp_fwd.bs       = 0u;
        s_isotp_fwd.bs_left  = 0u;
        s_isotp_fwd.stmin    = 0u;
        s_isotp_fwd.need_fc  = 1u;
        s_isotp_fwd.dropping = 0u;      /* 新包开始，前面丢弃状态作废 */
        s_isotp_fwd.fc_ready = 0u;      /* 丢掉上一包残留的流控帧 */

        g_fwd_isotp_ff++;

        return CanGw_SendStd(id, data, len);
    }

    /* ---- 连续帧 ---- */
    if (pci == 0x20u)
    {
        uint16_t          n;
        HAL_StatusTypeDef st;

        g_fwd_isotp_cf++;

        if (s_isotp_fwd.active == 0u)
        {
            /* 上一包被对端回 OVFLW 拒收：剩下的连续帧一律不发。
             * 原来这里照原样发出去，结果对端把它们全判成孤儿帧，
             * 串口就是连着几个 why=6，把真正的原因 (why=4) 挤出了环形缓冲。 */
            /* ★ 2026-11-24 审计修：丢弃要计数，而且不能永久卡住
             * · 原来静默 return HAL_OK：net_tcp 见 OK 就 ok++，drop 恒 0，
             *   指标显示"全部转发成功"实际数据被吞 —— 现场最难发现的一类缺陷
             * · 加 1 秒兜底：对端若只补 CF 不发新 FF，旧逻辑会永远卡在丢弃态，
             *   之后所有多帧报文都被吞 */
            if (s_isotp_fwd.dropping != 0u)
            {
                if ((HAL_GetTick() - s_isotp_fwd.drop_tick) > 1000u)
                {
                    s_isotp_fwd.dropping = 0u;      /* 超时兜底，恢复转发 */
                }
                else
                {
                    g_fwd_isotp_cf_dropped++;
                    return HAL_OK;
                }
            }

            /* 真的没收到首帧：流控救不了，照原样发，让对端自己丢 */
            return CanGw_SendStd(id, data, len);
        }

        /* 块发满了，或者首帧之后还没拿到第一个 CTS：先等流控帧 */
        if (s_isotp_fwd.need_fc != 0u)
        {
            HAL_StatusTypeDef fst = CanGw_WaitFc();

            s_isotp_fwd.need_fc = 0u;

            if (fst == HAL_ERROR)
            {
                /* 对端回 OVFLW：这包它收不下，后面的连续帧全丢，
                 * 不再往总线上灌。返回 OK 是为了让上层的 ok 计数保持干净。
                 * dropping 一置，后续连续帧在函数开头就被拦掉，连邮箱都不碰。 */
                s_isotp_fwd.dropping = 1u;
                s_isotp_fwd.drop_tick = HAL_GetTick();   /* 兜底计时的起点 */
                g_fwd_isotp_fc_to++;
                return HAL_OK;
            }

            if (fst == HAL_TIMEOUT)
            {
                /* 等超时：降级为继续发，否则整包卡死在这里 */
                g_fwd_isotp_fc_to++;
                s_isotp_fwd.bs_left = 0xFFu;
                s_isotp_fwd.stmin   = 0u;
            }
        }

        st = CanGw_SendStd(id, data, len);
        if (st != HAL_OK)
        {
            return st;
        }

        /* 这一帧带走多少字节：最后一帧可能不满 7 个 */
        n = (uint16_t)(s_isotp_fwd.total - s_isotp_fwd.sent);
        if (n > 7u)
        {
            n = 7u;
        }
        s_isotp_fwd.sent = (uint16_t)(s_isotp_fwd.sent + n);

        /* 包发完了，状态清干净：下一帧连续帧就当孤儿处理 */
        if (s_isotp_fwd.sent >= s_isotp_fwd.total)
        {
            s_isotp_fwd.active  = 0u;
            s_isotp_fwd.need_fc = 0u;

            return st;
        }

        /* 本块配额减一，减到 0 且对端要流控，就标记下一帧前必须等 */
        if ((s_isotp_fwd.bs != 0u) && (s_isotp_fwd.bs_left != 0xFFu))
        {
            if (s_isotp_fwd.bs_left > 0u)
            {
                s_isotp_fwd.bs_left--;
            }

            if (s_isotp_fwd.bs_left == 0u)
            {
                s_isotp_fwd.need_fc = 1u;
            }
        }

        /* 对端要求帧间隔就照做：STmin 是它给的，不是我们猜的 */
        if (s_isotp_fwd.stmin != 0u)
        {
            HAL_Delay(s_isotp_fwd.stmin);
        }

        return st;
    }

    /* 其它帧型（流控帧本身等）不属于转发方向，照原样发 */
    return CanGw_SendStd(id, data, len);
}

/*==============================================================================
 *  ★ 命令下发（2026-11-24 加）
 *
 *  ★ 做什么
 *    F407 往 0x201 发一帧命令，F103 执行后往 0x301 回一条 ACK。
 *    这是网关从"只会收"变成"能主动控制"的那条通路。
 *
 *  ★ 为什么不能靠 PC 转发
 *    原来想让 F103 干什么，只能电脑从 TCP 5000 发 CAN 命令进来。
 *    那样网关只是个转接头：断网就失控，也没法按总线上的条件自己动作。
 *
 *  ★ 帧格式（和协议契约一致）
 *    0x201: data[0]=命令码 data[1..2]=参数 data[3..7]=保留
 *    0x301: data[0]=回显命令码 data[1]=结果（0 成功）
 *
 *  ★ 怎么验证
 *    命令通道发 CMD 02，F103 停止记录；CQ 看到 ack_cnt 涨且 res=0。
 *============================================================================*/

/* ★ ack_* 由接收中断写，last_cmd/cmd_cnt 由任务上下文写，两组不交叉 */
static CanGwCmd_t s_cmd;

HAL_StatusTypeDef CanGw_SendCmd(uint8_t cmd, uint8_t p1, uint8_t p2)
{
    uint8_t d[8];

    d[0] = cmd;
    d[1] = p1;
    d[2] = p2;
    d[3] = 0u;
    d[4] = 0u;
    d[5] = 0u;
    d[6] = 0u;
    d[7] = 0u;

    if (CanGw_SendStd(CAN_GW_ID_CMD, d, 8u) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_cmd.last_cmd = cmd;
    s_cmd.cmd_cnt++;

    return HAL_OK;
}

const CanGwCmd_t *CanGw_GetCmd(void)
{
    return &s_cmd;
}

/* ★ 告警快照（2026-11-24 加）：全部字段由接收中断写，任务侧只读 */
static CanGwAlarm_t s_alarm;

const CanGwAlarm_t *CanGw_GetAlarm(void)
{
    return &s_alarm;
}

/* ★ 收到一帧
 * · 中断里只做取值和计数，不做任何打印和刷屏
 * · 数据直接写进 g_can_last_rx.data，主循环读来显示
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header = {0};

    if (hcan->Instance != CAN1)
    {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, g_can_last_rx.data) != HAL_OK)
    {
        return;
    }

    g_can_last_rx.id      = (header.IDE == CAN_ID_STD) ? header.StdId : header.ExtId;
    g_can_last_rx.dlc     = (uint8_t)header.DLC;
    g_can_last_rx.tick_ms = HAL_GetTick();
    g_can_rx_frames++;

    /* ★ 抓帧：只要 ID 对上就抄一份，放在解码分支之前
     *   · 放在前面是因为要抓的是"任意 ID"，不限于 0x101/0x102 那两个分支
     *   · 只抄 8 个字节，中断里的开销可以忽略 */
    if (g_can_cap_id == g_can_last_rx.id)
    {
        uint8_t ci;

        for (ci = 0u; ci < 8u; ci++)
        {
            g_can_cap_data[ci] = g_can_last_rx.data[ci];
        }
        g_can_cap_dlc = g_can_last_rx.dlc;
        g_can_cap_hit = 1u;
        g_can_cap_cnt++;
    }

    /* ★ ISO-TP 流控帧识别（2026-11-23 加）
     * · 只认 0x3E0 且首字节高 4 位是 3 的帧，其它一概不碰
     * · 中断里只搬 3 个字节进状态，不做任何等待，等待放在发送侧
     * · 覆盖式写：对端连发两个流控帧时保留最后一个，这正是我们要的 */
    if ((g_can_last_rx.id == CAN_GW_ISOTP_ID_RESP) &&
        (g_can_last_rx.dlc >= 3u) &&
        ((g_can_last_rx.data[0] & 0xF0u) == 0x30u))
    {
        s_isotp_fwd.fc_fs    = (uint8_t)(g_can_last_rx.data[0] & 0x0Fu);
        s_isotp_fwd.fc_bs    = g_can_last_rx.data[1];
        s_isotp_fwd.fc_stmin = g_can_last_rx.data[2];
        s_isotp_fwd.fc_ready = 1u;
    }
    /* ★ 0x101 遥测帧：温度乘 100、湿度乘 10、气压单位 Pa，都是小端
     * · 解码放在中断里：只有几次移位和赋值，不打印、不阻塞
     * · 别的 ID 先只看计数，后面加 STAT/ALARM 时在这里补分支 */
    if ((g_can_last_rx.id == 0x101u) && (g_can_last_rx.dlc >= 8u))
    {
        s_data.temp_x100 = (int16_t)((uint16_t)g_can_last_rx.data[0] |
                                     ((uint16_t)g_can_last_rx.data[1] << 8));
        s_data.humi_x10 = (uint16_t)((uint16_t)g_can_last_rx.data[2] |
                                     ((uint16_t)g_can_last_rx.data[3] << 8));
        s_data.press_pa = (uint32_t)g_can_last_rx.data[4] |
                          ((uint32_t)g_can_last_rx.data[5] << 8) |
                          ((uint32_t)g_can_last_rx.data[6] << 16) |
                          ((uint32_t)g_can_last_rx.data[7] << 24);

        s_data.frames++;
        s_data.last_tick = g_can_last_rx.tick_ms;
    }
    /* ★ 0x102 状态心跳：seq(2) + status(1) + proto_ver(1) + rtc(4)
     * · seq 连号说明没丢帧，跳号就把差额记进 drop
     * · 状态位含义按协议契约 4.4，这里只存原始字节，怎么解释交给显示层 */
    else if ((g_can_last_rx.id == 0x102u) && (g_can_last_rx.dlc >= 8u))
    {
        uint16_t seq = (uint16_t)((uint16_t)g_can_last_rx.data[0] |
                                  ((uint16_t)g_can_last_rx.data[1] << 8));

        if (s_data.stat_frames != 0u)
        {
            uint16_t expect = (uint16_t)(s_data.seq + 1u);

            if (seq != expect)
            {
                /* 只有"往前跳"才算丢帧
                 * 对面复位后 seq 从 1 重来，这里会算出一个接近 65535 的差值
                 * 不区分的话，对面每重启一次 drop 就凭空涨几万 */
                uint16_t gap = (uint16_t)(seq - expect);

                if (gap < 0x8000u)
                {
                    s_data.drop += gap;
                }
            }
        }

        s_data.seq = seq;
        s_data.status = g_can_last_rx.data[2];
        s_data.proto_ver = g_can_last_rx.data[3];

        /* ★ 2026-11-24：布局版本检查
         * · 两端帧内容不一致时，按错位解析出来的数看着像真的，只是荒谬
         * · 发现不匹配只标记，不拒绝接收 —— 拒绝会让界面全空，更难判断 */
        s_data.layout_ok = (uint8_t)((s_data.proto_ver == GW_LAYOUT_VER) ? 1u : 0u);
        if (s_data.layout_ok == 0u)
        {
            s_data.layout_bad++;
        }
        s_data.rtc = (uint32_t)g_can_last_rx.data[4] |
                     ((uint32_t)g_can_last_rx.data[5] << 8) |
                     ((uint32_t)g_can_last_rx.data[6] << 16) |
                     ((uint32_t)g_can_last_rx.data[7] << 24);
        s_data.stat_frames++;
    }
    /* ★ 命令 ACK（2026-11-24 加）：F103 收到 0x201 之后回 0x301
     * · data[0] 回显命令码，data[1] 是执行结果，0 表示成功
     * · 中断里只记状态不打印，主循环和命令通道自己来读 */
    else if ((g_can_last_rx.id == CAN_GW_ID_ACK) && (g_can_last_rx.dlc >= 2u))
    {
        s_cmd.ack_code   = g_can_last_rx.data[0];
        s_cmd.ack_result = g_can_last_rx.data[1];
        s_cmd.ack_tick   = g_can_last_rx.tick_ms;
        s_cmd.ack_cnt++;
    }
    /* ★ 故障告警（2026-11-24 加）：F103 在 Event_LogFail 时发 0x080
     * · data[0]=通道 data[1]=原因 data[2..3]=该通道累计次数（小端）
     * · 中断里只搬字段，判断和显示交给命令通道与界面 */
    else if ((g_can_last_rx.id == CAN_GW_ID_ALARM) && (g_can_last_rx.dlc >= 4u))
    {
        s_alarm.ch     = g_can_last_rx.data[0];
        s_alarm.reason = g_can_last_rx.data[1];
        s_alarm.count  = (uint16_t)((uint16_t)g_can_last_rx.data[2] |
                                    ((uint16_t)g_can_last_rx.data[3] << 8));
        s_alarm.tick   = g_can_last_rx.tick_ms;
        s_alarm.total++;
    }
    /* ★ 扩展帧（2026-11-24 加）：电池 / 光照 / 加速度 XY
     * · 全部小端，和 0x101 一致；加速度是有符号，按 int16 解 */
    else if ((g_can_last_rx.id == CAN_GW_ID_EXTRA) && (g_can_last_rx.dlc >= 8u))
    {
        /* ★ 2026-11-24 审计修：版本不匹配就一个字节都不解析
         * · 原来只记 layout_bad 却照常解 —— 旧布局的字节按新布局读，
         *   出来的数看着像真的、只是荒谬（电池 65408 mV），还会写进 Modbus 和网页
         * · 只挡扩展帧：0x101/0x102 与布局无关，它们的前 8 字节从没变过 */
        if (s_data.layout_ok == 0u)
        {
            return;
        }
        /* ★ 2026-11-24 重排：ax / ay / az 三轴同帧，vbat 挪到帧尾
         * · 三轴必须来自同一帧，否则姿态解算差一拍就歪 */
        s_data.ax      = (int16_t)((uint16_t)g_can_last_rx.data[0] |
                                   ((uint16_t)g_can_last_rx.data[1] << 8));
        s_data.ay      = (int16_t)((uint16_t)g_can_last_rx.data[2] |
                                   ((uint16_t)g_can_last_rx.data[3] << 8));
        s_data.az      = (int16_t)((uint16_t)g_can_last_rx.data[4] |
                                   ((uint16_t)g_can_last_rx.data[5] << 8));
        s_data.vbat_mv = (uint16_t)((uint16_t)g_can_last_rx.data[6] |
                                    ((uint16_t)g_can_last_rx.data[7] << 8));
        s_data.extra_frames++;
    }
    /* ★ 第四帧（2026-11-24 加）：加速度 Z / 角速度 XYZ，全部 int16 小端 */
    else if ((g_can_last_rx.id == CAN_GW_ID_EXTRA2) && (g_can_last_rx.dlc >= 8u))
    {
        /* 同上：布局版本不匹配时扩展帧一律不解 */
        if (s_data.layout_ok == 0u)
        {
            return;
        }
        /* ★ 2026-11-24 重排：gx / gy / gz 三轴同帧，light 挪到帧尾 */
        s_data.gx    = (int16_t)((uint16_t)g_can_last_rx.data[0] |
                                 ((uint16_t)g_can_last_rx.data[1] << 8));
        s_data.gy    = (int16_t)((uint16_t)g_can_last_rx.data[2] |
                                 ((uint16_t)g_can_last_rx.data[3] << 8));
        s_data.gz    = (int16_t)((uint16_t)g_can_last_rx.data[4] |
                                 ((uint16_t)g_can_last_rx.data[5] << 8));
        s_data.light = (uint16_t)((uint16_t)g_can_last_rx.data[6] |
                                  ((uint16_t)g_can_last_rx.data[7] << 8));
        s_data.extra2_frames++;
    }
}

/* ★ 取最新遥测值 */
const CanGwData_t *CanGw_GetData(void)
{
    return &s_data;
}

/* ★ 维护链路有效标志
 * · 判据只有一个：最近 500 毫秒内收到过 0x101
 * · 没有用错误中断来判断，因为总线安静时不会有错误，但也没数据 */
void CanGw_Poll(void)
{
    if (s_data.frames == 0u)
    {
        s_data.valid = 0u;
        return;
    }

    s_data.valid = ((HAL_GetTick() - s_data.last_tick) < CAN_GW_TIMEOUT_MS)
                   ? 1u : 0u;
}

/* ★ 错误状态变化
 * · 错误中断挂在 CAN1_SCE 上，这里只统计不处理
 * · 应答错误说明总线上只有自己一个节点，或者对端没上电
 * · bus-off 说明波特率不匹配或者 CANH/CANL 接反、短路
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t err = 0u;

    if (hcan->Instance != CAN1)
    {
        return;
    }

    err = HAL_CAN_GetError(hcan);
    g_can_last_err = err;
    g_can_err_cnt++;

    if ((err & HAL_CAN_ERROR_ACK) != 0u)
    {
        g_can_ack_err++;
    }
    if ((err & HAL_CAN_ERROR_BOF) != 0u)
    {
        g_can_busoff_cnt++;
    }
}

/* ★ 中断入口
 * · HAL 的中断处理在 stm32f4xx_hal_can.c 里，这里只做转发
 * · stm32f4xx_it.c 里没有 CAN 的中断函数，不会重复定义
 */
void CAN1_TX_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}
