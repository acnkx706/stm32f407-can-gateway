/**
  ******************************************************************************
  * @file    main.c
  * @brief   双板网关主程序：I2C 从机收 F103 遥测，屏上显示，再分三路出口
  ******************************************************************************
  * 做了什么：
  * · I2C 从机（PB8/PB9，地址 0x50）接收 F103 发来的 8 字节遥测帧
  * · LVGL 8.3 画三页（环境 / 网络 / 记录仪），底部导航按钮切页
  * · 三条出口：CAN 转发、UDS 诊断、以太网（5000 命令 / 502 Modbus / 80 HTTP）
  *
  * 为什么 I2C 用中断加短超时：
  * · 主循环每 5 ms 跑一次 lv_timer_handler，被堵住就掉帧
  * · 所以从机收包走中断，主循环只负责"取走已经收好的那一帧"
  *
  * 怎么验证：
  * · 屏上三页数值与串口打印一致，切页立刻是新值
  * · 串口 Q 命令里的各项计数持续增长，不回落
  ******************************************************************************
  */

#include "main.h"
#include "led.h"
#include "usart.h"
#include "lcd_spi_154.h"
#include "lv_port.h"
#include "w5500_port.h"
#include "net_tcp.h"               /* ★ 2026-11-22：TCP 连接与命令通道（5000） */

#include "modbus_tcp.h"
#include "web_srv.h"
#include "can_gw.h"           /* 2026-11-22: Modbus TCP 从站，协议层独立成文件 */


void SystemClock_Config(void);

/* ★ 数据源选择
 * 做什么：CAN_ENABLE = 1 时由 CAN 供数，I2C 从机不启用
 * 为什么：PB8/PB9 被 I2C1 和 CAN1 抢，物理上不能同时用；
 *         CAN 是正式链路，I2C 只是 11-22 之前的临时通路
 * 影响什么：关掉 I2C 后 F103 的 I2C 数据不再进来，一切靠 CAN
 * 怎么验证：串口出现 [CAN] init ok，且 rx 每秒增加 10
 */
#define CAN_ENABLE          1

#define I2C_LINK_ADDR7     0x50u
#define I2C_LINK_FRAME     8u

static I2C_HandleTypeDef hi2c1;

typedef struct
{
    int16_t  temp_x100;
    uint16_t humi_x10;
    uint32_t press_pa;

    /* 2026-11-24 加：0x103 帧带来的量 */
    uint16_t vbat_mv;
    uint16_t light;
    int16_t  ax;
    int16_t  ay;
    int16_t  az;
    int16_t  gx;
    int16_t  gy;
    int16_t  gz;
} GwReg_t;

static GwReg_t   g_reg;
static uint32_t  g_data_frames = 0u;
static uint32_t  g_i2c_err_count = 0u;

/* I2C 从机收帧缓冲
 * · HAL 的从机接收是"给一块内存、收满再中断"，所以必须常驻
 * · 不能用局部数组：回调返回时 HAL 还在往这块内存里写
 */
static uint8_t   s_i2c_buf[I2C_LINK_FRAME];

/* LVGL 页面对象与当前页 */
static lv_obj_t *s_pages[3];        /* 0=环境 1=网络 2=记录仪 */
static uint8_t   s_page_cur = 0u;
static uint32_t  s_btn_cnt = 0u;
/* ★ 新界面的配色（2026-11-24）
 * · 整屏只用这几个颜色，保证风格统一；要调色只改这里
 * · 用宏而不是常量表：lv_color_make 是 inline 函数，展开后不占 RAM
 */
#define UI_C_PANEL   lv_color_make(0x12, 0x18, 0x20)   /* 顶栏底色 */
#define UI_C_DIM     lv_color_make(0x8A, 0x8A, 0x8A)   /* 次要文字 */
#define UI_C_OK      lv_color_make(0x00, 0xE0, 0x70)   /* 正常 */
#define UI_C_WARN    lv_color_make(0xFF, 0xC0, 0x00)   /* 警告 */
#define UI_C_ERR     lv_color_make(0xFF, 0x40, 0x40)   /* 异常 */
#define UI_C_ACCENT  lv_color_make(0x00, 0xA0, 0xFF)   /* 强调（低温、湿度）*/
#define UI_C_OFF     lv_color_make(0x40, 0x40, 0x40)   /* 熄灭 */

/* ★ 新界面的控件句柄（2026-11-24）
 * · 和上面那批旧句柄并存：改造分三步走，第一步只加声明，旧的 Ui_Create 照常工作
 * · 三步走完再删旧句柄，中间任何一步都保持可编译
 */
static lv_obj_t *s_led_can;         /* 顶栏 CAN 状态灯 */
static lv_obj_t *s_led_net;         /* 顶栏网口状态灯 */
static lv_obj_t *s_lab_can;         /* 顶栏 CAN 文字 */
static lv_obj_t *s_lab_clock;       /* 顶栏时钟 */

static lv_obj_t *s_arc_temp;        /* 环境页的温度圆弧 */
static lv_obj_t *s_lab_temp_val;    /* 圆弧中间的大数字 */
/* 单位 C 是固定文字，不需要句柄更新，所以不占一个变量 */
static lv_obj_t *s_bar_humi;        /* 湿度条 */
static lv_obj_t *s_lab_humi_val;
static lv_obj_t *s_bar_press;       /* 气压条 */
static lv_obj_t *s_lab_press_val;

static lv_obj_t *s_led_tcp;         /* 网络页的 TCP 指示灯 */
static lv_obj_t *s_lab_tcp;
static lv_obj_t *s_chart_rate;      /* 帧率曲线 */
static lv_chart_series_t *s_ser_rate;
static lv_obj_t *s_lab_mb;
static lv_obj_t *s_lab_canf;        /* 网络页的帧率数字 */
static lv_obj_t *s_lab_mem;

static lv_obj_t *s_lab_rtc;         /* 记录仪页的 RTC */
static lv_obj_t *s_dot_led[8];      /* 8 个通道指示灯 */
static lv_obj_t *s_dot_lab[8];      /* 指示灯旁边的字母 */
static lv_obj_t *s_lab_seq;
static lv_obj_t *s_lab_ver;

static lv_obj_t *s_btns[3];         /* 底部导航按钮，切页时改高亮 */

/* I2C1 引脚：PB8=SCL / PB9=SDA，AF4，开漏
 * · 为什么开漏：I2C 是线与总线，靠外部上拉，推挽会顶死
 * · 板上上拉约 40k（偏大），所以把速率压到 50 kHz
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};

    if (hi2c->Instance != I2C1)
    {
        return;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* ★ I2C 中断优先级
     * · 从机收包必须走中断：主循环要刷屏，没空一直轮询
     * · 抢占优先级 6，只比 HAL 时基高：收帧不能被半路截断
     */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/* ★ I2C1 事件中断
 * · startup 里给的是 WEAK 定义，这里覆盖它，原样转给 HAL
 * · 不在中断里做业务：取数据放在 HAL 的完成回调里
 */
void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

static HAL_StatusTypeDef I2C_Link_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 50000u;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = (uint32_t)I2C_LINK_ADDR7 << 1;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0u;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    return HAL_I2C_Init(&hi2c1);
}

static void Bridge_Parse(const uint8_t *d)
{
    g_reg.temp_x100 = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
    g_reg.humi_x10 = (uint16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8));
    g_reg.press_pa  = (uint32_t)d[4]
                    | ((uint32_t)d[5] << 8)
                    | ((uint32_t)d[6] << 16)
                    | ((uint32_t)d[7] << 24);
}

/* ★ 挂上从机接收
 * · HAL 的 Slave_Receive_IT 是一次性的：收满一份就停，不会自动再挂
 * · 所以每取走一帧都要再调一次，否则只能收到第一帧
 */
static void I2C_Link_Arm(void)
{
    (void)HAL_I2C_Slave_Receive_IT(&hi2c1, s_i2c_buf, I2C_LINK_FRAME);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
    {
        return;
    }

    Bridge_Parse(s_i2c_buf);
    g_data_frames++;
    I2C_Link_Arm();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
    {
        return;
    }

    g_i2c_err_count++;

    /* ★ 出错后整机重建：HAL 的从机状态机容易卡在忙态，DeInit 是可靠的清法 */
    (void)HAL_I2C_DeInit(&hi2c1);
    if (I2C_Link_Init() == HAL_OK)
    {
        I2C_Link_Arm();
    }
}

/* ★ 旋钮按键按下时做什么
 * 做什么：计数加一、更新屏幕上的计数、翻转板上 LED、串口打印一行
 * 为什么：按键如果没有任何可见反馈，就分不清是没按到还是软件没响应
 * 影响什么：只改一个标签和一个 IO，不碰网络和数据链路
 * 怎么验证：每按一次，屏幕 P 后面的数字加 1，板上 LED 翻转一次
 */
/* ★ 切换页面
 * 做什么：在数据页和网络状态页之间切换，用隐藏标志控制
 * 为什么用隐藏而不是删了重建：重建要消耗 LVGL 堆，屏幕还会闪一下
 * 影响什么：只改两个容器的隐藏标志，不重新排版
 * 怎么验证：按旋钮，DATA 按钮回到数据页，NET 按钮切到网络页
 */
/* ★ 造一个标签（2026-11-24）
 * · 只做三件事：创建、设字体、设颜色；位置由调用者另外设
 * · 不做成"全部参数"的版本：这个界面只有二十来个标签，多一层封装反而难读
 */
static lv_obj_t *Ui_MkLabel(lv_obj_t *parent, const char *txt,
                            const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *o = lv_label_create(parent);

    lv_label_set_text(o, txt);
    lv_obj_set_style_text_font(o, font, 0);
    lv_obj_set_style_text_color(o, color, 0);

    return o;
}

/* ★ 造一个指示灯（2026-11-24）
 * · 必须清掉 CLICKABLE：lv_led 默认可点，留着会被编码器的 group 选中，
 *   滚轮一晃焦点就跑到灯上，按下去什么都不会发生，这种问题很难查
 */
static lv_obj_t *Ui_MkLed(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                          lv_coord_t size, lv_color_t color)
{
    lv_obj_t *o = lv_led_create(parent);

    lv_obj_set_size(o, size, size);
    lv_obj_set_pos(o, x, y);
    lv_led_set_color(o, color);
    lv_led_set_brightness(o, 255);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);

    return o;
}
static void Ui_SetPage(uint8_t page)
{
    uint8_t i;

    if (page > 2u)
    {
        page = 0u;
    }

    for (i = 0u; i < 3u; i++)
    {
        if (i == page)
        {
            lv_obj_clear_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    /* ★ 2026-11-24 加：当前页的导航按钮高亮，否则看不出自己在哪一页
     * · 只改背景色，不重建按钮，切页是瞬间的
     * · 必须判空：Ui_Create 末尾会调 Ui_SetPage(0)，那时按钮可能还没建出来 */
    for (i = 0u; i < 3u; i++)
    {
        if (s_btns[i] != NULL)
        {
            lv_obj_set_style_bg_color(s_btns[i],
                                      (i == page) ? lv_color_make(0x00, 0x6E, 0x3C)
                                                  : lv_color_make(0x1C, 0x1C, 0x1C), 0);
        }
    }

    s_page_cur = page;
}

/* ★ 旋钮按键按下时做什么
 * 做什么：切到按钮对应的页面，翻转板上 LED，串口打一行
 * 为什么：按键要有可见反馈，否则分不清是没按到还是软件没响应
 * 影响什么：只切页面和一个 IO，不碰网络和数据链路
 * 怎么验证：按 DATA 回到数据页，按 NET 切到网络页，LED 翻转
 */
static void Ui_BtnClicked(lv_event_t *e)
{
    uint32_t idx;

    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    {
        return;
    }

    /* 按钮编号在创建时通过 user_data 带进来：0 是数据页，1 是网络页 */
    idx = (uint32_t)(uintptr_t)lv_event_get_user_data(e);

    s_btn_cnt++;
    Ui_SetPage((uint8_t)idx);

    HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);

    printf("[KEY] page %u, press %lu\r\n", (unsigned)idx,
           (unsigned long)s_btn_cnt);
}

static void Ui_Create(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_t *panel;
    lv_obj_t *btn;
    lv_obj_t *lab;
    lv_group_t *grp;
    lv_indev_t *indev;
    uint8_t i;

    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* ---- top status bar: CAN led + clock + net led ---- */
    panel = lv_obj_create(scr);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, 240, 26);
    lv_obj_set_pos(panel, 0, 0);
    lv_obj_set_style_bg_color(panel, UI_C_PANEL, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    s_led_can = Ui_MkLed(panel, 7, 8, 10, UI_C_ERR);
    s_lab_can = Ui_MkLabel(panel, "CAN", &lv_font_montserrat_14, UI_C_DIM);
    lv_obj_set_pos(s_lab_can, 21, 5);

    s_lab_clock = Ui_MkLabel(panel, "--:--:--", &lv_font_montserrat_16,
                             lv_color_white());
    lv_obj_set_pos(s_lab_clock, 86, 4);

    s_led_net = Ui_MkLed(panel, 196, 8, 10, UI_C_ERR);
    lab = Ui_MkLabel(panel, "NET", &lv_font_montserrat_14, UI_C_DIM);
    lv_obj_set_pos(lab, 210, 5);

    /* ---- three page containers, stacked, switched by hidden flag ---- */
    for (i = 0u; i < 3u; i++)
    {
        s_pages[i] = lv_obj_create(scr);
        lv_obj_remove_style_all(s_pages[i]);
        lv_obj_clear_flag(s_pages[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(s_pages[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(s_pages[i], 240, 158);
        lv_obj_set_pos(s_pages[i], 0, 26);
    }

    /* ---- page 0: environment, arc gauge + two bars ---- */
    s_arc_temp = lv_arc_create(s_pages[0]);
    lv_obj_set_size(s_arc_temp, 124, 124);
    lv_obj_set_pos(s_arc_temp, 58, 0);
    lv_arc_set_range(s_arc_temp, 0, 60);        /* 0 = -10 C, 60 = 50 C */
    lv_arc_set_bg_angles(s_arc_temp, 135, 45);  /* gap at the bottom */
    lv_arc_set_value(s_arc_temp, 30);
    lv_obj_remove_style(s_arc_temp, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(s_arc_temp, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(s_arc_temp, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_arc_temp, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_arc_temp, lv_color_make(0x28, 0x28, 0x28),
                               LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_arc_temp, UI_C_OK, LV_PART_INDICATOR);

    /* ★ 2026-11-24 改：数字和单位放进一个 flex 容器再整体居中
     * · 原来两个标签各自 align_to 圆心，数字位数一变（9.9 / -12.3）整体就不居中，
     *   而且 lv_obj_align_to 对 arc 这种粗描边对象算出来的中心本来就偏
     * · 换成容器 + 横向 flex：宽度自适应，永远落在弧的正中间 */
    {
        lv_obj_t *mid = lv_obj_create(s_pages[0]);

        lv_obj_remove_style_all(mid);
        lv_obj_clear_flag(mid, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(mid, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(mid, 124, 40);
        lv_obj_align(mid, LV_ALIGN_TOP_MID, 0, 42);
        lv_obj_set_flex_flow(mid, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(mid, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        s_lab_temp_val = Ui_MkLabel(mid, "--", &lv_font_montserrat_28,
                                    lv_color_white());
        (void)Ui_MkLabel(mid, "C", &lv_font_montserrat_14, UI_C_DIM);
    }

    lab = Ui_MkLabel(s_pages[0], "H", &lv_font_montserrat_14, UI_C_DIM);
    lv_obj_set_pos(lab, 6, 128);

    s_bar_humi = lv_bar_create(s_pages[0]);
    lv_obj_set_size(s_bar_humi, 148, 12);
    lv_obj_set_pos(s_bar_humi, 22, 129);
    lv_bar_set_range(s_bar_humi, 0, 1000);
    lv_bar_set_value(s_bar_humi, 500, LV_ANIM_OFF);
    lv_obj_clear_flag(s_bar_humi, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(s_bar_humi, lv_color_make(0x28, 0x28, 0x28),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_bar_humi, UI_C_ACCENT, LV_PART_INDICATOR);

    s_lab_humi_val = Ui_MkLabel(s_pages[0], "--.-%", &lv_font_montserrat_14,
                                lv_color_white());
    lv_obj_set_pos(s_lab_humi_val, 176, 128);

    lab = Ui_MkLabel(s_pages[0], "P", &lv_font_montserrat_14, UI_C_DIM);
    lv_obj_set_pos(lab, 6, 143);

    s_bar_press = lv_bar_create(s_pages[0]);
    lv_obj_set_size(s_bar_press, 148, 12);
    lv_obj_set_pos(s_bar_press, 22, 144);
    lv_bar_set_range(s_bar_press, 0, 200);      /* 950 .. 1150 hPa */
    lv_bar_set_value(s_bar_press, 100, LV_ANIM_OFF);
    lv_obj_clear_flag(s_bar_press, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(s_bar_press, lv_color_make(0x28, 0x28, 0x28),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_bar_press, UI_C_ACCENT, LV_PART_INDICATOR);

    s_lab_press_val = Ui_MkLabel(s_pages[0], "----.--", &lv_font_montserrat_14,
                                 lv_color_white());
    lv_obj_set_pos(s_lab_press_val, 176, 143);

    /* ---- page 1: network, tcp badge + frame rate chart ---- */
    s_led_tcp = Ui_MkLed(s_pages[1], 8, 8, 12, UI_C_ERR);
    s_lab_tcp = Ui_MkLabel(s_pages[1], "TCP DOWN", &lv_font_montserrat_14, UI_C_DIM);
    lv_obj_set_pos(s_lab_tcp, 26, 5);

    s_chart_rate = lv_chart_create(s_pages[1]);
    lv_obj_set_size(s_chart_rate, 200, 76);
    lv_obj_set_pos(s_chart_rate, 20, 26);
    lv_chart_set_type(s_chart_rate, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_chart_rate, 30);
    lv_chart_set_range(s_chart_rate, LV_CHART_AXIS_PRIMARY_Y, 0, 60);
    lv_chart_set_div_line_count(s_chart_rate, 3, 0);
    lv_obj_clear_flag(s_chart_rate, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(s_chart_rate, lv_color_make(0x0C, 0x12, 0x18), 0);
    lv_obj_set_style_border_width(s_chart_rate, 1, 0);
    lv_obj_set_style_border_color(s_chart_rate, lv_color_make(0x30, 0x38, 0x40), 0);

    s_ser_rate = lv_chart_add_series(s_chart_rate, UI_C_OK,
                                     LV_CHART_AXIS_PRIMARY_Y);

    s_lab_mb = Ui_MkLabel(s_pages[1], "Modbus --", &lv_font_montserrat_14,
                          lv_color_white());
    lv_obj_set_pos(s_lab_mb, 10, 108);

    s_lab_canf = Ui_MkLabel(s_pages[1], "CAN -- f/s", &lv_font_montserrat_14,
                            lv_color_white());
    lv_obj_set_pos(s_lab_canf, 10, 126);

    s_lab_mem = Ui_MkLabel(s_pages[1], "HEAP --", &lv_font_montserrat_14, UI_C_DIM);
    lv_obj_set_pos(s_lab_mem, 10, 144);

    /* ---- page 2: recorder, 8 channel leds + two lines ---- */
    s_lab_rtc = Ui_MkLabel(s_pages[2], "RTC ----", &lv_font_montserrat_16,
                           lv_color_white());
    lv_obj_set_pos(s_lab_rtc, 10, 2);

    {
        static const char *nm[8] =
        {
            "T", "H", "P", "I", "S", "R", "A", "B"
        };

        for (i = 0u; i < 8u; i++)
        {
            lv_coord_t x = (lv_coord_t)(14 + (i & 3u) * 56);
            lv_coord_t y = (lv_coord_t)(32 + (i >> 2) * 30);

            s_dot_led[i] = Ui_MkLed(s_pages[2], x, y, 14, UI_C_OFF);
            s_dot_lab[i] = Ui_MkLabel(s_pages[2], nm[i], &lv_font_montserrat_16,
                                      UI_C_DIM);
            lv_obj_set_pos(s_dot_lab[i], x + 20, y - 2);
        }
    }

    s_lab_seq = Ui_MkLabel(s_pages[2], "SEQ ----   DROP --",
                           &lv_font_montserrat_14, lv_color_white());
    lv_obj_set_pos(s_lab_seq, 10, 104);

    s_lab_ver = Ui_MkLabel(s_pages[2], "VER --   STAT 0x--",
                           &lv_font_montserrat_14, lv_color_white());
    lv_obj_set_pos(s_lab_ver, 10, 126);

    /* ---- bottom navigation ---- */
    grp = lv_group_create();

    {
        /* ★ 只用字体里确实有字形的符号（2026-11-24 查过）
         * · 所有 montserrat 只带 24 个符号，带名字的只有 HOME/DOWNLOAD/TINT/SHUFFLE
         * · 原来想用 WIFI 和 LIST，那两个码点字体里没有，会显示成方框 */
        static const char *ico[3] =
        {
            LV_SYMBOL_TINT, LV_SYMBOL_SHUFFLE, LV_SYMBOL_DOWNLOAD
        };
        static const char *cap[3] =
        {
            "ENV", "NET", "REC"
        };

        for (i = 0u; i < 3u; i++)
        {
            btn = lv_btn_create(scr);
            lv_obj_set_size(btn, 74, 46);
            lv_obj_set_pos(btn, (lv_coord_t)(3 + i * 80), 190);
            lv_obj_set_style_bg_color(btn, lv_color_make(0x1C, 0x1C, 0x1C), 0);
            lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
            lv_obj_set_style_radius(btn, 6, 0);
            lv_obj_set_style_border_width(btn, 0, 0);
            lv_obj_set_style_shadow_width(btn, 0, 0);

            lab = Ui_MkLabel(btn, ico[i], &lv_font_montserrat_20, lv_color_white());
            lv_obj_align(lab, LV_ALIGN_TOP_MID, 0, 4);

            lab = Ui_MkLabel(btn, cap[i], &lv_font_montserrat_14, UI_C_DIM);
            lv_obj_align(lab, LV_ALIGN_BOTTOM_MID, 0, -4);

            lv_obj_add_event_cb(btn, Ui_BtnClicked, LV_EVENT_CLICKED,
                                (void *)(uintptr_t)i);
            lv_group_add_obj(grp, btn);

            s_btns[i] = btn;
        }
    }

    indev = lv_indev_get_next(NULL);
    if (indev != NULL)
    {
        lv_indev_set_group(indev, grp);
    }

    /* buttons must exist before this call: Ui_SetPage touches s_btns */
    Ui_SetPage(0u);
}


/* ★ 刷新屏幕上两页的数值
 * 做什么：数据页跟数据帧走，网络页每 500 毫秒刷一次
 * 为什么分开：数据页变化慢没必要每 5 毫秒重绘，网络页统计量变化也慢
 * 影响什么：只改标签文字，LVGL 自己决定哪里需要重画
 * 怎么验证：两页数值与串口打印的一致，切换页面时立刻是新值
 */
static void Ui_Update(uint8_t link_up)
{
    static uint32_t last_frames = 0xFFFFFFFFu;
    static uint32_t head_tick   = 0u;
    static uint32_t rate_tick   = 0u;
    static uint32_t rate_last   = 0u;

    char buf[48];
    const CanGwData_t *cd;
    lv_mem_monitor_t mon;
    uint8_t st;
    uint8_t i;

    cd = CanGw_GetData();

    /* ---- top bar: every 200 ms, the clock is second-resolution ---- */
    if ((HAL_GetTick() - head_tick) >= 200u)
    {
        uint32_t t;

        head_tick = HAL_GetTick();

        /* dim instead of off when the link is down, so you can tell
         * "not connected" apart from "led broken" */
        lv_led_set_color(s_led_can, (cd->valid != 0u) ? UI_C_OK : UI_C_ERR);
        lv_led_set_brightness(s_led_can, (cd->valid != 0u) ? 220u : 70u);

        lv_led_set_color(s_led_net, (link_up != 0u) ? UI_C_OK : UI_C_ERR);
        lv_led_set_brightness(s_led_net, (link_up != 0u) ? 220u : 70u);

        t = cd->rtc % 86400u;
        (void)lv_snprintf(buf, sizeof(buf), "%02u:%02u:%02u",
                          (unsigned)(t / 3600u),
                          (unsigned)((t / 60u) % 60u),
                          (unsigned)(t % 60u));
        lv_label_set_text(s_lab_clock, buf);
    }

    /* ---- page 0: environment, the arc follows the data frames ---- */
    if (s_page_cur == 0u)
    {
        if (g_data_frames != last_frames)
        {
            int32_t  tv;
            int32_t  tick;
            uint16_t hv;
            int32_t  pv;
            uint8_t  t_bad;

            last_frames = g_data_frames;
            tv = (int32_t)g_reg.temp_x100;
            hv = g_reg.humi_x10;
            pv = (int32_t)(g_reg.press_pa / 100u);

            /* ★ 2026-11-24 加：坏值一律显示 --，不硬算
             * · F103 传感器失败时发 0xFFFF（湿度）或 0x8000（温度），
             *   硬算会变成 6553.5% 这种数，宽度还会把 240 像素的屏顶出去
             * · 判据按物理量程：DS18B20 -55~125 度，湿度 0~100%，气压 300~1100 hPa
             * · 气压多判一个下界：0 多半是链路上没数据，不是真的没气压 */
            t_bad = (uint8_t)(((tv < -5500) || (tv > 12500)) ? 1u : 0u);

            if (t_bad != 0u)
            {
                lv_label_set_text(s_lab_temp_val, "--");
                tick = 0;
            }
            else
            {
                (void)lv_snprintf(buf, sizeof(buf), "%ld.%02ld",
                                  (long)(tv / 100),
                                  (long)((tv < 0 ? -tv : tv) % 100));
                lv_label_set_text(s_lab_temp_val, buf);

                /* 60 ticks cover -10 .. 50 C; divide first so -0.5 does not
                 * get rounded up into the next tick */
                tick = (tv / 100) + 10;
                if (tick < 0)  { tick = 0; }
                if (tick > 60) { tick = 60; }
            }
            lv_arc_set_value(s_arc_temp, (int16_t)tick);

            if (hv > 1000u)
            {
                lv_bar_set_value(s_bar_humi, 0, LV_ANIM_OFF);
                lv_label_set_text(s_lab_humi_val, "--.-%");
            }
            else
            {
                lv_bar_set_value(s_bar_humi, (int32_t)hv, LV_ANIM_OFF);
                (void)lv_snprintf(buf, sizeof(buf), "%u.%u%%",
                                  (unsigned)(hv / 10u),
                                  (unsigned)(hv % 10u));
                lv_label_set_text(s_lab_humi_val, buf);
            }

            /* the pressure bar only spans 950 .. 1150 hPa. A bar covering
             * the full 300 .. 1100 range barely moves in this window */
            if ((pv < 300) || (pv > 1100))
            {
                lv_bar_set_value(s_bar_press, 0, LV_ANIM_OFF);
                lv_label_set_text(s_lab_press_val, "----.--");
            }
            else
            {
                tick = pv - 950;
                if (tick < 0)   { tick = 0; }
                if (tick > 200) { tick = 200; }
                lv_bar_set_value(s_bar_press, (int32_t)tick, LV_ANIM_OFF);

                (void)lv_snprintf(buf, sizeof(buf), "%u.%02u",
                                  (unsigned)(g_reg.press_pa / 100u),
                                  (unsigned)(g_reg.press_pa % 100u));
                lv_label_set_text(s_lab_press_val, buf);
            }

            /* blue below 10 C, yellow above 30 C, green in between；
             * 坏值时弧整圈压暗，一眼能看出"这条数据没有" */
            if (t_bad != 0u)
            {
                lv_obj_set_style_arc_color(s_arc_temp, UI_C_OFF,
                                           LV_PART_INDICATOR);
            }
            else if ((tv / 100) < 10)
            {
                lv_obj_set_style_arc_color(s_arc_temp, UI_C_ACCENT,
                                           LV_PART_INDICATOR);
            }
            else if ((tv / 100) > 30)
            {
                lv_obj_set_style_arc_color(s_arc_temp, UI_C_WARN,
                                           LV_PART_INDICATOR);
            }
            else
            {
                lv_obj_set_style_arc_color(s_arc_temp, UI_C_OK,
                                           LV_PART_INDICATOR);
            }
        }

        return;
    }

    /* ---- page 1: network ---- */
    if (s_page_cur == 1u)
    {
        if ((HAL_GetTick() - rate_tick) >= 1000u)
        {
            uint32_t r;

            rate_tick = HAL_GetTick();
            r = cd->frames - rate_last;
            rate_last = cd->frames;

            if (r > 60u) { r = 60u; }
            lv_chart_set_next_value(s_chart_rate, s_ser_rate, (lv_coord_t)r);

            (void)lv_snprintf(buf, sizeof(buf), "CAN %lu f/s", (unsigned long)r);
            lv_label_set_text(s_lab_canf, buf);
        }

        lv_led_set_color(s_led_tcp, (g_tcp_connected != 0u) ? UI_C_OK : UI_C_ERR);
        lv_led_set_brightness(s_led_tcp, (g_tcp_connected != 0u) ? 220u : 70u);
        lv_label_set_text(s_lab_tcp, (g_tcp_connected != 0u) ? "TCP CONNECTED"
                                                             : "TCP DOWN");

        (void)lv_snprintf(buf, sizeof(buf), "Modbus %lu req / %lu exc",
                          (unsigned long)g_mb_req_cnt,
                          (unsigned long)g_mb_exc_cnt);
        lv_label_set_text(s_lab_mb, buf);

        lv_mem_monitor(&mon);
        (void)lv_snprintf(buf, sizeof(buf), "HEAP free %lu B",
                          (unsigned long)mon.free_size);
        lv_label_set_text(s_lab_mem, buf);

        return;
    }

    /* ---- page 2: recorder, one led per status bit ---- */
    st = cd->status;

    (void)lv_snprintf(buf, sizeof(buf), "RTC %lu", (unsigned long)cd->rtc);
    lv_label_set_text(s_lab_rtc, buf);

    for (i = 0u; i < 8u; i++)
    {
        uint8_t on = (uint8_t)((st >> i) & 0x01u);

        lv_led_set_color(s_dot_led[i], (on != 0u) ? UI_C_OK : UI_C_OFF);
        lv_led_set_brightness(s_dot_led[i], (on != 0u) ? 220u : 45u);
        lv_obj_set_style_text_color(s_dot_lab[i],
                                    (on != 0u) ? lv_color_white() : UI_C_DIM, 0);
    }

    (void)lv_snprintf(buf, sizeof(buf), "SEQ %u    DROP %lu",
                      (unsigned)cd->seq, (unsigned long)cd->drop);
    lv_label_set_text(s_lab_seq, buf);

    (void)lv_snprintf(buf, sizeof(buf), "VER %u    STAT 0x%02X",
                      (unsigned)cd->proto_ver, (unsigned)st);
    lv_label_set_text(s_lab_ver, buf);
}


/* ★ 把采集值推给 Modbus 协议层
 * 做什么：把 I2C 收到的温湿度气压和计数器打包成 Modbus_Data_t 交出去
 * 为什么单独成函数：以后接上 CAN，数据源从这里换掉，协议层一行都不用改
 * 影响什么：只做一次结构体拷贝，不碰硬件，调用开销可以忽略
 * 怎么验证：主站读寄存器 0 得到温度，和串口打印的 T 值一致
 */
/* ★ 把 CAN 解出来的遥测同步到 g_reg
 * 做什么：每 5 毫秒从 can_gw 取一次最新值，写进 g_reg 和帧计数
 * 为什么：屏幕、网页、Modbus 全都读 g_reg，只在这一个地方对接数据源
 * 影响什么：只做结构体赋值，不碰硬件；没收到帧时保持上一次的值
 * 怎么验证：屏幕/网页/Modbus 三处数值应与 F103 串口上报一致
 */
static void App_SyncFromCan(void)
{
    const CanGwData_t *d = CanGw_GetData();

    if (d->frames == 0u)
    {
        return;
    }

    g_reg.temp_x100 = d->temp_x100;
    g_reg.humi_x10 = d->humi_x10;
    g_reg.press_pa = d->press_pa;
    g_data_frames = d->frames;

    /* 2026-11-24：扩展帧的量单独搬，收不到时保持上一次的值 */
    g_reg.vbat_mv = d->vbat_mv;
    g_reg.light   = d->light;
    g_reg.ax      = d->ax;
    g_reg.ay      = d->ay;
    g_reg.az      = d->az;
    g_reg.gx      = d->gx;
    g_reg.gy      = d->gy;
    g_reg.gz      = d->gz;
}
static void Modbus_PushData(uint8_t link_up)
{
    Modbus_Data_t d;

    d.temp_x100 = g_reg.temp_x100;
    d.humi_x10  = g_reg.humi_x10;
    d.press_pa  = g_reg.press_pa;
    d.i2c_rx    = g_data_frames;
    d.i2c_err   = g_i2c_err_count;
    d.link_up   = link_up;
    d.vbat_mv   = g_reg.vbat_mv;
    d.light     = g_reg.light;
    d.ax        = g_reg.ax;
    d.ay        = g_reg.ay;
    d.az        = g_reg.az;
    d.gx        = g_reg.gx;
    d.gy        = g_reg.gy;
    d.gz        = g_reg.gz;

    Modbus_SetData(&d);
}


int main(void)
{
    uint32_t last_tick = 0u;
    uint32_t last_stat = 0u;
    uint32_t last_link = 0u;
    uint8_t  last_link_state = 0xFFu;

    HAL_Init();
    USART1_Init();
    printf("\r\n\r\n[BOOT] LVGL + I2C\r\n");

    SystemClock_Config();
    USART1->BRR = (uint16_t)((84000000u + 57600u) / 115200u);
    printf("[CLOCK] ok, SYSCLK=%lu Hz\r\n", (unsigned long)HAL_RCC_GetHCLKFreq());

#if (CAN_ENABLE == 0)
    if (I2C_Link_Init() != HAL_OK)
    {
        printf("[I2C] init FAIL\r\n");
        while (1)
        {
        }
    }
    printf("[I2C] slave ok, addr=0x%02X\r\n", (unsigned)I2C_LINK_ADDR7);

    /* ★ 先挂一次从机接收
     * · 挂上之后要等它收满一份才会回调，所以开机就要挂
     * · 之后每取走一帧都要再挂一次
     */
    I2C_Link_Arm();
#else
    printf("[CAN] I2C slave disabled, PB8/PB9 -> CAN1\r\n");
#endif

    LED_Init();
    SPI_LCD_Init();
    printf("[LCD] init ok\r\n");

#if (CAN_ENABLE != 0)
    if (CanGw_Init(0u) == HAL_OK)
    {
        printf("[CAN] init ok, 500 kbps, PB8=RX PB9=TX\r\n");
    }
    else
    {
        printf("[CAN] init FAIL at step %u\r\n", (unsigned)g_can_init_step);
    }
#endif

    /* 2026-11-24 加：第二路 CAN（PB12/PB13）的回环自检
     * 做了什么：把 CAN2 配成内部回环，发一帧再收回，结果存进 g_can2_self_test
     * 为什么先用回环：收发器还没到、总线还断着，回环不依赖任何外部硬件，
     *   能把"CAN2 外设/引脚有问题"和"硬件没到"这两件事分开
     * 怎么验证：TCP 敲 Q，末尾 c2=0 表示通过，非 0 是出错步骤号
     * 真机到货后改成 CanGw_Init2(0u)，走正常总线模式
     */
#if (CAN_ENABLE != 0)
    if (CanGw_Init2(1u) == HAL_OK)
    {
        g_can2_self_test = CanGw_SelfTest2();
        printf("[CAN2] loopback selftest = %u (0=pass)\r\n", (unsigned)g_can2_self_test);
    }
    else
    {
        g_can2_self_test = 0xFEu;
        printf("[CAN2] init FAIL\r\n");
    }
#endif

    /* ★ 2026-11-22：W5500 上电初始化
     * 做了什么：顺序做四步，SPI 引脚 到 硬复位 到 读版本寄存器 到 写读 socket 寄存器
     * 为什么分步：一步到位出错看不出是哪一段坏的，step 停在几就是第几步
     * 影响什么：socket 0 的 2KB 收发缓冲被占用
     */
    if (W5500_PortInit() == 0)
    {
        (void)W5500_PortTest();
        /* W5500_OpenBurstTest(); */
        (void)W5500_NetInit();

        /* ★ 2026-11-23：socket 0 从"回显服务器"改成"网关命令通道"
         * 做什么：让电脑能往 CAN 总线打帧，F103 的 ISO-TP 才有对手方
         * 影响什么：回显没了，PING 顶替冒烟测试，测试脚本改用命令通道
         * 怎么验证：telnet 5000 敲 PING 回 PONG，敲 Q 回一行计数 */
        if (Net_TcpInit() != 0) printf("[TCP] init FAIL\r\n");
        else printf("[TCP] cmd channel on port %u (PING / Q / CAN)\r\n", NET_TCP_CMD_PORT);

        /* ★ 2026-11-22 新增：Modbus TCP 从站
         * 做什么：在 socket 1 上开 502 端口，对外提供 16 个保持寄存器
         * 为什么放在这里：Net_TcpInit 内部已经等过 PHY 链路，此时链路必 UP
         * 影响什么：只占 socket 1，socket 0 的回显不受影响
         * 怎么验证：串口出现 [MODBUS] slave on port 502, 16 regs
         */
        if (Modbus_TcpInit() != 0)
        {
            printf("[MODBUS] init FAIL\r\n");
        }
        else
        {
            printf("[MODBUS] slave on port %u, %u regs\r\n",
                   MODBUS_TCP_PORT, MODBUS_REG_COUNT);
        }

        /* ★ 2026-11-23 新增：HTTP 服务器
         * 做什么：在 socket 2 上开 80 端口，浏览器能看到实时数据
         * 为什么放在这里：和 Modbus 同理，必须等 PHY 链路 UP 之后再开 socket
         * 影响什么：只占 socket 2，页面压在 2 KB 以内，不动缓冲分配
         * 怎么验证：浏览器打开 http://192.168.10.50/ ，串口出现 [WEB] GET /
         */
        if (Web_SrvInit() != 0)
        {
            printf("[WEB] init FAIL\r\n");
        }
        else
        {
            printf("[WEB] http server on port %u\r\n", WEB_SRV_PORT);
        }
    }
    else
    {
        printf("[W5500] init FAIL at step %u\r\n",
               (unsigned)W5500_PortLastStep());
    }

    lv_port_init();
    Ui_Create();

    /* 临时诊断：量一下 LVGL 堆实际用了多少，测完删
     * LV_MEM_SIZE 现在配的是 64 KB，如果只用了几 KB 就能大幅缩减 */
    {
        lv_mem_monitor_t mon;

        lv_mem_monitor(&mon);
        printf("[MEM] total=%lu free=%lu used=%lu max=%lu %u%%\r\n",
               (unsigned long)mon.total_size,
               (unsigned long)mon.free_size,
               (unsigned long)(mon.total_size - mon.free_size),
               (unsigned long)mon.max_used,
               (unsigned)mon.used_pct);
    }
    printf("[LVGL] init ok, %dx%d\r\n", 240, 240);

    printf("[LOOP] running\r\n");

    printf("[LOOP] enter while\r\n");

    while (1)
    {
        /* ★ 主循环只做不阻塞的事
         * · 一旦被某件事堵住：屏会卡、命令会迟、告警会漏
         * · 所以每件事都必须"查一次就走"，或者自带超时
         */

        /* ★ LVGL 计时
         * · 每 5 ms 喂一次 lv_timer_handler：慢了掉帧，快了白烧 CPU
         */
        if ((HAL_GetTick() - last_tick) >= 5u)
        {
            last_tick = HAL_GetTick();
            lv_timer_handler();
            Ui_Update((last_link_state == 1u) ? 1u : 0u);

#if (CAN_ENABLE != 0)
            CanGw_Poll();
            App_SyncFromCan();
#endif

            Modbus_PushData((last_link_state == 1u) ? 1u : 0u);
        }

        /* ★ 2026-11-22：三个网络服务的轮询
         * · 每个都只查一次就返回，不在这里等任何一家的数据
         * · 顺序固定：TCP 连接 到 Modbus 到 HTTP，界面更新排在它们之前
         */
        Net_TcpPoll();
        Modbus_TcpPoll();
        Web_SrvPoll();

        /* 2026-11-22: the old loop hammered W5500 registers at ~56k SPI
         * transactions/sec with no gap.  At SCLK=21 MHz that starves the
         * chip's internal command arbiter and makes Sn_CR commands flaky.
         * A 1 ms gap adds negligible TCP latency but cuts poll rate ~50x. */
        HAL_Delay(1u);

        if ((HAL_GetTick() - last_link) >= 1000u)
        {
            uint8_t link;

            last_link = HAL_GetTick();
            link = W5500_LinkUp();

            if (link != last_link_state)
            {
                last_link_state = link;
                printf("[W5500] LINK %s\r\n", (link != 0u) ? "UP" : "DOWN");
            }
        }

        if ((HAL_GetTick() - last_stat) >= 1000u)
        {
            last_stat = HAL_GetTick();
            printf("RX=%lu err=%lu | T=%d.%02u H=%u.%u P=%lu hPa\r\n",
                   (unsigned long)g_data_frames,
                   (unsigned long)g_i2c_err_count,
                   (int)(g_reg.temp_x100 / 100),
                   (unsigned)((g_reg.temp_x100 < 0 ? -g_reg.temp_x100 : g_reg.temp_x100) % 100),
                   (unsigned)(g_reg.humi_x10 / 10u),
                   (unsigned)(g_reg.humi_x10 % 10u),
                   (unsigned long)(g_reg.press_pa / 100u));

#if (CAN_ENABLE != 0)
            /* 每秒一行 CAN 链路状态
             * 解码已经搬到 can_gw.c 的接收中断里，这里只看计数 */
            {
                const CanGwData_t *d = CanGw_GetData();

                printf("[CAN] rx=%lu st=%lu ack=%lu link=%u | seq=%u drop=%lu "
                       "ver=%u st=0x%02X rtc=%lu\r\n",
                       (unsigned long)d->frames,
                       (unsigned long)d->stat_frames,
                       (unsigned long)g_can_ack_err,
                       (unsigned)d->valid,
                       (unsigned)d->seq,
                       (unsigned long)d->drop,
                       (unsigned)d->proto_ver,
                       (unsigned)d->status,
                       (unsigned long)d->rtc);
            }
#endif
        }
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState      = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState  = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM      = 8;
    RCC_OscInitStruct.PLL.PLLN      = 336;
    RCC_OscInitStruct.PLL.PLLP      = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ      = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
