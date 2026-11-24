/**
  ******************************************************************************
  * @file    lv_port.c
  * @brief   LVGL 移植层：ST7789 显示 + EC11 编码器输入
  ******************************************************************************
  * ★ 显示怎么接
  * · LVGL 把屏幕切成若干条带渲染，每条渲染完调一次 flush_cb
  * · flush_cb 里用 LCD_SetAddress 定窗口，再用 LCD_WriteBuff 整块发出去
  * · 底层驱动在 LCD_WriteBuff 里把 SPI 切成 16 位模式，正好是 RGB565 大端序，
  *   所以不需要额外做字节交换
  *
  * ★ 缓冲为什么用 40 行
  * · 240 x 40 x 2 字节 = 19.2 KB，占 SRAM 的 10%
  * · 条带太短 → flush 次数多，SPI 启动开销占比高
  * · 条带太长 → 单次刷新阻塞时间变长，界面响应变钝
  *
  * ★ 编码器怎么接
  * · TIM4 编码器模式 TI12 四倍频，PB6/PB7
  * · 一格 = 4 个计数，换算成 1 个档位交给 LVGL 的 enc_diff
  *
  * ★ 怎么验证
  * · 屏幕出现 LVGL 画的控件
  * · 转旋钮能切换焦点高亮
  ******************************************************************************
  */

#include "lv_port.h"
#include "lcd_spi_154.h"
#include "stm32f4xx_hal.h"

/* ★ 这个函数在 lcd_spi_154.c 里实现了，但没写进头文件
 * · 不声明的话编译器按隐式声明处理，参数和返回值都对不上
 * · 这里补一个 extern，避免改驱动头文件
 */
extern void LCD_WriteBuff(uint16_t *DataBuff, uint16_t DataSize);

/* ★ 显示缓冲：单缓冲 40 条带
 * · 双缓冲能进一步提速，但要多占 19.2 KB，第一版先不上
 */
#define DISP_HOR_RES     240
#define DISP_VER_RES     240
#define DISP_BUF_LINES   40

/* ★ 按键接在哪个脚，只改这三行
 *  · 默认 PB13：PA15 在板上找不到（多半丝印写成 JTDI 或没引出来）
 *  · PB13 原来是 W5500 的 SPI2 时钟脚，换到 SPI1 之后空出来了
 *  · 端口必须和时钟宏一起改，GPIOA 配成 GPIOB 是配不通的
 *  · 绝不能接 PB5，那是 LCD 的数据线，接上去屏幕立刻全黑
 */
#define ENC_KEY_PORT    GPIOB
#define ENC_KEY_PIN     GPIO_PIN_13
#define ENC_KEY_CLK()   __HAL_RCC_GPIOB_CLK_ENABLE()

static lv_disp_draw_buf_t s_draw_buf;
static lv_color_t        s_buf1[DISP_HOR_RES * DISP_BUF_LINES];

/* ★ 编码器硬件 */
static TIM_HandleTypeDef s_htim4;
static int32_t           s_enc_last  = 0;
static int32_t           s_enc_accum = 0;

/* ★ 刷新回调
 * · 这里是同步阻塞发送，一帧最多 19.2 KB / (21 MHz / 16) ≈ 14.6 ms
 * · 后面要提性能的话可以改成 SPI DMA，这里是第一版先跑通
 */
static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    uint16_t w = (uint16_t)(area->x2 - area->x1 + 1);
    uint16_t h = (uint16_t)(area->y2 - area->y1 + 1);

    LCD_SetAddress((uint16_t)area->x1, (uint16_t)area->y1,
                   (uint16_t)area->x2, (uint16_t)area->y2);
    LCD_WriteBuff((uint16_t *)color_p, (uint16_t)(w * h));

    lv_disp_flush_ready(drv);
}

/* ★ 按键消抖
 * 做什么：连续 10 毫秒读数一致，才承认按键状态真的变了
 * 为什么：EC11 的按键是机械触点，按下和松开都会弹跳 5 到 10 毫秒，
 *         不去抖的话按一次会被 LVGL 当成按了好几次
 * 影响什么：最多多 10 毫秒响应延迟，短按不会显得拖沓
 * 怎么验证：按一次，屏幕上的计数只加 1
 */
static uint8_t Enc_KeyStable(void)
{
    static uint8_t  s_stable = 0u;
    static uint8_t  s_cand   = 0u;
    static uint32_t s_tick   = 0u;

    uint8_t raw = (HAL_GPIO_ReadPin(ENC_KEY_PORT, ENC_KEY_PIN) == GPIO_PIN_RESET)
                  ? 1u : 0u;

    if (raw != s_cand)
    {
        s_cand = raw;
        s_tick = HAL_GetTick();
    }
    else if ((HAL_GetTick() - s_tick) >= 10u)
    {
        s_stable = s_cand;
    }

    return s_stable;
}

/* ★ 编码器读取回调
 * · LVGL 每次刷新时来问"转了多少"
 * · 读 16 位计数器转成有符号差值，再除以 4 得到档位
 * · 不足一格的零头留着累积，不会丢
 */
static void encoder_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    int32_t now = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&s_htim4);
    int32_t delta = now - s_enc_last;

    s_enc_last   = now;
    s_enc_accum += delta;

    data->enc_diff = (int16_t)(s_enc_accum / 4);
    s_enc_accum   -= (int32_t)data->enc_diff * 4;

    /* 按键接在 PA15，低电平为按下
     * 这一行以前写死成 RELEASED，所以旋钮能转但按下去没反应，
     * 结果就是 LVGL 永远进不了菜单，只能来回切焦点 */
    data->state = (Enc_KeyStable() != 0u) ? LV_INDEV_STATE_PRESSED
                                          : LV_INDEV_STATE_RELEASED;
}

/* ★ 编码器自带的按键：PA15
 *  · 模块丝印 KEY，按键另一端接 GND，按下读到低电平
 *  · 必须开内部上拉，否则松开时引脚悬空，读数会乱跳
 *  · PA15 是 JTDI，本工程用 SWD 烧录只占 PA13/PA14，两者不冲突
 */
static void encoder_key_hw_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    ENC_KEY_CLK();

    gpio.Pin   = ENC_KEY_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ENC_KEY_PORT, &gpio);
}

/* ★ 编码器硬件初始化：TIM4 编码器模式 TI12
 * · PB6 = CH1、PB7 = CH2，AF2
 * · 上拉必须开：EC11 是机械触点，松开时靠上拉给出确定电平
 */
static void encoder_hw_init(void)
{
    GPIO_InitTypeDef        gpio = {0};
    TIM_Encoder_InitTypeDef enc  = {0};

    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    s_htim4.Instance               = TIM4;
    s_htim4.Init.Prescaler         = 0u;
    s_htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim4.Init.Period            = 0xFFFFu;
    s_htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = 0u;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = 0u;

    if (HAL_TIM_Encoder_Init(&s_htim4, &enc) != HAL_OK)
    {
        return;
    }

    HAL_TIM_Encoder_Start(&s_htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&s_htim4, 0u);
    s_enc_last = 0;
}

void lv_port_init(void)
{
    static lv_disp_drv_t  disp_drv;
    static lv_indev_drv_t indev_drv;

    lv_init();

    /* ---- 显示 ---- */
    lv_disp_draw_buf_init(&s_draw_buf, s_buf1, NULL,
                          DISP_HOR_RES * DISP_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = DISP_HOR_RES;
    disp_drv.ver_res  = DISP_VER_RES;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &s_draw_buf;
    (void)lv_disp_drv_register(&disp_drv);

    /* ---- 输入 ---- */
    encoder_key_hw_init();
    encoder_hw_init();

    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = encoder_read;
    (void)lv_indev_drv_register(&indev_drv);
}

int16_t lv_port_encoder_take(void)
{
    int32_t now = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&s_htim4);
    int32_t delta = now - s_enc_last;

    s_enc_last   = now;
    s_enc_accum += delta;

    {
        int16_t steps = (int16_t)(s_enc_accum / 4);
        s_enc_accum -= (int32_t)steps * 4;
        return steps;
    }
}
