/**
  ******************************************************************************
  * @file    main.c
  * @brief   LVGL + I2C ï¿½ï¿½ï¿½ï¿½Ô´ï¿½ï¿½ï¿½ï¿½Ä»ï¿½ï¿½Ê¾ F103 ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Êµï¿½ï¿½ï¿½Ý£ï¿½ï¿½ï¿½Å¥ï¿½Ð½ï¿½ï¿½ï¿½
  ******************************************************************************
  * ï¿½ï¿½ ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
  * ï¿½ï¿½ I2C ï¿½Ó»ï¿½ï¿½ï¿½PB8/PB9ï¿½ï¿½ï¿½ï¿½Ö· 0x50ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ F103 ï¿½ï¿½ 8 ï¿½Ö½ï¿½Ò£ï¿½ï¿½
  * ï¿½ï¿½ LVGL 8.3ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½æ£¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
  * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½I2C ï¿½Ð¶ï¿½/ï¿½ï¿½Ñ¯ï¿½Õµï¿½ -> ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä´ï¿½ï¿½ï¿½ï¿½ï¿½ -> LVGL label ï¿½ï¿½Ê¾
  *
  * ï¿½ï¿½ I2C ÎªÊ²Ã´ï¿½Ã¶Ì³ï¿½Ê±ï¿½ï¿½Ñ¯
  * ï¿½ï¿½ LVGL ÒªÃ¿ï¿½ï¿½Ê®ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ lv_timer_handlerï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ã½ï¿½ï¿½æ¿¨ï¿½ï¿½
  * ï¿½ï¿½ ï¿½ï¿½ï¿½Ô°Ñ´Ó»ï¿½ï¿½ï¿½ï¿½Õ³ï¿½Ê±Ñ¹ï¿½ï¿½ 10 msï¿½ï¿½ï¿½Õ²ï¿½ï¿½ï¿½ï¿½Í¼ï¿½ï¿½ï¿½ï¿½Ü½ï¿½ï¿½ï¿½
  * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð¿ï¿½ï¿½ï¿½Â©ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê½ï¿½ï¿½Í£ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½Ó°ï¿½ï¿?  *
  * ï¿½ï¿½ ï¿½ï¿½Ã´ï¿½ï¿½Ö¤
  * ï¿½ï¿½ ï¿½ï¿½Ä»ï¿½ï¿½ï¿½ï¿½ LVGL ï¿½Ä½ï¿½ï¿½ï¿½Í°ï¿½Å?  * ï¿½ï¿½ ×ªï¿½ï¿½Å¥ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú°ï¿½Å¥Ö®ï¿½ï¿½ï¿½Æ¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Å±ï¿?  * ï¿½ï¿½ T/H/P ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ F103 ï¿½ï¿½ï¿½ï¿½Êµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
  ******************************************************************************
  */

#include "main.h"
#include "led.h"
#include "usart.h"
#include "lcd_spi_154.h"
#include "lv_port.h"
#include "w5500_port.h"
#include "net_tcp.h"               /* ï¿½ï¿½ 2026-11-22ï¿½ï¿½TCP ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */          /* ï¿½ï¿½ 2026-11-22ï¿½ï¿½W5500 ï¿½ï¿½Ì«ï¿½ï¿½ï¿½ï¿½Ö²ï¿½ï¿½ */

#include "modbus_tcp.h"
#include "web_srv.h"
#include "can_gw.h"           /* 2026-11-22: Modbus TCP ´ÓÕ¾£¬Ð­Òé²ã¶ÀÁ¢³ÉÎÄ¼þ */


void SystemClock_Config(void);

/* ¡ï Êý¾ÝÔ´Ñ¡Ôñ
 * ×öÊ²Ã´£ºCAN_ENABLE = 1 Ê±ÓÉ CAN ¹©Êý£¬I2C ´Ó»ú²»ÆôÓÃ
 * ÎªÊ²Ã´£ºPB8/PB9 ±» I2C1 ºÍ CAN1 ÇÀ£¬ÎïÀíÉÏ²»ÄÜÍ¬Ê±ÓÃ£»
 *         CAN ÊÇÕýÊ½Á´Â·£¬I2C Ö»ÊÇ 11-22 Ö®Ç°µÄÁÙÊ±Í¨Â·
 * Ó°ÏìÊ²Ã´£º¹Øµô I2C ºó F103 µÄ I2C Êý¾Ý²»ÔÙ½øÀ´£¬Ò»ÇÐ¿¿ CAN
 * ÔõÃ´ÑéÖ¤£º´®¿Ú³öÏÖ [CAN] init ok£¬ÇÒ rx Ã¿ÃëÔö¼Ó 10
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

    /* 2026-11-24 ¼Ó£º0x103 Ö¡´øÀ´µÄÁ¿ */
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

/* ï¿½ï¿½ ï¿½Ð¶Ï½ï¿½ï¿½ï¿½ï¿½ÃµÄ»ï¿½ï¿½ï¿½
 * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½Ò»Ö±ï¿½ï¿½Ð§ï¿½ï¿½HAL ï¿½ï¿½ï¿½Úºï¿½Ì¨ï¿½ï¿½ï¿½ï¿½Ð´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Å»Øµï¿½
 */
static uint8_t   s_i2c_buf[I2C_LINK_FRAME];

/* LVGL ï¿½Ø¼ï¿½ï¿½ï¿½ï¿?*/
static lv_obj_t *s_pages[3];        /* 0=»·¾³ 1=ÍøÂç 2=¼ÇÂ¼ÒÇ */
static uint8_t   s_page_cur = 0u;
static uint32_t  s_btn_cnt = 0u;
/* ¡ï ÐÂ½çÃæµÄÅäÉ«£¨2026-11-24£©
 * ¡¤ ÕûÆÁÖ»ÓÃÕâ¼¸¸öÑÕÉ«£¬±£Ö¤·ç¸ñÍ³Ò»£»Òªµ÷É«Ö»¸ÄÕâÀï
 * ¡¤ ÓÃºê¶ø²»ÊÇ³£Á¿±í£ºlv_color_make ÊÇ inline º¯Êý£¬Õ¹¿ªºó²»Õ¼ RAM
 */
#define UI_C_PANEL   lv_color_make(0x12, 0x18, 0x20)   /* ¶¥À¸µ×É« */
#define UI_C_DIM     lv_color_make(0x8A, 0x8A, 0x8A)   /* ´ÎÒªÎÄ×Ö */
#define UI_C_OK      lv_color_make(0x00, 0xE0, 0x70)   /* Õý³£ */
#define UI_C_WARN    lv_color_make(0xFF, 0xC0, 0x00)   /* ¾¯¸æ */
#define UI_C_ERR     lv_color_make(0xFF, 0x40, 0x40)   /* Òì³£ */
#define UI_C_ACCENT  lv_color_make(0x00, 0xA0, 0xFF)   /* Ç¿µ÷£¨µÍÎÂ¡¢Êª¶È£©*/
#define UI_C_OFF     lv_color_make(0x40, 0x40, 0x40)   /* Ï¨Ãð */

/* ¡ï ÐÂ½çÃæµÄ¿Ø¼þ¾ä±ú£¨2026-11-24£©
 * ¡¤ ºÍÉÏÃæÄÇÅú¾É¾ä±ú²¢´æ£º¸ÄÔì·ÖÈý²½×ß£¬µÚÒ»²½Ö»¼ÓÉùÃ÷£¬¾ÉµÄ Ui_Create ÕÕ³£¹¤×÷
 * ¡¤ Èý²½×ßÍêÔÙÉ¾¾É¾ä±ú£¬ÖÐ¼äÈÎºÎÒ»²½¶¼±£³Ö¿É±àÒë
 */
static lv_obj_t *s_led_can;         /* ¶¥À¸ CAN ×´Ì¬µÆ */
static lv_obj_t *s_led_net;         /* ¶¥À¸Íø¿Ú×´Ì¬µÆ */
static lv_obj_t *s_lab_can;         /* ¶¥À¸ CAN ÎÄ×Ö */
static lv_obj_t *s_lab_clock;       /* ¶¥À¸Ê±ÖÓ */

static lv_obj_t *s_arc_temp;        /* »·¾³Ò³µÄÎÂ¶ÈÔ²»¡ */
static lv_obj_t *s_lab_temp_val;    /* Ô²»¡ÖÐ¼äµÄ´óÊý×Ö */
/* µ¥Î» C ÊÇ¹Ì¶¨ÎÄ×Ö£¬²»ÐèÒª¾ä±ú¸üÐÂ£¬ËùÒÔ²»Õ¼Ò»¸ö±äÁ¿ */
static lv_obj_t *s_bar_humi;        /* Êª¶ÈÌõ */
static lv_obj_t *s_lab_humi_val;
static lv_obj_t *s_bar_press;       /* ÆøÑ¹Ìõ */
static lv_obj_t *s_lab_press_val;

static lv_obj_t *s_led_tcp;         /* ÍøÂçÒ³µÄ TCP Ö¸Ê¾µÆ */
static lv_obj_t *s_lab_tcp;
static lv_obj_t *s_chart_rate;      /* Ö¡ÂÊÇúÏß */
static lv_chart_series_t *s_ser_rate;
static lv_obj_t *s_lab_mb;
static lv_obj_t *s_lab_canf;        /* ÍøÂçÒ³µÄÖ¡ÂÊÊý×Ö */
static lv_obj_t *s_lab_mem;

static lv_obj_t *s_lab_rtc;         /* ¼ÇÂ¼ÒÇÒ³µÄ RTC */
static lv_obj_t *s_dot_led[8];      /* 8 ¸öÍ¨µÀÖ¸Ê¾µÆ */
static lv_obj_t *s_dot_lab[8];      /* Ö¸Ê¾µÆÅÔ±ßµÄ×ÖÄ¸ */
static lv_obj_t *s_lab_seq;
static lv_obj_t *s_lab_ver;

static lv_obj_t *s_btns[3];         /* µ×²¿µ¼º½°´Å¥£¬ÇÐÒ³Ê±¸Ä¸ßÁÁ */

/* ï¿½ï¿½ I2C ï¿½ï¿½ï¿½Å£ï¿½PB8=SCL / PB9=SDAï¿½ï¿½AF4 ï¿½ï¿½Â©
 * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ë¿ªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ßµï¿½ï¿½ï¿½ï¿½ï¿½È«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú²ï¿½ï¿½ï¿½ï¿½è£¨Ô¼ 40kï¿½ï¿½
 * ï¿½ï¿½ 40k Æ«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ñ¹ï¿½ï¿½ 50 kHz
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

    /* ï¿½ï¿½ ï¿½Ó»ï¿½Òªï¿½ï¿½ï¿½Ð¶ï¿½ï¿½Õ£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ñ¯
     * ï¿½ï¿½ ï¿½ï¿½Ñ¯ï¿½æ±¾Ò»ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½HAL ï¿½ï¿½×´Ì¬ï¿½ï¿½ï¿½ï¿½Í£ï¿½ï¿½Ã¦Ì¬ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Îµï¿½ï¿½ï¿½Ö±ï¿½Ó±ï¿½ï¿½ï¿½
     * ï¿½ï¿½ ï¿½Ð¶Ï°æ±¾ï¿½ï¿½Ó²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö®ï¿½ï¿½Øµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ãµï¿½Ò²ï¿½ï¿½ï¿½Ã³ï¿½Ê?     */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/* ï¿½ï¿½ ï¿½Ð¶ï¿½ï¿½ï¿½ï¿? * ï¿½ï¿½ startup ï¿½Ä¼ï¿½ï¿½ï¿½ï¿½ï¿½ WEAK ï¿½ï¿½ï¿½å£¬ï¿½ï¿½ï¿½ï¿½ï¿½Ç¿ï¿½ï¿½ï¿½å¸²ï¿½Çµï¿? * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½ HAL ï¿½ï¿½ï¿½Ð¶Ï´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Õ²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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

/* ï¿½ï¿½ ï¿½ï¿½ï¿½Â¹ï¿½Ò»ï¿½Î´Ó»ï¿½ï¿½ï¿½ï¿½ï¿½
 * ï¿½ï¿½ HAL ï¿½Ä´Ó»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ÔµÄ£ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Î¾ï¿½Í£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù¹ï¿½
 * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½Øµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¸ï¿½Ö®ï¿½ï¿½Òªï¿½ï¿½ï¿½ï¿½
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

    /* ï¿½ï¿½ï¿½ï¿½Ö®ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½é£¬ï¿½ï¿½ï¿½ï¿?HAL ×´Ì¬ï¿½ï¿½ï¿½ï¿½Ò»Ö±ï¿½ï¿½ï¿½ï¿½Ã¦Ì¬ */
    (void)HAL_I2C_DeInit(&hi2c1);
    if (I2C_Link_Init() == HAL_OK)
    {
        I2C_Link_Arm();
    }
}

/* ï¿½ï¿½ ï¿½ï¿½ï¿½æ£ºï¿½ï¿½ï¿½ï¿½ + ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµ + ï¿½ï¿½ï¿½ï¿½ï¿½É¾Û½ï¿½ï¿½ï¿½Å¥
 * ï¿½ï¿½ ï¿½ï¿½Å¥Ö»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ãµ¼ï¿½ï¿½ï¿½ï¿½Ã»ï¿½ï¿½Êµï¿½Ê¹ï¿½ï¿½ï¿½
 */
/* ¡ï ÐýÅ¥°´¼ü°´ÏÂÊ±×öÊ²Ã´
 * ×öÊ²Ã´£º¼ÆÊý¼ÓÒ»¡¢¸üÐÂÆÁÄ»ÉÏµÄ¼ÆÊý¡¢·­×ª°åÉÏ LED¡¢´®¿Ú´òÓ¡Ò»ÐÐ
 * ÎªÊ²Ã´£º°´¼üÈç¹ûÃ»ÓÐÈÎºÎ¿É¼û·´À¡£¬¾Í·Ö²»ÇåÊÇÃ»°´µ½»¹ÊÇÈí¼þÃ»ÏìÓ¦
 * Ó°ÏìÊ²Ã´£ºÖ»¸ÄÒ»¸ö±êÇ©ºÍÒ»¸ö IO£¬²»ÅöÍøÂçºÍÊý¾ÝÁ´Â·
 * ÔõÃ´ÑéÖ¤£ºÃ¿°´Ò»´Î£¬ÆÁÄ» P ºóÃæµÄÊý×Ö¼Ó 1£¬°åÉÏ LED ·­×ªÒ»´Î
 */
/* ¡ï ÇÐ»»Ò³Ãæ
 * ×öÊ²Ã´£ºÔÚÊý¾ÝÒ³ºÍÍøÂç×´Ì¬Ò³Ö®¼äÇÐ»»£¬ÓÃÒþ²Ø±êÖ¾¿ØÖÆ
 * ÎªÊ²Ã´ÓÃÒþ²Ø¶ø²»ÊÇÉ¾ÁËÖØ½¨£ºÖØ½¨ÒªÏûºÄ LVGL ¶Ñ£¬ÆÁÄ»»¹»áÉÁÒ»ÏÂ
 * Ó°ÏìÊ²Ã´£ºÖ»¸ÄÁ½¸öÈÝÆ÷µÄÒþ²Ø±êÖ¾£¬²»ÖØÐÂÅÅ°æ
 * ÔõÃ´ÑéÖ¤£º°´ÐýÅ¥£¬DATA °´Å¥»Øµ½Êý¾ÝÒ³£¬NET °´Å¥ÇÐµ½ÍøÂçÒ³
 */
/* ¡ï ÔìÒ»¸ö±êÇ©£¨2026-11-24£©
 * ¡¤ Ö»×öÈý¼þÊÂ£º´´½¨¡¢Éè×ÖÌå¡¢ÉèÑÕÉ«£»Î»ÖÃÓÉµ÷ÓÃÕßÁíÍâÉè
 * ¡¤ ²»×ö³É"È«²¿²ÎÊý"µÄ°æ±¾£ºÕâ¸ö½çÃæÖ»ÓÐ¶þÊ®À´¸ö±êÇ©£¬¶àÒ»²ã·â×°·´¶øÄÑ¶Á
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

/* ¡ï ÔìÒ»¸öÖ¸Ê¾µÆ£¨2026-11-24£©
 * ¡¤ ±ØÐëÇåµô CLICKABLE£ºlv_led Ä¬ÈÏ¿Éµã£¬Áô×Å»á±»±àÂëÆ÷µÄ group Ñ¡ÖÐ£¬
 *   ¹öÂÖÒ»»Î½¹µã¾ÍÅÜµ½µÆÉÏ£¬°´ÏÂÈ¥Ê²Ã´¶¼²»»á·¢Éú£¬ÕâÖÖÎÊÌâºÜÄÑ²é
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

    /* ¡ï 2026-11-24 ¼Ó£ºµ±Ç°Ò³µÄµ¼º½°´Å¥¸ßÁÁ£¬·ñÔò¿´²»³ö×Ô¼ºÔÚÄÄÒ»Ò³
     * ¡¤ Ö»¸Ä±³¾°É«£¬²»ÖØ½¨°´Å¥£¬ÇÐÒ³ÊÇË²¼äµÄ
     * ¡¤ ±ØÐëÅÐ¿Õ£ºUi_Create Ä©Î²»áµ÷ Ui_SetPage(0)£¬ÄÇÊ±°´Å¥¿ÉÄÜ»¹Ã»½¨³öÀ´ */
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

/* ¡ï ÐýÅ¥°´¼ü°´ÏÂÊ±×öÊ²Ã´
 * ×öÊ²Ã´£ºÇÐµ½°´Å¥¶ÔÓ¦µÄÒ³Ãæ£¬·­×ª°åÉÏ LED£¬´®¿Ú´òÒ»ÐÐ
 * ÎªÊ²Ã´£º°´¼üÒªÓÐ¿É¼û·´À¡£¬·ñÔò·Ö²»ÇåÊÇÃ»°´µ½»¹ÊÇÈí¼þÃ»ÏìÓ¦
 * Ó°ÏìÊ²Ã´£ºÖ»ÇÐÒ³ÃæºÍÒ»¸ö IO£¬²»ÅöÍøÂçºÍÊý¾ÝÁ´Â·
 * ÔõÃ´ÑéÖ¤£º°´ DATA »Øµ½Êý¾ÝÒ³£¬°´ NET ÇÐµ½ÍøÂçÒ³£¬LED ·­×ª
 */
static void Ui_BtnClicked(lv_event_t *e)
{
    uint32_t idx;

    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    {
        return;
    }

    /* °´Å¥±àºÅÔÚ´´½¨Ê±Í¨¹ý user_data ´ø½øÀ´£º0 ÊÇÊý¾ÝÒ³£¬1 ÊÇÍøÂçÒ³ */
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

    /* ¡ï 2026-11-24 ¸Ä£ºÊý×ÖºÍµ¥Î»·Å½øÒ»¸ö flex ÈÝÆ÷ÔÙÕûÌå¾ÓÖÐ
     * ¡¤ Ô­À´Á½¸ö±êÇ©¸÷×Ô align_to Ô²ÐÄ£¬Êý×ÖÎ»ÊýÒ»±ä£¨9.9 / -12.3£©ÕûÌå¾Í²»¾ÓÖÐ£¬
     *   ¶øÇÒ lv_obj_align_to ¶Ô arc ÕâÖÖ´ÖÃè±ß¶ÔÏóËã³öÀ´µÄÖÐÐÄ±¾À´¾ÍÆ«
     * ¡¤ »»³ÉÈÝÆ÷ + ºáÏò flex£º¿í¶È×ÔÊÊÓ¦£¬ÓÀÔ¶ÂäÔÚ»¡µÄÕýÖÐ¼ä */
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
        /* ¡ï Ö»ÓÃ×ÖÌåÀïÈ·ÊµÓÐ×ÖÐÎµÄ·ûºÅ£¨2026-11-24 ²é¹ý£©
         * ¡¤ ËùÓÐ montserrat Ö»´ø 24 ¸ö·ûºÅ£¬´øÃû×ÖµÄÖ»ÓÐ HOME/DOWNLOAD/TINT/SHUFFLE
         * ¡¤ Ô­À´ÏëÓÃ WIFI ºÍ LIST£¬ÄÇÁ½¸öÂëµã×ÖÌåÀïÃ»ÓÐ£¬»áÏÔÊ¾³É·½¿ò */
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


/* ï¿½ï¿½ Ë¢ï¿½Â½ï¿½ï¿½ï¿½ï¿½Ïµï¿½ï¿½ï¿½Öµ
 * ï¿½ï¿½ Ö»ï¿½ï¿½ I2C ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ë²Å¸ï¿½ï¿½Â£ï¿½ï¿½ï¿½ï¿½ï¿½Ã¿Ö¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 */
/* ¡ï Ë¢ÐÂÆÁÄ»ÉÏÁ½Ò³µÄÊýÖµ
 * ×öÊ²Ã´£ºÊý¾ÝÒ³¸úÊý¾ÝÖ¡×ß£¬ÍøÂçÒ³Ã¿ 500 ºÁÃëË¢Ò»´Î
 * ÎªÊ²Ã´·Ö¿ª£ºÊý¾ÝÒ³±ä»¯ÂýÃ»±ØÒªÃ¿ 5 ºÁÃëÖØ»æ£¬ÍøÂçÒ³Í³¼ÆÁ¿±ä»¯Ò²Âý
 * Ó°ÏìÊ²Ã´£ºÖ»¸Ä±êÇ©ÎÄ×Ö£¬LVGL ×Ô¼º¾ö¶¨ÄÄÀïÐèÒªÖØ»­
 * ÔõÃ´ÑéÖ¤£ºÁ½Ò³ÊýÖµÓë´®¿Ú´òÓ¡µÄÒ»ÖÂ£¬ÇÐ»»Ò³ÃæÊ±Á¢¿ÌÊÇÐÂÖµ
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

            /* ¡ï 2026-11-24 ¼Ó£º»µÖµÒ»ÂÉÏÔÊ¾ --£¬²»Ó²Ëã
             * ¡¤ F103 ´«¸ÐÆ÷Ê§°ÜÊ±·¢ 0xFFFF£¨Êª¶È£©»ò 0x8000£¨ÎÂ¶È£©£¬
             *   Ó²Ëã»á±ä³É 6553.5% ÕâÖÖÊý£¬¿í¶È»¹»á°Ñ 240 ÏñËØµÄÆÁ¶¥³öÈ¥
             * ¡¤ ÅÐ¾Ý°´ÎïÀíÁ¿³Ì£ºDS18B20 -55~125 ¶È£¬Êª¶È 0~100%£¬ÆøÑ¹ 300~1100 hPa
             * ¡¤ ÆøÑ¹¶àÅÐÒ»¸öÏÂ½ç£º0 ¶à°ëÊÇÁ´Â·ÉÏÃ»Êý¾Ý£¬²»ÊÇÕæµÄÃ»ÆøÑ¹ */
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

            /* blue below 10 C, yellow above 30 C, green in between£»
             * »µÖµÊ±»¡ÕûÈ¦Ñ¹°µ£¬Ò»ÑÛÄÜ¿´³ö"ÕâÌõÊý¾ÝÃ»ÓÐ" */
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


/* ¡ï °Ñ²É¼¯ÖµÍÆ¸ø Modbus Ð­Òé²ã
 * ×öÊ²Ã´£º°Ñ I2C ÊÕµ½µÄÎÂÊª¶ÈÆøÑ¹ºÍ¼ÆÊýÆ÷´ò°ü³É Modbus_Data_t ½»³öÈ¥
 * ÎªÊ²Ã´µ¥¶À³Éº¯Êý£ºÒÔºó½ÓÉÏ CAN£¬Êý¾ÝÔ´´ÓÕâÀï»»µô£¬Ð­Òé²ãÒ»ÐÐ¶¼²»ÓÃ¸Ä
 * Ó°ÏìÊ²Ã´£ºÖ»×öÒ»´Î½á¹¹Ìå¿½±´£¬²»ÅöÓ²¼þ£¬µ÷ÓÃ¿ªÏú¿ÉÒÔºöÂÔ
 * ÔõÃ´ÑéÖ¤£ºÖ÷Õ¾¶Á¼Ä´æÆ÷ 0 µÃµ½ÎÂ¶È£¬ºÍ´®¿Ú´òÓ¡µÄ T ÖµÒ»ÖÂ
 */
/* ¡ï °Ñ CAN ½â³öÀ´µÄÒ£²âÍ¬²½µ½ g_reg
 * ×öÊ²Ã´£ºÃ¿ 5 ºÁÃë´Ó can_gw È¡Ò»´Î×îÐÂÖµ£¬Ð´½ø g_reg ºÍÖ¡¼ÆÊý
 * ÎªÊ²Ã´£ºÆÁÄ»¡¢ÍøÒ³¡¢Modbus È«¶¼¶Á g_reg£¬Ö»ÔÚÕâÒ»¸öµØ·½¶Ô½ÓÊý¾ÝÔ´
 * Ó°ÏìÊ²Ã´£ºÖ»×ö½á¹¹Ìå¸³Öµ£¬²»ÅöÓ²¼þ£»Ã»ÊÕµ½Ö¡Ê±±£³ÖÉÏÒ»´ÎµÄÖµ
 * ÔõÃ´ÑéÖ¤£ºÆÁÄ»/ÍøÒ³/Modbus Èý´¦ÊýÖµÓ¦Óë F103 ´®¿ÚÉÏ±¨Ò»ÖÂ
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

    /* 2026-11-24£ºÀ©Õ¹Ö¡µÄÁ¿µ¥¶À°á£¬ÊÕ²»µ½Ê±±£³ÖÉÏÒ»´ÎµÄÖµ */
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

    /* ï¿½ï¿½ ï¿½ï¿½ï¿½Ïµï¿½Ò»ï¿½ï¿½ï¿½Ð¶Ï½ï¿½ï¿½ï¿½
     * ï¿½ï¿½ HAL ï¿½Ä´Ó»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ÔµÄ£ï¿½ï¿½ï¿½ï¿½ï¿½Ò»Ö¡ï¿½ï¿½Í£ï¿½ï¿½ï¿½ï¿½ï¿½Øµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â¹ï¿½
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

    /* ï¿½ï¿½ 2026-11-22ï¿½ï¿½W5500 ï¿½ï¿½Ì«ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½
     * ï¿½ï¿½ Ë³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½+ï¿½ï¿½Î» ï¿½ï¿½ ï¿½ï¿½ï¿½æ±¾ï¿½ï¿½ï¿½ï¿½Ö¤ ï¿½ï¿½ Ð´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?ï¿½ï¿½ ï¿½ï¿½ socket ï¿½ï¿½ï¿½ï¿½
     * ï¿½ï¿½ ï¿½æ±¾ï¿½Å¶ï¿½ï¿½ï¿½ 0x04ï¿½ï¿½step=5ï¿½ï¿½ï¿½ï¿½Ëµï¿½ï¿½ SPI Í¨Â·ï¿½ï¿½ï¿½ï¿½
     * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?socket ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â·×´Ì¬ï¿½ï¿½
     *   ï¿½ï¿½Îªï¿½ï¿½Â·Òªï¿½ï¿½ï¿½ßºï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½È»ï¿½ï¿?DOWN
     * ï¿½ï¿½ Ó°ï¿½ì£ºÕ¼ï¿½ï¿½ socket 0 ï¿½ï¿½ 2KB ï¿½Õ·ï¿½ï¿½ï¿½ï¿½ï¿½ */
    if (W5500_PortInit() == 0)
    {
        (void)W5500_PortTest();
        /* W5500_OpenBurstTest(); */
        (void)W5500_NetInit();

        /* ¡ï 2026-11-23£ºsocket 0 ´Ó"»ØÏÔ·þÎñÆ÷"¸Ä³É"Íø¹ØÃüÁîÍ¨µÀ"
         * ×öÊ²Ã´£ºÈÃµçÄÔÄÜÍù CAN ×ÜÏß´òÖ¡£¬F103 µÄ ISO-TP ²ÅÓÐ¶ÔÊÖ·½
         * Ó°ÏìÊ²Ã´£º»ØÏÔÃ»ÁË£¬PING ¶¥ÌæÃ°ÑÌ²âÊÔ£¬²âÊÔ½Å±¾¸ÄÓÃÃüÁîÍ¨µÀ
         * ÔõÃ´ÑéÖ¤£ºtelnet 5000 ÇÃ PING »Ø PONG£¬ÇÃ Q »ØÒ»ÐÐ¼ÆÊý */
        if (Net_TcpInit() != 0) printf("[TCP] init FAIL\r\n");
        else printf("[TCP] cmd channel on port %u (PING / Q / CAN)\r\n", NET_TCP_CMD_PORT);

        /* ¡ï 2026-11-22 ÐÂÔö£ºModbus TCP ´ÓÕ¾
         * ×öÊ²Ã´£ºÔÚ socket 1 ÉÏ¿ª 502 ¶Ë¿Ú£¬¶ÔÍâÌá¹© 16 ¸ö±£³Ö¼Ä´æÆ÷
         * ÎªÊ²Ã´·ÅÔÚÕâÀï£ºNet_TcpInit ÄÚ²¿ÒÑ¾­µÈ¹ý PHY Á´Â·£¬´ËÊ±Á´Â·±Ø UP
         * Ó°ÏìÊ²Ã´£ºÖ»Õ¼ socket 1£¬socket 0 µÄ»ØÏÔ²»ÊÜÓ°Ïì
         * ÔõÃ´ÑéÖ¤£º´®¿Ú³öÏÖ [MODBUS] slave on port 502, 16 regs
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

        /* ¡ï 2026-11-23 ÐÂÔö£ºHTTP ·þÎñÆ÷
         * ×öÊ²Ã´£ºÔÚ socket 2 ÉÏ¿ª 80 ¶Ë¿Ú£¬ä¯ÀÀÆ÷ÄÜ¿´µ½ÊµÊ±Êý¾Ý
         * ÎªÊ²Ã´·ÅÔÚÕâÀï£ººÍ Modbus Í¬Àí£¬±ØÐëµÈ PHY Á´Â· UP Ö®ºóÔÙ¿ª socket
         * Ó°ÏìÊ²Ã´£ºÖ»Õ¼ socket 2£¬Ò³ÃæÑ¹ÔÚ 2 KB ÒÔÄÚ£¬²»¶¯»º³å·ÖÅä
         * ÔõÃ´ÑéÖ¤£ºä¯ÀÀÆ÷´ò¿ª http://192.168.10.50/ £¬´®¿Ú³öÏÖ [WEB] GET /
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

    /* ÁÙÊ±Õï¶Ï£ºÁ¿Ò»ÏÂ LVGL ¶ÑÊµ¼ÊÓÃÁË¶àÉÙ£¬²âÍêÉ¾
     * LV_MEM_SIZE ÏÖÔÚÅäµÄÊÇ 64 KB£¬Èç¹ûÖ»ÓÃÁË¼¸ KB ¾ÍÄÜ´ó·ùËõ¼õ */
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
        /* ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È«ï¿½ï¿½ï¿½Ð¶Ï£ï¿½ï¿½ï¿½Ñ­ï¿½ï¿½Ö»ï¿½Ü½ï¿½ï¿½ï¿½
         * ï¿½ï¿½ ï¿½Ð¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â¹Ò½ï¿½ï¿½Õ£ï¿½ï¿½ï¿½ï¿½ï²»ï¿½ï¿½Òªï¿½ï¿½Ô¤
         * ï¿½ï¿½ ï¿½ï¿½Ñ­ï¿½ï¿½ï¿½ï¿½ï¿½á±» I2C ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
         */

        /* ï¿½ï¿½ LVGL ï¿½ï¿½Ê±ï¿½ï¿½
         * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½Ã¿ï¿½Î¶ï¿½ï¿½Ü£ï¿½ï¿½ï¿½ï¿?5 ms ï¿½ï¿½ï¿½Ò£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿?CPU
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

        /* ï¿½ï¿½ 2026-11-22 ï¿½Ó£ï¿½W5500 ï¿½ï¿½Â·×´Ì¬ï¿½ï¿½ï¿?         * ï¿½ï¿½ Ã¿ï¿½ï¿½ï¿½Ò»ï¿½ï¿?PHYï¿½ï¿½Ö»ï¿½Ú±ä»¯Ê±ï¿½ï¿½Ó¡ï¿½ï¿½ï¿½ï¿½Ë¢ï¿½ï¿½
         * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß²ï¿½ï¿½Ã¸ï¿½Î»ï¿½ï¿½ï¿½Ü¿ï¿½ï¿½ï¿½×´Ì¬ï¿½ä»¯ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö³ï¿½ï¿½Å²ï¿?*/
        /* ï¿½ï¿½ 2026-11-22 ï¿½Ó£ï¿½TCP ×´Ì¬ï¿½ï¿½ï¿½ï¿½Ñ¯
         * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ã¿ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Î£ï¿½ï¿½Ú²ï¿½ï¿½ï¿½ socket ×´Ì¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê²Ã´
         * ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ñ­ï¿½ï¿½ï¿½ï¿½Ç°ï¿½æ£¬ï¿½ï¿½Ö¤ï¿½ï¿½Ó¦ï¿½ï¿½Ê± */
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
            /* Ã¿ÃëÒ»ÐÐ CAN Á´Â·×´Ì¬
             * ½âÂëÒÑ¾­°áµ½ can_gw.c µÄ½ÓÊÕÖÐ¶ÏÀï£¬ÕâÀïÖ»¿´¼ÆÊý */
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
