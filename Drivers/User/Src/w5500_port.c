/**
  ******************************************************************************
  * @file    w5500_port.c
  * @brief   W5500 ��ֲ�㣺SPI2 + GPIO + ioLibrary �� 5 ���ص�
  ******************************************************************************
  * �� ����ʲô
  *    �� STM32 HAL ʵ�� ioLibrary ��Ҫ���ٽ�����CS�����ֽ� SPI��ͻ�� SPI��
  *    ���ṩ"�����ʼ��?+ Ӳ����λ + ע��ص�?+ wizchip_initialize"һ������ڡ�?  *
  * �� Ϊʲô���� HAL_SPI_MspInit
  *    LCD ������lcd_spi_154.c�����Ѿ���һ�� HAL_SPI_MspInit��ֻ���� SPI3��
  *    �ٶ���һ�����ظ����š���������ֱ���� SPI2 ��ʱ�Ӻ� GPIO��
  *    HAL_SPI_Init �ڲ��� MspInit ʱ�� SPI2 ʲô�����������á�
  *
  * �� Ӱ��ʲô
  *    PB10/PB12/PB13/PB14/PB15 ��ʼ�����?W5500 ר�á�
  *    �ٽ����õ��� PRIMASK ����/�ָ��������ƻ������ߵ��ж�״̬��
  *
  * �� ��ô��֤
  *    �� W5500_PortTest()�����ڴ�ӡ VERSIONR = 0x04 ��ͨ����
  *    ʧ��ʱ�� W5500_PortLastStep() ͣ�ڵڼ�����
  ******************************************************************************
  */

#include "w5500_port.h"
#include "main.h"
#include "usart.h"

#include "wizchip_conf.h"
#include "w5500.h"

/*==============================================================================
 *  ���Ŷ��� ���� ������ֻ����һ��
 *============================================================================*/
#define W5500_SPI           SPI2
/* ==========================================================================
 *  2026-11-22: SPI selection switch.
 *  Set W5500_USE_SPI1 to 1 AFTER rewiring the module to SPI1 pins.
 *
 *  SPI2 (default, current wiring):
 *      PB13 SCK / PB14 MISO / PB15 MOSI / PB12 CS / PB10 RST
 *      APB1 = 42 MHz, max SCLK = 21 MHz (prescaler /2)
 *  SPI1 (needs rewiring):
 *      PA5  SCK / PA6  MISO / PA7  MOSI / PA4  CS / PB10 RST
 *      APB2 = 84 MHz, max SCLK = 42 MHz (prescaler /2)
 *
 *  Why: at 0.656 MHz the W5500 silently drops Sn_CR commands (96/96 failed).
 *       At 21 MHz commands are accepted but SPI reads still glitch now and
 *       then. SPI1 can double SCLK to 42 MHz; that is the next variable to try.
 *
 *  RST stays on PB10 in both configurations, so it needs its own port macro.
 * ========================================================================== */
#define W5500_USE_SPI1      1

#if (W5500_USE_SPI1 == 0)
  #define W5500_SPI           SPI2
  #define W5500_SPI_CLK_EN()  __HAL_RCC_SPI2_CLK_ENABLE()
  #define W5500_PORT          GPIOB
  #define W5500_SCK_PIN       GPIO_PIN_13
  #define W5500_MISO_PIN      GPIO_PIN_14
  #define W5500_MOSI_PIN      GPIO_PIN_15
  #define W5500_CS_PIN        GPIO_PIN_12
  #define W5500_RST_PORT      GPIOB
  #define W5500_RST_PIN       GPIO_PIN_10
  #define W5500_AF            GPIO_AF5_SPI2
  #define W5500_SPI_PRESCALER SPI_BAUDRATEPRESCALER_2   /* 42 MHz / 2 = 21 MHz */
#else
  #define W5500_SPI           SPI1
  #define W5500_SPI_CLK_EN()  __HAL_RCC_SPI1_CLK_ENABLE()
  #define W5500_PORT          GPIOA
  #define W5500_SCK_PIN       GPIO_PIN_5
  #define W5500_MISO_PIN      GPIO_PIN_6
  #define W5500_MOSI_PIN      GPIO_PIN_7
  #define W5500_CS_PIN        GPIO_PIN_4
  #define W5500_RST_PORT      GPIOB
  #define W5500_RST_PIN       GPIO_PIN_10
  #define W5500_AF            GPIO_AF5_SPI1
  #define W5500_SPI_PRESCALER SPI_BAUDRATEPRESCALER_2   /* SPI1: 84 MHz / 2 = 42 MHz */
  /* Use SPI_BAUDRATEPRESCALER_4 for 21 MHz (same speed, different bus) */
#endif

/* Both GPIOB (RST) and the SPI port clock must be on. */
#define W5500_GPIO_CLK_EN() do { \
        __HAL_RCC_GPIOB_CLK_ENABLE(); \
        __HAL_RCC_GPIOA_CLK_ENABLE(); \
    } while (0)

/* �� �ٽ��������?PRIMASK��֧��Ƕ�ף� */
static uint32_t s_primask = 0u;
static SPI_HandleTypeDef s_hspi2;
static uint8_t s_init_step = 0u;

/*==============================================================================
 *  ioLibrary �ص�
 *============================================================================*/

/* �ٽ�������/�˳������沢�ر�ȫ���жϣ��˳�ʱԭ���ָ� */
static void W5500_CrisEnter(void)
{
    s_primask = __get_PRIMASK();
    __disable_irq();
}

static void W5500_CrisExit(void)
{
    __set_PRIMASK(s_primask);
}

/* Ƭѡ��W5500 �͵�ƽ��Ч */
static void W5500_CsSelect(void)
{
    HAL_GPIO_WritePin(W5500_PORT, W5500_CS_PIN, GPIO_PIN_RESET);
}

static void W5500_CsDeselect(void)
{
    HAL_GPIO_WritePin(W5500_PORT, W5500_CS_PIN, GPIO_PIN_SET);

    /* �� 2026-11-22 �ӣ�CS ���ߺ��һ�����?     * �� ʵ�� RCR ���� 10 ���� 2 �ζ��� 0x10��ӦΪ 0x08��������ģʽ������һλ
     *   ? ���� SPI ʱ�Ӷ����һ��?     * �� W5500 �� CS ���ߺ���Ҫһ��ʱ����β��MCU ̫��Ļ���һ�η��ʿ��ܲ���?     * �� ������?1us �Ŀ�ת�����ۿɺ��� */
    {
        volatile uint32_t d = 8u;
        while (d > 0u) { d--; }
    }
}

/* ���ֽڶ�д��д��ȥ��ͬʱ��Ӧ���ջ��� */
/* SPI ����ͳ�ƣ���������"оƬ���� 0"��"����ʧ�ܶ�����ֵ"
 * �������� volatile����ѭ�����ӡ���������߼��ж�?*/
volatile uint32_t g_spi_ok_cnt  = 0u;
volatile uint32_t g_spi_err_cnt = 0u;

static uint8_t W5500_SpiReadByte(void)
{
    uint8_t tx = 0xFFu;
    uint8_t rx = 0x00u;
    HAL_StatusTypeDef st;

    st = HAL_SPI_TransmitReceive(&s_hspi2, &tx, &rx, 1u, 10u);

    if (st == HAL_OK)
    {
        g_spi_ok_cnt++;
    }
    else
    {
        g_spi_err_cnt++;
    }

    return rx;
}

static void W5500_SpiWriteByte(uint8_t wb)
{
    (void)HAL_SPI_Transmit(&s_hspi2, &wb, 1u, 10u);
}

/* ͻ����д��W5500 �� TX/RX ����һ�ΰ�����ֽڶ���������? *
 * �� 2026-11-22 �ģ�ͻ�����ĳ�ѭ�����ֽ�
 * �� ԭ���� HAL_SPI_Receive�����?1 �ֽڵļĴ�����VERSIONR�����öԣ�
 *   2 �ֽڵģ�RTR���������� 0x0710 ������ 0x07D0 ���� ͻ����ͨ·������
 * �� �ĳɺ͵��ֽڶ���ȫ��ͬ��·����HAL_SPI_TransmitReceive + 0xFF dummy����
 *   ��֤����·����һ����
 * �� ���ۣ�ÿ�ֽڶ�һ�� HAL ���ÿ������� 656kHz �¿ɺ��� */
static void W5500_SpiReadBurst(uint8_t *pBuf, uint16_t len)
{
    uint16_t i;
    uint8_t  tx;

    for (i = 0u; i < len; i++)
    {
        tx = 0xFFu;
        (void)HAL_SPI_TransmitReceive(&s_hspi2, &tx, &pBuf[i], 1u, 10u);
    }
}

static void W5500_SpiWriteBurst(uint8_t *pBuf, uint16_t len)
{
    uint16_t i;

    for (i = 0u; i < len; i++)
    {
        (void)HAL_SPI_Transmit(&s_hspi2, &pBuf[i], 1u, 10u);
    }
}

/*==============================================================================
 *  ��ʼ��
 *============================================================================*/

/* SPI2 ��������
 * �� ģʽ 0��CPOL=0 / CPHA=0������ W5500 Ҫ��
 * �� �ȸ� 10.5 MHz��APB1 42MHz / 4������ͨ�ٿ������ٵ� 21MHz */
static void W5500_SpiInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    W5500_GPIO_CLK_EN();
    W5500_SPI_CLK_EN();

    /* SCK / MISO / MOSI �������� */
    gpio.Pin       = W5500_SCK_PIN | W5500_MISO_PIN | W5500_MOSI_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = W5500_AF;
    HAL_GPIO_Init(W5500_PORT, &gpio);

    /* CS goes on the SPI port, RST may be on a different port. */
    gpio.Pin       = W5500_CS_PIN;
    gpio.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(W5500_PORT, &gpio);

    gpio.Pin       = W5500_RST_PIN;
    HAL_GPIO_Init(W5500_RST_PORT, &gpio);

    W5500_CsDeselect();
    HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET);

    s_hspi2.Instance               = W5500_SPI;
    s_hspi2.Init.Mode              = SPI_MODE_MASTER;
    s_hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    s_hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    s_hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    s_hspi2.Init.NSS               = SPI_NSS_SOFT;
    /* 2026-11-22: SCLK MUST stay at ~21 MHz on this board.
     * At PCLK1/64 (0.656 MHz) every register / SRAM / VERSIONR access
     * succeeded, but Sn_CR commands were silently discarded:
     * 96/96 OPEN attempts returned SOCK_CLOSED in the G0 control group.
     * At PCLK1/2 (21 MHz) OPEN/LISTEN are accepted probabilistically
     * (measured 1..6 retries to SOCK_INIT, then SOCK_LISTEN).
     * The bounded retry loops in socket.c compensate for that. */
    s_hspi2.Init.BaudRatePrescaler = W5500_SPI_PRESCALER;
    s_hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    s_hspi2.Init.CRCPolynomial     = 7u;
    (void)HAL_SPI_Init(&s_hspi2);
}

/* W5500 Ӳ����λ��RST �������� 500us �ٷſ���֮���?1ms ���ڲ���ʼ�� */
static void W5500_HardReset(void)
{
    /* 2026-11-22: match NiRen vendor reference (USER/W5500.c,
     * W5500_Hardware_Reset): RST low 50 ms, then 200 ms after release.
     * Previous 2 ms / 5 ms is far shorter. SPI registers answer early,
     * but the internal PLL / socket engine needs the longer window;
     * the vendor also waits for PHYCFGR.LINK before any register access. */
    HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(50u);
    HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(200u);
}

/* W5500 оƬ��ʼ�������?ioLibrary �� wizchip_init��
 * ��ʲô��������λ -> �� 50ms -> �ֶ�д 8 �� socket ���շ�������? *         -> д�ش����� -> �ض�У�飬���Ծ����ԣ����?3 ��
 * Ϊʲô��wizchip_init() �ڲ����ٵ�һ�� wizchip_sw_reset()��
 *         �� wizchip_sw_reset() ��λ��ֻ��һ�μĴ����������ߣ�
 *         ����֤��λ�Ѿ���ɡ���λû���ʱд�����С�ᶪʧ��? *         ���� socket ����Ϊ 0��OPEN ���Ĭʧ�ܡ�
 *         �������?ͬһ�̼���ʱ�ܿ� socket����ʱ����"�ļ�Ъ����
 * Ӱ��ʲô�����?wizchip_init��ֻ�� W5500_PortInit �����һ��? * ��ô��֤��8 �� socket �� TXBUF/RXBUF ȫ������ 2 �ŷ��سɹ� */
static int8_t W5500_ChipInit(void)
{
    uint8_t i, retry, ok;

    for (retry = 0u; retry < 3u; retry++)
    {
        wizchip_sw_reset();
        HAL_Delay(50u);

        for (i = 0u; i < 8u; i++)
        {
            setSn_TXBUF_SIZE(i, 2u);
            setSn_RXBUF_SIZE(i, 2u);
        }

        setRTR(0x07D0u);
        setRCR(8u);

        ok = 1u;
        for (i = 0u; i < 8u; i++)
        {
            if ((getSn_TXBUF_SIZE(i) != 2u) || (getSn_RXBUF_SIZE(i) != 2u))
            {
                ok = 0u;
                break;
            }
        }

        if (ok != 0u)
        {
            printf("[CHIP] socket buffer 2KB verified, reset-retry=%u\r\n",
                   (unsigned)retry);
            return 0;
        }
    }

    printf("[CHIP] FATAL: socket buffer init failed after 3 tries\r\n");
    return -1;
}

int8_t W5500_PortInit(void)
{
    s_init_step = 1u;

    /* �� ���� */
    W5500_SpiInit();
    s_init_step = 2u;

    /* �� Ӳ����λ */
    W5500_HardReset();
    s_init_step = 3u;

    /* �� ע�� 5 ���ص�
     * �� ˳������ν���������� wizchip_initialize ֮ǰȫ��ע���� */
    reg_wizchip_cris_cbfunc(W5500_CrisEnter, W5500_CrisExit);
    reg_wizchip_cs_cbfunc(W5500_CsSelect, W5500_CsDeselect);
    reg_wizchip_spi_cbfunc(W5500_SpiReadByte, W5500_SpiWriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_SpiReadBurst, W5500_SpiWriteBurst);

    /* �� оƬ��ʼ�� + ���� socket ����
     * �� �� ioLibrary ��׼�� wizchip_init�����ڲ�����������λ��д 8 �� socket �Ļ��� */
    {
        static uint8_t txsize[8] = {2u, 2u, 2u, 2u, 2u, 2u, 2u, 2u};
        static uint8_t rxsize[8] = {2u, 2u, 2u, 2u, 2u, 2u, 2u, 2u};

        if (wizchip_init(txsize, rxsize) != 0)
        {
            return -1;
        }

        /* 2026-11-22: wizchip_init() calls wizchip_sw_reset() internally,
         * and that function uses only ONE getMR() as "for delay"
         * (wizchip_conf.c line 407, a few tens of microseconds). The 8 socket
         * TXBUF/RXBUF writes that follow can be swallowed by the still-running
         * reset -> buffer size 0 -> OPEN is silently ignored, Sn_SR stays 0x00.
         * This function was also written but NEVER CALLED (Keil warning
         * #177-D). Now re-write the sizes AFTER reset settles, verify by
         * read-back and retry up to 3 times. */
        if (W5500_ChipInit() != 0)
        {
            return -3;
        }
    }
    s_init_step = 4u;

    /* �� ���汾�Ĵ���ȷ��оƬ�������?*/
    if (getVERSIONR() != W5500_VERSION_VAL)
    {
        return -2;
    }
    s_init_step = 5u;

    return 0;
}

/*==============================================================================
 *  �������?���� ��������ܻ�?IP
 *
 *  MAC �� 02 ��ͷ��"���ع�����ַ"���������ʵ�������? *  ���� 192.168.10.x ��Ϊ�˱ܿ���������·�ɵ� 192.168.1.x / 0.x
 *============================================================================*/
static wiz_NetInfo s_netinfo =
{
    {0x02u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u},   /* MAC */
    {192u, 168u, 10u, 50u},                       /* IP  */
    {255u, 255u, 255u, 0u},                       /* �������� */
    {192u, 168u, 10u, 1u},                        /* ���� */
    {192u, 168u, 10u, 1u},                        /* DNS */
    NETINFO_STATIC
};

/* �� PHY ��·״̬��1 = ���߲���ҶԶ�����?*/
uint8_t W5500_LinkUp(void)
{
    return (uint8_t)((getPHYCFGR() & 0x01u) != 0u);
}

/* �� �����ʼ�����?MAC/IP/����/���أ�Ȼ�󱨸� PHY ״̬
 *   ���� 1 = ��·�����ӣ�0 = ����û���Զ�ûͨ */
uint8_t W5500_NetInit(void)
{
    uint8_t phy;
    uint8_t i;

    wizchip_setnetinfo(&s_netinfo);

    printf("[W5500] MAC ");
    for (i = 0u; i < 6u; i++) { printf("%02X%s", s_netinfo.mac[i], (i < 5u) ? ":" : ""); }
    printf("\r\n");

    printf("[W5500] IP  %u.%u.%u.%u / %u.%u.%u.%u  GW %u.%u.%u.%u\r\n",
           s_netinfo.ip[0], s_netinfo.ip[1], s_netinfo.ip[2], s_netinfo.ip[3],
           s_netinfo.sn[0], s_netinfo.sn[1], s_netinfo.sn[2], s_netinfo.sn[3],
           s_netinfo.gw[0], s_netinfo.gw[1], s_netinfo.gw[2], s_netinfo.gw[3]);

    phy = getPHYCFGR();

    if ((phy & 0x01u) != 0u)
    {
        printf("[W5500] LINK UP, %s, %s duplex\r\n",
               ((phy & 0x02u) != 0u) ? "100M" : "10M",
               ((phy & 0x04u) != 0u) ? "full" : "half");
        return 1u;
    }

    printf("[W5500] LINK DOWN - check the RJ45 cable\r\n");
    return 0u;
}

uint8_t W5500_PortLastStep(void)
{
    return s_init_step;
}

uint8_t W5500_PortReadVersion(void)
{
    return getVERSIONR();
}

/* �� 2026-11-22 �ӣ��Ĵ������? * �� Ŀ�ģ��汾���ܶ��Բ��������мĴ������ܶ��ԡ�
 *   ���������?��ֵ֪"�ļĴ�����ȫ�Բ�˵�� SPI ��д���ɿ���
 * �� �оݣ�VERSIONR=0x04 / RTR=0x07D0 / RCR=0x08 / TXBUF=2 / RXBUF=2
 * �� ������?*/
void W5500_DumpRegs(void)
{
    uint8_t i;

    printf("[REG] VERSIONR=0x%02X (want 0x04)\r\n", (unsigned)getVERSIONR());
    printf("[REG] RTR=0x%04X (want 0x07D0)  RCR=0x%02X (want 0x08)\r\n",
           (unsigned)getRTR(), (unsigned)getRCR());

    for (i = 0u; i < 8u; i++)
    {
        printf("[REG] SN%u MR=0x%02X TXBUF=%u RXBUF=%u\r\n",
               (unsigned)i,
               (unsigned)getSn_MR(i),
               (unsigned)getSn_TXBUF_SIZE(i),
               (unsigned)getSn_RXBUF_SIZE(i));
    }

    /* �� 2026-11-22 �ӣ�ͬһ���Ĵ������� 10 �Σ����ͨ·�Ƿ��ȶ�?*/
    printf("[REG] RTR x10: ");
    for (i = 0u; i < 10u; i++) { printf("0x%04X ", (unsigned)getRTR()); }
    printf("\r\n");

    printf("[REG] RCR x10: ");
    for (i = 0u; i < 10u; i++) { printf("0x%02X ", (unsigned)getRCR()); }
    printf("\r\n");

    printf("[REG] VERSIONR x10: ");
    for (i = 0u; i < 10u; i++) { printf("0x%02X ", (unsigned)getVERSIONR()); }
    printf("\r\n");
}

/* ��д�ػ����ԣ�дһ����ֵ֪�ٶ�����
 * ��ʲô���� RTR �Ĵ���д 0x1234���ٶ������Ƚ�
 * Ϊʲô��spi_err=0 ˵������û��ʱ������Ҫȷ��д��ȥ�Ͷ���������ͬһ����
 * Ӱ��ʲô������ʱ�� RTR���ش���ʱ��������д��Ĭ��ֵ 0x07D0
 * ��ô��֤������ 0x1234 ��˵��д�Ͷ������� */
/* PB14 �������?*/
/* PB14 ǿ��������ԣ������ñ���ֱ�������ȶ����?*/
/* ���ҷ��FDM1 �̶�����ģʽ���ļĴ�����д */
static uint8_t W5500_BbByte(uint8_t tx)
{
    uint8_t txb = tx, rxb = 0u;
    (void)HAL_SPI_TransmitReceive(&s_hspi2, &txb, &rxb, 1u, 10u);
    return rxb;
}

static void W5500_FdmWrite(uint16_t addr, uint8_t block, uint8_t data)
{
    uint8_t ctrl = 0x01u | 0x04u | (uint8_t)(block << 3);

    HAL_GPIO_WritePin(W5500_PORT, W5500_CS_PIN, GPIO_PIN_RESET);
    (void)W5500_BbByte((uint8_t)(addr >> 8));
    (void)W5500_BbByte((uint8_t)(addr & 0xFFu));
    (void)W5500_BbByte(ctrl);
    (void)W5500_BbByte(data);
    HAL_GPIO_WritePin(W5500_PORT, W5500_CS_PIN, GPIO_PIN_SET);
}

static uint8_t W5500_FdmRead(uint16_t addr, uint8_t block)
{
    uint8_t ctrl = 0x01u | 0x00u | (uint8_t)(block << 3);
    uint8_t val;

    HAL_GPIO_WritePin(W5500_PORT, W5500_CS_PIN, GPIO_PIN_RESET);
    (void)W5500_BbByte((uint8_t)(addr >> 8));
    (void)W5500_BbByte((uint8_t)(addr & 0xFFu));
    (void)W5500_BbByte(ctrl);
    val = W5500_BbByte(0xFFu);
    HAL_GPIO_WritePin(W5500_PORT, W5500_CS_PIN, GPIO_PIN_SET);
    return val;
}

static void W5500_FdmWrite2(uint16_t addr, uint8_t block, uint16_t data)
{
    uint8_t ctrl = 0x02u | 0x04u | (uint8_t)(block << 3);

    HAL_GPIO_WritePin(W5500_PORT, W5500_CS_PIN, GPIO_PIN_RESET);
    (void)W5500_BbByte((uint8_t)(addr >> 8));
    (void)W5500_BbByte((uint8_t)(addr & 0xFFu));
    (void)W5500_BbByte(ctrl);
    (void)W5500_BbByte((uint8_t)(data >> 8));
    (void)W5500_BbByte((uint8_t)(data & 0xFFu));
    HAL_GPIO_WritePin(W5500_PORT, W5500_CS_PIN, GPIO_PIN_SET);
}

/* OPEN �����ģʽ
 * ��ʲô���������?6 �루���û����߼������ǵ�ʱ�䣩��
 *         Ȼ��ÿ�뷢һ�������� OPEN �������У��ظ� 20 �Ρ�
 *         �ڼ�û���κ����� SPI ������θɾ���ץ��
 * Ϊʲô��Ҫȷ�� OPEN �� 4 ���ֽڣ�00 01 0C 01��������û����ķ���оƬ��? *         ֮ǰ�Ĳ���ץ����"�� Sn_SR"����"д Sn_CR"�����۲�������
 * Ӱ��ʲô��ֻ�� init �׶����У������ڼ���ѭ����û��ʼ��
 * ��ô��֤��������ץ�� CS �͵�ƽ�ڼ��� 4 ���ֽڡ�32 �� SCLK�� */
void W5500_OpenBurstTest(void)
{
    uint8_t i;

    printf("[OB] ready in 3s...\r\n");
    HAL_Delay(3000u);

    printf("[OB] begin: continuous, 1 OPEN per 10ms, capture 50ms now\r\n");

    i = 0u;
    for (;;)
    {
        W5500_FdmWrite(0x0001u, 1u, 0x01u);

        i++;
        if ((i % 100u) == 0u)
        {
            printf("[OB] sent %u opens, last SR=0x%02X\r\n",
                   (unsigned)i, (unsigned)W5500_FdmRead(0x0003u, 1u));
        }

        HAL_Delay(10u);
    }
}

void W5500_VendorExactTest(void)
{
    uint8_t sr, i;

    printf("[VEX] start\r\n");

    W5500_FdmWrite(0x0000u, 0u, 0x80u);
    HAL_Delay(10);

    for (i = 0u; i < 8u; i++)
    {
        W5500_FdmWrite(0x001Eu, (uint8_t)(1u + 4u*i), 0x02u);
        W5500_FdmWrite(0x001Fu, (uint8_t)(1u + 4u*i), 0x02u);
    }

    W5500_FdmWrite2(0x001Cu, 1u, 1460u);
    W5500_FdmWrite2(0x0004u, 1u, 5000u);

    W5500_FdmWrite(0x0000u, 1u, 0x01u);
    HAL_Delay(1);
    W5500_FdmWrite(0x0001u, 1u, 0x01u);
    HAL_Delay(5);

    sr = W5500_FdmRead(0x0003u, 1u);
    printf("[VEX] after OPEN: SR=0x%02X MR=0x%02X CR=0x%02X IR=0x%02X\r\n",
           (unsigned)sr,
           (unsigned)W5500_FdmRead(0x0000u, 1u),
           (unsigned)W5500_FdmRead(0x0001u, 1u),
           (unsigned)W5500_FdmRead(0x0002u, 1u));

    if (sr == 0x13u)
    {
        printf("[VEX] OPEN OK !\r\n");
        W5500_FdmWrite(0x0001u, 1u, 0x02u);
        HAL_Delay(5);
        printf("[VEX] after LISTEN: SR=0x%02X (want 0x14)\r\n",
               (unsigned)W5500_FdmRead(0x0003u, 1u));
    }
    else
    {
        uint8_t sr_udp;

        printf("[VEX] TCP OPEN FAILED, trying UDP...\r\n");

        /* �� UDP ģʽ��MR = 0x02 */
        W5500_FdmWrite(0x0001u, 1u, 0x10u);
        HAL_Delay(5);
        W5500_FdmWrite(0x0000u, 1u, 0x02u);
        HAL_Delay(1);
        W5500_FdmWrite(0x0004u, 1u, 0x13u);
        W5500_FdmWrite(0x0005u, 1u, 0x88u);
        W5500_FdmWrite(0x0001u, 1u, 0x01u);
        HAL_Delay(5);

        sr_udp = W5500_FdmRead(0x0003u, 1u);
        printf("[VEX] UDP: SR=0x%02X MR=0x%02X (UDP �ɹ�ӦΪ SR=0x13 MR=0x02)\r\n",
               (unsigned)sr_udp, (unsigned)W5500_FdmRead(0x0000u, 1u));

        if (sr_udp == 0x13u)
        {
            printf("[VEX] UDP OPEN OK -> TCP �����⣬UDP ����\r\n");
        }
        else
        {
            printf("[VEX] UDP ALSO FAILED -> socket ������ȫ������\r\n");
        }
    }
}

void W5500_FdmSocketTest(void)
{
    uint8_t sr;

    printf("[FDM] VERSIONR = 0x%02X\r\n", (unsigned)W5500_FdmRead(0x0039u, 0u));

    W5500_FdmWrite(0x0001u, 1u, 0x10u);
    HAL_Delay(5);
    W5500_FdmWrite(0x0000u, 1u, 0x01u);
    W5500_FdmWrite(0x0004u, 1u, 0x13u);
    W5500_FdmWrite(0x0005u, 1u, 0x88u);
    W5500_FdmWrite(0x0001u, 1u, 0x01u);
    HAL_Delay(5);

    sr = W5500_FdmRead(0x0003u, 1u);
    printf("[FDM] after OPEN: SR=0x%02X MR=0x%02X CR=0x%02X\r\n",
           (unsigned)sr,
           (unsigned)W5500_FdmRead(0x0000u, 1u),
           (unsigned)W5500_FdmRead(0x0001u, 1u));

    if (sr == 0x13u) { printf("[FDM] OPEN OK\r\n"); }
    else
    {
        uint8_t k, s3;

        printf("[FDM] OPEN FAILED, trying more options\r\n");

        setSn_TXBUF_SIZE(0, 16u);
        setSn_RXBUF_SIZE(0, 16u);
        HAL_Delay(2);
        printf("[FDM] buf16: TX=%u RX=%u\r\n",
               (unsigned)getSn_TXBUF_SIZE(0), (unsigned)getSn_RXBUF_SIZE(0));

        W5500_FdmWrite(0x0001u, 1u, 0x10u);
        HAL_Delay(5);
        W5500_FdmWrite(0x0000u, 1u, 0x01u);
        W5500_FdmWrite(0x0001u, 1u, 0x01u);

        for (k = 0u; k < 50u; k++)
        {
            HAL_Delay(10);
            sr = W5500_FdmRead(0x0003u, 1u);
            if (sr != 0x00u) { break; }
        }
        printf("[FDM] poll SR=0x%02X after %u ms\r\n", (unsigned)sr, (unsigned)((k+1u)*10u));

        s3 = W5500_FdmRead(0x0003u, (uint8_t)(1u + 4u*3u));
        printf("[FDM] socket3 SR before=0x%02X\r\n", (unsigned)s3);
        W5500_FdmWrite(0x0000u, (uint8_t)(1u + 4u*3u), 0x01u);
        W5500_FdmWrite(0x0001u, (uint8_t)(1u + 4u*3u), 0x01u);
        HAL_Delay(10);
        s3 = W5500_FdmRead(0x0003u, (uint8_t)(1u + 4u*3u));
        printf("[FDM] socket3 SR after=0x%02X\r\n", (unsigned)s3);
    }
}

/* �ֶ��� socket 0���ƿ� ioLibrary �� socket() ���� */
void W5500_ManualSocketTest(void)
{
    uint8_t sr1, sr2;

    WIZCHIP_WRITE(Sn_CR(0), Sn_CR_CLOSE);
    HAL_Delay(5);
    sr1 = getSn_SR(0);

    WIZCHIP_WRITE(Sn_MR(0), 0x01u);
    WIZCHIP_WRITE(Sn_PORT(0), 0x13u);
    WIZCHIP_WRITE(Sn_PORT(0) + 1u, 0x88u);

    WIZCHIP_WRITE(Sn_CR(0), Sn_CR_OPEN);
    HAL_Delay(10);

    sr2 = getSn_SR(0);

    printf("[MANUAL] before SR=0x%02X  after-open SR=0x%02X  MR=0x%02X CR=0x%02X\r\n",
           (unsigned)sr1, (unsigned)sr2,
           (unsigned)getSn_MR(0), (unsigned)getSn_CR(0));

    /* �� �ؼ������?socket ��������С
     * W5500 �� OPEN �����ڻ�����Ϊ 0 ʱ�ᾲĬʧ�ܣ�CR ���㵫 SR ���䣩 */
    printf("[MANUAL] TXBUF0=%u RXBUF0=%u  TXBUF1=%u RXBUF1=%u\r\n",
           (unsigned)getSn_TXBUF_SIZE(0), (unsigned)getSn_RXBUF_SIZE(0),
           (unsigned)getSn_TXBUF_SIZE(1), (unsigned)getSn_RXBUF_SIZE(1));

    if (sr2 == 0x13u)
    {
        printf("[MANUAL] OPEN ok\r\n");
        WIZCHIP_WRITE(Sn_CR(0), Sn_CR_CLOSE);
    }
    else
    {
        printf("[MANUAL] OPEN FAILED\r\n");

        /* ���Բ��ȣ�����д��������С���ٿ�һ�� */
        setSn_TXBUF_SIZE(0, 2u);
        setSn_RXBUF_SIZE(0, 2u);
        HAL_Delay(2);
        printf("[MANUAL] after set buf: TXBUF0=%u RXBUF0=%u\r\n",
               (unsigned)getSn_TXBUF_SIZE(0), (unsigned)getSn_RXBUF_SIZE(0));

        WIZCHIP_WRITE(Sn_CR(0), Sn_CR_CLOSE);
        HAL_Delay(5);
        WIZCHIP_WRITE(Sn_MR(0), 0x01u);
        WIZCHIP_WRITE(Sn_PORT(0), 0x13u);
        WIZCHIP_WRITE(Sn_PORT(0) + 1u, 0x88u);
        WIZCHIP_WRITE(Sn_CR(0), Sn_CR_OPEN);
        HAL_Delay(10);
        printf("[MANUAL] retry SR=0x%02X\r\n", (unsigned)getSn_SR(0));
    }
}

/* SCLK / MOSI ������֤ */
void W5500_SclkMosiTest(void)
{
    GPIO_InitTypeDef g = {0};
    uint32_t t0;

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &g);

    printf("[SCLK] PB13 = HIGH 3.3V, 8s. Measure module SCLK pin.\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 8000u) { }

    printf("[SCLK] PB13 = LOW 0V, 8s. Measure again.\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 8000u) { }

    g.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &g);

    printf("[MOSI] PB15 = HIGH 3.3V, 8s. Measure module MOSI pin.\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 8000u) { }

    printf("[MOSI] PB15 = LOW 0V, 8s. Measure again.\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 8000u) { }

    g.Pin       = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);

    printf("[SCLK/MOSI] done\r\n");
}

/* ���� SPI��λ��ת���� VERSIONR */
void W5500_BitBangTest(void)
{
    GPIO_InitTypeDef g = {0};
    uint8_t i, bit, val, rx = 0u;
    volatile uint32_t dd;
    const uint8_t tx[3] = {0x00u, 0x39u, 0x04u};   /* 0x04 = ��0 ����֮ǰ��д�� 0x0C */

    g.Pin   = GPIO_PIN_13 | GPIO_PIN_15 | GPIO_PIN_12;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin  = GPIO_PIN_14;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    for (dd = 0u; dd < 50u; dd++) { }

    for (i = 0u; i < 3u; i++)
    {
        val = tx[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            if ((val & 0x80u) != 0u) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); }
            else                     { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); }
            for (dd = 0u; dd < 20u; dd++) { }

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            for (dd = 0u; dd < 20u; dd++) { }
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            for (dd = 0u; dd < 20u; dd++) { }

            val <<= 1;
        }
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    for (bit = 0u; bit < 8u; bit++)
    {
        for (dd = 0u; dd < 20u; dd++) { }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        for (dd = 0u; dd < 20u; dd++) { }

        rx <<= 1;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) { rx |= 1u; }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        for (dd = 0u; dd < 20u; dd++) { }
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    printf("[BITBANG] VERSIONR = 0x%02X (want 0x04)\r\n", (unsigned)rx);

    g.Pin       = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);
}

/* CS ����ǿ���������?*/
void W5500_CsDriveTest(void)
{
    GPIO_InitTypeDef g = {0};
    uint32_t t0;

    g.Pin   = GPIO_PIN_12;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    printf("[CS] PB12 = HIGH 3.3V, 10s. Measure module SCS pin.\r\n");
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 10000u) { }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    printf("[CS] PB12 = LOW 0V, 10s. Measure again.\r\n");
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 10000u) { }

    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    printf("[CS] done\r\n");
}

void W5500_PinDriveTest(void)
{
    GPIO_InitTypeDef g = {0};
    uint32_t t0;

    g.Pin   = GPIO_PIN_14;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    printf("[DRIVE] PB14 = HIGH 3.3V, 10s. Measure module MISO pin now.\r\n");
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 10000u) { }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    printf("[DRIVE] PB14 = LOW 0V, 10s. Measure again.\r\n");
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 10000u) { }

    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);

    printf("[DRIVE] done, PB14 back to SPI mode\r\n");
}

void W5500_PinDiag(void)
{
    GPIO_InitTypeDef g = {0};
    uint32_t moder, pupdr, afr1;

    moder = GPIOB->MODER;
    pupdr = GPIOB->PUPDR;
    afr1  = GPIOB->AFR[1];
    printf("[PIN] SPI mode: MODER14=%lu PUPDR14=%lu AF14=%lu\r\n",
           (unsigned)((moder >> 28) & 3u),
           (unsigned)((pupdr >> 28) & 3u),
           (unsigned)((afr1 >> 24) & 0xFu));

    g.Pin   = GPIO_PIN_14;
    g.Mode  = GPIO_MODE_INPUT;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_Delay(1);
    printf("[PIN] pull-up   = %u\r\n", (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14));

    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_Delay(1);
    printf("[PIN] pull-down = %u\r\n", (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14));

    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_Delay(1);
    printf("[PIN] no-pull   = %u\r\n", (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14));

    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);
}

void W5500_RwTest(void)
{
    uint16_t wr = 0x1234u;
    uint16_t rd;

    setRTR(wr);
    rd = getRTR();

    if (rd == wr)
    {
        printf("[RW] write 0x%04X -> read 0x%04X  PASS\r\n",
               (unsigned)wr, (unsigned)rd);
    }
    else
    {
        printf("[RW] write 0x%04X -> read 0x%04X  FAIL\r\n",
               (unsigned)wr, (unsigned)rd);
    }

    setRTR(0x07D0u);
}

uint8_t W5500_PortTest(void)
{
    uint8_t ver;

    ver = W5500_PortReadVersion();

    printf("[W5500] VERSIONR = 0x%02X (want 0x%02X) step=%u\r\n",
                  (unsigned)ver, (unsigned)W5500_VERSION_VAL,
                  (unsigned)s_init_step);

    if (ver == W5500_VERSION_VAL)
    {
        printf("[W5500] SPI path OK\r\n");
        return 1u;
    }

    printf("[W5500] FAIL - check wiring: SCLK/MISO/MOSI/CS/RST and 3.3V\r\n");
    return 0u;
}

/*==============================================================================
 *  Register-level forensics (2026-11-22)
 *
 *  Purpose: split "Sn_CR=OPEN has no effect" into observable layers.
 *  Instead of guessing a cause and patching, dump every relevant register
 *  before/after each step and let the data say which layer is broken.
 *
 *  Block select (BSB): 0=common regs, 1=socket0 regs,
 *                      2=socket0 TX SRAM, 3=socket0 RX SRAM.
 *  All traffic uses the vendor-style FDM1 byte frames that the logic
 *  analyser already verified (addr_h, addr_l, ctrl, data).
 *
 *  Halts at the end so the main loop does not flood the serial log.
 *  To resume: replace W5500_ForensicsTest() in main.c with Net_TcpInit().
 *============================================================================*/

static uint16_t Fx_R16(uint16_t addr, uint8_t blk)
{
    uint16_t hi = (uint16_t)W5500_FdmRead(addr, blk);
    uint16_t lo = (uint16_t)W5500_FdmRead((uint16_t)(addr + 1u), blk);
    return (uint16_t)((hi << 8) | lo);
}

static void Fx_W16(uint16_t addr, uint8_t blk, uint16_t v)
{
    W5500_FdmWrite(addr, blk, (uint8_t)(v >> 8));
    W5500_FdmWrite((uint16_t)(addr + 1u), blk, (uint8_t)(v & 0xFFu));
}

static void Fx_Common(const char *tag)
{
    printf("[FX] COMMON %s: MR=%02X IR=%02X IMR=%02X SIR=%02X SIMR=%02X PHY=%02X VER=%02X RTR=%04X RCR=%02X\r\n",
           tag,
           (unsigned)W5500_FdmRead(0x0000u, 0u),
           (unsigned)W5500_FdmRead(0x0015u, 0u),
           (unsigned)W5500_FdmRead(0x0016u, 0u),
           (unsigned)W5500_FdmRead(0x0017u, 0u),
           (unsigned)W5500_FdmRead(0x0018u, 0u),
           (unsigned)W5500_FdmRead(0x002Eu, 0u),
           (unsigned)W5500_FdmRead(0x0039u, 0u),
           (unsigned)Fx_R16(0x0019u, 0u),
           (unsigned)W5500_FdmRead(0x001Bu, 0u));
}

static void Fx_Socket(const char *tag)
{
    printf("[FX] S0 %s: MR=%02X CR=%02X IR=%02X SR=%02X PORT=%04X MSSR=%04X TOS=%02X TTL=%02X\r\n",
           tag,
           (unsigned)W5500_FdmRead(0x0000u, 1u),
           (unsigned)W5500_FdmRead(0x0001u, 1u),
           (unsigned)W5500_FdmRead(0x0002u, 1u),
           (unsigned)W5500_FdmRead(0x0003u, 1u),
           (unsigned)Fx_R16(0x0004u, 1u),
           (unsigned)Fx_R16(0x0012u, 1u),
           (unsigned)W5500_FdmRead(0x0015u, 1u),
           (unsigned)W5500_FdmRead(0x0016u, 1u));
    printf("[FX] S0BUF %s: TXBUF=%02X RXBUF=%02X TX_FSR=%04X TX_RD=%04X TX_WR=%04X RX_RSR=%04X RX_RD=%04X RX_WR=%04X IMR=%02X\r\n",
           tag,
           (unsigned)W5500_FdmRead(0x001Fu, 1u),
           (unsigned)W5500_FdmRead(0x001Eu, 1u),
           (unsigned)Fx_R16(0x0020u, 1u),
           (unsigned)Fx_R16(0x0022u, 1u),
           (unsigned)Fx_R16(0x0024u, 1u),
           (unsigned)Fx_R16(0x0026u, 1u),
           (unsigned)Fx_R16(0x0028u, 1u),
           (unsigned)Fx_R16(0x002Au, 1u),
           (unsigned)W5500_FdmRead(0x002Cu, 1u));
}

static void Fx_Sram(void)
{
    static const uint16_t a[3] = {0x0000u, 0x0001u, 0x07FFu};
    static const uint8_t  v[3] = {0xA5u, 0x5Au, 0x3Cu};
    uint8_t i, r;

    for (i = 0u; i < 3u; i++)
    {
        W5500_FdmWrite(a[i], 2u, v[i]);
        r = W5500_FdmRead(a[i], 2u);
        printf("[FX] SRAM TX[%04X] w=%02X r=%02X %s\r\n",
               (unsigned)a[i], (unsigned)v[i], (unsigned)r,
               (r == v[i]) ? "OK" : "**FAIL**");
    }
    for (i = 0u; i < 3u; i++)
    {
        W5500_FdmWrite(a[i], 3u, v[i]);
        r = W5500_FdmRead(a[i], 3u);
        printf("[FX] SRAM RX[%04X] w=%02X r=%02X %s\r\n",
               (unsigned)a[i], (unsigned)v[i], (unsigned)r,
               (r == v[i]) ? "OK" : "**FAIL**");
    }
}

/*============================================================================
 * ROUND-2 DECISION MATRIX helpers (2026-11-22)
 *
 * Why these exist: the first forensics pass only ever issued OPEN on
 * socket 0, in TCP mode, at ONE SPI clock.  Two variables were never swept:
 *   (a) which socket / which protocol mode, and
 *   (b) SCLK frequency.  This board runs SPI2 at PCLK1/64 = 0.656 MHz while
 *       the NiRen vendor F103 example runs SPI1 at 72/2 = 36 MHz (55x).
 * If ANY socket other than 0 opens, no hardware replacement is needed.
 * If a faster SCLK makes OPEN work, the "dead chip" conclusion is wrong.
 *============================================================================*/
static void Fx_TryOpen(uint8_t sn, uint8_t mr, const char *tag)
{
    uint8_t blk = (uint8_t)(1u + 4u * sn);
    uint8_t sr;

    W5500_FdmWrite(0x0001u, blk, 0x10u);   /* CLOSE first */
    HAL_Delay(2u);
    W5500_FdmWrite(0x0000u, blk, mr);      /* Sn_MR */
    W5500_FdmWrite(0x0004u, blk, 0x13u);  /* Sn_PORT = 5000 */
    W5500_FdmWrite(0x0005u, blk, 0x88u);
    W5500_FdmWrite(0x0001u, blk, 0x01u);  /* OPEN */
    HAL_Delay(10u);

    sr = W5500_FdmRead(0x0003u, blk);
    printf("[DEC] %s S%u MR=%02X -> SR=%02X CR=%02X IR=%02X MRrd=%02X\r\n",
           tag, (unsigned)sn, (unsigned)mr, (unsigned)sr,
           (unsigned)W5500_FdmRead(0x0001u, blk),
           (unsigned)W5500_FdmRead(0x0002u, blk),
           (unsigned)W5500_FdmRead(0x0000u, blk));
}

static void Fx_SpiDiv(uint32_t br, const char *tag)
{
    /* Change only CR1.BR[2:0] in place.  Do NOT call HAL_SPI_Init() again:
     * it would re-run GPIO/Msp init and disturb the bus state. */
    __HAL_SPI_DISABLE(&s_hspi2);
    SPI2->CR1 = (SPI2->CR1 & ~0x38u) | (br & 0x38u);
    __HAL_SPI_ENABLE(&s_hspi2);
    HAL_Delay(2u);
    printf("[DEC] -- SPI %s BR=%u VER=%02X --\r\n",
           tag, (unsigned)((SPI2->CR1 >> 3) & 0x07u),
           (unsigned)W5500_FdmRead(0x0039u, 0u));
}

/* One quiet CLOSE -> configure -> OPEN cycle, returns resulting Sn_SR.
 * No printf inside, so a 32-attempt retry sweep stays readable. */
static uint8_t Fx_OpenOnce(uint8_t sn)
{
    uint8_t blk = (uint8_t)(1u + 4u * sn);

    W5500_FdmWrite(0x0001u, blk, 0x10u);
    HAL_Delay(1u);
    W5500_FdmWrite(0x0000u, blk, 0x01u);
    W5500_FdmWrite(0x0004u, blk, 0x13u);
    W5500_FdmWrite(0x0005u, blk, 0x88u);
    W5500_FdmWrite(0x0001u, blk, 0x01u);
    HAL_Delay(3u);
    return W5500_FdmRead(0x0003u, blk);
}

void W5500_ForensicsTest(void)
{
    static const uint16_t waits[5] = {1u, 5u, 50u, 200u, 500u};
    uint8_t i;

    HAL_Delay(1500u);   /* let PHY link settle before touching sockets */

    printf("\r\n[FX] ===== W5500 REGISTER FORENSICS =====\r\n");

    Fx_Common("boot");
    Fx_Socket("boot");
    Fx_Sram();

    /* STEP 1: clear stale socket interrupt flags */
    W5500_FdmWrite(0x0002u, 1u, 0xFFu);
    printf("[FX] STEP1 Sn_IR=FF -> rd=%02X\r\n",
           (unsigned)W5500_FdmRead(0x0002u, 1u));

    /* STEP 2: probe a plain 16-bit socket register write path */
    Fx_W16(0x0024u, 1u, 0x1234u);
    printf("[FX] STEP2 Sn_TX_WR=1234 -> rd=%04X (want 1234)\r\n",
           (unsigned)Fx_R16(0x0024u, 1u));

    /* STEP 3..5: rebuild every OPEN prerequisite, read back each one */
    W5500_FdmWrite(0x0000u, 1u, 0x01u);
    printf("[FX] STEP3 Sn_MR=01 -> rd=%02X\r\n",
           (unsigned)W5500_FdmRead(0x0000u, 1u));

    Fx_W16(0x0004u, 1u, 5000u);
    printf("[FX] STEP4 Sn_PORT=1388 -> rd=%04X\r\n",
           (unsigned)Fx_R16(0x0004u, 1u));

    Fx_W16(0x0012u, 1u, 1460u);   /* real Sn_MSSR is 0x0012, NOT 0x001C */
    printf("[FX] STEP5 Sn_MSSR@0012=05B4 -> rd=%04X\r\n",
           (unsigned)Fx_R16(0x0012u, 1u));

    Fx_Socket("pre-OPEN");

    /* STEP 6: OPEN, then sample SR as fast as SPI allows (no delay) */
    W5500_FdmWrite(0x0001u, 1u, 0x01u);
    printf("[FX] STEP6 OPEN sent, immediate SR x16: ");
    for (i = 0u; i < 16u; i++)
    {
        printf("%02X ", (unsigned)W5500_FdmRead(0x0003u, 1u));
    }
    printf("\r\n");
    printf("[FX] STEP6b CR=%02X IR=%02X TX_FSR=%04X RX_RSR=%04X\r\n",
           (unsigned)W5500_FdmRead(0x0001u, 1u),
           (unsigned)W5500_FdmRead(0x0002u, 1u),
           (unsigned)Fx_R16(0x0020u, 1u),
           (unsigned)Fx_R16(0x0026u, 1u));

    for (i = 0u; i < 5u; i++)
    {
        HAL_Delay(waits[i]);
        printf("[FX] t+%ums SR=%02X IR=%02X TX_FSR=%04X RX_RSR=%04X MR0=%02X\r\n",
               (unsigned)waits[i],
               (unsigned)W5500_FdmRead(0x0003u, 1u),
               (unsigned)W5500_FdmRead(0x0002u, 1u),
               (unsigned)Fx_R16(0x0020u, 1u),
               (unsigned)Fx_R16(0x0026u, 1u),
               (unsigned)W5500_FdmRead(0x0000u, 0u));
    }

    Fx_Socket("post-OPEN");
    Fx_Common("post-OPEN");

    /* STEP 7: control - does the chip consume an ILLEGAL command byte? */
    W5500_FdmWrite(0x0001u, 1u, 0xFFu);
    printf("[FX] STEP7 CR=FF -> immediate CR=%02X SR=%02X\r\n",
           (unsigned)W5500_FdmRead(0x0001u, 1u),
           (unsigned)W5500_FdmRead(0x0003u, 1u));
    HAL_Delay(5u);
    printf("[FX] STEP7b after 5ms CR=%02X SR=%02X\r\n",
           (unsigned)W5500_FdmRead(0x0001u, 1u),
           (unsigned)W5500_FdmRead(0x0003u, 1u));

    /*========================================================================
     * ROUND 2: the missing decision experiments
     *========================================================================*/
    printf("\r\n[DEC] ===== ROUND 2: SOCKET / MODE / SCLK SWEEP =====\r\n");

    /* A) Every socket's buffer allocation.  OPEN is silently ignored when
     *    Sn_TXBUF_SIZE or Sn_RXBUF_SIZE reads back as 0. */
    printf("[DEC] BUFCFG ");
    for (i = 0u; i < 8u; i++)
    {
        printf("S%u=%02X/%02X ", (unsigned)i,
               (unsigned)W5500_FdmRead(0x001Fu, (uint8_t)(1u + 4u*i)),
               (unsigned)W5500_FdmRead(0x001Eu, (uint8_t)(1u + 4u*i)));
    }
    printf("\r\n");

    /* B) All 8 sockets, TCP, at the current (slow) SCLK */
    for (i = 0u; i < 8u; i++)
    {
        Fx_TryOpen(i, 0x01u, "TCPslow");
    }

    /* C) Different protocol modes on socket 0 */
    Fx_TryOpen(0u, 0x02u, "UDPslow");
    Fx_TryOpen(0u, 0x04u, "MACRAWslow");

    /* D) SCLK sweep.  PCLK1=42MHz: /16 /8 /4 /2 = 2.625/5.25/10.5/21 MHz */
    Fx_SpiDiv(SPI_BAUDRATEPRESCALER_16, "2.625MHz");
    Fx_TryOpen(0u, 0x01u, "TCP-16");
    Fx_SpiDiv(SPI_BAUDRATEPRESCALER_8, "5.25MHz");
    Fx_TryOpen(0u, 0x01u, "TCP-8");
    Fx_SpiDiv(SPI_BAUDRATEPRESCALER_4, "10.5MHz");
    Fx_TryOpen(0u, 0x01u, "TCP-4");
    Fx_SpiDiv(SPI_BAUDRATEPRESCALER_2, "21MHz");
    Fx_TryOpen(0u, 0x01u, "TCP-2");

    /* E) Re-scan all sockets at the fastest stable speed */
    for (i = 0u; i < 8u; i++)
    {
        Fx_TryOpen(i, 0x01u, "TCPfast");
    }

    /* F) Put the bus back before halting */
    Fx_SpiDiv(SPI_BAUDRATEPRESCALER_64, "back-0.656MHz");

    /* G) Causality control + practical recovery.
     *    G0 re-applies the SAME /64 divider (same DISABLE/ENABLE sequence)
     *       to prove the CR1 rewrite itself is not what "wakes" the chip.
     *    G1 switches to /2 and retries OPEN up to 32 times.
     *    G2 tries LISTEN on whichever socket reached SOCK_INIT.          */
    {
        uint8_t n, sr, hitSn;

        Fx_SpiDiv(SPI_BAUDRATEPRESCALER_64, "G0-same-0.656MHz");
        sr = 0u;
        for (n = 0u; n < 32u; n++)
        {
            sr = Fx_OpenOnce((uint8_t)(n % 8u));
            if (sr != 0x00u) { break; }
        }
        printf("[DEC] G0 /64 control: SR=%02X after %u tries\r\n",
               (unsigned)sr, (unsigned)((sr != 0u) ? (n + 1u) : 32u));

        Fx_SpiDiv(SPI_BAUDRATEPRESCALER_2, "G1-21MHz");
        sr = 0u;
        hitSn = 0xFFu;
        for (n = 0u; n < 32u; n++)
        {
            hitSn = (uint8_t)(n % 8u);
            sr = Fx_OpenOnce(hitSn);
            if (sr != 0x00u) { break; }
        }
        printf("[DEC] G1 /2 retries: SR=%02X on S%u after %u tries\r\n",
               (unsigned)sr, (unsigned)hitSn, (unsigned)((n < 32u) ? (n + 1u) : 32u));

        if (sr == 0x13u)
        {
            uint8_t blk = (uint8_t)(1u + 4u * hitSn);
            W5500_FdmWrite(0x0001u, blk, 0x02u);   /* LISTEN */
            HAL_Delay(10u);
            printf("[DEC] G2 LISTEN on S%u -> SR=%02X (want 14)\r\n",
                   (unsigned)hitSn,
                   (unsigned)W5500_FdmRead(0x0003u, blk));
        }
        else
        {
            printf("[DEC] G2 no SOCK_INIT reached, LISTEN skipped\r\n");
        }
    }

    printf("[DEC] ===== ROUND 2 DONE =====\r\n");

    printf("[FX] ===== DONE - halting. Restore Net_TcpInit() in main.c =====\r\n");
    for (;;) { }
}
