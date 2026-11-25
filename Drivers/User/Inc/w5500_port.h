#ifndef __W5500_PORT_H
#define __W5500_PORT_H

/*==============================================================================
 *  W5500 移植层的对外接口（F407 + ioLibrary_Driver）
 *
 * ★ 做什么
 *    声明 ioLibrary 接入所需的入口：一次初始化、注册 4 组回调、硬复位、版本校验
 *    总线与引脚的选择放在 .c 里由 W5500_USE_SPI1 决定，本文件不重复定义，免得两处打架
 *
 * ★ 引脚（2026-11-25 现状：W5500_USE_SPI1 = 1，走 SPI1）
 *    SCK  = PA5    SPI1_SCK   (AF5)
 *    MISO = PA6    SPI1_MISO  (AF5)
 *    MOSI = PA7    SPI1_MOSI  (AF5)
 *    CS   = PA4    GPIO 输出
 *    RST  = PB10   GPIO 输出
 *    INT  = 不接（用轮询，省一根线）
 *
 *    切回 SPI2 只需把 .c 里的 W5500_USE_SPI1 改成 0，那套接线是：
 *    PB13/PB14/PB15 三根 SPI，PB12 当 CS，RST 仍是 PB10
 *
 * ★ 影响什么
 *    SPI1 占用 PA4~PA7 与 PB10；和 SPI2 那套（PB12~PB15）互斥，只能二选一
 *    走 SPI1 时 PB12~PB15 空了出来，2026-11-24 已把 CAN2 放到 PB12/PB13（AF9）
 *
 * ★ 怎么验证
 *    W5500_PortTest() 返回 1，串口打印 VERSIONR = 0x04
 *    失败时用 W5500_PortLastStep() 看停在第几步，再判断是 SPI 通路还是 CS/RST
 *============================================================================*/

#include <stdint.h>

/* ★ W5500 版本寄存器地址（Block 0 的 VERSIONR） */
#define W5500_VERSIONR      0x0039u
#define W5500_VERSION_VAL   0x04u

/* ★ 初始化总入口：配 GPIO/SPI + 注册 4 组回调 + 硬复位 + wizchip_initialize
 *   返回 0 = 成功，负数 = 失败；失败后用 W5500_PortLastStep() 定位停在哪一步
 */
int8_t W5500_PortInit(void);

/* ★ 初始化停在第几步：1=GPIO 2=SPI 3=复位 4=wizchip 5=版本 */
uint8_t W5500_PortLastStep(void);

/* ★ 读一次版本寄存器并返回读到的字节；0x04 表示芯片在线 */
uint8_t W5500_PortReadVersion(void);

/* ★ 抓包用：每 10ms 对 socket0 发一次 OPEN，进去不返回（调试专用，正式固件别调用） */
void W5500_OpenBurstTest(void);

void W5500_VendorExactTest(void);

void W5500_FdmSocketTest(void);

void W5500_ManualSocketTest(void);

void W5500_SclkMosiTest(void);

void W5500_BitBangTest(void);

void W5500_CsDriveTest(void);

void W5500_PinDriveTest(void);

void W5500_PinDiag(void);

void W5500_RwTest(void);

void W5500_DumpRegs(void);

uint8_t W5500_PortTest(void);

/* Register-level forensics: dump common/socket/SRAM registers step by step.
 * Halts at the end - replace it with Net_TcpInit() in main.c to resume. */
void W5500_ForensicsTest(void);

/* ★ 网络初始化：写 MAC/IP/掩码/网关，再读回 PHY 状态；返回 1 = 链路已连上 */
uint8_t W5500_NetInit(void);

/* ★ PHY 链路状态；1 = 网线插着且对端有响应 */
uint8_t W5500_LinkUp(void);

/* ★ SPI 收发统计：ok = 成功次数，err = 失败次数 */
extern volatile uint32_t g_spi_ok_cnt;
extern volatile uint32_t g_spi_err_cnt;

#endif /* __W5500_PORT_H */
