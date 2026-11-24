#ifndef __W5500_PORT_H
#define __W5500_PORT_H

/*==============================================================================
 *  W5500 ��ֲ�㣨F407 + SPI2 + ioLibrary_Driver��
 *
 *  �� ��ʲô
 *    �� ioLibrary ��Ҫ�� 5 ���ص����ٽ��� / CS / ���ֽ� SPI / ͻ�� SPI��
 *    �� STM32 HAL ʵ�ֳ��������ṩһ����ʼ����ڡ�
 *
 *  �� ���ţ�ȫ���� GPIOB��
 *    SCLK = PB13   SPI2_SCK   (AF5)
 *    MISO = PB14   SPI2_MISO  (AF5)
 *    MOSI = PB15   SPI2_MOSI  (AF5)
 *    CS   = PB12   GPIO ���
 *    RST  = PB10   GPIO ���
 *    INT  = ���ӣ���ѯ���ã�ʡһ���ߣ�
 *
 *  �� Ӱ��ʲô
 *    PB10/PB12/PB13/PB14/PB15 һ����ʼ���Ͳ����ٵ������;��
 *    �� LCD �� SPI3��PB3/PB5������Ӱ�죬CAN/I2C �� PB8/PB9 Ҳ����ͻ��
 *
 *  �� ��ô��֤
 *    �� W5500_PortTest()������Ӧ��ӡ VERSIONR = 0x04��
 *    ��������˵�� SPI ���߻� CS/RST �����⣬�ȱ����������硣
 *============================================================================*/

#include <stdint.h>

/* �� W5500 �İ汾�Ĵ�����ַ��Block 0 �� VERSIONR�� */
#define W5500_VERSIONR      0x0039u
#define W5500_VERSION_VAL   0x04u

/* �� ��ʼ������ + ע��ص� + Ӳ����λ + wizchip_initialize()
 *   ���� 0 = �ɹ����� 0 = ʧ�ܣ�ʧ��ʱ�� W5500_PortLastStep() ��������һ����
 */
int8_t W5500_PortInit(void);

/* �� ��ʼ�����ڵڼ�����1=GPIO 2=SPI 3=��λ 4=wizchip 5=�汾�ţ� */
uint8_t W5500_PortLastStep(void);

/* �� ������һ�ΰ汾�Ĵ��������ض�����ֵ��0x04 ��ʾоƬ������ */
uint8_t W5500_PortReadVersion(void);

/* �� �Լ죺���汾�Ų���ӡ��������� 1 = ͨ�� */
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

/* �� �����ʼ����д MAC/IP/����/���أ����� 1 = ���������� */
uint8_t W5500_NetInit(void);

/* �� PHY ��·״̬��1 = ���߲���ҶԶ����� */
uint8_t W5500_LinkUp(void);

/* SPI ����ͳ�ƣ�ok=�ɹ����� err=ʧ�ܴ��� */
extern volatile uint32_t g_spi_ok_cnt;
extern volatile uint32_t g_spi_err_cnt;

#endif /* __W5500_PORT_H */
