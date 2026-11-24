#ifndef __WEB_SRV_H
#define __WEB_SRV_H

/*==============================================================================
 *  HTTP 服务器（W5500 socket 2~7，端口 80）
 *
 *  ★ 做什么
 *    在 TCP 之上实现一个最小 HTTP 服务器，浏览器访问能看实时数据。
 *    页面由程序现场生成，不是从 Flash 里搬静态文件。
 *
 *  ★ 为什么不用 ioLibrary 自带的 httpServer
 *    那套库是给静态文件设计的：reg_httpServer_webContent() 在注册时就用
 *    strlen 把长度定死，网页里的数字一变长度就错；而且内部用 malloc 分配文件名。
 *    自己写两百行，动态内容直接拼字符串，还能把 HTTP 本身讲清楚。
 *
 *  ★ 影响什么
 *    占满 socket 2~7，六个 socket 各 2 KB 收发缓冲，可以同时服务六个连接。
 *    页面存在 Flash 里，整个响应压在 2 KB 以内，一帧发得完。
 *
 *  ★ 怎么验证
 *    电脑浏览器打开 http://192.168.10.50/ ，能看到温湿度气压并每秒刷新；
 *    串口打印 "[WEB] GET /" 和每次响应的字节数。
 *============================================================================*/

#include <stdint.h>

/* ★ HTTP 标准端口 */
#define WEB_SRV_PORT     80u

/* ★ 打开 socket 2 并在 80 端口监听
 *    返回 0 = 成功，-1 = socket 打开失败，-2 = listen 失败 */
int8_t Web_SrvInit(void);

/* ★ 状态机轮询，必须由主循环定期调用
 *    内部自限速 2 ms，一次请求处理完就主动关连接 */
void Web_SrvPoll(void);

extern volatile uint32_t g_web_req_cnt;      /* 累计处理的请求数 */
extern volatile uint32_t g_web_rx_bytes;     /* 累计收到字节 */
extern volatile uint32_t g_web_tx_bytes;     /* 累计发出字节 */
extern volatile uint8_t  g_web_busy;         /* 1 = 正在发响应 */

#endif /* __WEB_SRV_H */
