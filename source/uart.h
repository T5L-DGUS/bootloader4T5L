/**
 * @file uart.h
 * @brief BOOT底座OTA使用的精简UART5驱动。
 * 提供环形缓冲收发、AB CD OTA帧与5A A5 recovery帧提取。
 * @author yangming
 * @version 1.0.0
 */

#ifndef UART_H
#define UART_H

#include "sys.h"

/**
 * @brief UART控制块，包含收发环形缓冲指针与状态标志。
 * 注意: RxHead/RxTimeout等成员在中断与主循环间共享，
 *       主循环侧修改前需按UartReadFrame的方式短暂关闭接收中断。
 */
typedef struct UartxDefine
{
    uint16_t TxHead;        /* 发送环形缓冲写指针(主循环侧) */
    uint16_t TxTail;        /* 发送环形缓冲读指针(中断侧) */
    uint16_t RxHead;        /* 接收环形缓冲写指针(中断侧) */
    uint16_t RxTail;        /* 接收环形缓冲读指针(主循环侧) */
    uint8_t RxTimeout;      /* 帧间隙超时计数，Timer0递减 */
    uint8_t RxFlag:2;       /* 接收状态: UART_NON_REC/UART_RECING */
    uint8_t TxBusy:1;       /* 发送进行中标志 */
} UART_TYPE;

#define UART_NON_REC 0U        /* 当前无正在接收的数据 */
#define UART_RECING 1U         /* 正在接收数据(等待帧间隙超时) */

#define UART_RECOVERY_NONE 0U    /* 无recovery控制帧 */
#define UART_RECOVERY_UPGRADE 1U /* recovery帧请求进入升级模式 */
#define UART_RECOVERY_LOAD 2U    /* recovery帧请求加载指定块 */

/**
 * @brief UART5控制块实例，仅支持Uart5一个实例。
 */
extern UART_TYPE Uart5;

/**
 * @brief 初始化UART5用于OTA传输。
 * @param[in] baudrate 波特率(正常运行921600)。
 * 全局影响: 清空全部缓冲，配置SCON3T/SCON3R与波特率分频，
 *           打开UART5收发中断和总中断EA。
 */
void Uart5Init(uint32_t baudrate);

/**
 * @brief 停止UART5收发中断并复位接收状态。
 * 全局影响: 清除ES3T/ES3R，复位Uart5接收字段与recovery类型。
 */
void Uart5Stop(void);

/**
 * @brief 通过UART5发送字节数据。
 * @param[in] uart 仅支持&Uart5；buf 源数据；len 字节长度。
 * 行为: 发送缓冲满时忙等待；首个字节启动后由发送中断接管。
 */
void UartSendData(UART_TYPE *uart, uint8_t *buf, uint16_t len);

/**
 * @brief 解析UART5接收缓存并分发完整帧。
 * 行为: 提取AB CD OTA帧交OtaReceive，提取10字节5A A5帧做
 *       recovery检查；解析后剩余半帧数据前移保留。
 * 全局影响: 消耗接收环形缓冲，可能触发OTA状态变化。
 */
void UartReadFrame(UART_TYPE *uart);

/**
 * @brief 读取并清除最近一次recovery控制帧。
 * @param[in] control_buf 输出4字节控制值，可为NULL。
 * @return UART_RECOVERY_*类型，读取后类型复位为UART_RECOVERY_NONE。
 */
uint8_t UartRecoveryGetControl(uint8_t *control_buf);

#endif /* UART_H */
