/**
 * @file    uart.c
 * @brief   精简UART5驱动与AB CD帧提取实现。
 * @author  yangming
 * @version 1.0.0
 */

#include "uart.h"
#include "boot.h"
#include "ota.h"

UART_TYPE Uart5;

static uint8_t xdata Uart5TxBuffer[uartUART5_TXBUF_SIZE + 1U];
static uint8_t xdata Uart5RxBuffer[uartUART5_RXBUF_SIZE + 1U];
static uint8_t xdata Uart5Frame[4500U];
static uint16_t Uart5Partial;
static uint8_t Uart5RecoveryType;
static uint8_t Uart5RecoveryControl[BOOT_CTRL_BYTES];

/**
 * @brief 清空字节缓存。
 * @param[out] buf 待清空缓存。
 * @param[in] len 字节长度。
 */
static void UartClearBuffer(uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for(i = 0U; i < len; i++)
    {
        buf[i] = 0U;
    }
}

/**
 * @brief 检查最新收到的10字节是否为recovery控制写入帧。
 */
static void UartCheckRecoveryFrame(uint8_t xdata *frame)
{
    uint8_t i;
    uint8_t control_buf[BOOT_CTRL_BYTES];

    if((frame[0] != 0x5AU) ||
       (frame[1U] != 0xA5U) ||
       (frame[2U] != 0x07U) ||
       (frame[3U] != 0x82U) ||
       (frame[4U] != (uint8_t)(BOOT_CTRL_ADDR >> 8)) ||
       (frame[5U] != (uint8_t)BOOT_CTRL_ADDR))
    {
        return;
    }

    for(i = 0U; i < BOOT_CTRL_BYTES; i++)
    {
        control_buf[i] = frame[6U + i];
        Uart5RecoveryControl[i] = control_buf[i];
    }

    if((control_buf[0] == BOOT_CTRL_UPGRADE_0) &&
       (control_buf[1] == BOOT_CTRL_UPGRADE_1) &&
       (control_buf[2] == BOOT_CTRL_UPGRADE_2) &&
       (control_buf[3] == BOOT_CTRL_UPGRADE_3))
    {
        Uart5RecoveryType = UART_RECOVERY_UPGRADE;
    }
    else if((control_buf[0] == BOOT_CTRL_LOAD_0) &&
            (control_buf[1] == BOOT_CTRL_LOAD_1))
    {
        Uart5RecoveryType = UART_RECOVERY_LOAD;
    }
}

/**
 * @brief 初始化UART5用于OTA传输。
 * @param[in] baudrate UART波特率。
 */
void Uart5Init(uint32_t baudrate)
{
    uint32_t divider;

    UartClearBuffer((uint8_t *)&Uart5, sizeof(Uart5));
    UartClearBuffer(Uart5TxBuffer, sizeof(Uart5TxBuffer));
    UartClearBuffer(Uart5RxBuffer, sizeof(Uart5RxBuffer));
    UartClearBuffer(Uart5RecoveryControl, sizeof(Uart5RecoveryControl));
    Uart5RecoveryType = UART_RECOVERY_NONE;
    Uart5Partial = 0U;

    SCON3T = 0x80U;
    SCON3R = 0x80U;

    if(sys_2k_ratio != 0U)
    {
        divider = sysFCLK / 16UL / baudrate;
    }
    else
    {
        divider = sysFCLK / 8UL / baudrate;
    }
    BODE3_DIV_H = (uint8_t)(divider >> 8);
    BODE3_DIV_L = (uint8_t)divider;

    ES3T = 1U;
    ES3R = 1U;
    EA = 1U;
}

/**
 * @brief 停止UART5收发中断。
 */
void Uart5Stop(void)
{
    ES3T = 0U;
    ES3R = 0U;
    Uart5.RxHead = 0U;
    Uart5.RxTail = 0U;
    Uart5.RxTimeout = 0U;
    Uart5.RxFlag = UART_NON_REC;
    Uart5.TxBusy = 0U;
    Uart5RecoveryType = UART_RECOVERY_NONE;
}

/**
 * @brief UART5接收中断。
 */
void Uart5RxIsr(void) interrupt 13
{

    if((SCON3R & 0x01U) == 0x01U)
    {
        if(Uart5.RxHead < uartUART5_RXBUF_SIZE)
        {
            Uart5RxBuffer[Uart5.RxHead++] = SBUF3_RX;
            Uart5.RxFlag = UART_RECING;
            Uart5.RxTimeout = uartUART5_TIMEOUTSET;
        }

        SCON3R &= 0xFEU;
    }
}

/**
 * @brief UART5发送中断。
 */
void Uart5TxIsr(void) interrupt 12
{
    if((SCON3T & 0x01U) == 0x01U)
    {
        SCON3T &= 0xFEU;
        if(Uart5.TxHead != Uart5.TxTail)
        {
            SBUF3_TX = Uart5TxBuffer[Uart5.TxTail++];
            if(Uart5.TxTail >= uartUART5_TXBUF_SIZE)
            {
                Uart5.TxTail = 0U;
            }
        }
        else
        {
            Uart5.TxBusy = 0U;
        }
    }
}

/**
 * @brief 通过UART5发送字节数据。
 * @param[in] uart UART控制指针，仅支持Uart5。
 * @param[in] buf 源字节数据。
 * @param[in] len 字节长度。
 */
void UartSendData(UART_TYPE *uart, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint16_t next_head;

    if((uart != &Uart5) || (buf == NULL))
    {
        return;
    }

    for(i = 0U; i < len; i++)
    {
        do
        {
            next_head = uart->TxHead + 1U;
            if(next_head >= uartUART5_TXBUF_SIZE)
            {
                next_head = 0U;
            }
        }while(next_head == uart->TxTail);

        Uart5TxBuffer[uart->TxHead] = *buf++;
        uart->TxHead = next_head;
    }

    if(uart->TxBusy == 0U)
    {
        uart->TxBusy = 1U;
        SCON3T |= 0x01U;
    }
}

/**
 * @brief 解析UART5缓存数据并分发AB CD OTA帧。
 * @param[in] uart UART控制指针。
 */
void UartReadFrame(UART_TYPE *uart)
{
    uint16_t incoming, i, offset, length;
    uint8_t receive_enabled;
    if(uart != &Uart5 || uart->RxFlag == UART_NON_REC || uart->RxTimeout != 0U) return;
    receive_enabled = ES3R;
    ES3R = 0U;
    incoming = uart->RxHead;
    if(incoming + Uart5Partial > sizeof(Uart5Frame)) Uart5Partial = 0U;
    for(i = 0U; i < incoming; ++i) Uart5Frame[Uart5Partial + i] = Uart5RxBuffer[i];
    Uart5Partial += incoming;
    uart->RxHead = uart->RxTail = 0U;
    uart->RxFlag = UART_NON_REC;
    ES3R = receive_enabled;
    offset = 0U;
    while(Uart5Partial - offset >= 2U)
    {
        if(Uart5Frame[offset] == 0xABU && Uart5Frame[offset + 1U] == 0xCDU)
        {
            if(Uart5Partial - offset < 4U) break;
            length = ((uint16_t)Uart5Frame[offset + 2U] << 8) | Uart5Frame[offset + 3U];
            if(length < 1U || length > sizeof(Uart5Frame) - 4U) { ++offset; continue; }
            length += 4U;
            if(Uart5Partial - offset < length) break;
            OtaReceive(&Uart5Frame[offset], length);
            offset += length;
        }
        else if(Uart5Frame[offset] == 0x5AU && Uart5Frame[offset + 1U] == 0xA5U)
        {
            if(Uart5Partial - offset < 3U) break;
            length = Uart5Frame[offset + 2U] + 3U;
            if(Uart5Partial - offset < length) break;
            if(length == 10U) UartCheckRecoveryFrame(&Uart5Frame[offset]);
            offset += length;
        }
        else ++offset;
    }
    for(i = offset; i < Uart5Partial; ++i) Uart5Frame[i - offset] = Uart5Frame[i];
    Uart5Partial -= offset;
}

/**
 * @brief 读取并清除recovery控制帧。
 * @param[out] control_buf 4字节启动控制值。
 * @return UART_RECOVERY_*类型。
 */
uint8_t UartRecoveryGetControl(uint8_t *control_buf)
{
    uint8_t i;
    uint8_t recovery_type;

    UartReadFrame(&Uart5);
    recovery_type = Uart5RecoveryType;
    if((control_buf != NULL) && (recovery_type != UART_RECOVERY_NONE))
    {
        for(i = 0U; i < BOOT_CTRL_BYTES; i++)
        {
            control_buf[i] = Uart5RecoveryControl[i];
        }
    }

    Uart5RecoveryType = UART_RECOVERY_NONE;
    return recovery_type;
}
