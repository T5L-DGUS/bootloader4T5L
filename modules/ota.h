/**
 * @file    ota.h
 * @brief   启动加载器OTA下载与应用接口。
 * @details 通过UART5接收AB CD协议升级包，写入NAND后直接按下载上下文应用到目标区域。
 * @author  yangming
 * @version 1.0.0
 */

#ifndef OTA_H
#define OTA_H

#include "sys.h"

#define otaHEADER_BYTES 4096U

/**
 * @brief AB CD协议接收到的OTA文件描述信息。
 */
typedef struct
{
    uint8_t itype;
    uint8_t apply;
    uint16_t unid;
    uint32_t size;
    uint32_t crc32;
    uint16_t flash_start;
} OtaFileInfo;

/**
 * @brief OTA下载状态。
 */
typedef struct
{
    uint8_t download_end_flag;
    uint8_t total_num;
    uint8_t now_num;
    uint16_t flash_start_num;
    uint32_t off_position;
    uint32_t off_len;
    uint32_t all_size;
    uint32_t downloaded_size;
    OtaFileInfo file[otaDOWNLOAD_MAX];
} OtaContext;

void OtaInit(void);
void OtaReceive(uint8_t xdata *frame, uint16_t len);
void OtaTask(void);
void OtaTimerTick1ms(void);
uint8_t OtaActionFromDownload(void);

#endif /* OTA_H */
