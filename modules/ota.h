/**
 * @file ota.h
 * @brief 启动加载器OTA下载与应用接口。
 * 通过UART5接收AB CD协议升级包，先写入NAND并校验，
 *          再按下载上下文应用到目标区域(NOR/字库)。
 * @author yangming
 * @version 1.0.0
 */

#ifndef OTA_H
#define OTA_H

#include "sys.h"

/* 单个OTA文件描述中预留的工作缓存大小(字节)，与4KB NAND块对齐。 */
#define otaHEADER_BYTES 4096U

/**
 * @brief AB CD协议04文件信息帧解析出的单文件描述信息。
 */
typedef struct
{
    uint8_t itype;         /* 文件类型: 1=字库, 2..8=内部NOR文件 */
    uint8_t apply;         /* 应用方式标志 */
    uint16_t unid;         /* 文件唯一ID(决定目标NOR编号或字库区) */
    uint32_t size;         /* 文件大小(字节)，须为4KB倍数 */
    uint32_t crc32;        /* 文件级CRC32期望值 */
    uint16_t flash_start;  /* 文件在NAND中的起始4KB块号 */
} OtaFileInfo;

/**
 * @brief OTA下载会话上下文，覆盖otaDOWNLOAD_MAX个文件。
 * 下载端按04(信息) -> 05(数据) -> 06(结果)序列逐文件推进。
 */
typedef struct
{
    uint8_t download_end_flag;     /* 全部文件下载完成标志(0x11=已确认) */
    uint8_t total_num;             /* 本次升级包内文件总数 */
    uint8_t now_num;               /* 当前正在处理的文件序号 */
    uint16_t flash_start_num;      /* 下一个可用NAND 4KB块号 */
    uint32_t off_position;         /* 当前文件已请求的数据偏移 */
    uint32_t off_len;              /* 每次请求的数据长度(字节) */
    uint32_t all_size;             /* 当前文件总大小 */
    uint32_t downloaded_size;      /* 当前文件已接收字节数(进度用) */
    OtaFileInfo file[otaDOWNLOAD_MAX]; /* 各文件描述信息 */
} OtaContext;

/**
 * @brief 初始化OTA下载上下文与状态机。
 * 全局影响: 清零OtaContext，复位状态机与各标志。
 */
void OtaInit(void);

/**
 * @brief OTA帧分发入口，按命令字路由到04/05/06/07/08处理。
 * @param[in] frame 接收到的完整AB CD帧；len 帧总长(字节)。
 * 约束: frame非NULL且len>=5，帧头与长度字段不符时直接丢弃。
 */
void OtaReceive(uint8_t xdata *frame, uint16_t len);

/**
 * @brief OTA任务级处理: 超时重发、结果确认与下载完成检测。
 * 调用时机: 主循环中周期调用。
 */
void OtaTask(void);

/**
 * @brief OTA超时节拍，由Timer0 1ms中断调用。
 * 内部10分频后驱动OtaTimeout递减。
 * 约束: 中断上下文调用，仅操作OtaTimeout。
 */
void OtaTimerTick1ms(void);

/**
 * @brief 将已下载到NAND的文件按类型应用到目标区域。
 * @return 全部应用成功返回1，任一失败返回0。
 * 全局影响: 触发NAND->NOR拷贝、外部字库写入等Flash操作。
 */
uint8_t OtaActionFromDownload(void);

#endif /* OTA_H */
