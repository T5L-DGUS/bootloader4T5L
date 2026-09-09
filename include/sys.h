/**
 * @file sys.h
 * @brief T5L启动加载器系统服务接口。
 * 提供BOOT底座和OTA模块使用的DGUS VP访问、延时和VP拷贝辅助接口。
 * @author yangming
 * @version 1.0.0
 */

#ifndef SYS_H
#define SYS_H

#include "T5LOSConfig.h"

/* DGUS内置命令寄存器VP地址定义。 */
#define sysDGUS_FLASH_RW_CMD_ADDR 0x0008U  /* NOR Flash读写命令入口 */
#define sysDGUS_BOOT_RESET_ADDR 0x0004U    /* 软复位命令寄存器 */
#define sysDGUS_PIC_SET_ADDR 0x0084U       /* 页面切换命令寄存器 */
#define sysDGUS_NAND_CMD_ADDR 0x00AAU      /* NAND操作命令入口 */
#define sysDGUS_NAND_CRC_ADDR 0x00AEU      /* NAND CRC32结果寄存器 */

/**
 * @brief VP区域拷贝请求结构。
 * sourceVP/targetVP为VP字地址，len为拷贝字数；
 * mode为1时拷贝后额外清空源区域。
 */
typedef struct _VP_EXCHANGE
{
    uint32_t sourceVP;
    uint32_t targetVP;
    uint16_t len;
    uint8_t mode;
} VP_EXCHANGE;

/**
 * @brief 从DGUS VP空间读取数据。
 * @param[in] addr VP字节地址；buf 输出缓存；len 字节长度。
 * 约束: 直接操作APP RAM总线，调用前应确保无并发VP访问。
 */
void read_dgus_vp(uint32_t addr, uint8_t *buf, uint16_t len);

/**
 * @brief 向DGUS VP空间写入数据。
 * @param[in] addr VP字节地址；buf 源数据；len 字节长度。
 * 约束: 同read_dgus_vp。
 */
void write_dgus_vp(uint32_t addr, uint8_t *buf, uint16_t len);

/**
 * @brief 忙等待微秒延时(近似值，依赖编译优化设置)。
 */
void delay_us(uint16_t us);

/**
 * @brief 忙等待毫秒延时(近似值)。
 */
void delay_ms(uint16_t ms);

/**
 * @brief 从lib配置读取屏幕比例并设置运行时主频与T0重载值。
 * 全局影响: 修改sys_2k_ratio/sysFOSC/sysFCLK/timeT0_TICK。
 */
void SysLoadClockFromLib(void);

/**
 * @brief 在VP区域之间拷贝数据。
 * @param[in] exchange_msg 拷贝请求，不允许为NULL。
 * 约束: 源/目标区域均须位于dgusVP_COPY_LIMIT以下且不重叠。
 */
void vpExchange(VP_EXCHANGE *exchange_msg);

/**
 * @brief 将片内NOR Flash数据读回DGUS VP。
 * @param[in] flash_addr NOR字地址；dgus_vp_addr 目标VP地址；
 *       len_words 字数。
 */
void FlashToDgus(uint32_t flash_addr, uint16_t dgus_vp_addr, uint16_t len_words);

/**
 * @brief 将DGUS VP数据写入片内NOR Flash。
 * @param[in] flash_addr NOR字地址；dgus_vp_addr 源VP地址；len_words 字数。
 * 约束: 写入前区域须已擦除，本函数不做擦除。
 */
void DgusToFlash(uint32_t flash_addr, uint16_t dgus_vp_addr, uint16_t len_words);

/**
 * @brief 按页面ID切换DGUS页面并等待OS完成命令。
 * @param[in] page_id 页面ID。
 */
void SwitchPageById(uint16_t page_id);

/**
 * @brief 触发T5L软复位，复位后从BOOT重新启动。
 */
void SoftReset(void);

#endif /* SYS_H */
