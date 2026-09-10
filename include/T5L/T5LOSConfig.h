/**
 * @file    T5LOSConfig.h
 * @brief   精简T5L启动加载器平台配置。
 * @details 仅保留BOOT底座工程所需的平台定义，沿用Template4T5L头文件布局，
 *          不启用无关业务模块。
 * @author  yangming
 * @version 1.0.0
 */

#ifndef T5LOS_CONFIG_H
#define T5LOS_CONFIG_H

#include "t5los8051.h"

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;
typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;

#ifndef NULL
#define NULL ((void *)0)
#endif

#define TRUE 1U
#define FALSE 0U
#define __NOP()


#define SysEnterCritical()    EA=0;
#define SysExitCritical()     EA=1;


/**
 * @brief BOOT屏时钟配置。
 * @details 启动时从lib配置地址读取screen_ratio，0表示2K屏。
 */
#define sysLIB_SCREEN_RATIO_ADDR 0x0580U
#define sysLIB_SCREEN_RATIO_2K 0x0000U
#define sys2K_FOSC 383385600UL
#define sysNORMAL_FOSC 206438400UL

extern uint16_t sys_2k_ratio;
extern uint32_t sysFOSC;
extern uint32_t sysFCLK;

/**
 * @brief Flash操作后DGUS命令轮询延时，单位毫秒。
 */
#define FLASH_ACCESS_CYCLE 50U

/**
 * @brief 启动控制字配置。
 */
#define BOOT_CTRL_ADDR 0x0020U
#define BOOT_PAGE_SWITCH_ADDR 0x0022U
#define BOOT_PROGRESS_ADDR 0x0023U
#define BOOT_UPGRADE_PAGE_ADDR 0x0024U
#define BOOT_FINISH_PAGE_ADDR 0x0025U
#define BOOT_PAGE_SWITCH_VALUE 0x5AA5U
#define BOOT_CONFIG_WORDS 6U
#define BOOT_DEFAULT_START_BLOCK 112U
#define BOOT_RECOVERY_WINDOW_MS 500UL
#define BOOT_UPGRADE_IDLE_TIMEOUT_MS 30000UL
#define BOOT_POST_UPGRADE_LOAD_TIMEOUT_MS 30000UL

/**
 * @brief Timer0 1ms节拍配置。
 */
#define sysT0_TICK_FROM_FOSC(fosc) ((uint16_t)(65536UL - (fosc) / 12UL / 1000UL))
extern uint16_t timeT0_TICK;

/**
 * @brief UART5 OTA传输配置。
 */
#define uartUART5_TXBUF_SIZE 256U
#define uartUART5_RXBUF_SIZE 4500U
#define uartUART5_TIMEOUTSET 5U
#define uartUART5_BAUDRATE 921600UL

/**
 * @brief UART2调试打印配置。
 */
#define debugUART2_ENABLED 1
#define debugUART2_BAUDRATE 115200UL
#define debugLOG_KEY_FLOW_ENABLED 1

/**
 * @brief OTA下载配置。
 */
#define otaCRC32_CHECK_ENABLED 1
#define otaDOWNLOAD_MAX 20U
#define otaCACHE_VP_A 0x7000U
#define otaCACHE_VP_B 0x6800U
/* Reserve both 4KB windows (0x6800-0x77FF) exclusively during OTA.
 * Avoid 0x7800-0x7FFF: previous board tests reported corrupt data there. */

#endif /* T5LOS_CONFIG_H */
