/**
 * @file    boot.h
 * @brief   启动加载器应用加载与校验接口。
 * @author  yangming
 * @version 1.0.0
 */

#ifndef BOOT_H
#define BOOT_H

#include "sys.h"

#define BOOT_CTRL_UPGRADE_0 0x5AU
#define BOOT_CTRL_UPGRADE_1 0xA5U
#define BOOT_CTRL_UPGRADE_2 0x5AU
#define BOOT_CTRL_UPGRADE_3 0xA5U
#define BOOT_CTRL_LOAD_0 0xAAU
#define BOOT_CTRL_LOAD_1 0x55U
#define BOOT_CTRL_BYTES 4U

#define BOOT_RESTART_READY_ADDR 0x3785U
#define BOOT_RESTART_GO_ADDR 0x3786U
void BootClearRestartState(void);
void BootLoadApp(void);
uint8_t BootIsUpgradeRequested(void);
uint16_t BootResolveStartBlock(void);
uint8_t BootWaitLoadCommand(uint32_t timeout_ms);
void BootWaitRecoveryCommand(void);
void BootClearControl(void);
void BootSetControl(uint8_t *control_buf, uint8_t persist);
void BootWriteProgress(uint8_t progress);
void BootSwitchConfiguredPage(uint16_t page_addr);
uint8_t BootCodeCheck(uint16_t start_block);
void BootEnterUpgradeMode(void);

#define BootReloadConfigFromFlash() \
    FlashToDgus((uint32_t)BOOT_CTRL_ADDR, BOOT_CTRL_ADDR, BOOT_CONFIG_WORDS)

#endif /* BOOT_H */
