/**
 * @file boot.h
 * @brief 启动加载器应用加载与校验接口。
 * 定义启动控制字(VP 0x0020)的协议格式、NOR起始块解析规则，
 *          以及应用加载、升级模式进入等对外接口。
 * @author yangming
 * @version 1.0.0
 */

#ifndef BOOT_H
#define BOOT_H

#include "sys.h"

/* === 启动控制字(BOOT_CTRL_ADDR)协议定义 ===
 * 控制字共4字节，写入VP 0x0020(2字)，含义由首字节决定：
 * - 5A A5 5A A5 : 请求进入UART5升级模式；
 * - AA 55 xx xx : 指定应用起始NOR块号0xXXXX；
 * - 其他值      : 使用默认起始块BOOT_DEFAULT_START_BLOCK。 */
#define BOOT_CTRL_UPGRADE_0 0x5AU  /* 升级请求魔数第1字节 */
#define BOOT_CTRL_UPGRADE_1 0xA5U  /* 升级请求魔数第2字节 */
#define BOOT_CTRL_UPGRADE_2 0x5AU  /* 升级请求魔数第3字节 */
#define BOOT_CTRL_UPGRADE_3 0xA5U  /* 升级请求魔数第4字节 */
#define BOOT_CTRL_LOAD_0 0xAAU     /* 加载指定块前缀第1字节 */
#define BOOT_CTRL_LOAD_1 0x55U     /* 加载指定块前缀第2字节 */
#define BOOT_CTRL_BYTES 4U         /* 启动控制字长度(字节) */

/* UI交互用的按钮状态VP地址(清除残留触控用)。 */
#define BOOT_RESTART_READY_ADDR 0x3785U
#define BOOT_RESTART_GO_ADDR 0x3786U

/**
 * @brief 清除重启按钮残留状态。
 * 全局影响: 写VP BOOT_RESTART_READY_ADDR/BOOT_RESTART_GO_ADDR为0。
 */
void BootClearRestartState(void);

/**
 * @brief 将暂存在VP空间的APP代码拷贝到执行RAM并跳转。
 * 约束: 本函数经链接器固定于?PR?BOOTLOADAPP?BOOT(0xFF70)交接段，
 *       跳转后不会返回；调用前必须已通过BootCodeCheck校验。
 * 全局影响: 清空执行RAM与寄存器，控制权移交APP。
 */
void BootLoadApp(void);

/**
 * @brief 判断VP 0x0020启动控制字是否为升级请求。
 * @return 控制字为5A A5 5A A5时返回1，否则返回0。
 * 全局读取: VP 0x0020。
 */
uint8_t BootIsUpgradeRequested(void);

/**
 * @brief 从VP 0x0020解析应用起始NOR块号。
 * @return 控制字为AA 55 xx xx时返回块号0xXXXX，
 *         否则返回默认块号BOOT_DEFAULT_START_BLOCK。
 * 全局读取: VP 0x0020。
 */
uint16_t BootResolveStartBlock(void);

/**
 * @brief 等待升级端通过UART5发送加载命令(AA 55 xx xx)。
 * @param[in] timeout_ms 超时毫秒数；0表示无限等待。
 * @return 收到并接受加载命令返回1，超时返回0。
 * 全局影响: 收到命令后写入VP 0x0020并同步NOR Flash；
 *           等待期间持续解析UART5帧、应答08查询。
 */
uint8_t BootWaitLoadCommand(uint32_t timeout_ms);

/**
 * @brief 开放UART5 recovery窗口并等待恢复控制帧。
 * 全局影响: 初始化并关闭UART5/Timer0，窗口结束后从NOR Flash
 *           重载VP 0x0020配置(BootReloadConfigFromFlash)。
 */
void BootWaitRecoveryCommand(void);

/**
 * @brief 清除VP 0x0020启动控制字并同步保存到NOR Flash。
 * 全局影响: 写VP 0x0020及NOR Flash对应区域。
 */
void BootClearControl(void);

/**
 * @brief 写入启动控制字。
 * @param[in] control_buf 4字节控制值(大端序，不允许为NULL)；
 *       persist 非0时同步保存到NOR Flash。
 * 全局影响: 写VP 0x0020，persist时写NOR Flash。
 */
void BootSetControl(uint8_t *control_buf, uint8_t persist);

/**
 * @brief 写入升级进度到显示VP。
 * @param[in] progress 进度值0..100，大于100按100处理。
 * 全局影响: 写VP BOOT_PROGRESS_ADDR。
 */
void BootWriteProgress(uint8_t progress);

/**
 * @brief 按配置决定是否切换DGUS页面。
 * @param[in] page_addr 存放目标页面ID的VP地址。
 * 行为: 仅当BOOT_PAGE_SWITCH_ADDR的值等于BOOT_PAGE_SWITCH_VALUE时
 *       才执行SwitchPageById，否则不做任何操作。
 */
void BootSwitchConfiguredPage(uint16_t page_addr);

/**
 * @brief 从指定NOR起始块加载APP代码到执行RAM并做CRC校验。
 * @param[in] start_block NOR 4KB块号。
 * @return CRC校验通过返回1，否则返回0。
 * 全局影响: 临时改写VP 0xF000缓存区与代码暂存VP区，结束后恢复0xF000。
 */
uint8_t BootCodeCheck(uint16_t start_block);

/**
 * @brief 进入UART5升级模式，下载、应用完成后触发软复位；
 * 空闲超时或取消时退出升级模式并清除控制字。
 * 全局影响: 初始化/关闭UART5与Timer0，操作NAND/NOR与显示页面。
 */
void BootEnterUpgradeMode(void);

/* 从NOR Flash重载VP 0x0020配置区(宏形式，调用FlashToDgus)。 */
#define BootReloadConfigFromFlash() \
    FlashToDgus((uint32_t)BOOT_CTRL_ADDR, BOOT_CTRL_ADDR, BOOT_CONFIG_WORDS)

#endif /* BOOT_H */
