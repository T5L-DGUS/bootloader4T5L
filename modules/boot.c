/**
 * @file boot.c
 * @brief 启动加载器应用加载与校验实现。
 * APP RAM拷贝、CRC校验和跳转行为保持原T5L51_SHJB BOOT工程一致。
 * @author yangming
 * @version 1.0.0
 */

#include "boot.h"
#include "debug_uart.h"
#include "uart.h"

/* === 应用代码搬运与CRC参数 === */
#define bootAPP_DATA_BYTES 65280UL              /* APP执行RAM代码区大小(字节) */
#define bootAPP_DWORD_COUNT (uint16_t)(bootAPP_DATA_BYTES / 4UL) /* 32位读取次数 */
#define bootAPP_CRC_EXTRA_BYTES 2U              /* 附加校验字节(存储CRC占2字节) */
#define bootF000_CACHE_WORDS 2048U              /* VP 0xF000缓存字数(4KB) */
#define bootF000_CACHE_BYTES (bootF000_CACHE_WORDS * 2U)
#define bootCODE_TARGET_VP_START 0x10000UL      /* 代码暂存VP起始字地址 */
#define bootCODE_COPY_BLOCK_WORDS 0x0800U       /* 单次搬运字数(4KB) */
#define bootCODE_COPY_BLOCK_COUNT 16U           /* 总搬运次数(16*4KB=64KB) */
#define bootCMD_WAIT_STEP_MS 10U                /* DGUS命令轮询步长(ms) */
#define bootCMD_WAIT_TIMEOUT_MS 3000U           /* DGUS命令等待超时(ms) */

/* 读取VP 0x0020控制字的宏封装。 */
#define BootReadControl(control_buf) read_dgus_vp(BOOT_CTRL_ADDR, (control_buf), 2U)

/**
 * @brief VP 0xF000内容备份缓存，读取NOR代码块时暂存其原始内容，用后恢复。
 */
static uint8_t xdata BootVpF000Cache[bootF000_CACHE_BYTES];

/**
 * @brief 轮询等待DGUS命令寄存器空闲(使能字节回0)。
 * @param[in] cmd_addr 命令寄存器VP地址。
 * @return 空闲返回1，超时返回0。
 * 全局读取: VP cmd_addr。
 */
static uint8_t BootWaitDgusCmdIdle(uint32_t cmd_addr)
{
    uint8_t cmd_state[2];
    uint16_t elapsed_ms;

    elapsed_ms = 0U;
    cmd_state[0] = 0xFFU;
    while(elapsed_ms < bootCMD_WAIT_TIMEOUT_MS)
    {
        delay_ms(bootCMD_WAIT_STEP_MS);
        read_dgus_vp(cmd_addr, cmd_state, 1U);
        if(cmd_state[0] == 0U)
        {
            return 1U;
        }
        elapsed_ms += bootCMD_WAIT_STEP_MS;
    }

    DBG_LOG_HEX16("[BOOT] cmd wait timeout addr=", cmd_addr);
    return 0U;
}

/**
 * @brief 读取1个VP字。
 * @param[in] addr VP字地址。
 * @return VP字值。
 */
static uint16_t BootReadVpWord(uint16_t addr)
{
    uint8_t buf[2];

    read_dgus_vp(addr, buf, 1U);
    return ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
}

/**
 * @brief 写入1个VP字。
 * @param[in] addr VP字地址。
 * @param[in] value VP字值。
 */
static void BootWriteVpWord(uint16_t addr, uint16_t value)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)value;
    write_dgus_vp(addr, buf, 1U);
}

/**
 * @brief 写入启动控制值，并可同步保存到片内NOR Flash。
 * @param[in] control_buf 4字节控制值。
 * @param[in] persist 非0时保存到NOR Flash。
 */
void BootSetControl(uint8_t *control_buf, uint8_t persist)
{
    if(control_buf == NULL)
    {
        return;
    }

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    DebugLog("[BOOT] set ctrl=");
    DebugLogHex8(control_buf[0]);
    DebugLog(" ");
    DebugLogHex8(control_buf[1]);
    DebugLog(" ");
    DebugLogHex8(control_buf[2]);
    DebugLog(" ");
    DebugLogHex8(control_buf[3]);
    DebugLog(" persist=");
    DebugLogU8(persist);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */

    write_dgus_vp(BOOT_CTRL_ADDR, control_buf, 2U);
    if(persist != 0U)
    {
        DgusToFlash((uint32_t)BOOT_CTRL_ADDR, BOOT_CTRL_ADDR, 2U);
    }
}


/**
 * @brief 写入升级进度。
 * @param[in] progress 0到100。
 */
void BootWriteProgress(uint8_t progress)
{
#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    static uint8_t last_progress = 0xFFU;
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */

    if(progress > 100U)
    {
        progress = 100U;
    }

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    if(progress != last_progress)
    {
        DebugLog("[BOOT] progress=");
        DebugLogU8(progress);
        DebugLog("\r\n");
        last_progress = progress;
    }
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */

    BootWriteVpWord(BOOT_PROGRESS_ADDR, (uint16_t)progress);
}

/**
 * @brief 按配置决定是否切换到指定配置地址中的页面。
 * @param[in] page_addr 存放页面ID的VP地址。
 */
void BootSwitchConfiguredPage(uint16_t page_addr)
{
    uint16_t page_switch;
    uint16_t page_id;

    page_switch = BootReadVpWord(BOOT_PAGE_SWITCH_ADDR);
    if(page_switch != BOOT_PAGE_SWITCH_VALUE)
    {
        DBG_LOG_HEX16("[BOOT] page switch disabled value=", page_switch);
        return;
    }

    page_id = BootReadVpWord(page_addr);
#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    DebugLog("[BOOT] switch page=");
    DebugLogU16(page_id);
    DebugLog(" from cfg=");
    DebugLogHex16(page_addr);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
    SwitchPageById(page_id);
}

/**
 * @brief 判断VP 0x0020是否请求进入升级模式。
 * @return 控制值为5A A5 5A A5时返回1，否则返回0。
 */
uint8_t BootIsUpgradeRequested(void)
{
    uint8_t control_buf[BOOT_CTRL_BYTES];

    BootReadControl(control_buf);
    if((control_buf[0] == BOOT_CTRL_UPGRADE_0) &&
       (control_buf[1] == BOOT_CTRL_UPGRADE_1) &&
       (control_buf[2] == BOOT_CTRL_UPGRADE_2) &&
       (control_buf[3] == BOOT_CTRL_UPGRADE_3))
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 从VP 0x0020解析NOR起始块。
 * @return 存在AA55xxxx时返回动态块号，否则返回默认块号。
 */
uint16_t BootResolveStartBlock(void)
{
    uint8_t control_buf[BOOT_CTRL_BYTES];

    BootReadControl(control_buf);
    if((control_buf[0] == BOOT_CTRL_LOAD_0) &&
       (control_buf[1] == BOOT_CTRL_LOAD_1))
    {
        return ((uint16_t)control_buf[2] << 8) | (uint16_t)control_buf[3];
    }

    return BOOT_DEFAULT_START_BLOCK;
}

/**
 * @brief 清除UI按钮残留触控，避免上一次升级/完成画面遗留的按下状态被误读。
 */
void BootClearRestartState(void)
{
    BootWriteVpWord(BOOT_RESTART_READY_ADDR, 0U);
    BootWriteVpWord(BOOT_RESTART_GO_ADDR, 0U);
}

/**
 * @brief 等待升级端发送加载命令(AA 55 xx xx)。
 * 应用成功后由R11/APP下发加载块号，收到即写入控制字，
 * 复位后BOOT将直接从该块加载应用，无需人工确认。
 * @param[in] timeout_ms 超时毫秒数；0表示无限等待。
 * @return 收到并接受加载命令返回1，超时返回0。
 * 全局影响: 接受命令时写VP 0x0020并同步NOR Flash；
 *           等待期间持续解析UART5帧以应答07保活/08查询。
 */
uint8_t BootWaitLoadCommand(uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0UL;
    uint8_t control_buf[BOOT_CTRL_BYTES];
    BootClearRestartState();
    while(timeout_ms == 0UL || elapsed_ms < timeout_ms)
    {
        if(UartRecoveryGetControl(control_buf) == 0U)
            BootReadControl(control_buf);
        if(control_buf[0] == BOOT_CTRL_LOAD_0 && control_buf[1] == BOOT_CTRL_LOAD_1)
        {
            BootSetControl(control_buf, 1U);
            BootClearRestartState();
            DBG_LOG_LINE("[BOOT] load command accepted; automatic restart");
            return 1U;
        }
        UartReadFrame(&Uart5);
        delay_ms(10U);
        if(timeout_ms != 0UL) elapsed_ms += 10UL;
    }
    return 0U;
}

/**
 * @brief 清除VP 0x0020启动控制值并同步到NOR Flash。
 */
void BootClearControl(void)
{
    uint8_t zero_buf[BOOT_CTRL_BYTES];

    DBG_LOG_LINE("[BOOT] clear ctrl");
    zero_buf[0] = 0U;
    zero_buf[1] = 0U;
    zero_buf[2] = 0U;
    zero_buf[3] = 0U;
    BootSetControl(zero_buf, 1U);
}

/**
 * @brief 将已暂存的APP代码从VP空间拷贝到执行RAM并跳转。
 * @warning 该函数通过链接器固定在boot交接段。
 */
void BootLoadApp(void)
{
#pragma ASM
        ANL     PSW,#0E7H
        MOV     D_PAGESEL,#01H
        MOV     DPTR,#0000H
        MOV     DPC,#01H
        MOV     ADR_H,#00H
        MOV     ADR_M,#80H
        MOV     ADR_L,#00H
        MOV     ADR_INC,#01H
        MOV     RAMMODE,#0AFH
        JNB     APP_ACK,$
        MOV     R0,#64
READC1: MOV     R1,#255
READC2: SETB    APP_EN
        JB      APP_EN,$
        MOV     A,DATA3
        MOVX    @DPTR,A
        MOV     A,DATA2
        MOVX    @DPTR,A
        MOV     A,DATA1
        MOVX    @DPTR,A
        MOV     A,DATA0
        MOVX    @DPTR,A
        DJNZ    R1,READC2
        DJNZ    R0,READC1

        MOV     R0,#08H
        MOV     R1,#248
DATACLR:MOV     @R0,#00
        INC     R0
        DJNZ    R1,DATACLR

        MOV     D_PAGESEL,#02H
        MOV     DPTR,#8000H
RAMCLR: CLR     A
        MOVX    @DPTR,A
        MOVX    @DPTR,A
        MOVX    @DPTR,A
        MOVX    @DPTR,A
        MOV     A,DPH
        JNZ     RAMCLR
        MOV     R0,#00H
        MOV     R1,#00H
        MOV     R2,#00H
        MOV     R3,#00H
        MOV     R4,#00H
        MOV     R5,#00H
        MOV     R6,#00H
        MOV     R7,#00H
        MOV     DPC,#00H
#pragma ENDASM
    ((void (*)(void))0)();
}

/**
 * @brief 将1字节数据送入原BOOT使用的Modbus风格CRC16。
 * @param[in] crc 当前CRC值。
 * @param[in] data_byte 输入字节。
 * @return 更新后的CRC值。
 */
static uint16_t BootCrc16Update(uint16_t crc, uint8_t data_byte)
{
    uint8_t bit_index;

    crc ^= data_byte;
    for(bit_index = 0U; bit_index < 8U; bit_index++)
    {
        if((crc & 0x0001U) != 0U)
        {
            crc = (crc >> 1) ^ 0xA001U;
        }
        else
        {
            crc >>= 1;
        }
    }

    return crc;
}

/**
 * @brief 从APP RAM计算用户代码CRC。
 * @return CRC值；有效APP连同存储CRC一起计算后结果为0。
 */
static uint16_t BootCodeCrc16(void)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    ADR_H = 0U;
    ADR_M = 0x80U;
    ADR_L = 0x00U;
    ADR_INC = 1U;
    RAMMODE = 0xAFU;
    while(!APP_ACK)
    {
    }

    for(i = 0U; i < bootAPP_DWORD_COUNT; i++)
    {
        APP_EN = 1U;
        while(APP_EN)
        {
        }
        crc = BootCrc16Update(crc, DATA3);
        crc = BootCrc16Update(crc, DATA2);
        crc = BootCrc16Update(crc, DATA1);
        crc = BootCrc16Update(crc, DATA0);
    }

    APP_EN = 1U;
    while(APP_EN)
    {
    }
    crc = BootCrc16Update(crc, DATA3);
    crc = BootCrc16Update(crc, DATA2);

    RAMMODE = 0U;
    return crc;
}

/**
 * @brief 将1个4KB NOR代码块读入暂存VP区域。
 * @param[in] flash_addr 源NOR字地址。
 */
static void BootReadNorCodeBlock(uint32_t flash_addr)
{
    uint8_t cmd[12];

    cmd[0] = 0x5AU;
    cmd[1] = (uint8_t)(flash_addr >> 16);
    cmd[2] = (uint8_t)(flash_addr >> 8);
    cmd[3] = (uint8_t)flash_addr;
    cmd[4] = 0xF0U;
    cmd[5] = 0x00U;
    cmd[6] = (uint8_t)(bootCODE_COPY_BLOCK_WORDS >> 8);
    cmd[7] = (uint8_t)bootCODE_COPY_BLOCK_WORDS;
    cmd[8] = 0U;
    cmd[9] = 0U;
    cmd[10] = 0U;
    cmd[11] = 0U;

    write_dgus_vp(sysDGUS_FLASH_RW_CMD_ADDR + 1U, &cmd[2], 3U);
    write_dgus_vp(sysDGUS_FLASH_RW_CMD_ADDR, cmd, 1U);
    (void)BootWaitDgusCmdIdle(sysDGUS_FLASH_RW_CMD_ADDR);
    delay_ms(FLASH_ACCESS_CYCLE);
}

/**
 * @brief 从指定NOR起始块拷贝代码到APP RAM并校验CRC。
 * @param[in] start_block NOR 4KB块号。
 * @return 用户代码有效时返回1，否则返回0。
 */
uint8_t BootCodeCheck(uint16_t start_block)
{
    uint32_t flash_addr;
    uint32_t target_vp;
    uint16_t crc;
    uint8_t i;
    VP_EXCHANGE exchange_msg;

    DBG_LOG_U16("[BOOT] code check start block=", start_block);
    read_dgus_vp(0xF000U, BootVpF000Cache, bootF000_CACHE_WORDS);

    flash_addr = (uint32_t)start_block << 11;
    target_vp = bootCODE_TARGET_VP_START;

    for(i = 0U; i < bootCODE_COPY_BLOCK_COUNT; i++)
    {
#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
        DebugLog("[BOOT] copy code idx=");
        DebugLogU8(i);
        DebugLog(" flash=");
        DebugLogHex32(flash_addr);
        DebugLog(" target_vp=");
        DebugLogHex32(target_vp);
        DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
        BootReadNorCodeBlock(flash_addr);

        exchange_msg.sourceVP = 0xF000UL;
        exchange_msg.targetVP = target_vp;
        exchange_msg.len = bootF000_CACHE_WORDS;
        exchange_msg.mode = 0U;
        vpExchange(&exchange_msg);

        flash_addr += bootCODE_COPY_BLOCK_WORDS;
        target_vp += bootCODE_COPY_BLOCK_WORDS;
    }

    write_dgus_vp(0xF000U, BootVpF000Cache, bootF000_CACHE_WORDS);

    crc = BootCodeCrc16();
    if(crc == 0U)
    {
        DBG_LOG_LINE("[BOOT] code crc ok");
        return 1U;
    }

    DBG_LOG_HEX16("[BOOT] code crc fail crc=", crc);
    return 0U;
}
