/**
 * @file ota.c
 * @brief UART5 OTA下载与BOOT应用实现。
 * @author yangming
 * @version 1.0.0
 */

#include "ota.h"
#include "boot.h"
#include "debug_uart.h"
#include "timer.h"
#include "uart.h"

#ifndef otaLOG_DETAIL
#define otaLOG_DETAIL 0
#endif
#define otaFRAME_HEAD_0 0xABU
#define otaFRAME_HEAD_1 0xCDU
#define otaCMD_FILE_INFO 0x04U
#define otaCMD_READ_DATA 0x05U
#define otaCMD_FILE_RESULT 0x06U

#define otaSTEP_IDLE 0xFFU
#define otaSTEP_WAIT_DATA 0x50U
#define otaSTEP_WAIT_RESULT_ACK 0x62U
#define otaTIMEOUT_RELOAD 200U
#define otaDATA_START_BLOCK 64U
#define otaHANDSHAKE_LEN 5U
/* T5LU containers are decoded on R11; BOOT receives individual files. */

#define otaNAND_START_ADDR 0x04000000UL
#define otaNOR_RESERVED_ID 21U
#define otaNAND_BLOCK_BYTES 4096UL
#define otaCMD_WAIT_STEP_MS 10U
#define otaCMD_WAIT_TIMEOUT_MS 30000UL
#define otaCOPY_WAIT_TIMEOUT_MS 300000UL

#define OtaReadBe16(buf) \
    ((uint16_t)((((uint16_t)(buf)[0]) << 8) | (uint16_t)(buf)[1]))

#define OtaReadBe32(buf) \
    ((((uint32_t)(buf)[0]) << 24) | \
     (((uint32_t)(buf)[1]) << 16) | \
     (((uint32_t)(buf)[2]) << 8) | \
     (uint32_t)(buf)[3])

#define OtaWriteBe16(buf, value) \
    do { \
        (buf)[0] = (uint8_t)((value) >> 8); \
        (buf)[1] = (uint8_t)(value); \
    }while(0)

#define OtaWriteBe32(buf, value) \
    do { \
        (buf)[0] = (uint8_t)((value) >> 24); \
        (buf)[1] = (uint8_t)((value) >> 16); \
        (buf)[2] = (uint8_t)((value) >> 8); \
        (buf)[3] = (uint8_t)(value); \
    }while(0)

#define OtaSetTimeout(step_value) \
    do { \
        OtaStep = (step_value); \
        OtaTimeout = otaTIMEOUT_RELOAD; \
    }while(0)

static uint8_t xdata OtaVpBlock[otaHEADER_BYTES];

static OtaContext xdata OtaStatus;
static uint16_t xdata OtaTimeout;
static uint8_t OtaStep = otaSTEP_IDLE;
static uint8_t OtaLastResult = 2U;
static uint8_t OtaDownloadCompleteFlag;
static uint8_t OtaFrameActivityFlag;
static uint8_t OtaIoFailed;
static uint8_t OtaApplyResult;
static uint8_t OtaAbort;
static uint32_t OtaWireCrc;
/* One VP window is owned by NAND until write + block CRC complete;
 * the other window accepts the next packet. No extra 4KB xdata is needed. */
static uint8_t OtaWritePending;
static uint16_t OtaReceiveVp = otaCACHE_VP_A;
static uint16_t OtaPendingVp;
static uint32_t OtaPendingAddr;
static uint32_t OtaPendingOffset;
static uint32_t OtaPendingCrc;
static void OtaFinishPendingWrite(void);

/**
 * @brief 计算与R11一致的反射CRC32，独立校验UART实际接收的字节流。
 * @param[in] crc 当前CRC值，首字节传入0。
 * @param[in] bytes 输入数据缓存(xdata)。
 * @param[in] len 数据字节长度。
 * @return 更新后的CRC32值。
 */
static uint32_t OtaWireCrcUpdate(uint32_t crc, uint8_t xdata *bytes, uint16_t len)
{
    uint8_t bit_index;
    crc = ~crc;
    while(len--) {
        crc ^= *bytes++;
        for(bit_index=0U; bit_index<8U; ++bit_index) crc=(crc>>1)^((crc&1UL)?0xEDB88320UL:0UL);
    }
    return ~crc;
}

static void OtaLogFailure(char *stage, uint32_t where, uint32_t expected, uint32_t actual)
{
#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    DebugLog("[OTA] error stage=");DebugLog(stage);
    DebugLog(" id=");DebugLogU16(OtaStatus.file[OtaStatus.now_num].unid);
    DebugLog(" at=");DebugLogHex32(where);
    DebugLog(" expected=");DebugLogHex32(expected);
    DebugLog(" actual=");DebugLogHex32(actual);DebugLog("\r\n");
#else
    (void)stage;(void)where;(void)expected;(void)actual;
#endif
}

static uint8_t OtaWaitDgusCmdIdleFor(uint32_t cmd_addr, uint32_t timeout_ms)
{
    uint8_t cmd_state[2];
    uint32_t elapsed_ms;

    elapsed_ms = 0U;
    cmd_state[0] = 0xFFU;
    while(elapsed_ms < timeout_ms)
    {
        read_dgus_vp(cmd_addr, cmd_state, 1U);
        if(cmd_state[0] == 0U)
        {
            return 1U;
        }
        delay_ms(otaCMD_WAIT_STEP_MS);
        elapsed_ms += otaCMD_WAIT_STEP_MS;
    }

    OtaIoFailed = 1U;
    OtaLogFailure("timeout",cmd_addr,0UL,((uint32_t)cmd_state[0]<<8)|cmd_state[1]);
    return 0U;
}

static uint8_t OtaWaitDgusCmdIdle(uint32_t cmd_addr)
{
    return OtaWaitDgusCmdIdleFor(cmd_addr, otaCMD_WAIT_TIMEOUT_MS);
}

/**
 * @brief 计算xdata缓冲的Modbus风格CRC16。
 */
static uint16_t OtaCrc16(uint8_t xdata *update_data, uint16_t len)
{
    uint16_t crc;
    uint16_t i;
    uint8_t j;

    crc = 0xFFFFU;
    for(i = 0U; i < len; i++)
    {
        crc ^= *update_data++;
        for(j = 0U; j < 8U; j++)
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
    }

    return crc;
}

/**
 * @brief 32位向上整除，结果按uint16_t上限饱和。
 */
static uint16_t OtaCeilDiv32(uint32_t value, uint32_t divisor)
{
    uint32_t result;

    if(value == 0UL)
    {
        return 1U;
    }

    result = value / divisor;
    if((value % divisor) != 0UL)
    {
        result++;
    }

    if(result > 0xFFFFUL)
    {
        result = 0xFFFFUL;
    }

    return (uint16_t)result;
}


/**
 * @brief 启动NAND到NOR的拷贝操作。
 */
static void OtaSubmitNandCommand(uint8_t *cmd)
{
    /* DGUS observes the enable word asynchronously. Publish every parameter
     * and clear the result area BEFORE enabling; never write the tail after. */
    if(OtaIoFailed || !OtaWaitDgusCmdIdle(sysDGUS_NAND_CMD_ADDR)) return;
    write_dgus_vp(sysDGUS_NAND_CMD_ADDR + 1U, &cmd[2], 5U);
    write_dgus_vp(sysDGUS_NAND_CMD_ADDR, cmd, 1U);
    /* Multi-library copies can exceed the normal write/CRC timeout.
     * Wait for the enable byte to clear; never reissue a still-running copy. */
    (void)OtaWaitDgusCmdIdleFor(sysDGUS_NAND_CMD_ADDR,
        cmd[1] == 0x06U ? otaCOPY_WAIT_TIMEOUT_MS : otaCMD_WAIT_TIMEOUT_MS);
}

/**
 * @brief 启动NAND到NOR的大批量拷贝(命令0x06)。
 * @param[in] nand_addr NAND源字节地址。
 * @param[in] nor_id 目标NOR编号。
 * @param[in] block_count 拷贝的4KB块数。
 */
static void OtaCopyNandToNor(uint32_t nand_addr, uint8_t nor_id, uint16_t block_count)
{
    uint8_t cmd[12];

    cmd[0] = 0x5AU;
    cmd[1] = 0x06U;
    OtaWriteBe32(&cmd[2], nand_addr);
    cmd[6] = 0U;
    cmd[7] = nor_id;
    OtaWriteBe16(&cmd[8], block_count);
    cmd[10] = 0U;
    cmd[11] = 0U;

    OtaSubmitNandCommand(cmd);
}

/**
 * @brief 将NOR文件块读取到DGUS VP缓存。
 */
static void OtaReadNorToVp(uint8_t nor_id, uint32_t nor_offset,
                           uint16_t vp_addr, uint16_t len_words)
{
    uint8_t cmd[12];

    cmd[0] = 0x5AU;
    cmd[1] = 0x01U;
    cmd[2] = nor_id;
    cmd[3] = (uint8_t)(nor_offset >> 16);
    cmd[4] = (uint8_t)(nor_offset >> 8);
    cmd[5] = (uint8_t)nor_offset;
    OtaWriteBe16(&cmd[6], vp_addr);
    OtaWriteBe16(&cmd[8], len_words);
    cmd[10] = 0U;
    cmd[11] = 0U;

    OtaSubmitNandCommand(cmd);
}

/**
 * @brief 通过0x0008命令将1个4KB VP缓存块写入NOR。
 */
static void OtaWriteF000ToNor(uint32_t flash_addr)
{
    uint8_t cmd[12];

    cmd[0] = 0xA5U;
    cmd[1] = (uint8_t)(flash_addr >> 16);
    cmd[2] = (uint8_t)(flash_addr >> 8);
    cmd[3] = (uint8_t)flash_addr;
    cmd[4] = 0xF0U;
    cmd[5] = 0x00U;
    cmd[6] = 0x08U;
    cmd[7] = 0x00U;
    cmd[8] = 0U;
    cmd[9] = 0U;
    cmd[10] = 0U;
    cmd[11] = 0U;

    write_dgus_vp(sysDGUS_FLASH_RW_CMD_ADDR + 1U, &cmd[2], 3U);
    write_dgus_vp(sysDGUS_FLASH_RW_CMD_ADDR, cmd, 1U);
    (void)OtaWaitDgusCmdIdle(sysDGUS_FLASH_RW_CMD_ADDR);
    delay_ms(FLASH_ACCESS_CYCLE);
}

/**
 * @brief 启动从VP缓存到NAND的写入操作。
 */
static void OtaStartNandWrite(uint32_t nand_addr, uint16_t vp_addr, uint16_t block_count)
{
    uint8_t cmd[12];

    cmd[0] = 0x5AU;
    cmd[1] = 0x04U;
    OtaWriteBe32(&cmd[2], nand_addr);
    OtaWriteBe16(&cmd[6], vp_addr);
    cmd[8] = (uint8_t)block_count;
    cmd[9] = (uint8_t)(block_count >> 8);
    cmd[10] = 0U;
    cmd[11] = 0U;
    /* Publish the command and return while DGUS writes this VP window. */
    if(OtaIoFailed || !OtaWaitDgusCmdIdle(sysDGUS_NAND_CMD_ADDR)) return;
    write_dgus_vp(sysDGUS_NAND_CMD_ADDR + 1U, &cmd[2], 5U);
    write_dgus_vp(sysDGUS_NAND_CMD_ADDR, cmd, 1U);
}

/**
 * @brief 启动NAND CRC32计算。
 */
static void OtaStartNandCrc32(uint32_t nand_addr, uint16_t block_count)
{
    uint8_t cmd[12];

    cmd[0] = 0x5AU;
    cmd[1] = 0x05U;
    OtaWriteBe32(&cmd[2], nand_addr);
    OtaWriteBe16(&cmd[6], block_count);
    cmd[8] = 0U;
    cmd[9] = 0U;
    cmd[10] = 0U;
    cmd[11] = 0U;
    OtaSubmitNandCommand(cmd);
}

/**
 * @brief 读取NAND CRC32结果。
 */
static uint32_t OtaReadNandCrc32(void)
{
    uint8_t crc_buf[4];

    read_dgus_vp(sysDGUS_NAND_CRC_ADDR, crc_buf, 2U);




    /* DGUS returns D3:D0 in network (big-endian) order. */
    return OtaReadBe32(crc_buf);
}

/**
 * @brief 清空共用4KB工作缓存。
 */
static void OtaClearWorkBlock(void)
{
    uint16_t i;

    for(i = 0U; i < otaHEADER_BYTES; i++)
    {
        OtaVpBlock[i] = 0U;
    }
}

/**
 * @brief 将OTA分包载荷拷贝到共用4KB工作缓存。
 */
static void OtaCopyPacketToWorkBlock(uint8_t xdata *packet_data, uint16_t packet_len)
{
    uint16_t i;

    OtaClearWorkBlock();
    for(i = 0U; i < packet_len; i++)
    {
        OtaVpBlock[i] = packet_data[i];
    }
}

/**
 * @brief 通过UART5发送1帧AB CD OTA数据。
 */
static void OtaSendFrame(uint8_t *buf, uint16_t len)
{
    OtaWriteBe16(&buf[2], len - 4U);
    UartSendData(&Uart5, buf, len);
}

/**
 * @brief 按下载字节数更新升级进度。
 */
static void OtaUpdateDownloadProgress(uint16_t packet_len)
{
    uint32_t progress;

    OtaStatus.downloaded_size += packet_len;
    if(OtaStatus.all_size == 0UL)
    {
        return;
    }
    if(OtaStatus.downloaded_size > OtaStatus.all_size)
    {
        OtaStatus.downloaded_size = OtaStatus.all_size;
    }

    progress = (OtaStatus.downloaded_size * 100UL) / OtaStatus.all_size;
    if(progress > 100UL)
    {
        progress = 100UL;
    }

    BootWriteProgress((uint8_t)progress);
}

/**
 * @brief 发送升级握手指令。
 */
static void OtaSendHandshake(void)
{
    uint8_t handshake[otaHANDSHAKE_LEN];

    DBG_LOG_LINE("[OTA] send handshake");
    handshake[0] = 0xAAU;
    handshake[1] = 0x55U;
    handshake[2] = 0x00U;
    handshake[3] = 0x01U;
    handshake[4] = 0xF3U;
    UartSendData(&Uart5, handshake, otaHANDSHAKE_LEN);
}

/**
 * @brief 发送05数据请求。
 */
static void OtaSendData05(void)
{
    uint8_t send_buf[32];
    uint16_t send_len;
    OtaFileInfo *file;

    send_len = 0U;
    file = &OtaStatus.file[OtaStatus.now_num];

    send_buf[send_len++] = otaFRAME_HEAD_0;
    send_buf[send_len++] = otaFRAME_HEAD_1;
    send_buf[send_len++] = 0U;
    send_buf[send_len++] = 0U;
    send_buf[send_len++] = otaCMD_READ_DATA;
    send_buf[send_len++] = file->itype;
    send_buf[send_len++] = file->apply;
    OtaWriteBe16(&send_buf[send_len], file->unid);
    send_len += 2U;
    OtaWriteBe32(&send_buf[send_len], OtaStatus.off_position);
    send_len += 4U;
    OtaWriteBe32(&send_buf[send_len], OtaStatus.off_len);
    send_len += 4U;

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED && otaLOG_DETAIL
    DebugLog("[OTA] send 05 idx=");
    DebugLogU8(OtaStatus.now_num);
    DebugLog(" off=");
    DebugLogU32(OtaStatus.off_position);
    DebugLog(" len=");
    DebugLogU32(OtaStatus.off_len);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
    OtaSendFrame(send_buf, send_len);
}

/**
 * @brief 发送06文件结果响应。
 */
static void OtaSendData06(uint8_t result)
{
    uint8_t send_buf[16];
    uint16_t send_len;
    OtaFileInfo *file;

    send_len = 0U;
    OtaLastResult = result;
    file = &OtaStatus.file[OtaStatus.now_num];

    send_buf[send_len++] = otaFRAME_HEAD_0;
    send_buf[send_len++] = otaFRAME_HEAD_1;
    send_buf[send_len++] = 0U;
    send_buf[send_len++] = 0U;
    send_buf[send_len++] = otaCMD_FILE_RESULT;
    send_buf[send_len++] = file->itype;
    send_buf[send_len++] = file->apply;
    OtaWriteBe16(&send_buf[send_len], file->unid);
    send_len += 2U;
    send_buf[send_len++] = result;

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    DebugLog("[OTA] send 06 idx=");
    DebugLogU8(OtaStatus.now_num);
    DebugLog(" result=");
    DebugLogU8(result);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
    OtaSendFrame(send_buf, send_len);
}

/**
 * @brief 校验05数据分包CRC16。
 */
static uint8_t OtaPacketCrc16Ok(uint8_t xdata *frame, uint16_t packet_len)
{
    uint16_t crc_calc;
    uint16_t crc_recv;

    crc_calc = OtaCrc16(&frame[26], packet_len);
    crc_recv = OtaReadBe16(&frame[26U + packet_len]);

    return (crc_calc == crc_recv) ? 1U : 0U;
}

/**
 * @brief 初始化OTA下载状态。
 */
void OtaInit(void)
{
    uint16_t i;
    uint8_t *ctx;

    DBG_LOG_LINE("[OTA] init");
    /* Never release a VP source while a previous session still writes it. */
    if(OtaWritePending) {
        OtaFinishPendingWrite();
        if(OtaWritePending) return;
    }
    ctx = (uint8_t *)&OtaStatus;
    for(i = 0U; i < sizeof(OtaStatus); i++)
    {
        ctx[i] = 0U;
    }

    OtaStatus.flash_start_num = otaDATA_START_BLOCK;
    OtaStep = otaSTEP_IDLE;
    OtaTimeout = 0U;
    OtaLastResult = 2U;
    OtaDownloadCompleteFlag = 0U;
    OtaIoFailed = 0U;
    OtaApplyResult = 0U;
    OtaAbort = 0U;
    OtaReceiveVp = otaCACHE_VP_A;
}

/**
 * @brief 处理04文件信息命令。
 */
static void OtaHandleFileInfo(uint8_t xdata *frame, uint16_t len)
{
    uint8_t name_len;
    uint16_t status_index;
    uint16_t crc_index;
    uint16_t blocks;
    OtaFileInfo *file;

    if(len < 25U)
    {
        DBG_LOG_U16("[OTA] 04 short len=", len);
        return;
    }

    name_len = frame[19];
    status_index = 20U + (uint16_t)name_len;
    crc_index = status_index + 1U;

    if((status_index >= len) || ((crc_index + 4U) > len))
    {
        DBG_LOG_U16("[OTA] 04 bad name len=", name_len);
        return;
    }
    if(frame[status_index] != 0U)
    {
        DBG_LOG_HEX8("[OTA] 04 status skip=", frame[status_index]);
        return;
    }
    if(frame[6] >= otaDOWNLOAD_MAX)
    {
        DBG_LOG_U8("[OTA] 04 index overflow idx=", frame[6]);
        return;
    }

    if(frame[11] < 1U || frame[11] > 8U || frame[12] != 1U) return;
    if(frame[11] == 1U && (OtaReadBe16(&frame[13]) > 127U || OtaReadBe32(&frame[15]) > 262144UL)) return;
    if(frame[11] != 1U && OtaReadBe16(&frame[13]) > 224U) return;
    if(frame[5] == 0U || frame[5] > otaDOWNLOAD_MAX || frame[6] >= frame[5] ||
       OtaReadBe32(&frame[15]) == 0UL || (OtaReadBe32(&frame[15]) & 4095UL) != 0UL) return;
    if(OtaStep != otaSTEP_IDLE && frame[6] == OtaStatus.now_num)
    {
        if(OtaStep == otaSTEP_WAIT_DATA) OtaSendData05();
        else if(OtaStep == otaSTEP_WAIT_RESULT_ACK) OtaSendData06(OtaLastResult);
        return;
    }
    if(frame[6] != 0U && frame[6] != OtaStatus.now_num + 1U) return;
    if(OtaStep != otaSTEP_IDLE) return;
    if(OtaReadBe32(&frame[15]) / 4096UL > (uint32_t)(0xFFFFU - OtaStatus.flash_start_num)) return;
    if(frame[6] == 0U)
    {
        OtaInit();
        if(OtaIoFailed) return;
    }

    OtaStatus.total_num = frame[5];
    if(OtaStatus.total_num > otaDOWNLOAD_MAX)
    {
        OtaStatus.total_num = otaDOWNLOAD_MAX;
    }

    OtaStatus.now_num = frame[6];
    OtaStatus.off_position = 0UL;
    OtaStatus.off_len = otaNAND_BLOCK_BYTES;
    OtaStatus.all_size = OtaReadBe32(&frame[7]);
    file = &OtaStatus.file[OtaStatus.now_num];
    file->itype = frame[11];
    file->apply = frame[12];
    file->unid = OtaReadBe16(&frame[13]);
    file->size = OtaReadBe32(&frame[15]);
    file->crc32 = OtaReadBe32(&frame[crc_index]);
    OtaWireCrc = 0UL;
    file->flash_start = OtaStatus.flash_start_num;

    blocks = OtaCeilDiv32(file->size, otaNAND_BLOCK_BYTES);
    OtaStatus.flash_start_num += blocks;
    OtaStatus.download_end_flag = 0U;
    OtaFrameActivityFlag = 1U;

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    DebugLog("[OTA] file info idx=");
    DebugLogU8(OtaStatus.now_num);
    DebugLog("/");
    DebugLogU8(OtaStatus.total_num);
    DebugLog(" type=");
    DebugLogU8(file->itype);
    DebugLog(" apply=");
    DebugLogU8(file->apply);
    DebugLog(" unid=");
    DebugLogU16(file->unid);
    DebugLog(" size=");
    DebugLogU32(file->size);
    DebugLog(" crc=");
    DebugLogHex32(file->crc32);
    DebugLog(" nand_blk=");
    DebugLogU16(file->flash_start);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
    OtaSendData05();
    OtaSetTimeout(otaSTEP_WAIT_DATA);
}

/**
 * @brief 下一包已暂存后，等待上一块写入并校验；文件尾也必须调用。
 */
static void OtaFinishPendingWrite(void)
{
    uint16_t vp_addr;
    uint32_t nand_addr, block_crc, nand_crc;

    if(!OtaWritePending) return;
    if(!OtaWaitDgusCmdIdle(sysDGUS_NAND_CMD_ADDR)) return;
    OtaWritePending = 0U;
    if(OtaIoFailed) return;
    vp_addr = OtaPendingVp;
    nand_addr = OtaPendingAddr;
    block_crc = OtaPendingCrc;
    OtaStartNandCrc32(nand_addr, 1U);
    if(OtaIoFailed) return;
    delay_ms(50U);
    nand_crc=OtaReadNandCrc32();
    if(nand_crc != block_crc) {
        OtaLogFailure("NAND",OtaPendingOffset,block_crc,nand_crc);
        /* Rewriting an erase boundary is safe only before later blocks are sent.
         * Re-publish VP after settling, then allow exactly one erase/write retry.
         * Never erase/retry a non-boundary block: that could destroy earlier data. */
        if((nand_addr & 0x3FFFFUL) != 0UL) {
            OtaIoFailed = 1U;
            return;
        }
        DBG_LOG_LINE("[OTA] retry erase-boundary block once");
        delay_ms(200U);
        /* Work buffer holds the next packet; reload the retained source. */
        read_dgus_vp(vp_addr, OtaVpBlock, otaHEADER_BYTES / 2U);
        write_dgus_vp(vp_addr, OtaVpBlock, otaHEADER_BYTES / 2U);
        OtaStartNandWrite(nand_addr, vp_addr, 1U);
        if(OtaIoFailed) return;
        OtaWritePending = 1U;
        if(!OtaWaitDgusCmdIdle(sysDGUS_NAND_CMD_ADDR)) return;
        OtaWritePending = 0U;
        OtaStartNandCrc32(nand_addr, 1U);
        if(OtaIoFailed) return;
        delay_ms(50U);
        nand_crc=OtaReadNandCrc32();
        if(nand_crc != block_crc) {
            OtaLogFailure("NAND-retry",OtaPendingOffset,block_crc,nand_crc);
            OtaIoFailed = 1U;
            return;
        }
        DBG_LOG_LINE("[OTA] NAND block retry verified");
    }
    OtaUpdateDownloadProgress(otaHEADER_BYTES);
}

/**
 * @brief 将1个分包载荷写入NAND。
 */
static void OtaWritePacketToNand(uint8_t xdata *frame, uint16_t packet_len)
{
    uint16_t vp_addr;
    uint16_t i;
    uint32_t block_crc;
    uint32_t now_packet;
    uint32_t nand_addr;

    now_packet = (uint32_t)OtaStatus.file[OtaStatus.now_num].flash_start +
                 (OtaStatus.off_position / otaNAND_BLOCK_BYTES);

    vp_addr = OtaReceiveVp;

    OtaCopyPacketToWorkBlock(&frame[26], packet_len);
    write_dgus_vp(vp_addr, OtaVpBlock, otaHEADER_BYTES / 2U);
    
    /* Verify the actual VP cache, not only the UART receive buffer. */
    read_dgus_vp(vp_addr, OtaVpBlock, otaHEADER_BYTES / 2U);
    for(i = 0U; i < packet_len; ++i) {
        if(OtaVpBlock[i] != frame[26U + i]) {
            OtaLogFailure("VP",OtaStatus.off_position+i,frame[26U+i],OtaVpBlock[i]);
            OtaIoFailed = 1U;
            return;
        }
    }
    nand_addr = otaNAND_START_ADDR + (now_packet * otaNAND_BLOCK_BYTES);
#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    if(OtaStatus.off_position==0UL || OtaStatus.off_position%262144UL==0UL || OtaStatus.off_position+packet_len==OtaStatus.file[OtaStatus.now_num].size) {
    DebugLog("[OTA] write packet idx=");
    DebugLogU8(OtaStatus.now_num);
    DebugLog(" packet_blk=");
    DebugLogU32(now_packet);
    DebugLog(" nand=");
    DebugLogHex32(nand_addr);
    DebugLog(" vp=");
    DebugLogHex16(vp_addr);
    DebugLog(" len=");
    DebugLogU16(packet_len);
    DebugLog("\r\n");
    }
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
    block_crc = OtaWireCrcUpdate(0UL, OtaVpBlock, packet_len);
    /* Stage the next packet before joining the previous write. */
    OtaFinishPendingWrite();
    if(OtaIoFailed) return;
    OtaStartNandWrite(nand_addr, vp_addr, 1U);
    if(OtaIoFailed) return;
    OtaPendingVp = vp_addr;
    OtaPendingAddr = nand_addr;
    OtaPendingOffset = OtaStatus.off_position;
    OtaPendingCrc = block_crc;
    OtaWritePending = 1U;
    OtaReceiveVp = (vp_addr == otaCACHE_VP_A) ? otaCACHE_VP_B : otaCACHE_VP_A;
}

/**
 * @brief 通过NAND控制器校验当前文件CRC32。
 */
static uint8_t OtaFileCrcOk(void)
{
    uint16_t blocks;
    uint32_t nand_addr;
    uint32_t crc32;
    OtaFileInfo *file;

#if !otaCRC32_CHECK_ENABLED
    DBG_LOG_LINE("[OTA] file crc skipped");
    return 1U;
#else
    file = &OtaStatus.file[OtaStatus.now_num];
    if(OtaWireCrc != file->crc32) {
        DBG_LOG_LINE("[OTA] UART content CRC mismatch; NAND apply blocked");
        return 0U;
    }
    blocks = OtaCeilDiv32(file->size, otaNAND_BLOCK_BYTES);
    nand_addr = otaNAND_START_ADDR + ((uint32_t)file->flash_start * otaNAND_BLOCK_BYTES);

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED && otaLOG_DETAIL
    DebugLog("[OTA] file crc start idx=");
    DebugLogU8(OtaStatus.now_num);
    DebugLog(" nand=");
    DebugLogHex32(nand_addr);
    DebugLog(" blocks=");
    DebugLogU16(blocks);
    DebugLog(" expect=");
    DebugLogHex32(file->crc32);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */

    OtaStartNandCrc32(nand_addr, blocks);
    delay_ms(50U);
    crc32 = OtaReadNandCrc32();

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    DebugLog("[OTA] file crc ");
    if(crc32 == file->crc32)
    {
        DebugLog("ok");
    }
    else
    {
        DebugLog("fail");
    }
    DebugLog(" expected=");
    DebugLogHex32(file->crc32);
    DebugLog(" uart=");
    DebugLogHex32(OtaWireCrc);
    DebugLog(" got=");
    DebugLogHex32(crc32);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
    return (!OtaIoFailed && crc32 == file->crc32) ? 1U : 0U;
#endif
}

/**
 * @brief 处理05分包数据命令。
 */
static void OtaHandlePacketData(uint8_t xdata *frame, uint16_t len)
{
    uint16_t payload_len;
    OtaFileInfo *file;

    if(OtaStep != otaSTEP_WAIT_DATA) return;
    if(len < 30U)
    {
        DBG_LOG_U16("[OTA] 05 short len=", len);
        return;
    }

    payload_len = OtaReadBe16(&frame[2]);
    if(payload_len < 24U)
    {
        DBG_LOG_U16("[OTA] 05 bad payload header=", payload_len);
        return;
    }
    payload_len -= 24U;

    if((payload_len > otaNAND_BLOCK_BYTES) || ((26U + payload_len + 2U) > len))
    {
        DBG_LOG_U16("[OTA] 05 bad payload len=", payload_len);
        return;
    }

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED && otaLOG_DETAIL
    DebugLog("[OTA] recv 05 idx=");
    DebugLogU8(OtaStatus.now_num);
    DebugLog(" off=");
    DebugLogU32(OtaStatus.off_position);
    DebugLog(" len=");
    DebugLogU16(payload_len);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */

    if(OtaPacketCrc16Ok(frame, payload_len) == 0U)
    {
        DBG_LOG_LINE("[OTA] packet crc fail retry");
        OtaSendData05();
        OtaSetTimeout(otaSTEP_WAIT_DATA);
        return;
    }

    file = &OtaStatus.file[OtaStatus.now_num];
    if(OtaStep != otaSTEP_WAIT_DATA || frame[5] != 1U || frame[6] != file->itype ||
       frame[7] != file->apply || OtaReadBe16(&frame[8]) != file->unid ||
       OtaReadBe32(&frame[10]) != file->size || OtaReadBe32(&frame[14]) != OtaStatus.off_position ||
       OtaReadBe32(&frame[18]) != OtaStatus.off_len || OtaReadBe32(&frame[22]) != payload_len ||
       payload_len != 4096U) return;
    OtaStep = otaSTEP_IDLE;
    OtaFrameActivityFlag = 1U;
    OtaWireCrc = OtaWireCrcUpdate(OtaWireCrc, &frame[26], payload_len);
    OtaWritePacketToNand(frame, payload_len);
    if(OtaIoFailed) { OtaSendData06(3U); OtaSetTimeout(otaSTEP_WAIT_RESULT_ACK); return; }

    file = &OtaStatus.file[OtaStatus.now_num];
    if((OtaStatus.off_position + OtaStatus.off_len) >= file->size)
    {
        OtaFinishPendingWrite();
        if(!OtaIoFailed && OtaFileCrcOk() != 0U)
        {
            OtaSendData06(2U);
            OtaSetTimeout(otaSTEP_WAIT_RESULT_ACK);
            if((OtaStatus.now_num + 1U) >= OtaStatus.total_num)
            {
                OtaStatus.download_end_flag = 0x01U;
            }
        }
        else
        {
            OtaSendData06(3U);
            OtaSetTimeout(otaSTEP_WAIT_RESULT_ACK);
        }
    }
    else
    {
        OtaStatus.off_position += OtaStatus.off_len;
        OtaSendData05();
        OtaSetTimeout(otaSTEP_WAIT_DATA);
    }
}

/**
 * @brief 处理06结果确认。
 */
static void OtaHandleFileResultAck(uint8_t xdata *frame, uint16_t len)
{
    OtaFileInfo *file = &OtaStatus.file[OtaStatus.now_num];
    if(len != 10U || OtaStep != otaSTEP_WAIT_RESULT_ACK || frame[5] != file->itype ||
       frame[6] != file->apply || OtaReadBe16(&frame[7]) != file->unid || frame[9] != 0U) return;
    OtaStep = otaSTEP_IDLE;
    OtaFrameActivityFlag = 1U;
    if(OtaStatus.download_end_flag == 0x01U)
    {
        OtaStatus.download_end_flag = 0x11U;
    }
}

/**
 * @brief 将1帧AB CD数据分发到OTA状态机。
 */
void OtaReceive(uint8_t xdata *frame, uint16_t len)
{
    uint16_t payload_len;

    if((frame == NULL) || (len < 5U))
    {
        DBG_LOG_U16("[OTA] rx invalid len=", len);
        return;
    }
    if((frame[0] != otaFRAME_HEAD_0) || (frame[1] != otaFRAME_HEAD_1))
    {
        DBG_LOG_LINE("[OTA] rx bad head");
        return;
    }

    payload_len = OtaReadBe16(&frame[2]);
    if((payload_len + 4U) != len)
    {
        DBG_LOG_U16("[OTA] rx bad frame len=", len);
        return;
    }

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED && otaLOG_DETAIL
    DebugLog("[OTA] rx cmd=");
    DebugLogHex8(frame[4]);
    DebugLog(" payload=");
    DebugLogU16(payload_len);
    DebugLog(" len=");
    DebugLogU16(len);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */

    switch(frame[4])
    {
        case otaCMD_FILE_INFO:
            OtaHandleFileInfo(frame, len);
            break;
        case otaCMD_READ_DATA:
            OtaHandlePacketData(frame, len);
            break;
        case otaCMD_FILE_RESULT:
            OtaHandleFileResultAck(frame, len);
            break;
        case 0x07U:
            if(len == 6U && frame[5] == 0U) OtaFrameActivityFlag = 1U;
            if(len == 6U && frame[5] == 1U && OtaApplyResult != 3U) OtaAbort = 1U;
            break;
        case 0x08U:
            if(len == 6U && frame[5] == 0U && OtaApplyResult != 0U)
            {
                uint8_t result_buf[6] = {0xABU, 0xCDU, 0U, 2U, 8U, 0U};
                result_buf[5] = OtaApplyResult;
                OtaSendFrame(result_buf, 6U);
            }
            break;
        default:
            DBG_LOG_HEX8("[OTA] rx unknown cmd=", frame[4]);
            break;
    }
}

/**
 * @brief 执行超时、重试和完成检测。
 */
void OtaTask(void)
{
    if(OtaStep != otaSTEP_IDLE)
    {
        if(OtaTimeout == 0U)
        {
            if((OtaStep & 0xF0U) == 0x50U)
            {
                DBG_LOG_HEX8("[OTA] timeout retry step=", OtaStep);
                OtaSendData05();
                OtaSetTimeout(otaSTEP_WAIT_DATA);
            }
            else if((OtaStep & 0xF0U) == 0x60U)
            {
                DBG_LOG_HEX8("[OTA] timeout resend result step=", OtaStep);
                OtaSendData06(OtaLastResult);
                OtaSetTimeout(otaSTEP_WAIT_RESULT_ACK);
            }
            else
            {
                DBG_LOG_HEX8("[OTA] timeout idle step=", OtaStep);
                OtaStep = otaSTEP_IDLE;
            }
        }
    }

    if(OtaStatus.download_end_flag == 0x11U)
    {
    
        OtaStatus.download_end_flag = 0U;
        OtaDownloadCompleteFlag = 1U;
    }
}

/**
 * @brief OTA 1ms超时节拍，由Timer0调用。
 */
void OtaTimerTick1ms(void)
{
    static uint8_t tick_10ms = 0U;

    tick_10ms++;
    if(tick_10ms >= 10U)
    {
        tick_10ms = 0U;
        if(OtaTimeout != 0U)
        {
            OtaTimeout--;
        }
    }
}

/**
 * @brief 应用下载包中的1个内部NOR文件。
 */
static void OtaApplyInternalFile(OtaFileInfo *file)
{
    uint32_t nand_addr;
    uint32_t block_count;

    nand_addr = (uint32_t)file->flash_start;
    nand_addr *= otaNAND_BLOCK_BYTES;
    nand_addr += otaNAND_START_ADDR;

    if(file->unid >= 0x00C0U)
    {
        block_count = OtaCeilDiv32(file->size, 8388608UL);
    }
    else
    {
        block_count = OtaCeilDiv32(file->size, 262144UL);
    }
    if(block_count > 255UL)
    {
        block_count = 255UL;
    }

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    DebugLog("[OTA] apply internal unid=");
    DebugLogU16(file->unid);
    DebugLog(" nand=");
    DebugLogHex32(nand_addr);
    DebugLog(" blocks=");
    DebugLogU32(block_count);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
    OtaCopyNandToNor(nand_addr, (uint8_t)file->unid, (uint16_t)block_count);
}

/**
 * @brief 应用下载包中的1个外部字库文件。
 */
static void OtaApplyLibraryFile(OtaFileInfo *file)
{
    uint32_t nand_addr;
    uint32_t extern_flash_addr;
    uint32_t flash_addr;
    uint8_t block_index;
    uint8_t block_count;
    uint16_t block_count_words;

    nand_addr = (uint32_t)file->flash_start;
    nand_addr *= otaNAND_BLOCK_BYTES;
    nand_addr += otaNAND_START_ADDR;

#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
    DebugLog("[OTA] apply lib unid=");
    DebugLogU16(file->unid);
    DebugLog(" size=");
    DebugLogU32(file->size);
    DebugLog(" nand=");
    DebugLogHex32(nand_addr);
    DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */

    OtaCopyNandToNor(nand_addr, otaNOR_RESERVED_ID, 1U);

    block_count_words = OtaCeilDiv32(file->size, otaNAND_BLOCK_BYTES);
    if(block_count_words > 64U)
    {
        block_count_words = 64U;
    }
    block_count = (uint8_t)block_count_words;

    extern_flash_addr = 0UL;
    flash_addr = (uint32_t)file->unid * 2048UL;

    for(block_index = 0U; block_index < block_count; block_index++)
    {
#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED && otaLOG_DETAIL
        DebugLog("[OTA] apply lib block=");
        DebugLogU8(block_index);
        DebugLog(" nor_off=");
        DebugLogHex32(extern_flash_addr);
        DebugLog(" flash=");
        DebugLogHex32(flash_addr);
        DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
        if(OtaIoFailed) return;
        OtaReadNorToVp(otaNOR_RESERVED_ID, extern_flash_addr, 0xF000U, 0x0800U);
        if(OtaIoFailed) return;
        OtaWriteF000ToNor(flash_addr);

        extern_flash_addr += 2048UL;
        flash_addr += 2048UL;
    }
}

/**
 * @brief 应用当前下载上下文中的升级包。
 */
uint8_t OtaActionFromDownload(void)
{
    uint8_t file_index;
    OtaFileInfo *file;

    OtaFinishPendingWrite();
    if(OtaIoFailed) return 0U;
    DBG_LOG_U8("[OTA] apply start total=", OtaStatus.total_num);
    for(file_index = 0U; file_index < OtaStatus.total_num; file_index++)
    {
        if(OtaIoFailed) return 0U;
        file = &OtaStatus.file[file_index];
#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
        DebugLog("[OTA] apply file idx=");
        DebugLogU8(file_index);
        DebugLog(" type=");
        DebugLogU8(file->itype);
        DebugLog(" unid=");
        DebugLogU16(file->unid);
        DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
        if((file->itype == 1U) && (file->unid <= 127U))
        {
            OtaApplyLibraryFile(file);
        }
        else if((file->itype >= 2U) && (file->itype <= 8U) && (file->unid <= 255U))
        {
            OtaApplyInternalFile(file);
        }
        else
        {
            DBG_LOG_U8("[OTA] skip file idx=", file_index);
            __NOP();
        }
    }
    if(OtaIoFailed) {
        DBG_LOG_LINE("[OTA] apply failed; restart remains disabled");
        return 0U;
    }
    DBG_LOG_LINE("[OTA] apply done");
    return 1U;
}

/**
 * @brief 上电后先等待UART5 recovery控制帧。
 */
void BootWaitRecoveryCommand(void)
{
    uint32_t elapsed_ms;
    uint8_t recovery_type;
    uint8_t control_buf[BOOT_CTRL_BYTES];

    elapsed_ms = 0UL;

    DBG_LOG_U32("[BOOT] recovery window ms=", BOOT_RECOVERY_WINDOW_MS);
    Uart5Init(uartUART5_BAUDRATE);
    TimerInit();

    while(elapsed_ms < BOOT_RECOVERY_WINDOW_MS)
    {
        recovery_type = UartRecoveryGetControl(control_buf);
        if(recovery_type != UART_RECOVERY_NONE)
        {
#if debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED
            DebugLog("[BOOT] recovery cmd type=");
            DebugLogU8(recovery_type);
            DebugLog(" value=");
            DebugLogHex8(control_buf[0]);
            DebugLog(" ");
            DebugLogHex8(control_buf[1]);
            DebugLog(" ");
            DebugLogHex8(control_buf[2]);
            DebugLog(" ");
            DebugLogHex8(control_buf[3]);
            DebugLog("\r\n");
#endif /* debugUART2_ENABLED && debugLOG_KEY_FLOW_ENABLED */
            BootSetControl(control_buf, 1U);
            break;
        }

        delay_ms(1U);
        elapsed_ms++;
    }

    TimerStop();
    Uart5Stop();
    DBG_LOG_LINE("[BOOT] recovery window done");
    BootReloadConfigFromFlash();
    DBG_LOG_LINE("[BOOT] config reloaded");
}

/**
 * @brief 进入UART5升级模式，完成或超时后返回正常加载流程。
 */
void BootEnterUpgradeMode(void)
{
    uint32_t idle_ms;
    uint16_t ready_ms;

    idle_ms = 0UL;
    ready_ms = 0U;

    DBG_LOG_LINE("[BOOT] enter upgrade mode");
    OtaInit();
    BootClearRestartState();
    OtaFrameActivityFlag = 0U;
    Uart5Init(uartUART5_BAUDRATE);
    TimerInit();
    BootWriteProgress(0U);
    BootSwitchConfiguredPage(BOOT_UPGRADE_PAGE_ADDR);
    OtaSendHandshake();

    while(idle_ms < BOOT_UPGRADE_IDLE_TIMEOUT_MS || OtaApplyResult == 3U)
    {
        UartReadFrame(&Uart5);
        OtaTask();
        if(OtaAbort) break;

        if(OtaDownloadCompleteFlag != 0U)
        {
        
            {
                uint8_t result_buf[6] = {0xABU, 0xCDU, 0U, 2U, 8U, 0U};
                OtaApplyResult = OtaActionFromDownload() ? 2U : 3U;
                result_buf[5] = OtaApplyResult;
                OtaSendFrame(result_buf, 6U);
            }
            OtaDownloadCompleteFlag = 0U;
            if(OtaApplyResult != 2U) {
                /* Keep recovery available; do not report completion or reset after a failed write. */
                idle_ms = 0UL;
                continue;
            }
        
            BootWriteProgress(100U);
            BootSwitchConfiguredPage(BOOT_FINISH_PAGE_ADDR);
            DBG_LOG_LINE("[BOOT] wait R11 load command for automatic restart");
            while(BootWaitLoadCommand(0UL) == 0U) {}
            TimerStop();
            Uart5Stop();
            DBG_LOG_LINE("[BOOT] soft reset");
            SoftReset();
            while(1)
            {
            }
            return;
        }

        if(OtaFrameActivityFlag != 0U)
        {
            OtaFrameActivityFlag = 0U;
            idle_ms = 0UL;
        }
        else
        {
            delay_ms(1U);
            idle_ms++;
            if(OtaStatus.total_num == 0U && ++ready_ms >= 1000U) {
                OtaSendHandshake();
                ready_ms = 0U;
            }
        }
    }

    OtaFinishPendingWrite();
    DBG_LOG_LINE("[BOOT] upgrade idle timeout");
    TimerStop();
    Uart5Stop();
    BootClearControl();
}
