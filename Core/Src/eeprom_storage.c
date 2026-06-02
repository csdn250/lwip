#include "eeprom_storage.h"
#include "i2c.h"
#include "app_log.h"
#include "SEGGER_RTT.h"

#define EEPROM_STORAGE_DEV_ADDR (0x50U << 1)
#define EEPROM_STORAGE_SIZE_BYTES (8U * 1024U)
#define EEPROM_STORAGE_PAGE_SIZE 32U
#define EEPROM_STORAGE_MEM_ADDR_SIZE I2C_MEMADD_SIZE_16BIT
#define EEPROM_STORAGE_WRITE_TIMEOUT 100U
#define EEPROM_STORAGE_READY_TRIALS 20U

static uint8_t eeprom_storage_is_range_valid(uint32_t address,
                                             uint16_t len)
{
    if (0U == len)
    {
        return 1U;
    }

    if (EEPROM_STORAGE_SIZE_BYTES <= address)
    {
        return 0U;
    }

    if ((EEPROM_STORAGE_SIZE_BYTES - address) < len)
    {
        return 0U;
    }

    return 1U;
}

void eeprom_storage_init(void)
{
    if (0U != eeprom_storage_is_ready())
    {
        SEGGER_RTT_WriteString(0, "eeprom ready\r\n");
    }
    else
    {
        SEGGER_RTT_WriteString(0, "eeprom not ready\r\n");
        app_log_key_event(APP_LOG_EVENT_I2C_EEPROM_ERROR,
                          "eeprom not ready");
    }
}

uint8_t eeprom_storage_is_ready(void)
{
    return (HAL_OK == (HAL_I2C_IsDeviceReady(&hi2c2,
                                             EEPROM_STORAGE_DEV_ADDR,
                                             EEPROM_STORAGE_READY_TRIALS,
                                             EEPROM_STORAGE_WRITE_TIMEOUT)))
               ? 1U
               : 0U;
}

HAL_StatusTypeDef eeprom_storage_read(uint32_t address,
                                      uint8_t *data,
                                      uint16_t len)
{
    // 1.判断参数是否非法
    if ((NULL == data) && (len > 0U))
    {
        return HAL_ERROR;
    }

    // 2.判断地址范围是否越界
    if (0U == eeprom_storage_is_range_valid(address, len))
    {
        return HAL_ERROR;
    }

    // 3.判断长度
    if (0U == len)
    {
        return HAL_OK;
    }

    return HAL_I2C_Mem_Read(&hi2c2,
                            EEPROM_STORAGE_DEV_ADDR,
                            (uint16_t)address,
                            EEPROM_STORAGE_MEM_ADDR_SIZE,
                            data,
                            len,
                            EEPROM_STORAGE_WRITE_TIMEOUT);
}

HAL_StatusTypeDef eeprom_storage_write(uint32_t address,
                                       const uint8_t *data,
                                       uint16_t len)
{
    HAL_StatusTypeDef status;
    uint16_t write_len;
    uint16_t page_remain;

    // 1.判断参数是否非法
    if ((NULL == data) && (len > 0U))
    {
        return HAL_ERROR;
    }

    // 2.地址是否越界
    if (0U == eeprom_storage_is_range_valid(address, len))
    {
        return HAL_ERROR;
    }

    // 3.while循环写入数据
    //页剩余计算、地址推进、len 递减、每页写完等待 ready
    while (0U < len)
    {
        page_remain = (uint16_t)(EEPROM_STORAGE_PAGE_SIZE -
                                 (address % EEPROM_STORAGE_PAGE_SIZE));

        write_len = (len < page_remain) ? len : page_remain;

        status = HAL_I2C_Mem_Write(&hi2c2,
                                   EEPROM_STORAGE_DEV_ADDR,
                                   (uint16_t)address,
                                   EEPROM_STORAGE_MEM_ADDR_SIZE,
                                   (uint8_t *)data,
                                   write_len,
                                   EEPROM_STORAGE_WRITE_TIMEOUT);
        if (HAL_OK != status)
        {
            return status;
        }

        status = HAL_I2C_IsDeviceReady(&hi2c2,
                                       EEPROM_STORAGE_DEV_ADDR,
                                       EEPROM_STORAGE_READY_TRIALS,
                                       EEPROM_STORAGE_WRITE_TIMEOUT);

        if (HAL_OK != status)
        {
            return status;
        }

        address += write_len;
        data += write_len;
        len = (uint16_t)(len - write_len);
    }

    return HAL_OK;
}
