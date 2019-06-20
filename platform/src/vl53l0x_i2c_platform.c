#include "nrf_drv_twi.h"
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_platform.h"
#include "nrf_log.h"
#include "app_util_platform.h"

#define TWI_SCL_M                9   //!< Master SCL pin
#define TWI_SDA_M                8   //!< Master SDA pin

/**
 * @brief TWI master instance
 *
 * Instance of TWI master driver that would be used for communication with VL53L0X.
 */
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(0);

/**
 * @brief  Initialise platform comms.
 *
 * @param  comms_type      - selects between I2C and SPI
 * @param  comms_speed_khz - unsigned short containing the I2C speed in kHz
 *
 * @return status - status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_comms_initialise(uint8_t  comms_type, uint16_t comms_speed_khz){

	if(comms_type == SPI){
		NRF_LOG_ERROR("SPI not supported. Use I2C.\r\n");
		return 1;
	} else if(comms_type != I2C){
		NRF_LOG_ERROR("Invalid communication protocol with VL53L0X. Use I2C.\r\n");
		return 1;
	}

	uint32_t nrf_speed;

	if(comms_speed_khz == 400){
		nrf_speed = NRF_TWI_FREQ_400K;
	} else if(comms_speed_khz == 250){
		nrf_speed = NRF_TWI_FREQ_250K;
	} else if(comms_speed_khz == 100){
		nrf_speed = NRF_TWI_FREQ_100K;
	} else {
		NRF_LOG_ERROR("Invalid TWI comms speed.");
		return 1;
	}

	ret_code_t ret;
	const nrf_drv_twi_config_t config =
	{
	   .scl                = TWI_SCL_M,
	   .sda                = TWI_SDA_M,
	   .frequency          = nrf_speed,
	   .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
	   .clear_bus_init     = false
	};

    ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

	if (NRF_SUCCESS == ret)
	{
		nrf_drv_twi_enable(&m_twi_master);
    		NRF_LOG_DEBUG("TWI init successful\r\n");
	} else {
		NRF_LOG_ERROR("TWI init failed\r\n");
	}

	return ret;
};

/**
 * @brief  Close platform comms.
 *
 * @return status - status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_comms_close(void){
	nrf_drv_twi_disable(&m_twi_master);
	return 0;
}

/**
 * @brief Writes the supplied byte buffer to the device
 *
 * Wrapper for SystemVerilog Write Multi task
 *
 * @code
 *
 * Example:
 *
 * uint8_t *spad_enables;
 *
 * int status = VL53L0X_write_multi(RET_SPAD_EN_0, spad_enables, 36);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  pdata - pointer to uint8_t buffer containing the data to be written
 * @param  count - number of bytes in the supplied byte buffer
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count)
{
	ret_code_t ret;

	/* device supports only limited number of bytes written in sequence */
	if (count > (VL53L0X_MAX_STRING_LENGTH_PLT))
	{
		return NRF_ERROR_INVALID_LENGTH;
	}

	/* All written data has to be in the same page */
	if ((address / (VL53L0X_MAX_STRING_LENGTH_PLT)) != ((index + count - 1) / (VL53L0X_MAX_STRING_LENGTH_PLT)))
	{
		return NRF_ERROR_INVALID_ADDR;
	}

	do{
		uint8_t buffer[1 + VL53L0X_MAX_STRING_LENGTH_PLT]; /* index + data */
		buffer[0] = (uint8_t)index;
		memcpy(buffer + 1, pdata, count);
		ret = nrf_drv_twi_tx(&m_twi_master, address >> 1, buffer, count + 1, false);
	} while (0);
	return ret;
}

/**
 * @brief  Reads the requested number of bytes from the device
 *
 * Wrapper for SystemVerilog Read Multi task
 *
 * @code
 *
 * Example:
 *
 * uint8_t buffer[COMMS_BUFFER_SIZE];
 *
 * int status = status  = VL53L0X_read_multi(DEVICE_ID, buffer, 2)
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  pdata - pointer to the uint8_t buffer to store read data
 * @param  count - number of uint8_t's to read
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_read_multi(uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count)
{
    ret_code_t ret;

    if (count > (VL53L0X_MAX_STRING_LENGTH_PLT))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    do
    {
       ret = nrf_drv_twi_tx(&m_twi_master, address >> 1, &index, 1, true);
       if (NRF_SUCCESS != ret)
       {
    	   return 1;
       }
       ret = nrf_drv_twi_rx(&m_twi_master, address >> 1, pdata, count);
    }while (0);
    return ret;
}

/**
 * @brief  Writes a single byte to the device
 *
 * Wrapper for SystemVerilog Write Byte task
 *
 * @code
 *
 * Example:
 *
 * uint8_t page_number = MAIN_SELECT_PAGE;
 *
 * int status = VL53L0X_write_byte(PAGE_SELECT, page_number);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uint8_t data value to write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */


int32_t VL53L0X_write_byte(uint8_t address,  uint8_t index, uint8_t data)
{
    ret_code_t ret;

    do
    {
        uint8_t buffer[2]; /* Addr + data */
        buffer[0] = index;
        buffer[1] = data;
        ret = nrf_drv_twi_tx(&m_twi_master, address >> 1, buffer, 2, false);
    }while (0);
    return ret;

}

/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device (first byte written is the MS byte).
 * Uses SystemVerilog Write Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint16_t nvm_ctrl_pulse_width = 0x0004;
 *
 * int status = VL53L0X_write_word(NVM_CTRL__PULSE_WIDTH_MSB, nvm_ctrl_pulse_width);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uin16_t data value write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data)
{
    ret_code_t ret;

    do
    {
        uint8_t buffer[3]; /* Addr + data */
        buffer[0] = index;
        buffer[1] = data >> 8;
        buffer[2] = data;
        ret = nrf_drv_twi_tx(&m_twi_master, address >> 1, buffer, 2, false);
    }while (0);

    return ret;

}

/**
 * @brief  Writes a single dword (32-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device (first byte written is the MS byte).
 * Uses SystemVerilog Write Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint32_t nvm_data = 0x0004;
 *
 * int status = VL53L0X_write_dword(NVM_CTRL__DATAIN_MMM, nvm_data);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uint32_t data value to write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t  data)
{
    ret_code_t ret;

    do
    {
        uint8_t buffer[5]; /* Addr + data */
        buffer[0] = index;
        buffer[1] = data >> 24;
        buffer[2] = data >> 16;
        buffer[3] = data >> 8;
        buffer[4] = data;
        ret = nrf_drv_twi_tx(&m_twi_master, address >> 1, buffer, 2, false);
    }while (0);

    return ret;
}

/**
 * @brief  Reads a single byte from the device
 *
 * Uses SystemVerilog Read Byte task.
 *
 * @code
 *
 * Example:
 *
 * uint8_t device_status = 0;
 *
 * int status = VL53L0X_read_byte(STATUS, &device_status);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index  - uint8_t register index value
 * @param  pdata  - pointer to uint8_t data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata)
{
    ret_code_t ret;

    do
    {
       ret = nrf_drv_twi_tx(&m_twi_master, address >> 1, &index, 1, true);
       if (NRF_SUCCESS != ret)
       {
    	   return 1;
       }
       ret = nrf_drv_twi_rx(&m_twi_master, address >> 1, pdata, 1);
       if (NRF_SUCCESS != ret)
       {
    	   return 1;
       }
    }while (0);
    return ret;
}

/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 * Uses SystemVerilog Read Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint16_t timeout = 0;
 *
 * int status = VL53L0X_read_word(TIMEOUT_OVERALL_PERIODS_MSB, &timeout);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index  - uint8_t register index value
 * @param  pdata  - pointer to uint16_t data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata)
{
    ret_code_t ret;

    do
    {
    	uint8_t buffer[2];
    	ret = nrf_drv_twi_tx(&m_twi_master, address >> 1, &index, 1, true);
    	if (NRF_SUCCESS != ret)
		{
		    return 1;
	    }
	    ret = nrf_drv_twi_rx(&m_twi_master, address >> 1, buffer, 2);
	    *pdata = (buffer[0] << 8) + buffer[1];
    }while (0);
    return ret;
}

/**
 * @brief  Reads a single dword (32-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 * Uses SystemVerilog Read Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint32_t range_1 = 0;
 *
 * int status = VL53L0X_read_dword(RANGE_1_MMM, &range_1);
 *
 * @endcode
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  pdata - pointer to uint32_t data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    ret_code_t ret;

    do
    {
    	uint8_t buffer[4];
    	ret = nrf_drv_twi_tx(&m_twi_master, address >> 1, &index, 1, true);
    	if (NRF_SUCCESS != ret)
		{
		    return 1;
	    }
	    ret = nrf_drv_twi_rx(&m_twi_master, address >> 1, buffer, 4);
	    *pdata = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
    }while (0);
    return ret;
}


