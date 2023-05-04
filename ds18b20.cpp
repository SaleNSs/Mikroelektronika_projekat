#include "ds18b20.h"

gpio_num_t DS_GPIO;
uint8_t initSuccessfull = 0u;
uint8_t bitResolution = 12u;
uint8_t devices = 0u;

DeviceAddress ROM_NO;
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
bool LastDeviceFlag;


/**
 * The function ds18b20_write() sends one bit to bus. There
 * is two write modes, "Write 1" and "Write 0"
 * 
 * Arguments:
 *   bit      Bit to be transmitted to the bus (uint8_t) 
 * 
 * Return:
 * void
 * 
 * */ 
void ds18b20_write(uint8_t bit)
{
	if (bit & 1u) // "Write 1"
	{
		gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
		noInterrupts();
		gpio_set_level(DS_GPIO, 0u);
		ets_delay_us(6u);				                // Hold OUTPUT pin LOW for 6us			  
		gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);   // After which change it to INPUT
		ets_delay_us(64u);                              // And wait for 64us
		interrupts();
	}
	else // "Write 0"
	{
		gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
		noInterrupts();
		gpio_set_level(DS_GPIO, 0u);
		ets_delay_us(60u);                              // Hold OUTPUT pin LOW for 60us
		gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);   // After which change it to INPUT
		ets_delay_us(10u);                              // And wait for 10us
		interrupts();
	}
}


/**
 * The function ds18b20_read() reads one bit from the bus
 * 
 * Arguments:
 * none             
 * 
 * Return:
 * uint8_t          a bit of data read from the bus
 * 
 * */ 
uint8_t ds18b20_read(void)
{
	uint8_t value = 0u;

	gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
	noInterrupts();
	gpio_set_level(DS_GPIO, 0u);
	ets_delay_us(5u);                               // Hold OUTPUT pin LOW for 6us
	gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);   // After which change it to INPUT
	ets_delay_us(8u);                               // And wait for 8us
	value = gpio_get_level(DS_GPIO);                // And read its state
	ets_delay_us(54u);
	interrupts();

	return (value);
}


/**
 * The function ds18b20_writeByte() sends one Byte on the bus
 * 
 * Arguments:
 * data           Byte of data to be send on the bus (uint8_t)              
 * 
 * Return:
 * void
 * 
 * */ 
void ds18b20_writeByte(uint8_t data)
{
	uint8_t i;
	uint8_t x;

	for (i = 0u; i < 8u; i++)
	{
		x = data >> i;
		x &= 0x01;
		ds18b20_write(x);
	}
	ets_delay_us(100u);
}


/**
 * The function ds18b20_readByte() reads one Byte from the bus
 * 
 * Arguments:
 * none
 * 
 * Return:
 * uint8_t             Byte read from the bus
 * 
 * */ 
uint8_t ds18b20_readByte(void)
{
	uint8_t i;
	uint8_t data = 0u;

	for (i = 0u; i < 8u; i++)
	{
		if (ds18b20_read())
		{
			data |= 0x01 << i;
		}
		ets_delay_us(15u);
	}

	return (data);
}


/**
 * The function ds18b20_reset() is used for sending reset pulse to the bus.
 * The reset pulse is used in init phase of communication between master and
 * slave of 1-Wire bus
 * 
 * Arguments:
 * none
 * 
 * Return:
 * bool             is reset pulse successful
 * 
 * */ 
bool ds18b20_reset(void)
{
	bool isResetSuccessful;

	gpio_set_direction(DS_GPIO, GPIO_MODE_OUTPUT);
	noInterrupts();
	gpio_set_level(DS_GPIO, 0u);
	ets_delay_us(480u);									 // Hold OUTPUT pin LOW for 480us
	gpio_set_level(DS_GPIO, 1u);						 // After which set it to HIGH
	gpio_set_direction(DS_GPIO, GPIO_MODE_INPUT);		 // And change it to INPUT
	ets_delay_us(70u);									 // Wait for 70us for the slave device(s) to respond with presence pulse
	isResetSuccessful = (gpio_get_level(DS_GPIO) == 0u); // which is logic 0 on the 1-Wire bus
	ets_delay_us(410u);									 // Reading presence pulse ends 480us (70 + 410) after transmit of the reset pulse
	interrupts();

	return isResetSuccessful;
}


/**
 * The function ds18b20_setResolution() is used for setting the resolution for all
 * sensors connected to the bus
 * 
 * Arguments:
 * tempSensorAddresses        array which contains all addressess of all sensors
 *                                  (const DeviceAddress)
 * numAddresses               number of sensors connected to the bus (uint8_t)
 * newResolution              new resolution to be set for all sensors (uint8_t)
 * 
 * Return:
 * void
 * 
 * */ 
bool ds18b20_setResolution(const DeviceAddress sensorAddresses[], uint8_t numAddresses, uint8_t newResolution)
{
	bool isSetResolutionRuccessful = false;
	uint8_t newValue = 0u;
	uint8_t i = 0u;

	newResolution = constrain(newResolution, 9u, 12u);

	ScratchPad scratchPad;
	for (i = 0u; i < numAddresses; i++)
	{
		if (ds18b20_isConnected((DeviceAddress *)sensorAddresses[i], scratchPad))
		{
			switch (newResolution)
			{
			case 12u:
				newValue = TEMP_12_BIT;
				break;
			case 11u:
				newValue = TEMP_11_BIT;
				break;
			case 10u:
				newValue = TEMP_10_BIT;
				break;
			case 9u:
			default:
				newValue = TEMP_9_BIT;
				break;
			}

			if (scratchPad[CONFIGURATION] != newValue)
			{
				scratchPad[CONFIGURATION] = newValue;
				ds18b20_writeScratchPad((DeviceAddress *)sensorAddresses[i], scratchPad);
			}

			isSetResolutionRuccessful = true;
		}
	}

	return isSetResolutionRuccessful;
}


/**
 * The function ds18b20_writeScratchPad() is used for writing to the scratchpad
 * memory of specific sensor based on input arguement deviceAddress
 * 
 * Arguments:
 * deviceAddress      pointer to pysical address of the sensor (const DeviceAddress *)
 * scratchPad         pointer to scratchpad memory array (const uint8_t *)
 * 
 * Return:
 * void
 * 
 * */ 
void ds18b20_writeScratchPad(const DeviceAddress *deviceAddress, const uint8_t *scratchPad)
{
	ds18b20_reset();
	ds18b20_select(deviceAddress);
	ds18b20_writeByte(WRITE_SCRATCH);
	ds18b20_writeByte(scratchPad[HIGH_ALARM_TEMP]);
	ds18b20_writeByte(scratchPad[LOW_ALARM_TEMP]);
	ds18b20_writeByte(scratchPad[CONFIGURATION]);
	ds18b20_reset();
}


/**
 * The function ds18b20_readScratchPad() is used for reading the scratchpad
 * memory of specific sensor based on input arguement deviceAddress
 * 
 * Arguments:
 * deviceAddress      pointer to pysical address of the sensor (const DeviceAddress *)
 * scratchPad         pointer to scratchpad memory array (uint8_t *)
 * 
 * Return:
 * bool               is reset after reading scratchpad memory successful
 * 
 * */ 
bool ds18b20_readScratchPad(const DeviceAddress *deviceAddress, uint8_t *scratchPad)
{
	uint8_t i;

	bool isResetSuccessful = ds18b20_reset();
	if (isResetSuccessful == 0)
	{
		return false;
	}

	ds18b20_select(deviceAddress);
	ds18b20_writeByte(READ_SCRATCH);

	// Read all registers
	for (i = 0u; i < 9u; i++)
	{
		scratchPad[i] = ds18b20_readByte();
	}

	isResetSuccessful = ds18b20_reset();

	return (isResetSuccessful == 1u);
}


/**
 * The function ds18b20_select() is used for selecting specific sensor
 * based on its address
 * 
 * Arguments:
 * [IN] address         pointer to pysical address of the sensor (const DeviceAddress *)
 * 
 * Return:
 * void
 * 
 * */ 
void ds18b20_select(const DeviceAddress *address)
{
	uint8_t i;

	ds18b20_writeByte(SELECT_DEVICE);
	for (i = 0u; i < 8u; i++)
	{
		ds18b20_writeByte(((uint8_t *)address)[i]);
	}
}


/**
 * The function ds18b20_requestTemperatures() reads all temperatures data from
 * all sensors connected to the bus
 * 
 * Arguments:
 * none
 * 
 * Return:
 * void
 * 
 * */ 
void ds18b20_requestTemperatures()
{
	ds18b20_reset();

	ds18b20_writeByte(SKIP_ROM);
	ds18b20_writeByte(GET_TEMP);
	unsigned long start = micros();

	while (!isConversionComplete() && (micros() - start < microsToWaitForConversion()))
	{
		vPortYield();
	}
}


/**
 * The function isConversionComplete() checks if the reading of one bit from bus
 * is successful
 * 
 * Arguments:
 * none
 * 
 * Return:
 * bool             is bit reading successful
 * 
 * */ 
bool isConversionComplete()
{
	uint8_t isCompleted = ds18b20_read();

	return (isCompleted == 1u);
}


/**
 * The function microsToWaitForConversion() returns the number of microseconds
 * to wait for conversions, based on global variable sensor resolution bitResolution
 * 
 * Arguments:
 * none
 * 
 * Return:
 * uint16_t             number of microseconds
 * 
 * */
uint16_t microsToWaitForConversion()
{
	switch (bitResolution)
	{
	case 9u:
		return 94u;
	case 10u:
		return 188u;
	case 11u:
		return 375u;
	default:
		return 750u;
	}
}


/**
 * The function ds18b20_isConnected() checks if the sensor with specific address is
 * connected to the bus
 * 
 * Arguments:
 * deviceAddress      A pointer to specific device phycical address (const DeviceAddress *)
 * scratchPad         A pointer to the scratchpad memory array (uint8_t *)
 * 
 * Return:
 * bool               Is device connected
 * 
 * */
bool ds18b20_isConnected(const DeviceAddress *deviceAddress, uint8_t *scratchPad)
{
	bool isReadScratchpadSuccessful = ds18b20_readScratchPad(deviceAddress, scratchPad);

	return isReadScratchpadSuccessful && !ds18b20_isAllZeros(scratchPad) && 
            (ds18b20_crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);
}


/**
 * The function ds18b20_crc8() calculates the crc based of scratchpad 
 * memory
 * 
 * Arguments:
 * addr               A pointer to scratchpad memory array (const uint8_t *)
 * len                Length of the scratchpad memory array (uint8_t)
 * 
 * Return:
 * uint8_t            Calculated CRC
 * 
 * */
uint8_t ds18b20_crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0u;

	while (len--)
	{
		crc = *addr++ ^ crc; // just re-using crc as intermediate
		crc = pgm_readByte(dscrc2x16_table + (crc & 0x0f)) ^ pgm_readByte(dscrc2x16_table + 16u + ((crc >> 4u) & 0x0f));
	}

	return crc;
}


/**
 * The function ds18b20_isAllZeros() checks if the scratchpad memory array 
 * contains all zero values
 * 
 * Arguments:
 * scratchPad        A pointer to scratchpad memory array (const uint8_t *const)
 * 
 * Return:
 * bool              Is all zero values
 * 
 * */
bool ds18b20_isAllZeros(const uint8_t *const scratchPad)
{
	uint8_t i;

	for (i = 0u; i < 9u; i++)
	{
		if (scratchPad[i] != 0u)
		{
			return false;
		}
	}

	return true;
}


/**
 * The function ds18b20_getTempC() reads temperature data from the sensor's scratchpad
 * memory based on deviceAddress argument
 * 
 * Arguments:
 * deviceAddress      A pointer to specific device phycical address (const DeviceAddress *)
 * 
 * Return:
 * bool               Is all zero values
 * 
 * */
float ds18b20_getTempC(const DeviceAddress *deviceAddress)
{
	ScratchPad scratchPad;
	if (ds18b20_isConnected(deviceAddress, scratchPad))
	{
		int16_t rawTemp = calculateTemperature(deviceAddress, scratchPad);
		if (rawTemp <= DEVICE_DISCONNECTED_RAW)
		{
			return DEVICE_DISCONNECTED_F;
		}

		return (float)rawTemp / 128.0f;
	}

	return DEVICE_DISCONNECTED_F;
}


/**
 * The function calculateTemperature() calculate temperature from the sensor based on device
 * address and scratchpad memory.
 * 
 * Arguments:
 * deviceAddress      A pointer to specific device phycical address (const DeviceAddress *)
 * scratchPad         A pointer to scratchpad memory array (uint8_t *)
 * 
 * Return:
 * int16_t            Raw temperature data from the sensor
 * 
 * */
int16_t calculateTemperature(const DeviceAddress *deviceAddress, uint8_t *scratchPad)
{
	int16_t fpTemperature = (((int16_t)scratchPad[TEMP_MSB]) << 11u) | 
                            (((int16_t)scratchPad[TEMP_LSB]) << 3u);

	return fpTemperature;
}


/**
 * The function ds18b20_init() is used to set the 1-Wire bus gpio pin, global
 * gpio variable and global flag for initialization success
 * 
 * Arguments:
 * deviceAddress      A GPIO pin number (gpio_num_t)
 * 
 * Return:
 * void
 * 
 * */
void ds18b20_init(gpio_num_t GPIO)
{
	DS_GPIO = GPIO;				   // set the global variable
	gpio_pad_select_gpio(DS_GPIO); // This is from esp library
	initSuccessfull = 1u;		   // set the initialization sequence as successful
}


/**
 * The function resetSearch() is used to reset all variables used
 * in searchSensors algorithm
 * 
 * Arguments:
 * none
 * 
 * Return:
 * void
 * 
 * */
void resetSearch(void)
{
	int i;
	devices = 0u;

	// reset the search state
	LastDiscrepancy = 0u;
	LastDeviceFlag = false;
	LastFamilyDiscrepancy = 0u;
	
	for (i = 7; i >= 0; i--)
	{
		ROM_NO[i] = 0u;
	}
}


/**
 * The function searchForSensors() is to detect all sensors connected on the
 * 1-Wire bus. There are two search modes, normal mode and conditional mode.
 * 
 * Arguments:
 * newAddr        A pointer to detected address (uint8_t *)
 * search_mode    A flag for setting search mode: 
 * 							true - normal mode
 * 							false - conditional mode
 * 
 * Return:
 * bool			  true  - device found, ROM number in ROM_NO buffer
 *       		  false - device not found, end of search
 * 
 * */
bool searchForSensors(uint8_t *newAddr, bool search_mode)
{
	uint8_t id_bit_number;
	uint8_t last_zero;
	uint8_t rom_byte_number;
	bool search_result;
	uint8_t id_bit;
	uint8_t cmp_id_bit;

	uint8_t rom_byte_mask;
	uint8_t search_direction;

	uint8_t i;

	// initialize for search
	id_bit_number = 1u;
	last_zero = 0u;
	rom_byte_number = 0u;
	rom_byte_mask = 1u;
	search_result = false;

	if (!LastDeviceFlag)
	{
		// 1-Wire reset
		if (!ds18b20_reset())
		{
			LastDiscrepancy = 0u;
			LastDeviceFlag = false;
			LastFamilyDiscrepancy = 0u;
			return false;
		}

		// issue the search command
		if (search_mode == true)
		{
			ds18b20_writeByte(NORMAL_SEARCH);
		}
		else
		{
			ds18b20_writeByte(CONDITIONAL_SEARCH);
		}

		// search
		do
		{
			// read a bit and its complement
			id_bit = ds18b20_read();	 // read one bit
			cmp_id_bit = ds18b20_read(); // read the complement of bit

			if ((id_bit == 1) && (cmp_id_bit == 1))
			{
				break;
			}
			else
			{
				if (id_bit != cmp_id_bit)
				{
					search_direction = id_bit;
				}
				else
				{
					if (id_bit_number < LastDiscrepancy)
					{
						search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
					}
					else
					{
						search_direction = (id_bit_number == LastDiscrepancy);
					}

					if (search_direction == 0)
					{
						last_zero = id_bit_number;

						if (last_zero < 9)
						{
							LastFamilyDiscrepancy = last_zero;
						}
					}
				}

				if (search_direction == 1)
				{
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				}
				else
				{
					ROM_NO[rom_byte_number] &= ~rom_byte_mask;
				}

				ds18b20_write(search_direction);

				id_bit_number++;
				rom_byte_mask <<= 1u;

				if (rom_byte_mask == 0u)
				{
					rom_byte_number++;
					rom_byte_mask = 1u;
				}
			}
		} 
		while (rom_byte_number < 8u); // loop until through all ROM bytes 0-7

		if (!(id_bit_number < 65u))
		{
			LastDiscrepancy = last_zero;

			if (LastDiscrepancy == 0u)
			{
				LastDeviceFlag = true;
			}
			search_result = true;
		}
	}

	if (!search_result || !ROM_NO[0])
	{
		devices = 0u;
		LastDiscrepancy = 0u;
		LastDeviceFlag = false;
		LastFamilyDiscrepancy = 0u;
		search_result = false;
	}
	else
	{
		for (i = 0u; i < 8u; i++)
		{
			newAddr[i] = ROM_NO[i];
		}
		devices++;
	}

	return search_result;
}


/**
 * The function getTempAddresses() is used to read all physical adresses
 * of all sensors connected to the bus
 * 
 * Arguments:
 * tempSensorAddresses        A pointer to an array of all device phycial
 *                                  addresses (DeviceAddress *)
 * 
 * Return:
 * void
 * 
 * */
void getTempAddresses(DeviceAddress *sensorAddresses)
{
	uint8_t sensorsFound = 0u;

	resetSearch();
	ets_delay_us(10);

	// search all addresses of connected devices
	while (searchForSensors(sensorAddresses[sensorsFound], true))
	{
		// DEBUG/INFO MESSAGE
		sensorsFound++;
		if (sensorsFound == NUMBER_OF_CONNECTED_DEVICES)
		{
			break;
		}
	}

	// if all addresses are not found
	while (sensorsFound != NUMBER_OF_CONNECTED_DEVICES)
	{
		sensorsFound = 0u;
		
        // DEBUG/INFO MESSAGE
		resetSearch();
		while (searchForSensors(sensorAddresses[sensorsFound], true))
		{
			sensorsFound++;
			if (sensorsFound == NUMBER_OF_CONNECTED_DEVICES)
			{
				break;
			}
		}
	}
}
