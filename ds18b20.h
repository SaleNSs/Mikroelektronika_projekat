#include <Arduino.h>
#include <esp_system.h>
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"

#ifndef DS18B20_H_  
#define DS18B20_H_

// OneWire commands
#define GET_TEMP 0x44	    
#define SKIP_ROM 0xCC	   
#define SELECT_DEVICE 0x55
#define COPY_SCRATCH 0x48
#define READ_SCRATCH 0xBE
#define WRITE_SCRATCH 0x4E
#define RECALL_SCRATCH 0xB8
#define READ_POWER_SUPPLY 0xB4
#define ALARM_SEARCH 0xEC

// Scratchpad locations
#define TEMP_LSB        0u
#define TEMP_MSB        1u
#define HIGH_ALARM_TEMP 2u
#define LOW_ALARM_TEMP  3u
#define CONFIGURATION   4u
#define INTERNAL_BYT    5u
#define COUNT_REMAIN    6u
#define COUNT_PER_C     7u
#define SCRATCHPAD_CRC  8u

// DSROM FIELDS
#define DS_ROM_FAMILY   0u
#define DS_ROM_CRC      7u

// Device resolution
#define TEMP_9_BIT  0x1F    //  9 bit
#define TEMP_10_BIT 0x3F    // 10 bit
#define TEMP_11_BIT 0x5F    // 11 bit
#define TEMP_12_BIT 0x7F    // 12 bit

#define NORMAL_SEARCH       0xF0
#define CONDITIONAL_SEARCH  0xEC

#define DEVICE_DISCONNECTED_C   -127
#define DEVICE_DISCONNECTED_F   -196.6
#define DEVICE_DISCONNECTED_RAW -7040
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define pgm_readByte(addr)   (*(const uint8_t *)(addr))

#define NUMBER_OF_CONNECTED_DEVICES 4U

typedef uint8_t DeviceAddress[8];
typedef uint8_t ScratchPad[9];


// CRC using polynomial X^8 + X^5 + X^4 + X^0
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};


#ifdef __cplusplus
    extern "C" {
#endif

#define noInterrupts() ets_intr_lock()
#define interrupts() ets_intr_unlock()

void ds18b20_init(gpio_num_t GPIO);

void ds18b20_write(uint8_t bit);
uint8_t ds18b20_read(void);
void ds18b20_writeByte(uint8_t data);
uint8_t ds18b20_readByte(void);

bool ds18b20_reset(void);

bool ds18b20_setResolution(const DeviceAddress sensorAddresses[], uint8_t numAddresses, uint8_t newResolution);
bool ds18b20_isConnected(const DeviceAddress *deviceAddress, uint8_t *scratchPad);
void ds18b20_writeScratchPad(const DeviceAddress *deviceAddress, const uint8_t *scratchPad);
bool ds18b20_readScratchPad(const DeviceAddress *deviceAddress, uint8_t *scratchPad);
void ds18b20_select(const DeviceAddress *address);
uint8_t ds18b20_crc8(const uint8_t *addr, uint8_t len);
bool ds18b20_isAllZeros(const uint8_t * const scratchPad);
bool isConversionComplete();
uint16_t microsToWaitForConversion();

void ds18b20_requestTemperatures();
float ds18b20_getTempC(const DeviceAddress *deviceAddress);
int16_t calculateTemperature(const DeviceAddress *deviceAddress, uint8_t* scratchPad);
void resetSearch(void);
bool searchForSensors(uint8_t *newAddr, bool search_mode);
void getTempAddresses(DeviceAddress *sensorAddresses);


#ifdef __cplusplus
    }
#endif

#endif