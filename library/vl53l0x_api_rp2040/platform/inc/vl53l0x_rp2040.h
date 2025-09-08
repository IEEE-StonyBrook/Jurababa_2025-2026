
#ifndef _VL53L0X_RP2040_H_
#define _VL53L0X_RP2040_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "hardware/i2c.h"
#include "vl53l0x_platform.h"

#define    BYTES_PER_WORD        2
#define    BYTES_PER_DWORD       4

enum RANGE_PROFILE {
    VL53L0X_DEFAULT_MODE    =   30000,
    VL53L0X_HIGHT_ACCURACY  =   200000,
    VL53L0X_LONG_RANGE      =   33000,
    VL53L0X_HIGH_SPEED      =   20000
} ;

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count);
int32_t VL53L0X_read_multi(uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count);
int32_t VL53L0X_write_byte(uint8_t address,  uint8_t index, uint8_t   data);
int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data);
int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t  data);
int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata);
int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata);
int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata);

VL53L0X_Error VL53L0X_device_initizlise(VL53L0X_Dev_t *pDevice, uint32_t RangeProfile);

VL53L0X_Error VL53L0X_dev_i2c_default_initialise(VL53L0X_Dev_t *pDevice, uint32_t RangeProfile);
VL53L0X_Error VL53L0X_dev_i2c_initialise(VL53L0X_Dev_t *pDevice,
    i2c_inst_t* i2c_port, uint sda, uint scl, uint16_t i2c_speed_k, uint32_t RangeProfile);
void vl53l0x_print_device_info(VL53L0X_Dev_t *pDevice);

VL53L0X_Error VL53L0X_SingleRanging(VL53L0X_Dev_t *pDevice, uint16_t *MeasureData);
VL53L0X_Error VL53L0X_ContinuousRanging(VL53L0X_Dev_t *pDevice, uint16_t *MeasuredData, uint16_t RangeCount, uint16_t *validCount);


#ifdef __cplusplus
}
#endif

#endif //_VL53L0X_RP2040_H_

