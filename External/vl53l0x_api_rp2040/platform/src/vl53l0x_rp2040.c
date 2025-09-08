
#include "stdio.h"   
#include "pico/stdlib.h"

#include "vl53l0x_rp2040.h"
#include "string.h"
#include "vl53l0x_api.h"

#define STATUS_OK              0x00
#define STATUS_FAIL            0x01

i2c_inst_t* vl53l0x_i2c_port = &i2c1_inst;
uint vl53l0x_i2c_sda = 14; // PICO_DEFAULT_I2C_SDA_PIN;
uint vl53l0x_i2c_scl = 15; // PICO_DEFAULT_I2C_SCL_PIN;

VL53L0X_Error VL53L0X_dev_i2c_default_initialise(VL53L0X_Dev_t *pDevice, uint32_t RangeProfile) {
    VL53L0X_Error Status;
    i2c_init(vl53l0x_i2c_port, 400 * 1000);
    gpio_set_function(vl53l0x_i2c_sda, GPIO_FUNC_I2C);
    gpio_set_function(vl53l0x_i2c_scl, GPIO_FUNC_I2C);
    gpio_pull_up(vl53l0x_i2c_sda);
    gpio_pull_up(vl53l0x_i2c_scl);
    Status = VL53L0X_device_initizlise(pDevice, RangeProfile);

    return Status;
}

VL53L0X_Error VL53L0X_dev_i2c_initialise(VL53L0X_Dev_t *pDevice,
    i2c_inst_t* i2c_port, uint sda, uint scl, uint16_t i2c_speed_k, uint32_t RangeProfile)
{
    VL53L0X_Error Status;
    vl53l0x_i2c_port = i2c_port;
    vl53l0x_i2c_sda = sda;
    vl53l0x_i2c_scl = scl;

    i2c_init(vl53l0x_i2c_port, i2c_speed_k * 1000);
    gpio_set_function(vl53l0x_i2c_sda, GPIO_FUNC_I2C);
    gpio_set_function(vl53l0x_i2c_scl, GPIO_FUNC_I2C);
    gpio_pull_up(vl53l0x_i2c_sda);
    gpio_pull_up(vl53l0x_i2c_scl);

    Status = VL53L0X_device_initizlise(pDevice, RangeProfile);
    return Status;
}

VL53L0X_Error VL53L0X_device_initizlise(VL53L0X_Dev_t *pDevice, uint32_t RangeProfile) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int i;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    Status = VL53L0X_DataInit(pDevice); 
    if(Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_StaticInit(pDevice); // Device Initialization
    if(Status != VL53L0X_ERROR_NONE) return Status;
   
    Status = VL53L0X_PerformRefCalibration(pDevice,
        	&VhvSettings, &PhaseCal); // Device Initialization
    if(Status != VL53L0X_ERROR_NONE) return Status;
    
    Status = VL53L0X_PerformRefSpadManagement(pDevice,
        		&refSpadCount, &isApertureSpads); // Device Initialization
    if(Status != VL53L0X_ERROR_NONE) return Status;
    
     Status = VL53L0X_SetLimitCheckEnable(pDevice,
        	VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if(Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_SetLimitCheckEnable(pDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if(Status != VL53L0X_ERROR_NONE) return Status;	
   
    Status = VL53L0X_SetLimitCheckValue(pDevice,
            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
            (FixPoint1616_t)(0.1*65536));
	if(Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_SetLimitCheckValue(pDevice,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
            (FixPoint1616_t)(60*65536));			
    if(Status != VL53L0X_ERROR_NONE) return Status;
    
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice,
        		RangeProfile);
	if(Status != VL53L0X_ERROR_NONE) return Status;
	   
    Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    if(Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_SetVcselPulsePeriod(pDevice, 
	        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    if(Status != VL53L0X_ERROR_NONE) return Status;

    return Status;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;
    uint8_t i2c_buff[count+1];
    i2c_buff[0] = index;
    memcpy(i2c_buff+1, pdata, count);
    if ( i2c_write_blocking(vl53l0x_i2c_port, address, i2c_buff, count+1, false) == PICO_ERROR_GENERIC) {
        status = STATUS_FAIL;
    }
    return status;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;

    int i2c_ret = i2c_write_blocking(vl53l0x_i2c_port, address, &index, 1, true);
    if (i2c_ret == PICO_ERROR_GENERIC) return STATUS_FAIL;
    i2c_ret = i2c_read_blocking(vl53l0x_i2c_port, address, pdata, count, false);
    if (i2c_ret == PICO_ERROR_GENERIC) return STATUS_FAIL;
    return status;
}


int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    int32_t status = STATUS_OK;

    status = VL53L0X_write_multi(address, index, &data, 1);

    return status;

}


int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    int32_t status = STATUS_OK;
    uint8_t  buffer[BYTES_PER_WORD];
    // Split 16-bit word into MS and LS uint8_t
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);
    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);
    return status;

}


int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    int32_t status = STATUS_OK;
    uint8_t  buffer[BYTES_PER_DWORD];
    // Split 32-bit word into MS ... LS bytes
    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);
    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);
    return status;
}


int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    int32_t status = STATUS_OK;
    int32_t cbyte_count = 1;
    status = VL53L0X_read_multi(address, index, pdata, cbyte_count);
    return status;
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    int32_t  status = STATUS_OK;
    	uint8_t  buffer[BYTES_PER_WORD];
    
    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD);

	*pdata = ((uint16_t)buffer[0] << 8) + (uint16_t)buffer[1];

    return status;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    int32_t status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_DWORD];
    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD);
    *pdata = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
    return status;
}


VL53L0X_Error VL53L0X_SingleRanging(VL53L0X_Dev_t *pDevice, uint16_t *MeasuredData) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    
    *MeasuredData=0; 
    Status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
    if(Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_PerformSingleRangingMeasurement(pDevice,
                &RangingMeasurementData);
    if (Status == VL53L0X_ERROR_NONE && RangingMeasurementData.RangeStatus == 0) {
        *MeasuredData = RangingMeasurementData.RangeMilliMeter;
    } else {
        Status = VL53L0X_ERROR_RANGE_ERROR;
    }
    /* for accuracy average several samples
    uint32_t ranging=0;
    uint32_t valid_count=0;
    int i;
    for(i=0; i<10; i++){
        Status = VL53L0X_PerformSingleRangingMeasurement(pDevice,
                &RangingMeasurementData);
        if (Status == VL53L0X_ERROR_NONE && RangingMeasurementData.RangeStatus == 0) {
            ranging += RangingMeasurementData.RangeMilliMeter;
            valid_count++;
        }
        
        if (Status != VL53L0X_ERROR_NONE) break;
    }
    
    if (valid_count == 0) {
        Status = VL53L0X_ERROR_RANGE_ERROR;
    } else {
        *MeasuredData = ranging/valid_count;
    }
    */ 
    return Status;
}

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_Dev_t *pDevice) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t dataReady=0;
    absolute_time_t timeout = make_timeout_time_ms(200);
    do { 
        Status = VL53L0X_GetMeasurementDataReady(pDevice, &dataReady);
        if ((dataReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
            break;
        }
    } while (absolute_time_diff_us(get_absolute_time(), timeout) > 0);
    if (!dataReady) Status = VL53L0X_ERROR_TIME_OUT;
    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_Dev_t *pDevice) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=1;
    absolute_time_t timeout = make_timeout_time_ms(200);
    do {
        Status = VL53L0X_GetStopCompletedStatus(pDevice, &StopCompleted);
        if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
            break;
        }
    } while (absolute_time_diff_us(get_absolute_time(), timeout) > 0);

    if (StopCompleted) {
         Status = VL53L0X_ERROR_TIME_OUT;
    }
    return Status;
}

VL53L0X_Error VL53L0X_ContinuousRanging(VL53L0X_Dev_t *pDevice, uint16_t *MeasuredData, uint16_t RangeCount, uint16_t *validCount){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;

    Status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); 
    if (Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_StartMeasurement(pDevice);
    if (Status != VL53L0X_ERROR_NONE) return Status;
    *validCount=0;
    uint16_t vCount=0;
    for (int i=0; i < RangeCount; i++) {

        Status = WaitMeasurementDataReady(pDevice);
        if (Status != VL53L0X_ERROR_NONE) break;

        Status = VL53L0X_GetRangingMeasurementData(pDevice, &RangingMeasurementData);
        if (Status == VL53L0X_ERROR_NONE ) { 
            if (RangingMeasurementData.RangeStatus == 0) {
                MeasuredData[vCount++] = RangingMeasurementData.RangeMilliMeter;
                //printf("valid:%d, m:%d \n",vCount, RangingMeasurementData.RangeMilliMeter);
            // Clear the interrupt
            }
            VL53L0X_ClearInterruptMask(pDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            VL53L0X_PollingDelay(pDevice);
        }
    }
    *validCount = vCount;

    Status = VL53L0X_StopMeasurement(pDevice);
    if (Status != VL53L0X_ERROR_NONE) return Status;

    Status = WaitStopCompleted(pDevice);
    if (Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_ClearInterruptMask(pDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return Status;
}


