#include "sdk_config.h"
#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_sdm.h"

#include "sequence_comms.h"

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

VL53L0X_Dev_t MyDevice;
VL53L0X_Dev_t *pMyDevice = &MyDevice;

uint16_t latestRange = 0 ;

static void print_pal_error(VL53L0X_Error Status){
	if(Status != 0){
		char buf[VL53L0X_MAX_STRING_LENGTH];
		VL53L0X_GetPalErrorString(Status, buf);
		NRF_LOG_DEBUG("API Status: %i : %s\n", (uint32_t)Status, *buf);
	}
}

static bool isMeasurementReady(void){
	uint8_t deviceReady;
	VL53L0X_GetMeasurementDataReady(pMyDevice, &deviceReady);
	return (bool)deviceReady;
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

//    // Initialize Comms
    MyDevice.I2cDevAddr      =  0x52;
    MyDevice.comms_type      =  1;
    MyDevice.comms_speed_khz =  400;

    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    uint8_t deviceRevisionMajor;
    uint8_t deviceRevisionMinor;

    uint8_t vhvCalibrationValue;
    uint8_t phaseCalibrationValue;

    err_code = VL53L0X_comms_initialise(I2C, pMyDevice->comms_speed_khz);

		if(VL53L0X_ERROR_NONE == err_code){

			err_code = VL53L0X_DataInit(pMyDevice);
			if(VL53L0X_ERROR_NONE != err_code) NRF_LOG_ERROR("DataInit: %d\r\n", err_code);

			uint16_t osc_calibrate_val=0;
			err_code = VL53L0X_RdWord(&MyDevice, VL53L0X_REG_OSC_CALIBRATE_VAL,&osc_calibrate_val);
			NRF_LOG_DEBUG("%i\n",osc_calibrate_val);

			err_code = VL53L0X_StaticInit(pMyDevice);
			if(VL53L0X_ERROR_NONE != err_code) NRF_LOG_ERROR("StaticInit: %d\r\n", err_code);

			uint32_t refSpadCount;
			uint8_t isApertureSpads;

			VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads);
			if(VL53L0X_ERROR_NONE != err_code) {
				NRF_LOG_ERROR("SpadCal: %d\r\n", err_code)
			} else {
				NRF_LOG_DEBUG("refSpadCount: %d\r\nisApertureSpads: %d\r\n", refSpadCount,isApertureSpads)
			}

			err_code = VL53L0X_PerformRefCalibration(pMyDevice, &vhvCalibrationValue, &phaseCalibrationValue);
			print_pal_error(err_code);
			NRF_LOG_DEBUG("Calibration Values:\r\n VHV:%d phaseCal:%d\r\n", vhvCalibrationValue, phaseCalibrationValue);

			err_code = VL53L0X_GetVersion(pVersion);
			print_pal_error(err_code);
			NRF_LOG_INFO("VL53L0X API Version: %d.%d.%d (revision %d)\r\n", pVersion->major, pVersion->minor ,pVersion->build, pVersion->revision);

			err_code = VL53L0X_GetProductRevision(pMyDevice, &deviceRevisionMajor, &deviceRevisionMinor);

			if(VL53L0X_ERROR_NONE == err_code){
				NRF_LOG_INFO("VL53L0X product revision: %d.%d\r\n", deviceRevisionMajor, deviceRevisionMinor);
			} else {
				NRF_LOG_ERROR("Version Retrieval Failed.  ");
				print_pal_error(err_code);
			}

			VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);

		if (err_code == VL53L0X_ERROR_NONE) {
			err_code = VL53L0X_SetLimitCheckValue(pMyDevice,
					VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
					(FixPoint1616_t) (0.25 * 65536));
		}

		if (err_code == VL53L0X_ERROR_NONE) {
			err_code = VL53L0X_SetLimitCheckValue(pMyDevice,
					VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
					(FixPoint1616_t) (18 * 65536));
		}

		if (err_code == VL53L0X_ERROR_NONE) {
			err_code = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
					200000);
		}

    } else {
    	NRF_LOG_ERROR("Data Init Failed\r\n");
    }

	VL53L0X_RangingMeasurementData_t rangingData;

    // Enter main loop.
    for (;; )
    {

    	VL53L0X_StartMeasurement(pMyDevice);

    	do{
	    // TODO do this in a more power efficient way
    	}while(!isMeasurementReady());

    	// TODO HANDLE THESE ERRORS

    	VL53L0X_GetRangingMeasurementData(pMyDevice, &rangingData);
    	VL53L0X_ClearInterruptMask(pMyDevice, 0);

    	NRF_LOG_DEBUG("Range in MM: %d\r\n", rangingData.RangeMilliMeter);
    }
}
