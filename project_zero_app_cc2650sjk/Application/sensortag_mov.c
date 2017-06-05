/******************************************************************************

 @file  sensortag_mov.c

 @brief This file contains the Movement Processor sub-application. It uses the
        MPU-9250 Wake-on-movement feature to allow the
        MPU to turn off the gyroscope and magnetometer when no activity is
        detected.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************

 Copyright (c) 2015-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <xdc/runtime/Log.h>

#include "gatt.h"
#include "gattservapp.h"

#include "board.h"
#include "sensortag_mov.h"
#include "SensorMpu9250.h"
#include "SensorUtil.h"
#include "util.h"
#include "string.h"

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file

/*********************************************************************
 * MACROS
 */
#define ST_CFG_SENSOR_DISABLE           0x00
#define ST_CFG_SENSOR_IDLE  	        0x01
#define ST_CFG_SENSOR_ENABLE            0x02
#define ST_CFG_ERROR                    0xFF

/*********************************************************************
 * CONSTANTS and MACROS
 */
// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD     2

// Event flag for this sensor
#define SENSOR_EVT                ST_GYROSCOPE_SENSOR_EVT

// Movement task states
#define APP_STATE_ERROR           0xFF
#define APP_STATE_OFF             0
#define APP_STATE_IDLE            1
#define APP_STATE_ACTIVE          2

// Movement task configuration
#define MOVEMENT_INACT_TIMEOUT    10     // 10 seconds
#define GYR_SHAKE_THR             10.0
#define WOM_THR                   68

// Configuration bit-masks (bits 0-6 defined in sensor_mpu9250.h)
#define MOV_WOM_ENABLE            0x0080
#define MOV_MASK_WOM_THRESHOLD    0x3C00 // TBD
#define MOV_MASK_INACT_TIMEOUT    0xC000 // TBD

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

extern Semaphore_Struct semSensorAlert;
//extern unit8_t gInterruptGenValue;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
 static Clock_Struct gPeriodicClock;
 static uint8_t gSensorAlertInfo = 0;
 // Application state variables

// MPU config:
// bit 0-2:   accelerometer enable(z,y,x)
// bit 3-5:   gyroscope enable (z,y,x)
// bit 6:     magnetometer enable
// bit 7:     WOM enable
// bit 8-9:   accelerometer range (2,4,8,16)
static uint16_t gMPUconfig;

static uint8_t  gAppState;
static uint8_t 	gMovThreshold;
static uint8_t  gMPUintStatus;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SensorTagMov_clockHandler(UArg arg);
static void appStateSet(uint8_t newState);
void SensorTagMov_processInterrupt(void);
//static void SensorTagMov_processCharChangeEvt(uint8_t paramID);
/*********************************************************************
 * PROFILE CALLBACKS
 */
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void ss_gMPUconfig_setting(uint16_t val);

Clock_Struct *get_gPeriodicClock(void)
{
	return &gPeriodicClock;
}


uint8_t get_SensorAlertInfo()
{
	return gSensorAlertInfo;
}

/*********************************************************************
 * @fn      SensorTagMov_init
 *
 * @brief   Initialization function for the SensorTag movement sub-application
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_init(void)
{

    // Create continuous clock for internal periodic events.
    Util_constructClock(&gPeriodicClock, SensorTagMov_clockHandler, 1, SENSOR_DEFAULT_PERIOD, false, 0);
    gMPUconfig = MPU_AX_SEN | ( ACC_RANGE_8G << 8);

	if (SensorMpu9250_init())
	{

		ss_gMPUconfig_setting( ST_CFG_SENSOR_DISABLE | (ACC_RANGE_8G << 8) );
		appStateSet(APP_STATE_OFF);

        ss_gMPUconfig_setting( MPU_AX_SEN | ( ACC_RANGE_8G << 8) );
        appStateSet(APP_STATE_ACTIVE);

		SensorMpu9250_registerCallback(SensorTagMov_processInterrupt);
	}

	appStateSet(APP_STATE_IDLE);
    SensorMpu9250_registerCallback(SensorTagMov_processInterrupt);

}

/*********************************************************************
 * @fn      SensorTagMov_processSensorEvent
 *
 * @brief   SensorTag Movement sensor event processor.
 *
 * @param   none
 *
 * @return  none
 */
uint8_t SensorTagMov_processSensor(uint16_t *data)
{
	uint8_t ret =0;
	gMPUintStatus = SensorMpu9250_irqStatus();
	if(gAppState == APP_STATE_ACTIVE)
	{
		if (gMPUintStatus & MPU_DATA_READY)
		{
			SensorMpu9250_accRead(data);
			ret = 1;
		}
	}
	else if(gAppState == APP_STATE_IDLE)
	{
		if (gMPUintStatus & MPU_MOVEMENT)
		{
			gMPUconfig = MPU_AX_SEN | ( ACC_RANGE_8G << 8);
			appStateSet(APP_STATE_ACTIVE);
			DELAY_MS(1);
		}
	}
	return ret;
}


/*********************************************************************
 * @fn      SensorTagAcc_processSensorEvent
 *
 * @brief   SensorTag Movement sensor event processor.
 *
 * @param   none
 *
 * @return  none
 */
uint8_t SensorTagAcc_processSensor(uint16_t *AccData)
{
	uint8_t ret = 0;
	//sgkim
	gMPUintStatus = SensorMpu9250_irqStatus();
	
	if(gMPUintStatus & MPU_DATA_READY)
	{
		SensorMpu9250_accRead(AccData);
		ret = 1;
	}

    return ret;
}
/*********************************************************************
 * @fn      SensorTagMov_reset
 *
 * @brief   Reset characteristics and disable sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_reset(void)
{
  gMPUconfig = ST_CFG_SENSOR_DISABLE | (ACC_RANGE_8G << 8);
  appStateSet(APP_STATE_OFF);
  gMPUconfig = MPU_AX_SEN | ( ACC_RANGE_8G << 8);
  appStateSet(APP_STATE_ACTIVE);
}

/*********************************************************************
 * @fn      SensorTagMov_reset
 */
void ss_gMPUconfig_setting(uint16_t val)
{
	gMPUconfig = val;
}



/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      SensorTagMov_processInterrupt
 *
 * @brief   Interrupt handler for MPU
 *
 * @param   none
 *
 * @return  none
 */


void SensorMpu9250_intrrupt_disable(void);
void SensorMpu9250_intrrupt_enable(void);
void SensorMpu9250_INT_ENABLE_SET(uint8_t val);

//sgkim
int gInterruptGenValue = 0;

void SensorTagMov_processInterrupt(void)
{

    Log_info0("interrupt gen");

    //SensorMpu9250_intrrupt_disable();
    //SensorMpu9250_INT_ENABLE_SET( 1<<0 );
    //SensorMpu9250_intrrupt_enable();

#if 0 // kangkw
    SensorMpu9250_intrrupt_disable();
    gMPUconfig = MPU_AX_SEN | ( ACC_RANGE_8G << 8);
    SensorMpu9250_enable(gMPUconfig & 0xFF);
    Util_startClock(&gPeriodicClock);
#else
	// Wake up the application thread
    gInterruptGenValue = 1;
	Semaphore_post(Semaphore_handle(&semSensorAlert));
#endif

    Log_info0("get interrupt and start clk timer");

}
/*********************************************************************
 * @fn      SensorTagMov_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - not used
 *
 * @return  none
 */
void SensorTagMov_clockHandler(UArg arg)
{
	// Schedule readout periodically
	gInterruptGenValue = 0;
	Semaphore_post(Semaphore_handle(&semSensorAlert));
}

/*******************************************************************************
 * @fn      appStateSet
 *
 * @brief   Set the application state
 *
 */
static void appStateSet(uint8_t newState)
{

	if (newState == APP_STATE_OFF)
	{
		gAppState = APP_STATE_OFF;
		SensorMpu9250_enable(0);
		SensorMpu9250_powerOff();
		// Stop scheduled data measurements
		if( Util_isActive(&gPeriodicClock) == TRUE)
		{
			Util_stopClock(&gPeriodicClock);
			Log_info1("[appStateSet:%d] APP_STATE_OFF Util_stopClock()", __LINE__);
		}
	}
	else if(newState == APP_STATE_IDLE)
	{
		gAppState = APP_STATE_IDLE;
		gMovThreshold   = WOM_THR;
		gMPUintStatus  = 0;
		SensorMpu9250_powerOn();

		if (SensorMpu9250_reset())
		{
			SensorMpu9250_enableWom(gMovThreshold);
		}
		// Stop scheduled data measurements
		if( Util_isActive(&gPeriodicClock) == TRUE)
		{
			Util_stopClock(&gPeriodicClock);
			Log_info1("[appStateSet:%d] APP_STATE_IDLE Util_stopClock()", __LINE__);
		}
	}
	else if (newState == APP_STATE_ACTIVE)
	{
		gAppState = APP_STATE_ACTIVE;
		gMovThreshold   = WOM_THR;
		gMPUintStatus  = 0;
		SensorMpu9250_powerOn();  // Make sure pin interrupt is disabled
		SensorMpu9250_enable(gMPUconfig & 0xFF);
		// Start scheduled data measurements
		if( Util_isActive(&gPeriodicClock) == FALSE)
		{
			Util_startClock(&gPeriodicClock);
			Log_info1("[appStateSet:%d] APP_STATE_ACTIVE Util_startClock()", __LINE__);
		}
	}
}

/*********************************************************************
 * @fn      SensorTagMov_processCharChangeEvt
 *
 * @brief   SensorTag Movement event handling
 *
 * @param   paramID - identifies which characteristic has changed
 *
 * @return  none
 */
void SensorTagMov_processChange(uint8_t paramID)
{
	switch (paramID)
	{
		case ST_CFG_SENSOR_DISABLE:
			gMPUconfig = ST_CFG_SENSOR_DISABLE | (ACC_RANGE_8G << 8);
			appStateSet(APP_STATE_OFF);
			break;
		case ST_CFG_SENSOR_IDLE:
			gMPUconfig = MPU_AX_SEN | (ACC_RANGE_8G << 8);
			appStateSet(APP_STATE_IDLE);
			break;

		case ST_CFG_SENSOR_ENABLE:
			gMPUconfig = MPU_AX_SEN | ( ACC_RANGE_8G << 8);
			appStateSet(APP_STATE_ACTIVE);
			break;
		default:
		// Should not get here
		break;
	}
}


/*********************************************************************
*********************************************************************/

