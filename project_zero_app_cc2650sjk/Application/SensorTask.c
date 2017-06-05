
/*********************************************************************
 * INCLUDES
 */
#include <string.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>
#include <driverlib/aon_batmon.h>

#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

// Stack headers
#include <hci_tl.h>
#include <gap.h>
#include <gatt.h>
#include <gapgattserver.h>
#include <gattservapp.h>
#include <osal_snv.h>
#include <gapbondmgr.h>
#include <peripheral.h>
#include <icall_apimsg.h>

#include <devinfoservice.h>
#include "util.h"
#include "Board.h"
#include "SensorI2C.h"
#include "sensortag_mov.h"
#include "SensorUtil.h"
#include "SensorMpu9250.h"
#include "project_zero.h"
#include "SensorTask.h"
#include "sjk_service.h"

typedef void (*SensorMpu9250CallbackFn_t)(void);
void SensorTagMov_clockHandler(UArg arg);
Clock_Struct *get_gPeriodicClock(void);

void SensorMpu9250_enable(uint16_t axes);
bool SensorMpu9250_enableWom(uint8_t threshold);
void SensorMpu9250_interrupt_pin_init(void);
void SensorMpu9250_powerOn(void);
void SensorMpu9250_registerCallback(SensorMpu9250CallbackFn_t pfn);
bool SensorMpu9250_reset(void);
void SensorTagMov_processInterrupt(void);
void sensorMagEnable(bool);

// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD     2
#define WOM_THR                       0x1F // R/W
// Accelerometer ranges
#define ACC_RANGE_2G      0
#define ACC_RANGE_4G      1
#define ACC_RANGE_8G      2
#define ACC_RANGE_16G     3
#define ACC_RANGE_INVALID 0xFF

// Axis bitmaps
#define MPU_AX_GYR        0x07
#define MPU_AX_ACC        0x38
#define MPU_AX_MAG        0x40
#define MPU_AX_ALL        0x7F
//#define MPU_AX_SEN        0x3F
#define MPU_AX_SEN        0x3A
// Interrupt status bit
#define MPU_DATA_READY    0x01
#define MPU_MOVEMENT      0x40


#define SIGNAL_PATH_RESET             0x68 // R/W
#define ACCEL_INTEL_CTRL              0x69 // R/W
#define USER_CTRL                     0x6A // R/W
#define PWR_MGMT_1                    0x6B // R/W
#define PWR_MGMT_2                    0x6C // R/W
#define FIFO_COUNT_H                  0x72 // R/W
#define FIFO_COUNT_L                  0x73 // R/W
#define FIFO_R_W                      0x74 // R/W
#define WHO_AM_I                      0x75 // R/W
#define INT_ENABLE                    0x38 // R/W
#define SMPLRT_DIV                    0x19 // R/W
#define CONFIG                        0x1A // R/W
#define GYRO_CONFIG                   0x1B // R/W
#define ACCEL_CONFIG                  0x1C // R/W
#define ACCEL_CONFIG_2                0x1D // R/W
#define LP_ACCEL_ODR                  0x1E // R/W
#define WOM_THR                       0x1F // R/W
#define FIFO_EN                       0x23 // R/W
#define INT_PIN_CFG                   0x37 // R/W
#define INT_ENABLE                    0x38 // R/W
#define INT_STATUS                    0x3A // R


#define ST_CFG_SENSOR_DISABLE        0x00
#define ST_CFG_SENSOR_IDLE           0x01
#define ST_CFG_SENSOR_ENABLE         0x02

// Sensor selection/de-selection
#define SENSOR_SELECT()               SensorI2C_select(SENSOR_I2C_1, Board_MPU9250_ADDR)
#define SENSOR_SELECT_MAG()           SensorI2C_select(SENSOR_I2C_1, Board_MPU9250_MAG_ADDR)
#define SENSOR_DESELECT()             SensorI2C_deselect()


void SensorMpu9250_intrrupt_disable(void);
void SensorMpu9250_intrrupt_enable(void);
uint8_t SensorMpu9250_irqStatus(void);
void sensorMagInit(void);
bool SensorMpu9250_magTest(void);


extern Semaphore_Struct semSensorAlert;
extern void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData, uint16_t len );

// Struct for message about button state
typedef struct
{
  PIN_Id   pinId;
  uint8_t  state;
} button_state_t;


extern void user_handleButtonPress(button_state_t *pState);
extern int gInterruptGenValue;

uint8_t gAccDataXYZ[BS_SJK1_LEN*2*3];
extern uint8_t bs_SJK1Val[BS_SJK1_LEN];


void Sensor_taskFxn(UArg a0, UArg a1)
{

    uint8_t ret, val;
    int i;
    static Clock_Struct *pgPeriodicClock;

    SensorI2C_open();


    //sgkim1
    pgPeriodicClock = get_gPeriodicClock();
    // Create continuous clock for internal periodic events.
    //Util_constructClock(pgPeriodicClock, SensorTagMov_clockHandler, 1, SENSOR_DEFAULT_PERIOD, false, 0);
    //sgkim-timer-set
    Util_constructClock(pgPeriodicClock, SensorTagMov_clockHandler, 1, 5 , false, 0);

    // Pins used by MPU
    SensorMpu9250_interrupt_pin_init();
    SensorMpu9250_registerCallback(SensorTagMov_processInterrupt);

    // MPU Setting
    SensorMpu9250_powerOn();
    if(! SensorMpu9250_reset() ) Log_info0("SensorMpu9250 Rest Error");

    //SensorMpu9250_enableWom(0x1F);
    // Select magnetometer (all axes at once)
    // Make sure accelerometer is running, CYCLE, CLKSEL
    //val = 0x09;
    //ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    val = SensorMpu9250_magTest();
    Log_info1("retu val = %d",val);
    sensorMagInit();
    sensorMagEnable(1);
    Log_info0("mag init end");

    SENSOR_SELECT();

    // Make sure accelerometer is running, CYCLE, CLKSEL
    val = 0x09;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Select gyro and accelerometer , all enable
    //val = 0x00;
    val = 0x2F;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_2, &val, 1));

    // Set Accel LPF setting to 184 Hz Bandwidth
    //val = 0x01;
    //val = (1 << 3);  //4KHz Rate Accelerometer
    val = 0x01;  //4KHz Rate Accelerometer
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG_2, &val, 1));

    // Enable Motion Interrupt
    val = (1 << 6) ;
    ST_ASSERT(SensorI2C_writeReg(INT_ENABLE, &val, 1));

    // Enable Accel Hardware Intelligence
    val = ((1 << 7) | (1 << 6));
    ST_ASSERT(SensorI2C_writeReg(ACCEL_INTEL_CTRL, &val, 1));

    // Set Motion Threshhld (0x00 ~ 0xFF, 4mg, 0 ~ 1020mg)
    val = (0x4F);
    ST_ASSERT(SensorI2C_writeReg(WOM_THR, &val, 1));

    // Set Frequency of Wake-up (0 ~ 11)
    val = 11; // 500 Hz
    ST_ASSERT(SensorI2C_writeReg(LP_ACCEL_ODR, &val, 1));

    // Select the current range 2-00, 4-01, 8-10, 16-11
    //val = ( 3 << 3); //4G setting
    //ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG, &val, 1));
    //ACC_RANGE_2G,ACC_RANGE_4G,ACC_RANGE_8G,ACC_RANGE_16G     3
    SensorMpu9250_accSetRange(ACC_RANGE_8G);


    // Enable Cycle Mode (Accel Low Power Mode)
    val = 0x29;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    SENSOR_DESELECT();

    SensorMpu9250_intrrupt_enable();

    Log_info0("start main while");

    int rcvCount = 0;

    //sgkim
	while (1)
	{
	    static uint16_t RawAcc[3];
	    //Wait for the Timer Clock post
		Semaphore_pend(Semaphore_handle(&semSensorAlert), BIOS_WAIT_FOREVER);  // BIOS_NO_WAIT   BIOS_WAIT_FOREVER
		//Log_info1("gInterruptGenValue = %d", gInterruptGenValue);
		if(gInterruptGenValue == 1)
		{
		    SensorMpu9250_intrrupt_disable();
		    SensorMpu9250_enable((MPU_AX_SEN | ( ACC_RANGE_8G << 8)) & 0xFF);
		    if( Util_isActive(pgPeriodicClock) == FALSE)
		    {
		    	Util_startClock(pgPeriodicClock);
		    	Log_info0("Util_startClock()");
		    }
		}
		if(rcvCount < BS_SJK1_LEN)
		{
		    //get Sensor DAta
		    memset(RawAcc,0,sizeof(RawAcc));
		    ret = SensorTagAcc_processSensor(RawAcc);
		    //RawAcc[0]=rcvCount;
		    //RawAcc[1]=rcvCount+2;
		    //RawAcc[2]=rcvCount+4;
		    memcpy(bs_SJK1Val + rcvCount, RawAcc,sizeof(RawAcc));
		    //Log_info4("rcvCount = %5d Ax=%7d  Ay=%7d  Az=%7d",  rcvCount, *((int16_t *)(bs_SJK1Val + rcvCount)), *((int16_t *)(bs_SJK1Val + rcvCount + 2)), *((int16_t *)(bs_SJK1Val + rcvCount + 4)) );
		    rcvCount += 6;
		}
		else
		{
            // Stop scheduled data measurements
            if( Util_isActive(pgPeriodicClock) == TRUE)
            {
                Util_stopClock(pgPeriodicClock);
                Log_info0("Util_stopClock()");
            }
#if 0
            for(i=0; i<60; i+=6)
            {
            	Log_info4("i = %5d Ax=%7d  Ay=%7d  Az=%7d",  i, *((int16_t *)(bs_SJK1Val + i)), *((int16_t *)(bs_SJK1Val + i + 2)), *((int16_t *)(bs_SJK1Val + i + 4)) );
            	//Task_sleep(700);
            }
#endif
            uint8_t mydata;
            mydata = rcvCount;
            user_enqueueRawAppMsg( APP_MY_MSG, (uint8_t *)&mydata, sizeof(mydata) );

            SensorMpu9250_intrrupt_enable();
            SensorMpu9250_irqStatus();
            rcvCount = 0;
		}
	} //while

}
