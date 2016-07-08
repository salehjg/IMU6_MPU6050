 /***************************************************
  This is a library for MPU6050.

  ------> http://www.easymcu.ir
  
  Check out the links above for our tutorials and wiring diagrams
  
  Redistributed by Saleh Jamali(SalehJG) for EasyMCU
  All text above must be included in any redistribution
 ****************************************************/


//*****************************************************************************
// Usefull Library With I2C 4 Setup MPU6050
// Copyright :              WWW.RoboticNGO.com      &      www.ECA.ir
// Author :                 S_Ahmad (Seyyed Ahmad Mousavi)
// Remarks :
// known Problems :         None
// Version :                1.5
// Date :                   1392/10/23
// Company :                www.RoboticNGO.com      &      www.ECA.ir
//
// -----------------
//                  |                 ----------------
//                  |- 5v  ----- Vcc -| MPU 6050     |
// MicroController  |- GND ----- GND -| Acceleration,|
// Board            |- SDA ----- SDA -| Gyro, Temp   |
//                  |- SCL ----- SCL -|   Module     |
//                  |                 ----------------
//------------------
//
//*****************************************************************************
 
#ifndef __MPU6050_HPP__
#define __MPU6050_HPP__

#include "common_func.h"
#include <math.h>

#define i2cx i2c1


	#define MPU6050_ADDRESS (0x68<<1)         //0b1101000   (7-bit DADR without r/w bit) [AD0 pin of MPU6050 tied GND]
	#define X 0
	#define Y 1
	#define Z 2	
	#define off 0
	#define on  1
	//======================================================================================================================
	//MPU6050 REGISTER ADDRESS MAP:
	//The reset value is 0x00 for all registers other than the registers below.
	// Register PWR_MGMT_1 (0x6B): 0x40.
	// Register WHO_AM_I   (0x75): 0x68.

	//    Register Name    Addr(Dec.) Serial I/F    Bit7     Bit6     Bit5     Bit4     Bit3     Bit2     Bit1     Bit0
	#define RA_XG_OFFS_TC          0   //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
	#define RA_YG_OFFS_TC          1   //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
	#define RA_ZG_OFFS_TC          2   //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD

	#define RA_X_FINE_GAIN         3   // R/W   X_FINE_GAIN[7:0]
	#define RA_Y_FINE_GAIN         4   // R/W   Y_FINE_GAIN[7:0]
	#define RA_Z_FINE_GAIN         5   // R/W   Z_FINE_GAIN[7:0]

	#define RA_XA_OFFS_H           6   // R/W   XA_OFFS[15:0]
	#define RA_XA_OFFS_L_TC        7   // R/W
	#define RA_YA_OFFS_H           8   // R/W   YA_OFFS[15:0]
	#define RA_YA_OFFS_L_TC        9   // R/W
	#define RA_ZA_OFFS_H           10  // R/W   ZA_OFFS[15:0]
	#define RA_ZA_OFFS_L_TC        11  // R/W

	#define RA_SELF_TEST_X         13  // R/W  XA_TEST[4-2]     XG_TEST[4-0]
	#define RA_SELF_TEST_Y         14  // R/W  YA_TEST[4-2]     YG_TEST[4-0]
	#define RA_SELF_TEST_Z         15  // R/W  ZA_TEST[4-2]     ZG_TEST[4-0]
	#define RA_SELF_TEST_A         16  // R/W    RESERVED       XA_TEST[1-0]     YA_TEST[1-0]     ZA_TEST[1-0]

	#define RA_XG_OFFS_USRH        19  // R/W   XG_OFFS_USR[15:0]
	#define RA_XG_OFFS_USRL        20  // R/W
	#define RA_YG_OFFS_USRH        21  // R/W   YG_OFFS_USR[15:0]
	#define RA_YG_OFFS_USRL        22  // R/W
	#define RA_ZG_OFFS_USRH        23  // R/W   ZG_OFFS_USR[15:0]
	#define RA_ZG_OFFS_USRL        24  // R/W

	#define RA_SMPLRT_DIV          25  // R/W  SMPLRT_DIV[7:0]

	#define RA_CONFIG              26  // R/W    -     -     EXT_SYNC_SET[2:0]     DLPF_CFG[2:0]
	#define RA_GYRO_CONFIG         27  // R/W    -     -     -     FS_SEL [1:0]     -     -     -
	#define RA_ACCEL_CONFIG        28  // R/W  XA_ST     YA_ST     ZA_ST     AFS_SEL[1:0]    -     -     -

	#define RA_FF_THR              29  // R/W	FF_THR[7:0]
	#define RA_FF_DUR              30  // R/W	FF_DUR[7:0]
	#define RA_MOT_THR             31  // R/W	MOT_THR[7:0]
	#define RA_MOT_DUR             32  // R/W	MOT_DUR[7:0]
	#define RA_ZRMOT_THR           33  // R/W	ZRMOT_THR[7:0]
	#define RA_ZRMOT_DUR           34  // R/W	ZRMOT_DUR[7:0]

	#define RA_FIFO_EN             35  // R/W  TEMP _FIFO_EN     XG _FIFO_EN     YG _FIFO_EN     ZG _FIFO_EN     ACCEL _FIFO_EN     SLV2 _FIFO_EN     SLV1 _FIFO_EN     SLV0 _FIFO_EN

	#define RA_I2C_MST_CTRL        36  // R/W  MULT _MST_EN     WAIT _FOR_ES     SLV_3 _FIFO_EN     I2C_MST _P_NSR     I2C_MST_CLK[3:0]
	#define RA_I2C_SLV0_ADDR       37  // R/W  I2C_SLV0 _RW     I2C_SLV0_ADDR[6:0]
	#define RA_I2C_SLV0_REG        38  // R/W  I2C_SLV0_REG[7:0]
	#define RA_I2C_SLV0_CTRL       39  // R/W  I2C_SLV0 _EN     I2C_SLV0 _BYTE_SW     I2C_SLV0 _REG_DIS     I2C_SLV0 _GRP     I2C_SLV0_LEN[3:0]
	#define RA_I2C_SLV1_ADDR       40  // R/W  I2C_SLV1 _RW     I2C_SLV1_ADDR[6:0]
	#define RA_I2C_SLV1_REG        41  // R/W  I2C_SLV1_REG[7:0]
	#define RA_I2C_SLV1_CTRL       42  // R/W  I2C_SLV1 _EN     I2C_SLV1 _BYTE_SW     I2C_SLV1 _REG_DIS     I2C_SLV1 _GRP     I2C_SLV1_LEN[3:0]
	#define RA_I2C_SLV2_ADDR       43  // R/W  I2C_SLV2 _RW     I2C_SLV2_ADDR[6:0]
	#define RA_I2C_SLV2_REG        44  // R/W  I2C_SLV2_REG[7:0]
	#define RA_I2C_SLV2_CTRL       45  // R/W  I2C_SLV2 _EN     I2C_SLV2 _BYTE_SW     I2C_SLV2 _REG_DIS     I2C_SLV2 _GRP     I2C_SLV2_LEN[3:0]
	#define RA_I2C_SLV3_ADDR       46  // R/W  I2C_SLV3 _RW     I2C_SLV3_ADDR[6:0]
	#define RA_I2C_SLV3_REG        47  // R/W  I2C_SLV3_REG[7:0]
	#define RA_I2C_SLV3_CTRL       48  // R/W  I2C_SLV3 _EN     I2C_SLV3 _BYTE_SW 	I2C_SLV3 _REG_DIS 	I2C_SLV3 _GRP 	I2C_SLV3_LEN[3:0]
	#define RA_I2C_SLV4_ADDR 	   49  // R/W  I2C_SLV4 _RW 	I2C_SLV4_ADDR[6:0]
	#define RA_I2C_SLV4_REG 	   50  // R/W  I2C_SLV4_REG[7:0]
	#define RA_I2C_SLV4_DO 	       51  // R/W  I2C_SLV4_DO[7:0]
	#define RA_I2C_SLV4_CTRL 	   52  // R/W  I2C_SLV4 _EN 	I2C_SLV4  _INT_EN 	I2C_SLV4 _REG_DIS 	I2C_MST_DLY[4:0]
	#define RA_I2C_SLV4_DI 	       53  // R    I2C_SLV4_DI[7:0]
	#define RA_I2C_MST_STATUS 	   54  // R    PASS_ THROUGH 	I2C_SLV4 _DONE 	I2C_LOST _ARB 	I2C_SLV4 _NACK 	I2C_SLV3 _NACK 	I2C_SLV2 _NACK 	I2C_SLV1 _NACK 	I2C_SLV0 _NACK

	#define RA_INT_PIN_CFG 	       55  // R/W  INT_LEVEL 	INT_OPEN 	LATCH _INT_EN 	INT_RD _CLEAR 	FSYNC_ INT_LEVEL 	FSYNC _INT_EN 	I2C _BYPASS _EN 	-
	#define RA_INT_ENABLE 	       56  // R/W   - 	- 	- 	FIFO _OFLOW _EN 	I2C_MST _INT_EN 	- 	- 	DATA _RDY_EN
	#define RA_INT_STATUS   	   58  // R 	- 	- 	- 	FIFO _OFLOW _INT 	I2C_MST _INT 	- 	- 	DATA _RDY_INT

	#define RA_ACCEL_XOUT_H 	   59  // R    ACCEL_XOUT[15:8]
	#define RA_ACCEL_XOUT_L 	   60  // R    ACCEL_XOUT[7:0]
	#define RA_ACCEL_YOUT_H 	   61  // R    ACCEL_YOUT[15:8]
	#define RA_ACCEL_YOUT_L 	   62  // R    ACCEL_YOUT[7:0]
	#define RA_ACCEL_ZOUT_H 	   63  // R    ACCEL_ZOUT[15:8]
	#define RA_ACCEL_ZOUT_L        64  // R    ACCEL_ZOUT[7:0]

	#define RA_TEMP_OUT_H 	       65  // R    TEMP_OUT[15:8]
	#define RA_TEMP_OUT_L 	       66  // R    TEMP_OUT[7:0]

	#define RA_GYRO_XOUT_H	       67  // R    GYRO_XOUT[15:8]
	#define RA_GYRO_XOUT_L	       68  // R    GYRO_XOUT[7:0]
	#define RA_GYRO_YOUT_H	       69  // R    GYRO_YOUT[15:8]
	#define RA_GYRO_YOUT_L	       70  // R    GYRO_YOUT[7:0]
	#define RA_GYRO_ZOUT_H 	       71  // R    GYRO_ZOUT[15:8]
	#define RA_GYRO_ZOUT_L 	       72  // R    GYRO_ZOUT[7:0]

	#define RA_EXT_SENS_DATA_00    73  // R    EXT_SENS_DATA_00[7:0]
	#define RA_EXT_SENS_DATA_01    74  // R    EXT_SENS_DATA_01[7:0]
	#define RA_EXT_SENS_DATA_02    75  // R    EXT_SENS_DATA_02[7:0]
	#define RA_EXT_SENS_DATA_03    76  // R    EXT_SENS_DATA_03[7:0]
	#define RA_EXT_SENS_DATA_04    77  // R    EXT_SENS_DATA_04[7:0]
	#define RA_EXT_SENS_DATA_05    78  // R    EXT_SENS_DATA_05[7:0]
	#define RA_EXT_SENS_DATA_06    79  // R    EXT_SENS_DATA_06[7:0]
	#define RA_EXT_SENS_DATA_07	   80  // R    EXT_SENS_DATA_07[7:0]
	#define RA_EXT_SENS_DATA_08    81  // R    EXT_SENS_DATA_08[7:0]
	#define RA_EXT_SENS_DATA_09    82  // R    EXT_SENS_DATA_09[7:0]
	#define RA_EXT_SENS_DATA_10	   83  // R    EXT_SENS_DATA_10[7:0]
	#define RA_EXT_SENS_DATA_11    84  // R    EXT_SENS_DATA_11[7:0]
	#define RA_EXT_SENS_DATA_12    85  // R    EXT_SENS_DATA_12[7:0]
	#define RA_EXT_SENS_DATA_13    86  // R    EXT_SENS_DATA_13[7:0]
	#define RA_EXT_SENS_DATA_14    87  // R    EXT_SENS_DATA_14[7:0]
	#define RA_EXT_SENS_DATA_15    88  // R    EXT_SENS_DATA_15[7:0]
	#define RA_EXT_SENS_DATA_16	   89  // R    XT_SENS_DATA_16[7:0]
	#define RA_EXT_SENS_DATA_17    90  // R    EXT_SENS_DATA_17[7:0]
	#define RA_EXT_SENS_DATA_18    91  // R    EXT_SENS_DATA_18[7:0]
	#define RA_EXT_SENS_DATA_19    92  // R    EXT_SENS_DATA_19[7:0]
	#define RA_EXT_SENS_DATA_20    93  // R    EXT_SENS_DATA_20[7:0]
	#define RA_EXT_SENS_DATA_21	   94  // R    EXT_SENS_DATA_21[7:0]
	#define RA_EXT_SENS_DATA_22	   95  // R    EXT_SENS_DATA_22[7:0]
	#define RA_EXT_SENS_DATA_23    96  // R    EXT_SENS_DATA_23[7:0]

	#define RA_MOT_DETECT_STATUS   97  // R    MOT_XNEG[7]   MOT_XPOS[6]   MOT_YNEG[5]   MOT_YPOS[4]   MOT_ZNEG[3]   MOT_ZPOS[2]   MOT_ZRMOT[0]

	#define RA_I2C_SLV0_DO	       99  // R/W  I2C_SLV0_DO[7:0]
	#define RA_I2C_SLV1_DO 	       100 // R/W  I2C_SLV1_DO[7:0]
	#define RA_I2C_SLV2_DO 	       101 // R/W  I2C_SLV2_DO[7:0]
	#define RA_I2C_SLV3_DO 	       102 // R/W  I2C_SLV3_DO[7:0]

	#define RA_I2C_MST_DELAY_CTRL  103 // R/W  DELAY_ES _SHADOW 	- 	- 	I2C_SLV4 _DLY_EN 	I2C_SLV3 _DLY_EN 	I2C_SLV2 _DLY_EN 	I2C_SLV1 _DLY_EN 	I2C_SLV0 _DLY_EN
	#define RA_SIGNAL_PATH_RESET   104 // R/W 	- 	- 	- 	- 	- 	GYRO _RESET 	ACCEL _RESET 	TEMP _RESET
	#define RA_MOT_DETECT_CTRL     105 // R/W    	ACCEL_ON_DELAY[5:4] 	FF_COUNT[3:2]
	#define RA_USER_CTRL 	       106 // R/W 	- 	FIFO_EN 	I2C_MST_EN 	I2C_IF_DIS 	- 	FIFO_RESET 	I2C_MST_RESET 	SIG_COND_RESET
	#define RA_PWR_MGMT_1          107 // R/W  DEVICE _RESET 	SLEEP 	CYCLE 	- 	TEMP_DIS 	CLKSEL[2:0]
	#define RA_PWR_MGMT_2 	       108 // R/W  LP_WAKE_CTRL[1:0] 	STBY_XA 	STBY_YA 	STBY_ZA 	STBY_XG 	STBY_YG 	STBY_ZG

	#define RA_BANK_SEL            109 // R/W   PRFTCH_EN[6]	 CFG_USER_BANK[5]	 MEM_SEL[4:0]
	#define RA_MEM_START_ADDR      110 // R/W   START_ADDR[7:0]
	#define RA_MEM_R_W             111 // R/W   MEM_R_W[7:0]
	#define RA_DMP_CFG_1           112 // R/W
	#define RA_DMP_CFG_2           113 // R/W

	#define RA_FIFO_COUNTH 	       114 // R/W  FIFO_COUNT[15:8]
	#define RA_FIFO_COUNTL 	       115 // R/W  FIFO_COUNT[7:0]
	#define RA_FIFO_R_W 	       116 // R/W  FIFO_DATA[7:0]
	#define RA_WHO_AM_I 	       117 // R 	- 	WHO_AM_I[6:1] 	-

	//======================================================================================================================
	#define FsDiv8          7       // Fs/(1+7)
	#define FsDiv7          6       // Fs/(1+6)
	#define FsDiv6          5       // Fs/(1+5)
	#define FsDiv5          4       // Fs/(1+4)
	#define FsDiv4          3       // Fs/(1+3)
	#define FsDiv3          2       // Fs/(1+2)
	#define FsDiv2          1       // Fs/(1+1)
	#define FsDiv1          0       // Fs/(1+0)
	#define SampleRateDiv   FsDiv5
	// You can find the Fs value from the DLPF section
	//======================================================================================================================
	#define EXT_SYNC_SET_Dis    0   // Input disabled
	#define EXT_SYNC_SET_Temp   1   // TEMP_OUT_L[0]
	#define EXT_SYNC_SET_GX     2   // GYRO_XOUT_L[0]
	#define EXT_SYNC_SET_GY     3   // GYRO_YOUT_L[0]
	#define EXT_SYNC_SET_GZ     4   // GYRO_ZOUT_L[0]
	#define EXT_SYNC_SET_AX     5   // ACCEL_XOUT_L[0]
	#define EXT_SYNC_SET_AY     6   // ACCEL_YOUT_L[0]
	#define EXT_SYNC_SET_AZ     7   // ACCEL_ZOUT_L[0]
	#define EXT_SYNC_SET        EXT_SYNC_SET_Dis

			   // Accelerometer   |         Gyroscope         |
			   // (Fs = 1kHz) 	  |                           |
			   // BW(Hz) Delay(ms)|	BW(Hz) 	Delay(ms) Fs(kHz) |
	#define DLPF_CFG_0 	 0 // 260 	0 	      |  256 	0.98 	   8      |
	#define DLPF_CFG_1 	 1 // 184 	2.0 	  |  188 	1.9 	   1      |
	#define DLPF_CFG_2 	 2 // 94 	3.0 	  |  98 	2.8 	   1      |
	#define DLPF_CFG_3 	 3 // 44 	4.9 	  |  42 	4.8 	   1      |
	#define DLPF_CFG_4 	 4 // 21 	8.5 	  |  20 	8.3 	   1      |
	#define DLPF_CFG_5 	 5 // 10 	13.8 	  |  10 	13.4 	   1      |
	#define DLPF_CFG_6 	 6 // 5 	19.0 	  |  5 	    18.6 	   1      |
	#define DLPF_CFG     DLPF_CFG_3
	//======================================================================================================================
	//Range_250  : 0b00000
	//Range_500  : 0b01000
	//Range_1000 : 0b10000
	//Range_2000 : 0b11000
	#define GYRO_FullScale_Range_250     0x0  
	#define GYRO_FullScale_Range_500     1<<3 
	#define GYRO_FullScale_Range_1000    1<<4 
	#define GYRO_FullScale_Range_2000    3<<3 

	#define GFS_SEL                      GYRO_FullScale_Range_2000

	#define XG_ST    0  // Active Or Deactive self-test for X gyroscope axis
	#define YG_ST    0  // Active Or Deactive self-test for Y gyroscope axis
	#define ZG_ST    0  // Active Or Deactive self-test for Z gyroscope axis
	//======================================================================================================================
	#define GYRO_Sensitivity_Range_250   131.0   // LSB/(º/s)
	#define GYRO_Sensitivity_Range_500   65.5    // LSB/(º/s)
	#define GYRO_Sensitivity_Range_1000  32.8    // LSB/(º/s)
	#define GYRO_Sensitivity_Range_2000  16.4    // LSB/(º/s)

	
	#if   GFS_SEL == GYRO_FullScale_Range_250
	#define GYRO_Sensitivity       GYRO_Sensitivity_Range_250
	#elif GFS_SEL == GYRO_FullScale_Range_500
	#define GYRO_Sensitivity       GYRO_Sensitivity_Range_500
	#elif GFS_SEL == GYRO_FullScale_Range_1000
	#define GYRO_Sensitivity       GYRO_Sensitivity_Range_1000
	#elif GFS_SEL == GYRO_FullScale_Range_2000
	
	#define GYRO_Sensitivity       GYRO_Sensitivity_Range_2000
	#endif

	//======================================================================================================================
	/*
	2g:  0b00000     // 0
	4g:  0b01000     // 1
	8g:  0b10000     // 2
	16g: 0b11000     // 3
	*/
	#define ACCEL_FullScale_Range_2g     0x0
	#define ACCEL_FullScale_Range_4g     (1<<3)
	#define ACCEL_FullScale_Range_8g     (1<<4)
	#define ACCEL_FullScale_Range_16g    (0x18)
	#define AFS_SEL                      ACCEL_FullScale_Range_16g

	#define XA_ST    off  // Active Or Deactive self-test for X accelerometer axis
	#define YA_ST    off  // Active Or Deactive self-test for Y accelerometer axis
	#define ZA_ST    off  // Active Or Deactive self-test for Z accelerometer axis
	//======================================================================================================================
	#define ACCEL_Sensitivity_Range_2g   (16384.0f)   // LSB/g
	#define ACCEL_Sensitivity_Range_4g   (8192.0f)    // LSB/g
	#define ACCEL_Sensitivity_Range_8g   (4096.0f)    // LSB/g
	#define ACCEL_Sensitivity_Range_16g  (2048.0f)    // LSB/g

	
	#if AFS_SEL == ACCEL_FullScale_Range_2g
	#define ACCEL_Sensitivity                   ACCEL_Sensitivity_Range_2g
	#elif AFS_SEL == ACCEL_FullScale_Range_4g
	#define ACCEL_Sensitivity            ACCEL_Sensitivity_Range_4g
	#elif AFS_SEL == ACCEL_FullScale_Range_8g
	#define ACCEL_Sensitivity            ACCEL_Sensitivity_Range_8g
	#elif AFS_SEL == ACCEL_FullScale_Range_16g
	
	#define ACCEL_Sensitivity            ACCEL_Sensitivity_Range_16g
	#endif

	//======================================================================================================================
	#define TEMP_Sensitivity    340.0f   // LSB/ºC
	//======================================================================================================================
	#define TEMP_FIFO_En        0b10000000
	#define X_GYRO_FIFO_En      0b01000000
	#define Y_GYRO_FIFO_En      0b00100000
	#define Z_GYRO_FIFO_En      0b00010000
	#define ACCEL_FIFO_En       0b00001000
	#define SLV2_FIFO_En        0b00000100
	#define SLV1_FIFO_En        0b00000010
	#define SLV0_FIFO_En        0b00000001
	#define ANY_FIFO_Bit_En     0
	#define FIFO_En_Parameters  ANY_FIFO_Bit_En
	//======================================================================================================================
	#define Int8MHz              0
	#define PLL_XG_ref           1
	#define PLL_YG_ref           2
	#define PLL_ZG_ref           3
	#define PLL_ext32768Hz_ref   4
	#define PLL_ext19200KHz_ref  5
	#define Stop                 7
	#define CLKSEL               PLL_XG_ref

	#define TEMP_DIS        off  // When set to 1, this bit disables the temperature sensor
	#define CYCLE           off  // When this bit is set to 1 and SLEEP is disabled, the MPU-6050 will cycle between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL (register 108).
	#define SLEEP           off  // When set to 1, this bit puts the MPU-60X0 into sleep mode.
	#define DEVICE_RESET    off  // When set to 1, this bit resets all internal registers to their default values.The bit automatically clears to 0 once the reset is done.The default values for each register can be found in RA_MPU6050.h.
	//======================================================================================================================
	#define LP_WAKE_CTRL_1_25Hz   0 // Wake-up Frequency : 1.25 Hz
	#define LP_WAKE_CTRL_5Hz      1 // Wake-up Frequency : 5 Hz
	#define LP_WAKE_CTRL_20Hz     2 // Wake-up Frequency : 20 Hz
	#define LP_WAKE_CTRL_40Hz     3 // Wake-up Frequency : 40 Hz
	#define LP_WAKE_CTRL          LP_WAKE_CTRL_1_25Hz

	#define STBY_XA   off // When set to 1, this bit puts the X axis accelerometer into standby mode.
	#define STBY_YA   off // When set to 1, this bit puts the Y axis accelerometer into standby mode.
	#define STBY_ZA   off // When set to 1, this bit puts the Z axis accelerometer into standby mode.
	#define STBY_XG   off // When set to 1, this bit puts the X axis gyroscope into standby mode.
	#define STBY_YG   off // When set to 1, this bit puts the Y axis gyroscope into standby mode.
	#define STBY_ZG   off // When set to 1, this bit puts the Z axis gyroscope into standby mode.
	//======================================================================================================================
	#define  GYRO_SIGMA_MULTIPLE  3.0f 
	
	
	
	class MPU6050
	{
		public:
			MPU6050(uint8_t portNum = 1);
			
			void write(unsigned char Reg , unsigned char Data); //write_i2c
			unsigned char read( unsigned char Reg )  ; 			//read_i2c
			void reset();										//MPU6050_Reset
			unsigned char test();								//MPU6050_Test_I2C
			//void MPU6050_Sleep(char ON_or_OFF);
			void init();										//MPU6050_Init
			void getAccelOffset();								//Get_Accel_Offset
			void getAccelVal();									//Get_Accel_Val
			void getAvrgAccel_Val();							//Get_AvrgAccel_Val
			void getAccelAngles();								//Get_Accel_Angles
			void getGyroOffset();								//Get_Gyro_Offset
			void getGyroVal();									//Get_Gyro_Val
			void getAvrgGyroVal();								//Get_AvrgGyro_Val
			void getTempVal();									//Get_Temp_Val
			void setLastTime(void);								//setLastTime
			ulong getLastTime(void);							//getLastTime

			float Accel_Raw_Val[3];
			float AvrgAccel_Raw_Val[3];
			float Accel_In_g[3];
			float Accel_Offset_Val[3];
			float Accel_Angle[3];

			float Gyro_Raw_Val[3];
			float AvrgGyro_Raw_Val[3];
			float Gyro_Offset_Val[3];
			float GyroRate_Val[3];
			float Gyrothreshold[3];

			float Temp_Val;		
			ulong last_time;			
			/*=========================================================================*/
		private:
			uint8_t i2cPortNum;
			


	};

#endif

/************************************************************************/

