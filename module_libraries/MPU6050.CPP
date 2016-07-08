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


#include "MPU6050.h"


MPU6050::MPU6050(uint8_t portNum)
{
	i2cPortNum = portNum;
	i2cx.init(F_400HZ);
	
	Accel_Offset_Val[0]=Accel_Offset_Val[1]=Accel_Offset_Val[2]=0;
	Gyro_Offset_Val[0]=Gyro_Offset_Val[1]=Gyro_Offset_Val[2]=0;
}
//======================================================================================================================
//======================================================================================================================
//======================================================================================================================
void MPU6050::write( unsigned char Reg , unsigned char Data)
{
		i2cx.write(MPU6050_ADDRESS, Reg, Data);    
}
//======================================================================================================================
unsigned char MPU6050::read( unsigned char Reg )  //BusAddres: 7-bit address /!\: without r/w bit!!
{
	unsigned char Data;
	
	Data = i2cx.read(MPU6050_ADDRESS, Reg);	
	
	return Data;
}
//======================================================================================================================
//======================================================================================================================
//======================================================================================================================
// This function can restor MPU6050 to default
void MPU6050::reset()
{
    // When set to 1, DEVICE_RESET bit in RA_PWR_MGMT_1 resets all internal registers to their default values.
    // The bit automatically clears to 0 once the reset is done.
    // The default values for each register can be found in RA_MPU6050.h
    write( RA_PWR_MGMT_1, 0x80);
    // Now all reg reset to default values
		delay.ms(100);
}
//======================================================================================================================
// This function can test i2c communication MPU6050
unsigned char MPU6050::test()
{
    unsigned char Data = 0x00;
    Data=read( RA_WHO_AM_I);
    if(Data == 0x68)
        return 1;       // Means Comunication With MPU6050 is Corect
    else
        return 0;       // Means ERROR, Stopping
}
//======================================================================================================================
// MPU6050 sensor initialization
void MPU6050::init()
{
	//Sets sample rate to 1000/1+4 = 200Hz
	write( RA_SMPLRT_DIV, SampleRateDiv);
	//Disable FSync, 42Hz DLPF
	write( RA_CONFIG, (EXT_SYNC_SET<<3)|(DLPF_CFG));
	//Disable sensor output to FIFO buffer
	write( RA_FIFO_EN, FIFO_En_Parameters);

	//Freefall threshold of |0mg|
	write( RA_FF_THR, 0x00);
	//Freefall duration limit of 0
	write( RA_FF_DUR, 0x00);
	//Motion threshold of 0mg
	write( RA_MOT_THR, 0x00);
	//Motion duration of 0s
	write( RA_MOT_DUR, 0x00);
	//Zero motion threshold
	write( RA_ZRMOT_THR, 0x00);
	//Zero motion duration threshold
	write( RA_ZRMOT_DUR, 0x00);

	//Reset sensor signal paths
	write( RA_SIGNAL_PATH_RESET, 0x00);
	
	//Motion detection control
	write( RA_MOT_DETECT_CTRL, 0x00);
	//Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
	write( RA_USER_CTRL, 0x00);

	//Sets clock source to gyro reference w/ PLL
	write( RA_PWR_MGMT_1, (SLEEP<<6)|(CYCLE<<5)|(TEMP_DIS<<3)|CLKSEL);
	//Controls frequency of wakeups in accel low power mode plus the sensor standby modes
	write( RA_PWR_MGMT_2, (LP_WAKE_CTRL<<6)|(STBY_XA<<5)|(STBY_YA<<4)|(STBY_ZA<<3)|(STBY_XG<<2)|(STBY_YG<<1)|(STBY_ZG));
	//Data transfer to and from the FIFO buffer
	write( RA_FIFO_R_W, 0x00);


	//Disable all axis gyro self tests, scale of 2000 degrees/s
	write( RA_GYRO_CONFIG , GFS_SEL );
	//Disable accel self tests, scale of +-16g, no DHPF
	write( RA_ACCEL_CONFIG, AFS_SEL );
	//  MPU6050 Setup Complete
}
//======================================================================================================================
// get accel offset X,Y,Z
void MPU6050::getAccelOffset()
{
  #define    NumAve4AO      100
  float Ave=0;
  unsigned char i= NumAve4AO;
  while(i--)
  {
    Accel_Offset_Val[X] = (float)((short)(  ((read( RA_ACCEL_XOUT_H)<<8)|
                            read( RA_ACCEL_XOUT_L)    ))   );
    Ave = (float) Ave + (Accel_Offset_Val[X] / NumAve4AO);
    delay.us(100);
  }
  Accel_Offset_Val[X] = Ave;
  Ave = 0;
  i = NumAve4AO;
  while(i--)
  {
    Accel_Offset_Val[Y] = (float)((short)(  ((read( RA_ACCEL_YOUT_H)<<8)|
                            read( RA_ACCEL_YOUT_L)    ))   );
    Ave = (float) Ave + (Accel_Offset_Val[Y] / NumAve4AO);
    delay.us(100);
  }
  Accel_Offset_Val[Y] = Ave;
  Ave = 0;
  i = NumAve4AO;
  while(i--)
  {
    Accel_Offset_Val[Z] = (float)((short)(  ((read( RA_ACCEL_ZOUT_H)<<8)|
                            read( RA_ACCEL_ZOUT_L)    ))   );
    Ave = (float) Ave + (Accel_Offset_Val[Z] / NumAve4AO);
    delay.us(100);
  }
  Accel_Offset_Val[Z] = Ave;
}
//======================================================================================================================
// Gets raw accelerometer data, performs no processing
void MPU6050::getAccelVal()
{
    Accel_Raw_Val[X] = (float)(  (short)( ((read( RA_ACCEL_XOUT_H)<<8)|
                         read( RA_ACCEL_XOUT_L)    ))   );
		
    Accel_Raw_Val[Y] = (float)(  (short)( ((read( RA_ACCEL_YOUT_H)<<8)|
                         read( RA_ACCEL_YOUT_L)    ))   );
		
    Accel_Raw_Val[Z] = (float)(  (short)( ((read( RA_ACCEL_ZOUT_H)<<8)|
                         read( RA_ACCEL_ZOUT_L)    ))   );

    Accel_In_g[X] = Accel_Raw_Val[X] - Accel_Offset_Val[X];
    Accel_In_g[Y] = Accel_Raw_Val[Y] - Accel_Offset_Val[Y];
    Accel_In_g[Z] = Accel_Raw_Val[Z] - Accel_Offset_Val[Z];

    Accel_In_g[X] = Accel_In_g[X] / ACCEL_Sensitivity ;
    Accel_In_g[Y] = Accel_In_g[Y] / ACCEL_Sensitivity ;
    Accel_In_g[Z] = Accel_In_g[Z] / ACCEL_Sensitivity ;
}
//======================================================================================================================
// Gets n average raw accelerometer data, performs no processing
void MPU6050::getAvrgAccel_Val()
{
  #define    NumAve4A      50
  float Ave=0;
  unsigned char i= NumAve4A;
  while(i--)
  {
    AvrgAccel_Raw_Val[X] = (float)(   (short)((read( RA_ACCEL_XOUT_H)<<8)|
                             read( RA_ACCEL_XOUT_L)   ));
    Ave = (float) Ave + (AvrgAccel_Raw_Val[X] / NumAve4A);
    delay.us(10);
  }
  AvrgAccel_Raw_Val[X] = Ave;
  Ave = 0;
  i = NumAve4A;
  while(i--)
  {
    AvrgAccel_Raw_Val[Y] = (float)(   (short)((read( RA_ACCEL_YOUT_H)<<8)|
                             read( RA_ACCEL_YOUT_L)   ));
    Ave = (float) Ave + (AvrgAccel_Raw_Val[Y] / NumAve4A);
    delay.us(10);
  }
  AvrgAccel_Raw_Val[Y] = Ave;
  Ave = 0;
  i = NumAve4A;
  while(i--)
  {
    AvrgAccel_Raw_Val[Z] = (float)(   (short)((read( RA_ACCEL_ZOUT_H)<<8)|
                             read( RA_ACCEL_ZOUT_L)   ));
    Ave = (float) Ave + (AvrgAccel_Raw_Val[Z] / NumAve4A);
    delay.us(10);
  }
  AvrgAccel_Raw_Val[Z] = Ave;

  Accel_In_g[X] = AvrgAccel_Raw_Val[X] - Accel_Offset_Val[X];
  Accel_In_g[Y] = AvrgAccel_Raw_Val[Y] - Accel_Offset_Val[Y];
  Accel_In_g[Z] = AvrgAccel_Raw_Val[Z] - Accel_Offset_Val[Z];

  Accel_In_g[X] = Accel_In_g[X] / ACCEL_Sensitivity;  //  g/LSB
  Accel_In_g[Y] = Accel_In_g[Y] / ACCEL_Sensitivity;  //  g/LSB
  Accel_In_g[Z] = Accel_In_g[Z] / ACCEL_Sensitivity;  //  g/LSB

}
//======================================================================================================================
// Gets angles from accelerometer data
void MPU6050::getAccelAngles()
{
// If you want be averaged of accelerometer data, write (on),else write (off)
#define  GetAvrg  on

#if GetAvrg == on
    getAvrgAccel_Val();
//  Calculate The Angle Of Each Axis
    Accel_Angle[X] = 57.295*atan((float) AvrgAccel_Raw_Val[X] / sqrt(pow((float)AvrgAccel_Raw_Val[Z],2)+pow((float)AvrgAccel_Raw_Val[Y],2)));
    Accel_Angle[Y] = 57.295*atan((float) AvrgAccel_Raw_Val[Y] / sqrt(pow((float)AvrgAccel_Raw_Val[Z],2)+pow((float)AvrgAccel_Raw_Val[X],2)));
    Accel_Angle[Z] = 57.295*atan((float) sqrt(pow((float)AvrgAccel_Raw_Val[X],2)+pow((float)AvrgAccel_Raw_Val[Y],2))/ AvrgAccel_Raw_Val[Z] );
#else
    getAccelVal();
//  Calculate The Angle Of Each Axis
    Accel_Angle[X] = 57.295*atan((float) Accel_Raw_Val[X] / sqrt(pow((float)Accel_Raw_Val[Z],2)+pow((float)Accel_Raw_Val[Y],2)));
    Accel_Angle[Y] = 57.295*atan((float) Accel_Raw_Val[Y] / sqrt(pow((float)Accel_Raw_Val[Z],2)+pow((float)Accel_Raw_Val[X],2)));
    Accel_Angle[Z] = 57.295*atan((float) sqrt(pow((float)Accel_Raw_Val[X],2)+pow((float)Accel_Raw_Val[Y],2))/ Accel_Raw_Val[Z] );
#endif

}
//======================================================================================================================
// get gyro offset X,Y,Z
void MPU6050::getGyroOffset()
{
  #define    NumAve4GO      100

  float Ave = 0;
  unsigned char i = NumAve4GO;
  Gyro_Offset_Val[X] = Gyro_Offset_Val[Y] = Gyro_Offset_Val[Z] = 0;

  while(i--)
  {
    Gyro_Offset_Val[X] = (float)(   (short)((read( RA_GYRO_XOUT_H)<<8)|
                           read( RA_GYRO_XOUT_L)   ));
    Ave = (float) Ave + (Gyro_Offset_Val[X] / NumAve4GO);
    delay.ms(10);
  }
  Gyro_Offset_Val[X] = Ave;
  Gyrothreshold[X]   = ABS(GYRO_SIGMA_MULTIPLE*Ave);
  Ave = 0;
  i = NumAve4GO;
  while(i--)
  {
    Gyro_Offset_Val[Y] = (float)(   (short)((read( RA_GYRO_YOUT_H)<<8)|
                           read( RA_GYRO_YOUT_L)   ));
    Ave = (float) Ave + (Gyro_Offset_Val[Y] / NumAve4GO);
    delay.ms(10);
  }
  Gyro_Offset_Val[Y] = Ave;
  Gyrothreshold[Y]   = ABS(GYRO_SIGMA_MULTIPLE*Ave);
  Ave = 0;
  i = NumAve4GO;
  while(i--)
  {
      Gyro_Offset_Val[Z] = (float)(   (short)((read( RA_GYRO_ZOUT_H)<<8)|
                             read( RA_GYRO_ZOUT_L)   ));
    Ave = (float) Ave + (Gyro_Offset_Val[Z] / NumAve4GO);
    delay.ms(10);
  }
  Gyro_Offset_Val[Z] = Ave;
  Gyrothreshold[Z]   = ABS(GYRO_SIGMA_MULTIPLE*Ave);

}
//======================================================================================================================
// Function to read the gyroscope rate data and convert it into degrees/s
void MPU6050::getGyroVal()
{
    Gyro_Raw_Val[X] = (float)(  (short)(((read( RA_GYRO_XOUT_H)<<8) |
                        read( RA_GYRO_XOUT_L)   ))   );
    Gyro_Raw_Val[Y] = (float)(  (short)(((read( RA_GYRO_YOUT_H)<<8) |
                        read( RA_GYRO_YOUT_L)      ))   );
    Gyro_Raw_Val[Z] = (float)(  (short)(((read( RA_GYRO_ZOUT_H)<<8) |
                        read( RA_GYRO_ZOUT_L)       ))   );
    GyroRate_Val[X] = Gyro_Raw_Val[X] - Gyro_Offset_Val[X];
    GyroRate_Val[Y] = Gyro_Raw_Val[Y] - Gyro_Offset_Val[Y];
    GyroRate_Val[Z] = Gyro_Raw_Val[Z] - Gyro_Offset_Val[Z];
	
	if( ABS(GyroRate_Val[X]) < Gyrothreshold[X]) GyroRate_Val[X]=0;
	if( ABS(GyroRate_Val[Y]) < Gyrothreshold[Y]) GyroRate_Val[Y]=0;
	if( ABS(GyroRate_Val[Z]) < Gyrothreshold[Z]) GyroRate_Val[Z]=0;

    GyroRate_Val[X] = GyroRate_Val[X] / GYRO_Sensitivity;
    GyroRate_Val[Y] = GyroRate_Val[Y] / GYRO_Sensitivity;
    GyroRate_Val[Z] = GyroRate_Val[Z] / GYRO_Sensitivity;

}
//======================================================================================================================
// Function to read the Avrrage of gyroscope rate data and convert it into degrees/s
void MPU6050::getAvrgGyroVal()
{
  #define    NumAve4G      50

  float Ave = 0;
  unsigned char i = NumAve4G;
  AvrgGyro_Raw_Val[X] = AvrgGyro_Raw_Val[Y] = AvrgGyro_Raw_Val[Z] = 0;

  while(i--)
  {
    AvrgGyro_Raw_Val[X] = (float)(   (short)((  read( RA_GYRO_XOUT_H)<<8)|
                            read( RA_GYRO_XOUT_L)   ));
    Ave = (float) Ave + (AvrgGyro_Raw_Val[X] / NumAve4G);
    delay.us(1);
  }
  AvrgGyro_Raw_Val[X] = Ave;
  Ave = 0;
  i = NumAve4G;
  while(i--)
  {
    AvrgGyro_Raw_Val[Y] = (float)(   (short)((  read( RA_GYRO_YOUT_H)<<8)|
                            read( RA_GYRO_YOUT_L)   ));
    Ave = (float) Ave + (AvrgGyro_Raw_Val[Y] / NumAve4G);
    delay.us(1);
  }
  AvrgGyro_Raw_Val[Y] = Ave;
  Ave = 0;
  i = NumAve4G;
  while(i--)
  {
    AvrgGyro_Raw_Val[Z] = (float)(   (short)((  read( RA_GYRO_ZOUT_H)<<8)|
                            read( RA_GYRO_ZOUT_L)   ));
    Ave = (float) Ave + (AvrgGyro_Raw_Val[Z] / NumAve4G);
    delay.us(1);
  }
  AvrgGyro_Raw_Val[Z] = Ave;

  GyroRate_Val[X] = AvrgGyro_Raw_Val[X] - Gyro_Offset_Val[X];
  GyroRate_Val[Y] = AvrgGyro_Raw_Val[Y] - Gyro_Offset_Val[Y];
  GyroRate_Val[Z] = AvrgGyro_Raw_Val[Z] - Gyro_Offset_Val[Z];

  GyroRate_Val[X] = GyroRate_Val[X] / GYRO_Sensitivity;   // (º/s)/LSB
  GyroRate_Val[Y] = GyroRate_Val[Y] / GYRO_Sensitivity;   // (º/s)/LSB
  GyroRate_Val[Z] = GyroRate_Val[Z] / GYRO_Sensitivity;   // (º/s)/LSB

}
//======================================================================================================================
// Function to read the Temperature
void MPU6050::getTempVal()
{
	Temp_Val = (float)(     (short)((  read( RA_TEMP_OUT_H)  << 8  )  |  read( RA_TEMP_OUT_L))       );
	Temp_Val = (Temp_Val /TEMP_Sensitivity) + 36.53;   // Compute the temperature in degrees              
}
//======================================================================================================================
void MPU6050::setLastTime(void)
{
	last_time = timer.millis();
}
//======================================================================================================================
ulong MPU6050::getLastTime(void)
{
	return last_time;
}

