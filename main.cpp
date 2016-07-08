 /***************************************************
  6-DOF IMU - v1.0

  ------> http://www.easymcu.ir
  
  Check out the links above for our tutorials and wiring diagrams
  
  Redistributed by Saleh Jamali(SalehJG) for EasyMCU
  All text above must be included in any redistribution
 ****************************************************/



#include "headers.h"

IMU imu;
MPU6050 		* mpu = new MPU6050(1);

#define TRACE(x)		serial1.print(x);
#define TRACEln(x)	serial1.println(x);


int main()
{
	int ii;

	//-----------------------------------------------------------------------------------------
		//unsigned long baudrate = 9600, uint8_t databits = 8, uint8_t parity = 0, uint8_t stopbits = 1
	serial1.init(115200,8,0,1);
	TRACE("Balancing Robot V1.0\n");
	
	
	TRACE((mpu->test()==0) ? "i2c has failed...\n\n" : "MPU Project V1.0\n\n");
	TRACE("\t=>Restarting MPU6050...");
	mpu->reset();
	TRACE((mpu->test()==0) ? "i2c has failed...\n" : "OK\n");
	//-----------------------------------------------------------------------------------------
	
	mpu->init();
	
	for(ii=0;ii<10;ii++)
	{
		io.toggle(LED3);
		delay.ms(100);
	}
	
	//mpu->Get_Accel_Offset();
	mpu->getGyroOffset();	
	mpu->getAccelVal();
	
	imu.HADXL_P[0] = mpu->Accel_In_g[X];		//x;
	imu.HADXL_P[1] = mpu->Accel_In_g[Y];		//y;
	imu.HADXL_P[2] = mpu->Accel_In_g[Z];		//z;
	
	for(ii=0;ii<10;ii++)
	{
		io.toggle(LED3);
		delay.ms(50);
	}
	
	
	
	
	uint32_t routin_100HZ_start = timer.millis();
	uint32_t routin_90ms_start = timer.millis();
	while(1)
	{
		if(timer.deltaTmillis(routin_100HZ_start)>9) 	// --------> execute every 10ms -> 100Hz routin
		{
			routin_100HZ_start = timer.millis();	

			mpu->getAccelVal();
			imu.interval = 10;//timer.deltaTmillis(gyro.getLastTime());
			mpu->getGyroVal();


			imu.calculate(
										radians(mpu->GyroRate_Val[X]), 
										radians(mpu->GyroRate_Val[Y]), 
										radians(mpu->GyroRate_Val[Z]), 
										mpu->Accel_In_g[X], 
										mpu->Accel_In_g[Y],
										mpu->Accel_In_g[Z]
									 );					
			imu.update();
			
		} // End of 100Hz routin				
		/*******************************************************************/
		if(timer.deltaTmillis(routin_90ms_start)>150) 	// --------> execute every 10ms -> 100Hz routin
		{					
			TRACE("Interval: "); 			TRACEln(imu.interval);						
			TRACE("Roll:     ");			TRACEln(imu.roll);
			TRACE("Pitch:    ");			TRACEln(imu.pitch);
			TRACE("Yaw:      ");			TRACEln(imu.yaw);		
			TRACEln();	

			routin_90ms_start = millis();
		}
	}
}



