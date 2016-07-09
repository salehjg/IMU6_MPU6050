/***************************************************
  This is a library for IMU.

  ------> http://www.easymcu.ir
  
  Check out the links above for our tutorials and wiring diagrams
  
  Written by Sebastian Madgwick. 
  Ported for EasyMCU in C++ by Morteza Zandi + some modifications.
  All text above must be included in any redistribution
	
	BSD license, All text above must be included in any redistribution
 ****************************************************/
 
#ifndef __IMU_H__
#define __IMU_H__

#include "imu.h"
#include "type.h"
#include <math.h>

//was 0.3
#define beta 0.3f
#define RAD2DEG (57.2957795130823208768)


	class IMU
	{
		public:
			IMU();
			void init(void);
			void calculate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
			void calculate(float gx, float gy, float gz, float ax, float ay, float az);
			void update(void);	//float *roll, float *pitch, float *yaw
			float interval;
			//float beta;				// algorithm gain
			float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
			float HADXL_P[3];
			
			float invSqrt(float x);
			float InvSqrt_Opt(float x);	
		
			float roll;
			float pitch;
			float yaw;		
	};


#endif
