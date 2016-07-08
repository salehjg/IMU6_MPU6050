/***************************************************
  This is a library for IMU.

  ------> http://www.easymcu.ir
  
  Check out the links above for our tutorials and wiring diagrams
  
  Written by Sebastian Madgwick. 
  Ported for EasyMCU in C++ by Morteza Zandi + some modifications.
  All text above must be included in any redistribution
 ****************************************************/
 
	#include"imu.h"

	IMU::IMU()
	{
			q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	
	}
		
	void IMU::init(void)
	{
	}
	
	//---------------------------------------------------------------------
	
		void IMU::calculate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float hx, hy;
		float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

		// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
//		if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
//			MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
//			return;
//		}

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = InvSqrt_Opt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Normalise magnetometer measurement
			recipNorm = InvSqrt_Opt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0f * q0 * mx;
			_2q0my = 2.0f * q0 * my;
			_2q0mz = 2.0f * q0 * mz;
			_2q1mx = 2.0f * q1 * mx;
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_2q0q2 = 2.0f * q0 * q2;
			_2q2q3 = 2.0f * q2 * q3;
			q0q0 = q0 * q0;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q1 = q1 * q1;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q2 = q2 * q2;
			q2q3 = q2 * q3;
			q3q3 = q3 * q3;

			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = sqrt(hx * hx + hy * hy);
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;

			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			recipNorm = InvSqrt_Opt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalize step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
//		q0 += qDot1 * ((float)1.0f / 200);		//sampleFreq
//		q1 += qDot2 * ((float)1.0f / 200);
//		q2 += qDot3 * ((float)1.0f / 200);
//		q3 += qDot4 * ((float)1.0f / 200);
		
//		q0 += qDot1 * (0.0248);
//		q1 += qDot2 * (0.0248);
//		q2 += qDot3 * (0.0248);
//		q3 += qDot4 * (0.0248);		
		
		q0 += qDot1 * (interval/1000.0f);
		q1 += qDot2 * (interval/1000.0f);
		q2 += qDot3 * (interval/1000.0f);
		q3 += qDot4 * (interval/1000.0f);			

		// Normalize quaternion
		recipNorm = InvSqrt_Opt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
	}

	//---------------------------------------------------------------------------------------------------


	void IMU::calculate(float gx, float gy, float gz, float ax, float ay, float az) {
		float recipNorm, acc_weight=0;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalize accelerometer measurement
			recipNorm = InvSqrt_Opt(ax * ax + ay * ay + az * az);

			acc_weight = 1-(2 * ABS(1-recipNorm));
			if((acc_weight >= 0) && (acc_weight <= 1)) /*{ acc_weight = 0; acc_div = 0; }*/
					 {				
						
						ax *= recipNorm;
						ay *= recipNorm;
						az *= recipNorm;
						 
						 	HADXL_P[0] = ax;
						  HADXL_P[1] = ay;
						  HADXL_P[2] = az;
					 }
					 else
					 {
						 ax = HADXL_P[0];
						 ay = HADXL_P[1];
						 az = HADXL_P[2];
					 }

						// Auxiliary variables to avoid repeated arithmetic
						_2q0 = 2.0f * q0;
						_2q1 = 2.0f * q1;
						_2q2 = 2.0f * q2;
						_2q3 = 2.0f * q3;
						_4q0 = 4.0f * q0;
						_4q1 = 4.0f * q1;
						_4q2 = 4.0f * q2;
						_8q1 = 8.0f * q1;
						_8q2 = 8.0f * q2;
						q0q0 = q0 * q0;
						q1q1 = q1 * q1;
						q2q2 = q2 * q2;
						q3q3 = q3 * q3;

						// Gradient decent algorithm corrective step
						s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
						s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
						s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
						s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
						recipNorm = InvSqrt_Opt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalize step magnitude
						s0 *= recipNorm;
						s1 *= recipNorm;
						s2 *= recipNorm;
						s3 *= recipNorm;

						// Apply feedback step
						qDot1 -= beta * s0;
						qDot2 -= beta * s1;
						qDot3 -= beta * s2;
						qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
//		q0 += qDot1 * ((float)1.0f / sampleFreq);
//		q1 += qDot2 * ((float)1.0f / sampleFreq);
//		q2 += qDot3 * ((float)1.0f / sampleFreq);
//		q3 += qDot4 * ((float)1.0f / sampleFreq);
		
//		q0 += qDot1 * (0.0248);
//		q1 += qDot2 * (0.0248);
//		q2 += qDot3 * (0.0248);
//		q3 += qDot4 * (0.0248);	
		if(interval > 200)	// because of first time or for error time
			interval = 150;
		
		q0 += qDot1 * (interval/1000.0f);
		q1 += qDot2 * (interval/1000.0f);
		q2 += qDot3 * (interval/1000.0f);
		q3 += qDot4 * (interval/1000.0f);		

		// Normalize quaternion
		recipNorm = InvSqrt_Opt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
	}

	//---------------------------------------------------------------------------------------------------
	// Fast inverse square-root
	// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

	float IMU::invSqrt(float x) {
		float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		return y;
	}
	
	float IMU::InvSqrt_Opt(float x){		//reverse
		long i = 0x5F1F1412 - (*(long*)&x >> 1);
		float tmp = *((float*)&i);
		return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
	}

	/*
	* get Euler angles
	* aerospace sequence, to obtain sensor attitude:
	* 1. rotate around sensor Z plane by yaw
	* 2. rotate around sensor Y plane by pitch
	* 3. rotate around sensor X plane by roll
	*/
	void IMU::update(void) {	//float *roll, float *pitch, float *yaw
		
		roll=  atan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2))*RAD2DEG;
		pitch =  asin(2 * (q0 * q2 - q3 * q1))*RAD2DEG*(-1);
		yaw =  (atan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3))*RAD2DEG*(-1));
	}

/*******************************************************************/
	