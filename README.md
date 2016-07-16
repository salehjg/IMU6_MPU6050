# IMU6_MPU6050
6-DOF IMU based on MPU6050 &amp; Zcross Board

/!\ UART1(serial1) used as an output for printing IMU data.

I2C1:
  P11 = SDA
  P12 = SCL

	
SERIAL1:
  P21 = TX
  P22 = RX

	
Quick start:
	1.Create a raw EMCU project for Zcross board.
	2.Copy content of "module_libraries" folder to <EMCU_DIR>\module_libraries
	3.Replace config.h, modules_lib.h, main.cpp with provided source files in the repository.

	
