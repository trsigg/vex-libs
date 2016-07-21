#pragma config(Sensor, in1,    Yaw,            sensorGyro)
#pragma config(Sensor, dgtl3,  leftE,          sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  rightE,         sensorQuadEncoder)
#pragma config(Motor,  port2,           lfd,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           lbd,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           rfd,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           rbd,           tmotorVex393_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "parallelDrive.c"

parallel_drive drive;
robotPosition position;

float gyroError = 0;
float gyro = 0;

task main() {
	initializeDrive(drive);
	setLeftMotors(drive, 2, lfd, lbd);
	setRightMotors(drive, 2, rfd, rbd);
	attachGyro(drive, Yaw);
	attachEncoderL(drive, leftE);
	attachEncoderR(drive, rightE, true);

	while (true) {
		driveRuntime(drive);
		position = *updatePosition(drive);

		gyroError = gyroVal(drive, RADIANS) - drive.position.theta;
		gyro = gyroVal(drive, RADIANS);

		if (vexRT[Btn5D] == 1) drive.width = calculateWidth(drive);

		if (vexRT[Btn6D] == 1) {
			setRobotPosition(drive, 0, 0, 0);
			resetGyro(drive);
		}
	}
}
