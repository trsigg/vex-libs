/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file in the same directory as your code

	2. Include this line near the top of your code:
			| #include "pd_autoMove.c"

	3. To turn, call turn(driveName, degreesToTurn) where driveName is a parallel_drive object with a gyro attached
	    Optional arguments can be used to configure the drive power during turning, whether to run as a task or function, and the duration of the delay at the end of the turn

	4. The variable turnData.isTurning holds the status of the turn (true if turning, false otherwise)
*/

#include "parallelDrive.c"

int limit(int input, int min, int max) {
	if (input <= max && input >= min) {
		return input;
	}
	else {
		return (input > max ? max : min);
	}
}


//turning region
struct turnData {
	parallel_drive *drive;
	int degreesToTurn; //positive for clockwise, negative for counterclockwise
	int power; //motor power during turning
	int waitAtEnd; //delay after turning (default 250ms for braking)
	bool isTurning; //whether turn is executing (useful for running as task)
	int direction; //internal variable (sign of degreesToTurn)
};

bool turnIsComplete() {
	return abs(SensorValue[turnData.drive.gyro]) >= abs(turnData.degreesToTurn)
}

void turnEnd() {
	//brake
	setDrivePower(turnData.drive, -turnData.direction * 10, turnData.direction * 10); //might need to be *(turnData.drive). I dunno.
	int brakeDelay = limit(0, 250, turnData.waitAtEnd);
	wait1Msec(brakeDelay);
	setDrivePower(turnData.drive, 0, 0);

	turnData.isTurning = false;
	wait1Msec(turnData.waitAtEnd);
}

task turnTask() {
	while (!turnIsComplete()) { EndTimeSlice(); }
	turnEnd();
}

void turn(parallel_drive &drive, float _degreesToTurn_, int _power_=50, bool runAsTask=false, int _waitAtEnd_=250) {
	if (drive.hasGyro) {
		turnData.drive = drive;
		turnData.degreesToTurn = _degreesToTurn_ * 10; //gyro outputs are in degrees*10
		turnData.power = _power_;
		turnData.waitAtEnd = (_waitAtEnd_>250 ? _waitAtEnd_-250 : 0);
		turnData.isTurning = true;
		turnData.direction = sgn(_degreesToTurn_)

		SensorValue[ turnData.drive.gyro ] = 0; //clear the gyro
		setDrivePower(turnData.drive, turnData.direction*turnData.power, -turnData.direction*turnData.power); //begin turn

		if (runAsTask) {
			startTask(turnTask);
		}
		else {
			while (!turnIsComplete()) { EndTimeSlice(); }
			turnEnd();
		}
	}
}
//end turning region
