/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file in the same directory as your code

	2. Include this line near the top of your code:
			| #include "pd_autoMove.c"

-----------------  FOR TURNING  -----------------
	1. Call turn(driveName, degreesToTurn) where driveName is a parallel_drive object with a gyro attached
	    Optional arguments can be used to configure the drive power during turning, whether to run as a task or function, and the duration of the delay at the end of the turn

	2. The variable turnData.isTurning holds the status of the turn (true if turning, false otherwise)

-----------------  FOR DRIVING  -----------------
	1. Call driveStraight(driveName, clicks) where driveName is a parallel_drive object and clicks is the distance to drive in encoder clicks
		Optional arguments can be used to configure the delay at the end of a drive maneuver, motor power during driving, whether to run as a task or function,
		the timeout duration, correction coefficient, sample time, initial difference in power between master and slave sides of the drive, and sensors used for correction

	2. The variable driveData.isDriving holds the status of the maneuver (true if driving, false otherwise)
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
	return abs(gyroVal(turnData.drive)) >= abs(turnData.degreesToTurn)
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

void turn(parallel_drive &drive, float degreesToTurn, int power=50, bool runAsTask=false, int waitAtEnd=250) {
	if (drive.hasGyro) {
		//initialize variables
		turnData.drive = drive;
		turnData.degreesToTurn = degreesToTurn * 10; //gyro outputs are in degrees*10
		turnData.power = power;
		turnData.waitAtEnd = (waitAtEnd>250 ? waitAtEnd-250 : 0);
		turnData.isTurning = true;
		turnData.direction = sgn(degreesToTurn)

		gyroVal(turnData.drive) = 0; //clear the gyro
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


//driveStraight region
enum correctionType { NONE, GYRO, ENCODER, AUTO };

struct driveData {
	parallel_drive *drive;
	int clicks; //distance to drive, in encoder clicks
	int delayAtEnd; //duration of pause at end of driving
	int power; //motor power while driving
	int timeout; //amount of time after which a drive action times out (ceases)
	float coeff; //correction coefficient, controls how agressively drive reacts to errors
	int sampleTime; //time between motor power adjustments
	correctionType correctionType; //which sensor inputs are used for correction
	bool isDriving; //whether driving action is being executed (true if driving, false othrewise)
	//interal variables
	int direction; //sign of clicks
	int totalClicks; //distance traveled so far
	int slavePower; //power of right side of drive
	int error; //calculated from gyro or encoders
};


void driveStraightRuntime() {
	setDrivePower(driveData.drive, driveData.slavePower * driveData.direction, driveData.power * driveData.direction);

	if (driveData.correctionType == GYRO) {
		driveData.error = gyroVal(driveData.drive);
	} else if (driveData.correctionType == ENCODER) {
		driveData.error = encoderVal_R(driveData.drive) - encoderVal_L(driveData.drive);
	} else {
		driveData.error = 0;
	}

	driveData.slavePower += driveData.error * driveData.direction / driveData.coeff;

	driveData.totalClicks += encoderVal(driveData.drive);
	clearEncoders(driveData.drive);
}

void driveStraightEnd() {
	setDrivePower(driveData.drive, 0, 0);
	wait1Msec(driveData.delayAtEnd);
	driveData.isDriving = false;
}

task driveStraightTask() {
	while (abs(totalClicks) < clicks  && time1(driveTimer) < timeout) {
		driveStraightRuntime();

		wait1Msec(driveData.sampleTime);
	}
	driveStraightEnd();
}

void setCorrectionType(type) {
	if (type==GYRO && driveData.drive.hasGyro) {
		driveData.correctionType = GYRO;
	} else if (type==ENCODER && driveData.drive.hasEncoderL && driveData.drive.hasEncoderR) {
		driveData.correctionType = ENCODER;
	} else {
		driveData.correctionType = NONE;
	}
}

void driveStraight(parallel_drive &drive, int clicks, int delayAtEnd=250, int power=60, bool startAsTask=false, int timeout=15000, coeff=300, int sampleTime=100, int powDiff=5, correctionType correctionType=AUTO) {
	//initialize variables
	driveData.drive = drive;
	driveData.clicks = abs(clicks);
	driveData.direction = sgn(clicks);
	driveData.power = power;
	driveData.delayAtEnd = delayAtEnd;
	driveData.timeout = timeout;
	driveData.coeff = coeff;
	driveData.sampleTime = sampleTime;
	driveData.isDriving = true;

	driveData.totalClicks = 0;
	driveData.slavePower = power - powDiff;
	driveData.error = 0;

	if (correctionType == AUTO) {
		setCorrectionType(GYRO);

		if (driveData.correctionType == NONE) {
			setCorrectionType(ENCODER)
		}
	} else {
		setCorrectionType(correctionType);
	}

	//initialize sensors
	clearEncoders(driveData.drive);
	clearGyro(driveData.drive);

	clearTimer(driveTimer);

	if (startAsTask) {
		startTask(driveStraightTask);
	}
	else { //runs as function
		while (abs(totalClicks) < clicks  && time1(driveTimer) < timeout) {
			driveStraightRuntime();
			wait1Msec(driveData.sampleTime);
		}
		driveStraightEnd();
	}
}
//end driveStraight region
