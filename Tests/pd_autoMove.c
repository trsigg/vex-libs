/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file as well as coreIncludes.c, parallelDrive.c, PID.c, and timer.c in the same directory as your code

	2. Include this line near the top of your code:
			| #include "pd_autoMove.c"

	3. Do not create a parallel_drive object in your code as normal, instead substitute driveName in the define statement below with the name of the drive.
		Configure the drive as normal. (still call initializeDrive(), attach motors, etc.) */

#define autoDrive drive

/*
-----------------  FOR TURNING  -----------------
	1. Call turn(driveName, angle) where driveName is a parallel_drive object with a gyro attached
	    Optional arguments can be used to configure the angle input type, whether to run as a task or function,
	    the initial and maximum motor powers during the maneuver, and the duration of the delay at the end of the turn

	2. The variable turnData.isTurning holds the status of the turn (true if turning, false otherwise)

-----------------  FOR DRIVING  -----------------
	1. Call driveStraight(driveName, distance) where driveName is a parallel_drive object and distance is in either encoder clicks or inches
		Optional arguments can be used to configure whether to run as a task or function, the initial and maximum motor powers during the maneuver, PID correction coefficients, sensors used for correction,
		whether distance is measured in inches or encoder clicks, the minimum speed necessary to avoid a timeout (distance/second), the timeout duration, the delay at the end of a drive maneuver, sample time, and brake power

	2. The variable driveData.isDriving holds the status of the maneuver (true if driving, false otherwise)

	Note: the functions _turn_() and _driveStraight_() are much less user friendly, but can be used in place of turn() and driveStraight() to more finely configure robot behavior
*/

#include "coreIncludes.c"
#include "parallelDrive.c"
#include "PID.c"
#include "timer.c"

#define TURN_BRAKE_DURATION 100 //maximum duration of braking at end of turn
#define DRIVE_BRAKE_POWER 30 //power used during driveStraight braking
#define DRIVE_BRAKE_DURATION 100 //maximum duration of braking at end of driveStraight

parallel_drive autoDrive;


//turning region
typedef struct {
	float angle; //positive for clockwise, negative for counterclockwise
	int waitAtEnd; //delay after finishing turn (default 100ms for braking)
	int brakePower; //the motor power while braking
	bool isTurning; //whether turn is executing (useful for running as task)
	float a, b, c; //ramping equation constants
	int direction; //sign of angle
} turnStruct;

turnStruct turnData;

bool turnIsComplete() {
	return /*abs(gyroVal(autoDrive, DEGREES)) >= abs(turnData.angle)*/false;
}

void turnRuntime() {
	int gyro = abs(gyroVal(autoDrive, DEGREES));
	int power = turnData.a*pow(gyro, 2) +  turnData.b*gyro + turnData.c;

	setDrivePower(autoDrive, turnData.direction*power, -turnData.direction*power);
}

void turnEnd() {
	//brake
	setDrivePower(autoDrive, -turnData.direction * turnData.brakePower, turnData.direction * turnData.brakePower);
	int brakeDelay = limit(0, TURN_BRAKE_DURATION, turnData.waitAtEnd);
	wait1Msec(brakeDelay);
	setDrivePower(autoDrive, 0, 0);

	wait1Msec(turnData.waitAtEnd>TURN_BRAKE_DURATION ? turnData.waitAtEnd-TURN_BRAKE_DURATION : 0);
	turnData.isTurning = false;
}

task turnTask() {
	while (!turnIsComplete()) {
		turnRuntime();
		EndTimeSlice();
	}
	turnEnd();
}

void _turn_(parallel_drive &drive, float angle, float a, float b, float c, bool runAsTask=false, int waitAtEnd=100, int brakePower=20) { //Internal function. Use at your own risk.
	if (drive.hasGyro) {
		//initialize variables
		autoDrive = drive;
		turnData.angle = abs(angle);
		turnData.a = a;
		turnData.b = b;
		turnData.c = c;
		turnData.direction = sgn(angle);
		turnData.waitAtEnd = waitAtEnd;
		turnData.isTurning = true;

		resetGyro(autoDrive);

		if (runAsTask) {
			startTask(turnTask);
		}
		else {
			while (!turnIsComplete()) { turnRuntime(); }
			turnEnd();
		}
	}
}

void turn(parallel_drive &drive, float angle, angleType angleType=DEGREES, bool runAsTask=false, int initialPower=40, int maxPower=100, int finalPower=0, int waitAtEnd=100, int brakePower=20) {
	angle = convertAngle(angle, DEGREES, angleType);
	float a = (pow(angle, 2) * (finalPower+initialPower-2*maxPower) - 2*pow(exp(4 * log(angle)) * (finalPower-maxPower) * (initialPower-maxPower), 0.5)) / pow(angle, 4);
	float b = (finalPower-initialPower)/angle - a*angle;

	_turn_(drive, angle, a, b, initialPower, runAsTask, waitAtEnd, brakePower);
}
//end turning region


//driveStraight region
typedef enum correctionType { NONE, GYRO, ENCODER, AUTO };

typedef struct {
	float distance;
	bool rawValue; //whether distance is measured in encoder clicks or inches
	float minSpeed; //minimum speed during maneuver to prevent timeout (distance per 100ms)
	int timeout; //amount of time after which a drive action sensing lower speed than minSpeed ceases (ms)
	int sampleTime; //time between motor power adjustments
	int waitAtEnd; //duration of pause at end of driving
	int brakePower; // motor power during braking
	correctionType correctionType; //which sensor inputs are used for correction
	bool isDriving; //whether driving action is being executed (true if driving, false othrewise)
	//interal variables
	PID pid;
	float a, b, c; //constants in equation for determining motor power
	float totalDist; //distance traveled so far
	int direction; //sign of distance
	float error; //calculated from gyro or encoders
	long timer; //for tracking timeout
} driveStruct;

driveStruct driveData;

bool drivingComplete() {
	return abs(driveData.totalDist)>driveData.distance  || time(driveData.timer)>driveData.timeout;
}

void driveStraightRuntime() {
	int power = driveData.a*pow(driveData.totalDist, 2) + driveData.b*driveData.totalDist + driveData.c;
	float slaveCoeff = 1 + PID_pointerRuntime(driveData.pid);

	setDrivePower(autoDrive, slaveCoeff*driveData.direction*power, driveData.direction*power);

	float leftDist = encoderVal_L(autoDrive, driveData.rawValue);
	float rightDist = encoderVal_R(autoDrive, driveData.rawValue);

	//calculate error value
	if (driveData.correctionType == GYRO) {
		driveData.error = sin(gyroVal(autoDrive, RADIANS));
	} else if (driveData.correctionType == ENCODER) {
		driveData.error = rightDist - leftDist;
	} else {
		driveData.error = 0;
	}

	driveData.totalDist += (leftDist + rightDist) / 2;
	if (encoderVal(autoDrive) > driveData.minSpeed) driveData.timer = resetTimer(); //track timeout state
	resetEncoders(autoDrive);
}

void driveStraightEnd() {
	//brake
	setDrivePower(autoDrive, -driveData.direction * DRIVE_BRAKE_POWER, -driveData.direction * DRIVE_BRAKE_POWER);
	int brakeDelay = limit(0, DRIVE_BRAKE_DURATION, driveData.waitAtEnd);
	wait1Msec(brakeDelay);
	setDrivePower(autoDrive, 0, 0);

	wait1Msec(driveData.waitAtEnd>DRIVE_BRAKE_DURATION ? driveData.waitAtEnd-DRIVE_BRAKE_DURATION : 0);
	driveData.isDriving = false;
}

task driveStraightTask() {
	while (!drivingComplete()) {
		driveStraightRuntime();

		wait1Msec(driveData.sampleTime);
	}
	driveStraightEnd();
}

void setCorrectionType(correctionType type) {
	if (type==GYRO && autoDrive.hasGyro) {
		driveData.correctionType = GYRO;
	} else if (type==ENCODER && autoDrive.hasEncoderL && autoDrive.hasEncoderR) {
		driveData.correctionType = ENCODER;
	} else {
		driveData.correctionType = NONE;
	}
}

void _driveStraight_(parallel_drive &drive, float distance, float a, float b, float c, bool runAsTask=false, float kP=0.25, float kI=0.25, float kD=0.25, correctionType correctionType=AUTO, bool rawValue=false, float minSpeed=3, int timeout=800, int waitAtEnd=250, int sampleTime=50) {
	//initialize variables
	autoDrive = drive;
	driveData.distance = abs(distance);
	driveData.a = a;
	driveData.b = b;
	driveData.c = c;
	driveData.direction = sgn(distance);
	driveData.rawValue = rawValue;
	driveData.minSpeed = minSpeed * sampleTime / 1000;
	driveData.timeout = timeout;
	driveData.waitAtEnd = waitAtEnd;
	driveData.sampleTime = sampleTime;
	driveData.isDriving = true;
	//configure PID
	driveData.pid.input = &(driveData.error);
	driveData.pid.kP = kP;
	driveData.pid.kI = kI;
	driveData.pid.kD = kD;
	driveData.pid.target = 0;

	driveData.totalDist = 0;
	driveData.error = 0;

	if (correctionType == AUTO) {
		setCorrectionType(ENCODER);

		if (driveData.correctionType == NONE) {
			setCorrectionType(GYRO);
		}
	} else {
		setCorrectionType(correctionType);
	}

	//initialize sensors
	resetEncoders(autoDrive);
	resetGyro(autoDrive);

	driveData.timer = resetTimer();

	if (runAsTask) {
		startTask(driveStraightTask);
	}
	else { //runs as function
		while (!drivingComplete()) {
			driveStraightRuntime();
			wait1Msec(driveData.sampleTime);
		}
		driveStraightEnd();
	}
}

void driveStraight(parallel_drive &drive, float distance, bool runAsTask=false, int initialPower=40, int maxPower=120, int finalPower=0, float kP=0.25, float kI=0.25, float kD=0.25, correctionType correctionType=AUTO, bool rawValue=false, float minSpeed=3, int timeout=60000, int waitAtEnd=100, int sampleTime=50) {
	float a = (pow(distance, 2) * (finalPower+initialPower-2*maxPower) - 2*pow(exp(4 * log(distance)) * (finalPower-maxPower) * (initialPower-maxPower), 0.5)) / pow(distance, 4);
	float b = (finalPower-initialPower)/distance - a*distance;

	_driveStraight_(drive, distance, a, b, initialPower, runAsTask, kP, kI, kD, correctionType, rawValue, minSpeed, timeout, waitAtEnd, sampleTime);
}
//end driveStraight region
