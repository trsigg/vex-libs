/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file as well as coreIncludes.c, parallelDrive.c, PID.c, and timer.c in the same directory as your code

	2. Include this line near the top of your code:
			| #include "pd_autoMove.c"

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


parallel_drive autoDrive;


//turning region
typedef struct {
	int angle; //positive for clockwise, negative for counterclockwise
	int waitAtEnd; //delay after finishing turn (default 100ms for braking)
	int brakePower; //the motor power while braking
	bool isTurning; //whether turn is executing (useful for running as task)
	int direction; //internal variable, sign of angle
	float coeff, offset, exponent; //ramping equation constants
} turnStruct;

turnStruct turnData;

bool turnIsComplete() {
	return abs(gyroVal(autoDrive, RAW)) >= abs(turnData.angle);
}

void turnRuntime() {
	int gyro = abs(gyroVal(autoDrive, RAW));
	int power = turnData.coeff * gyro * pow(turnData.angle - gyro, turnData.exponent);

	setDrivePower(autoDrive, turnData.direction * power, -turnData.direction * power);
}

void turnEnd() {
	//brake
	setDrivePower(autoDrive, -turnData.direction * turnData.brakePower, turnData.direction * turnData.brakePower);
	int brakeDelay = limit(0, 100, turnData.waitAtEnd);
	wait1Msec(brakeDelay);
	setDrivePower(autoDrive, 0, 0);

	wait1Msec(turnData.waitAtEnd);
	turnData.isTurning = false;
}

task turnTask() {
	while (!turnIsComplete()) {
		turnRuntime();
		EndTimeSlice();
	}
	turnEnd();
}

void _turn_(parallel_drive &drive, int angle, float coeff, float offset, bool runAsTask=false, int waitAtEnd=100, int brakePower=10, float exponent=1) { //Internal function. Use at your own risk.
	if (drive.hasGyro) {
		//initialize variables
		autoDrive = drive;
		turnData.angle = abs(angle);
		turnData.waitAtEnd = (waitAtEnd>100 ? waitAtEnd-100 : 0);
		turnData.exponent = exponent;
		turnData.isTurning = true;
		turnData.direction = sgn(angle);

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

void turn(parallel_drive &drive, float angle, angleType angleType=DEGREES, bool runAsTask=false, float initialPower=40, float maxPower=100, int waitAtEnd=100, int brakePower) {
	angle = convertAngle(angle, RAW, angleType);
	float offset = (2*angle*maxPower - angle*initialPower - 2*angle*pow(maxPower*(maxPower-initialPower), 0.5)) / initialPower;
	float coeff = initialPower / (offset * angle);

	_turn_(drive, angle, coeff, offset, runAsTask, waitAtEnd, brakePower);
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
	float coeff, offset, exponent; //constants in equation for determining motor power
	int direction; //sign of distance
	int totalDist; //distance traveled so far
	float error; //calculated from gyro or encoders
	long timer; //for tracking timeout
} driveStruct;

driveStruct driveData;

bool drivingComplete() {
	return abs(driveData.totalDist)<driveData.distance  && time(driveData.timer)<driveData.timeout;
}

void driveStraightRuntime() {
	int power = driveData.coeff * driveData.totalDist * pow(driveData.distance - driveData.totalDist, driveData.exponent);
	float slaveCoeff = 1 + PID_pointerRuntime(pid);

	setDrivePower(autoDrive, slaveCoeff * power * driveData.direction, power * driveData.direction);

	//calculate error value
	if (driveData.correctionType == GYRO) {
		driveData.error = sin(gyroVal(autoDrive, RAW));
	} else if (driveData.correctionType == ENCODER) {
		driveData.error = encoderVal_R(autoDrive, driveData.rawValue) - encoderVal_L(autoDrive, driveData.rawValue);
	} else {
		driveData.error = 0;
	}

	driveData.totalDist += encoderVal(autoDrive);
	if (encoderVal(autoDrive)*100/driveData.sampleTime > driveData.minSpeed) driveData.timer = resetTimer(); //track timeout state
	resetEncoders(autoDrive);
}

void driveStraightEnd() {
	setDrivePower(autoDrive, 0, 0);
	wait1Msec(driveData.waitAtEnd);
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

void _driveStraight_(parallel_drive &drive, int distance, float coeff, float offset, bool runAsTask=false, float kP=0.25, float kI=0.25, float kD=0.25, correctionType correctionType=AUTO, bool rawValue=false, float minSpeed=3, int timeout=800, int waitAtEnd=250, int sampleTime=100, int brakePower=5, float exponent=1) {
	//initialize variables
	autoDrive = drive;
	driveData.distance = abs(distance);
	driveData.direction = sgn(distance);
	driveData.coeff = coeff;
	driveData.offset = offset;
	driveData.rawValue = rawValue;
	driveData.minSpeed = minSpeed/sampleTime;
	driveData.timeout = timeout;
	driveData.waitAtEnd = waitAtEnd;
	driveData.sampleTime = sampleTime;
	driveData.brakePower = brakePower;
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

void driveStraight(parallel_drive &drive, int distance, bool runAsTask=false, float initialPower=40, float maxPower=120, float kP=0.25, float kI=0.25, float kD=0.25, correctionType correctionType=AUTO, bool rawValue=false, float minSpeed=3, int timeout=800, int waitAtEnd=100, int sampleTime=100, int brakePower=10) {
	float offset = (2*distance*maxPower - distance*initialPower - 2*distance*pow(maxPower*(maxPower-initialPower), 0.5)) / initialPower;
	float coeff = initialPower / (offset * distance);

	_driveStraight_(drive, distance, coeff, offset, runAsTask, kP, kI, kD, correctionType, rawValue, minSpeed, timeout, waitAtEnd, sampleTime, brakePower);
}
//end driveStraight region
