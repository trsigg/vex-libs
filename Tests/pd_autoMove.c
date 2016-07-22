/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file as well as coreIncludes.c, parallelDrive.c, PID.c, and timer.c in the same directory as your code

	2. Include this line near the top of your code:
			| #include "pd_autoMove.c"

-----------------  FOR TURNING  -----------------
	1. Call turn(driveName, angle) where driveName is a parallel_drive object with a gyro attached
	    Optional arguments can be used to configure the angle input type, whether to run as a task or function,
	    the initial and maximum motor powers duting the maneuver, and the duration of the delay at the end of the turn

	2. The variable turnData.isTurning holds the status of the turn (true if turning, false otherwise)

-----------------  FOR DRIVING  -----------------
	1. Call driveStraight(driveName, clicks) where driveName is a parallel_drive object and clicks is the distance to drive in encoder clicks
		Optional arguments can be used to configure the delay at the end of a drive maneuver, motor power during driving, whether to run as a task or function,
		the timeout duration, correction coefficient, sample time, initial difference in power between master and slave sides of the drive, and sensors used for correction

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
	int clicks; //distance to drive, in encoder clicks
	int delayAtEnd; //duration of pause at end of driving
	int power; //motor power while driving
	int minSpeed; //minimum speed during maneuver to prevent timeout (encoder clicks per 100ms)
	int timeout; //amount of time after which a drive action sensing lower speed than minSpeed ceases (ms)
	float coeff; //correction coefficient, controls how agressively drive reacts to errors
	int sampleTime; //time between motor power adjustments
	correctionType correctionType; //which sensor inputs are used for correction
	bool isDriving; //whether driving action is being executed (true if driving, false othrewise)
	//interal variables
	int direction; //sign of clicks
	int totalClicks; //distance traveled so far
	int slavePower; //power of right side of drive
	int error; //calculated from gyro or encoders
	long timer; //for tracking timeout
} driveStruct;

driveStruct driveData;

bool drivingComplete() {
	return abs(driveData.totalClicks)<driveData.clicks  && time(driveData.timer)<driveData.timeout;
}

void driveStraightRuntime() {
	setDrivePower(autoDrive, driveData.slavePower * driveData.direction, driveData.power * driveData.direction);

	//calculate error value
	if (driveData.correctionType == GYRO) {
		driveData.error = gyroVal(autoDrive, RAW);
	} else if (driveData.correctionType == ENCODER) {
		driveData.error = encoderVal_R(autoDrive) - encoderVal_L(autoDrive);
	} else {
		driveData.error = 0;
	}

	//adjust slavePower based on error
	driveData.slavePower += driveData.error * driveData.direction / driveData.coeff;

	driveData.totalClicks += encoderVal(autoDrive);
	if (encoderVal(autoDrive)*100/driveData.sampleTime > driveData.minSpeed) driveData.timer = resetTimer(); //track timeout state
	resetEncoders(autoDrive);
}

void driveStraightEnd() {
	setDrivePower(autoDrive, 0, 0);
	wait1Msec(driveData.delayAtEnd);
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

void driveStraight(parallel_drive &drive, int clicks, int delayAtEnd=250, int power=60, bool startAsTask=false, int minSpeed=20, int timeout=800, float coeff=300, int sampleTime=100, int powDiff=5, correctionType correctionType=AUTO) {
	//initialize variables
	autoDrive = drive;
	driveData.clicks = abs(clicks);
	driveData.direction = sgn(clicks);
	driveData.power = power;
	driveData.delayAtEnd = delayAtEnd;
	driveData.minSpeed = minSpeed;
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
			setCorrectionType(ENCODER);
		}
	} else {
		setCorrectionType(correctionType);
	}

	//initialize sensors
	resetEncoders(autoDrive);
	resetGyro(autoDrive);

	driveData.timer = resetTimer();

	if (startAsTask) {
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
//end driveStraight region
