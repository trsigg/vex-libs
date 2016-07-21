/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file as well as coreIncludes.c, parallelDrive.c, PID.c, and timer.c in the same directory as your code

	2. Include this line near the top of your code:
			| #include "pd_autoMove.c"

-----------------  FOR TURNING  -----------------
	1. Call turn(driveName, degreesToTurn) where driveName is a parallel_drive object with a gyro attached
	    Optional arguments can be used to configure whether to run as a task or function, the duration of the delay at the end of the turn, and the equation used to determine motor powers throughout the maneuver

	2. The variable turnData.isTurning holds the status of the turn (true if turning, false otherwise)

-----------------  FOR DRIVING  -----------------
	1. Call driveStraight(driveName, clicks) where driveName is a parallel_drive object and clicks is the distance to drive in encoder clicks
		Optional arguments can be used to configure the delay at the end of a drive maneuver, motor power during driving, whether to run as a task or function,
		the timeout duration, correction coefficient, sample time, initial difference in power between master and slave sides of the drive, and sensors used for correction

	2. The variable driveData.isDriving holds the status of the maneuver (true if driving, false otherwise)
*/

#include "coreIncludes.c"
#include "parallelDrive.c"
#include "PID.c"
#include "timer.c"


//turning region
struct turnData {
	struct parallel_drive *drive;
	int degreesToTurn; //positive for clockwise, negative for counterclockwise
	int waitAtEnd; //delay after finishing turn (default 100ms for braking)
	float coeff; //coefficient in ramping equation. Use a negative input to have it autocalculated based on a motor power value instead (it will be set to a value such that the maximum motor power during the turn is the value provided)
	float exponent; //exponent in ramping equation.  Higher values mean drive accelerates faster and slows more gradually, and vice-versa
	bool isTurning; //whether turn is executing (useful for running as task)
	int direction; //internal variable, sign of degreesToTurn
};

bool turnIsComplete() {
	return abs(gyroVal(turnData.drive, RAW)) >= abs(turnData.degreesToTurn); //might need to be &turnData.drive. I dunno.
}

void turnRuntime() {
	int gyro = abs(gyroVal(turnData.drive, RAW));
	int power = turnData.coeff * gyro * pow(turnData.degreesToTurn - gyro, turnData.exponent);

	setDrivePower(turnData.drive, turnData.direction * power, -direction * power);
}

void turnEnd() {
	//brake
	setDrivePower(turnData.drive, -turnData.direction * 10, turnData.direction * 10);
	int brakeDelay = limit(0, 100, turnData.waitAtEnd);
	wait1Msec(brakeDelay);
	setDrivePower(turnData.drive, 0, 0);

	turnData.isTurning = false;
	wait1Msec(turnData.waitAtEnd);
}

task turnTask() {
	while (!turnIsComplete()) {
		turnRuntime();
		EndTimeSlice();
	}
	turnEnd();
}

void turn(parallel_drive &drive, float degreesToTurn, bool runAsTask=false, int waitAtEnd=100, float coeff=-115, float exponent=1) {
	if (drive.hasGyro) {
		//initialize variables
		turnData.drive = drive;
		turnData.degreesToTurn = abs(degreesToTurn * 10); //gyro outputs are in degrees*10
		turnData.waitAtEnd = (waitAtEnd>100 ? waitAtEnd-100 : 0);
		turnData.exponent = exponent;
		turnData.isTurning = true;
		turnData.direction = sgn(degreesToTurn);

		//set coefficient
		if (coeff < 0) {
			turnData.coeff = -coeff * pow(2/turnData.degreesToTurn, exponent+1); //solve for coefficient which produces specified maximum motor value
		} else {
			turnData.coeff = coeff;
		}

		clearGyro(turnData.drive);

		if (runAsTask) {
			startTask(turnTask);
		}
		else {
			while (!turnIsComplete()) { turnRuntime(); }
			turnEnd();
		}
	}
}
//end turning region


//driveStraight region
typedef enum correctionType { NONE, GYRO, ENCODER, AUTO };

struct driveData {
	parallel_drive *drive;
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
};

bool drivingComplete() {
	return abs(driveData.totalClicks)<driveData.clicks  && time(driveData.timer)<driveData.timeout;
}

void driveStraightRuntime() {
	setDrivePower(driveData.drive, driveData.slavePower * driveData.direction, driveData.power * driveData.direction);

	//calculate error value
	if (driveData.correctionType == GYRO) {
		driveData.error = gyroVal(driveData.drive, RAW);
	} else if (driveData.correctionType == ENCODER) {
		driveData.error = encoderVal_R(driveData.drive) - encoderVal_L(driveData.drive);
	} else {
		driveData.error = 0;
	}

	//adjust slavePower based on error
	driveData.slavePower += driveData.error * driveData.direction / driveData.coeff;

	driveData.totalClicks += encoderVal(driveData.drive);
	if (encoderVal(driveData.drive)*100/driveData.sampleTime > driveData.minSpeed) driveData.timer = resetTimer(); //track timeout state
	clearEncoders(driveData.drive);
}

void driveStraightEnd() {
	setDrivePower(driveData.drive, 0, 0);
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
	if (type==GYRO && driveData.drive->hasGyro) {
		driveData.correctionType = GYRO;
	} else if (type==ENCODER && driveData.drive->hasEncoderL && driveData.drive->hasEncoderR) {
		driveData.correctionType = ENCODER;
	} else {
		driveData.correctionType = NONE;
	}
}

void driveStraight(parallel_drive &drive, int clicks, int delayAtEnd=250, int power=60, bool startAsTask=false, int minSpeed=20, int timeout=800, float coeff=300, int sampleTime=100, int powDiff=5, correctionType correctionType=AUTO) {
	//initialize variables
	driveData.drive = drive;
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
	clearEncoders(driveData.drive);
	clearGyro(driveData.drive);

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
