#define autoDrive drive

#include "coreIncludes.c"
#include "parallelDrive.c"
#include "PID.c"
#include "rampHandler.c"
#include "timer.c"

enum correctionType { NONE, GYRO, ENCODER, AUTO };

parallel_drive autoDrive;

//#region defaults
typedef struct {
	angleType defAngleType;
	bool runAsTask;
	bool useGyro;
	bool reversed;	//reverses all turns (for mirroring auton routines)
	bool usePID;	//true for PID ramping, false for quad ramping
	int debugStartCol;
	int sampleTime;
	int waitAtEnd;
	int minErrorMargin;	//minimum possible error margin 	(TODO: find another system?)
	float rampConst1, rampConst2, rampConst3, rampConst4, rampConst5; // initialPower/kP, maxPower/kI, finalPower/kD, brakeDuration/pd acceptable error, brakePower/pd timeout
} turnDefsStruct;

typedef struct {
	correctionType defCorrectionType;
	bool runAsTask;
	bool rawValue;
	bool usePID;
	int brakeDuration;
	int movementTimeout;
	int waitAtEnd;
	int sampleTime;
	int debugStartCol;
	int minErrorMargin;
	float rampConst1, rampConst2, rampConst3, rampConst4, rampConst5;	//same as in turnDefsStruct
	float kP_c, kI_c, kD_c; //correction PID constants
	float minSpeed;
} driveDefsStruct;

turnDefsStruct turnDefaults;
driveDefsStruct driveDefaults;

void initializeAutoMovement() {
	//turning
	turnDefaults.defAngleType = DEGREES;
	turnDefaults.runAsTask = false;
	turnDefaults.useGyro = true;
	turnDefaults.reversed = false;
	turnDefaults.usePID = true;
	turnDefaults.debugStartCol = -1;
	turnDefaults.sampleTime = 30;
	turnDefaults.waitAtEnd = 100;
	turnDefaults.minErrorMargin = 2;
	turnDefaults.rampConst1 = 5.75; // initialPower  / kP
	turnDefaults.rampConst2 = 0.02; // maxPower      / kI
	turnDefaults.rampConst3 = 20;	  // finalPower    / kD
	turnDefaults.rampConst4 = 0.08; // brakeDuration / pd acceptable error (proportion of target)
	turnDefaults.rampConst5 = 250;	// brakePower    / pd timeout

	//driving
	driveDefaults.defCorrectionType = AUTO;
	driveDefaults.runAsTask = false;
	driveDefaults.rawValue = false;
	driveDefaults.movementTimeout = 500;
	driveDefaults.waitAtEnd = 100;
	driveDefaults.sampleTime = 50;
	driveDefaults.usePID = true;
	driveDefaults.debugStartCol = -1;
	driveDefaults.minErrorMargin = 1;
	driveDefaults.rampConst1 = 25;	//same as above
	driveDefaults.rampConst2 = 0.5;
	driveDefaults.rampConst3 = 70;
	driveDefaults.rampConst4 = 0.1;
	driveDefaults.rampConst5 = 150;
	driveDefaults.kP_c = 0.7;
	driveDefaults.kI_c = 0.007;
	driveDefaults.kD_c = 0.15;
	driveDefaults.minSpeed = 1;	//TODO: is not being assigned correctly
}
//#endregion

//#region turning
typedef struct {
	float angle; //positive for clockwise, negative for counterclockwise
	rampHandler ramper; //used for ramping motor powers
	int sampleTime;	//time between motor power adjustments
	float error;	//allowable deviation from target value
	int pdTimeout;	//time robot is required to be within <error> of the target before continuing
	long pdTimer;	//tracks timeout state when PD ramping
	int finalDelay; //delay after finishing turn (default 100ms for braking)
	int brakeDelay; //maximum duration of braking at end of turn
	int brakePower; //the motor power while braking
	bool usingGyro; //whether to use encoders or gyro for turns
	bool isTurning; //whether turn is executing (useful for running as task)
	int direction; //sign of angle
} turnStruct;

turnStruct turnData;

float turnProgress() {
	return turnData.usingGyro ? gyroVal(autoDrive, DEGREES) : driveEncoderVal(autoDrive);
}

bool turnIsComplete() {
	if (!turnData.isTurning)
		return true;

	if (turnData.ramper.algorithm == PD)
		return time(turnData.pdTimer) >= turnData.pdTimeout;
	else	//algorithm is QUAD
		return fabs(turnProgress()) >= turnData.angle;
}

void turnRuntime() {
	float progress = fabs(turnProgress());

	int power = rampRuntime(turnData.ramper, progress, turnDefaults.debugStartCol);

	setDrivePower(autoDrive, turnData.direction*power, -turnData.direction*power);

	if (turnData.ramper.algorithm==PD && fabs(progress - turnData.angle) > turnData.error)	//track timeout state
		turnData.pdTimer = resetTimer();
}

void turnEnd() {
	if (turnData.ramper.algorithm == QUAD) {
		//brake
		setDrivePower(autoDrive, -turnData.direction * turnData.brakePower, turnData.direction * turnData.brakePower);
		wait1Msec(turnData.brakeDelay);
	}

	setDrivePower(autoDrive, 0, 0);
	wait1Msec(turnData.finalDelay);
	turnData.isTurning = false;
}

task turnTask() {
	while (!turnIsComplete()) {
		turnRuntime();
		wait1Msec(turnData.sampleTime);
	}
	turnEnd();
}

void turn(float angle, bool runAsTask=turnDefaults.runAsTask, float in1=turnDefaults.rampConst1, float in2=turnDefaults.rampConst2, float in3=turnDefaults.rampConst3, float in4=turnDefaults.rampConst4, float in5=turnDefaults.rampConst5, bool usePID=turnDefaults.usePID, int sampleTime=turnDefaults.sampleTime, angleType angleType=turnDefaults.defAngleType, bool useGyro=turnDefaults.useGyro, int waitAtEnd=turnDefaults.waitAtEnd) { //for PD, in1=kP, in2=kI, in3=kD, in4=error, in5=pd timeout; for quad ramping, in1=initial, in2=maximum, in3=final, in4=brakePower, and in5=brakeDuration
	//initialize variables
	if (turnDefaults.reversed) angle *= -1;
	float formattedAngle = convertAngle(fabs(angle), DEGREES, angleType);
	turnData.angle = (useGyro ? formattedAngle : PI*autoDrive.width*formattedAngle/360.);
	turnData.direction = sgn(angle);
	turnData.sampleTime = sampleTime;
	turnData.usingGyro = useGyro;
	turnData.isTurning = true;

	if (usePID) {
		initializeRampHandler(turnData.ramper, PD, formattedAngle, in1, in2, in3, 0, false);	//temp
		turnData.error = max(in4*formattedAngle, turnDefaults.minErrorMargin);	//TODO: add lower limit
		turnData.pdTimeout = in5;
		turnData.pdTimer = resetTimer();
		turnData.finalDelay = waitAtEnd;
	}
	else {
		initializeRampHandler(turnData.ramper, QUAD, formattedAngle, in1, in2, in3);
		turnData.brakeDelay = limit(in5, 0, waitAtEnd);
		turnData.finalDelay = waitAtEnd - in5;
	}

	resetGyro(autoDrive);

	if (runAsTask) {
		startTask(turnTask);
	}
	else {
		while (!turnIsComplete()) {
			turnRuntime();
			wait1Msec(turnData.sampleTime);
		}
		turnEnd();
	}
}
//#endregion

//#region driveStraight
typedef struct {
	float distance;
	bool rawValue; //whether distance is measured in encoder clicks or inches
	float minSpeed; //minimum speed during maneuver to prevent timeout (distance per 100ms)
	float error;	//allowable deviation from target value
	int pdTimeout;	//time robot is required to be within <error> of the target before continuing
	int movementTimeout; //amount of time after which a drive action sensing lower speed than minSpeed ceases (ms)
	int sampleTime; //time between motor power adjustments
	int finalDelay; //duration of pause at end of driving
	int brakeDelay; //maximum duration of braking at end of turn
	int brakePower; // motor power during braking
	correctionType correctionType; //which sensor inputs are used for correction
	bool isDriving; //whether driving action is being executed (true if driving, false othrewise)
	//interal variables
	PID pid;
	rampHandler ramper;
	float leftDist, rightDist, totalDist; //distance traveled by each side of the drive (and their average)
	int direction; //sign of distance
	long pdTimer;	//for tracking pd timeout
	long movementTimer; //for tracking timeout (time without movement)
} driveStruct;

driveStruct driveData;

bool drivingComplete() {
	if (!driveData.isDriving)
		return true;

	if (time(driveData.movementTimer) >= driveData.movementTimeout)
		return true;

	if (driveData.ramper.algorithm == PD)
		return time(driveData.pdTimer) >= driveData.pdTimeout;
	else	//algorithm is QUAD
		return driveData.totalDist >= driveData.distance;
}

void driveStraightRuntime() {
	driveData.leftDist += driveEncoderVal(autoDrive, LEFT, (driveData.rawValue ? RAW_DIST : INCH)) * driveData.direction;
	driveData.rightDist += driveEncoderVal(autoDrive, RIGHT, (driveData.rawValue ? RAW_DIST : INCH)) * driveData.direction;
	driveData.totalDist = (driveData.leftDist + driveData.rightDist) / 2;

	//track timeout states
	if (driveEncoderVal(autoDrive) >= driveData.minSpeed)
		driveData.movementTimer = resetTimer();
	if (driveData.ramper.algorithm==PD && fabs(driveData.totalDist - driveData.distance) > driveData.error)
		driveData.pdTimer = resetTimer();

	resetDriveEncoders(autoDrive);

	//calculate error value and correction coefficient
	float error;

	switch (driveData.correctionType) {
		case ENCODER:
			error = driveData.rightDist - driveData.leftDist;
			break;
		case GYRO:
			error = sin(gyroVal(autoDrive, RADIANS)); //not sure if this is the right approach, but...
			break;
		default:
			error = 0;
	}

	int power = rampRuntime(driveData.ramper, driveData.totalDist, driveDefaults.debugStartCol);

	float correctionPercent = 1 + PID_runtime(driveData.pid, error) * sgn(power);	//sgn(target?)
	float rightPower = power * correctionPercent;
	float leftPower = power;

	if (rightPower > 127) {
		rightPower = 127;
		leftPower = 127 / (correctionPercent);
	}

	setDrivePower(autoDrive, driveData.direction*leftPower, driveData.direction*rightPower);
}

void driveStraightEnd() {
	if (driveData.ramper.algorithm == QUAD) {
		//brake
		setDrivePower(autoDrive, -driveData.direction * driveData.brakePower, -driveData.direction * driveData.brakePower);
		wait1Msec(driveData.brakeDelay);
	}

	setDrivePower(autoDrive, 0, 0);
	wait1Msec(driveData.finalDelay);
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
		while (fabs(gyroVal(autoDrive)) > 10) resetGyro(autoDrive); //I'm horrible, I know
	} else if (type==ENCODER && autoDrive.leftDrive.hasEncoder && autoDrive.rightDrive.hasEncoder) {
		driveData.correctionType = ENCODER;
	}
	else {
		driveData.correctionType = NONE;
	}
}

void driveStraight(float distance, bool runAsTask=driveDefaults.runAsTask, float in1=driveDefaults.rampConst1, float in2=driveDefaults.rampConst2, float in3=driveDefaults.rampConst3, float in4=driveDefaults.rampConst4, float in5=driveDefaults.rampConst5, bool usePID=driveDefaults.usePID, float kP=driveDefaults.kP_c, float kI=driveDefaults.kI_c, float kD=driveDefaults.kD_c, correctionType correctionType=driveDefaults.defCorrectionType, float minSpeed=driveDefaults.minSpeed, int movementTimeout=driveDefaults.movementTimeout, int waitAtEnd=driveDefaults.waitAtEnd) { //for PD, in1=kP, in2=kI, in3=kD, in4=error, in5=pd timeout; for quad ramping, in1=initial, in2=maximum, in3=final, in4=brakePower, and in5=brakeDuration
	//initialize variables
	driveData.distance = fabs(distance);
	driveData.direction = sgn(distance);
	driveData.rawValue = driveDefaults.rawValue;
	driveData.sampleTime = driveDefaults.sampleTime;
	driveData.minSpeed = minSpeed * driveData.sampleTime / 1000;
	driveData.movementTimeout = movementTimeout;
	driveData.isDriving = true;
	initializePID(driveData.pid, 0, kP, kI, kD, 0, false);	//PIDs have 0 sample time because sample delay is taken care of in main loop - temp false

	driveData.leftDist = 0;
	driveData.rightDist = 0;
	driveData.totalDist = 0;

	if (usePID) {
		initializeRampHandler(driveData.ramper, PD, driveData.distance, in1, in2, in3, 0, false);	//temp false
		driveData.error = max(in4*driveData.distance, driveDefaults.minErrorMargin);
		driveData.pdTimeout = in5;
		driveData.pdTimer = resetTimer();
		driveData.finalDelay = waitAtEnd;
	}
	else {
		initializeRampHandler(driveData.ramper, QUAD, driveData.distance, in1, in2, in3);
		driveData.brakeDelay = limit(in5, 0, waitAtEnd);
		driveData.finalDelay = waitAtEnd - in5;
	}

	if (correctionType == AUTO) {
		setCorrectionType(ENCODER);

		if (driveData.correctionType == NONE) {
			setCorrectionType(GYRO);
		}
	}
	else {
		setCorrectionType(correctionType);
	}

	//initialize sensors
	resetDriveEncoders(autoDrive);

	driveData.movementTimer = resetTimer();

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
//#endregion
