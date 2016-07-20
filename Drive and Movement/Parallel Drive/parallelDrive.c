/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file and coreIncludes.c in the same directory as your code

	2. Include this line near the top of your code:
			| #include "parallelDrive.c"

	3. To create drive, include the following lines in your code:
			| parallel_drive driveName;
			| initializeDrive(driveName);
			| setLeftMotors(driveName, numLeftMotors, left1, left2, left3, etc.);
			| setRightMotors(driveName, numRightMotors, right1, right2, right3, etc.);
	   Any valid variable name can be substituted for driveName
	   numLeftMotors and numRightMotors are the number of motors on that side of the drive (Maximum of 6)
	   Motor names (right1, right2, left1, left2, etc...) should correspond to the names assigned in motor setup.
	   The optional arguments of initializeDrive can be used to further configure the drive
	   	e.g. a user creating a drive with ramping and quadratic input mapping would substitute the second line of code with:
	   	| initializeDrive(driveName, true, 20, 10, 2);

	4. Whenever the drive should be updated (probably once every input cycle) include the following line of code
			| driveRuntime(driveName)
	   Where driveName is the same as in the previous step

	5. To explicitly set drive power, use setDrivePower(driveName, leftPower, rightPower), setRightPower(driveName, power), or setLeftPower(driveName, power)

	6. To attach sensors, call attachGyro(driveName, gyro), attachEncoderL(driveName, leftEncoder), or attachEncoderR(driveName, rightEncoder), where gyro, leftEncoder, and rightEncoder are the names assigned in sensor setup

	7. To explicitly control how encoders are used for distance measurement, call setEncoderConfig(driveName, config) where config is NONE, LEFT, RIGHT, or AVERAGE

	8. To access a sensor value, call encoderVal(driveName), encoderVal_L(driveName), encoderVal_R(driveName), or gyroVal(driveName)

	9. To clear a sensor, call clearEncoders(driveName), clearLeft(driveName), clearRight(driveName), or clearGyro(driveName)
*/

#include "coreIncludes.c"

typedef enum encoderConfig { NONE, LEFT, RIGHT, AVERAGE };

typedef struct {
		bool isRamped; //whether drive is ramped
		int msPerPowerChange; //if ramping, time between motor power changes, calculated using maxAcc100ms
		int deadband; //range of motor values around 0 for which motors are not engaged
		float powMap; //degree of polynomial to which inputs are mapped (1 for linear)
		float powerCoeff; //factor by which motor speeds are multiplied
		TVexJoysticks leftInput; //id of remote channel used to calculate power of left side of drive (usually Ch3)
		TVexJoysticks rightInput; //id of remote channel used to calculate power of right side of drive (usually Ch2)
		//internal variables
		long lastUpdatedLeft, lastUpdatedRight; //for ramping
		int numLeftMotors, numRightMotors;
		//motor ports used for drive
		tMotor rightMotors[6];
		tMotor leftMotors[6];
		//associated sensors
		encoderConfig encoderConfig;
		bool hasGyro, hasEncoderL, hasEncoderR;
		tSensors gyro, leftEncoder, rightEncoder;
} parallel_drive;


void initializeDrive(parallel_drive &drive, bool isRamped=false, int maxAcc100ms=20, int deadband=10, float powMap=1, float powerCoeff=1, TVexJoysticks leftInput=Ch3, TVexJoysticks rightInput=Ch2) {
	//initialize drive variables
	drive.isRamped = isRamped;
	drive.msPerPowerChange = 100 / maxAcc100ms;
	drive.deadband = deadband;
	drive.powMap = powMap;
	drive.powerCoeff = powerCoeff;
	drive.leftInput = leftInput;
	drive.rightInput = rightInput;
	drive.lastUpdatedLeft = nPgmTime;
	drive.lastUpdatedRight = nPgmTime;
	drive.numLeftMotors = 0;
	drive.numRightMotors = 0;
}

void attachMotor(parallel_drive &drive, tMotor motor, bool left) {
	if (left) {
		if (drive.numLeftMotors <= 5) {
			drive.leftMotors[drive.numLeftMotors] = motor;
			drive.numLeftMotors++;
		}
	} else if (drive.numRightMotors <= 5) {
		drive.rightMotors[drive.numRightMotors] = motor;
		drive.numRightMotors++;
	}
}

void setLeftMotors(parallel_drive &drive, int numMotors, tMotor motor1, tMotor motor2=port1, tMotor motor3=port1, tMotor motor4=port1, tMotor motor5=port1, tMotor motor6=port1) { //look, I know this is stupid.  But arrays in ROBOTC */really/* suck
	tMotor motors[6] = {motor1, motor2, motor3, motor4, motor5, motor6};

	for (int i=0; i<numMotors; i++) {
		attachMotor(drive, motors[i], true);
	}
}

void setRightMotors(parallel_drive &drive, int numMotors, tMotor motor1, tMotor motor2=port1, tMotor motor3=port1, tMotor motor4=port1, tMotor motor5=port1, tMotor motor6=port1) { //look, I know this is stupid.  But arrays in ROBOTC */really/* suck
	tMotor motors[6] = {motor1, motor2, motor3, motor4, motor5, motor6};

	for (int i=0; i<numMotors; i++) {
		attachMotor(drive, motors[i], false);
	}
}


//sensor setup region
void updateEncoderConfig(parallel_drive &drive) {
	if (drive.hasEncoderL) {
		if (drive.hasEncoderR) {
			drive.encoderConfig = AVERAGE;
		} else {
			drive.encoderConfig = LEFT;
		}
	} else {
		drive.encoderConfig = RIGHT; //safe assuming nothig but attachEncoder functions call this
	}
}

void attachEncoderL(parallel_drive &drive, tSensors encoder) {
	drive.leftEncoder = encoder;
	drive.hasEncoderL = true;
	updateEncoderConfig(drive);
}

void attachEncoderR(parallel_drive &drive, tSensors encoder) {
	drive.rightEncoder = encoder;
	drive.hasEncoderR = true;
	updateEncoderConfig(drive);
}


void attachGyro(parallel_drive &drive, tSensors gyro) {
	drive.gyro = gyro;
	drive.hasGyro = true;
}

void setEncoderConfig(parallel_drive &drive, encoderConfig config) {
	drive.encoderConfig = config;
}
//end sensor setup region


//sensor access region
int encoderVal(parallel_drive &drive) {
	if (drive.encoderConfig==AVERAGE && drive.hasEncoderL && drive.hasEncoderR) {
		return (SensorValue[drive.leftEncoder] + SensorValue[drive.rightEncoder]) / 2;
	} else if (drive.encoderConfig==LEFT && drive.hasEncoderL) {
		return SensorValue[drive.leftEncoder];
	} else if (drive.encoderConfig==RIGHT && drive.hasEncoderR) {
		return SensorValue[drive.rightEncoder];
	}

	return 0;
}

int encoderVal_L(parallel_drive &drive) {
	if (drive.hasEncoderL) {
		return SensorValue[drive.leftEncoder];
	} else {
		return 0;
	}
}

int encoderVal_R(parallel_drive &drive) {
	if (drive.hasEncoderR) {
		return SensorValue[drive.rightEncoder];
	} else {
		return 0;
	}
}

void clearLeft(parallel_drive &drive) {
	if (drive.hasEncoderL) {
		SensorValue[drive.leftEncoder] = 0;
	}
}

void clearRight(parallel_drive &drive) {
	if (drive.hasEncoderR) {
		SensorValue[drive.rightEncoder] = 0;
	}
}

void clearEncoders(parallel_drive &drive) {
	clearLeft(drive);
	clearRight(drive);
}

int gyroVal(parallel_drive &drive) {
	if (drive.hasGyro) {
		return SensorValue[drive.gyro];
	} else {
		return 0;
	}
}

void clearGyro(parallel_drive &drive) {
	if (drive.hasGyro) {
		SensorValue[drive.gyro] = 0;
	}
}
//end sensor access region


//set drive power region
void setLeftPower (parallel_drive &drive, int power) {
	for (int i=0; i<drive.numLeftMotors; i++) {
		motor[ drive.leftMotors[i] ] = power;
	}
}

void setRightPower (parallel_drive &drive, int power) {
	for (int i=0; i<drive.numRightMotors; i++) {
		motor[ drive.rightMotors[i] ] = power;
	}
}

void setDrivePower (parallel_drive &drive, int left, int right) {
	setLeftPower(drive, left);
	setRightPower(drive, right);
}
//end set drive power region


void setDriveSide(parallel_drive &drive, bool leftSide) {
	int input = vexRT[leftSide ? drive.leftInput : drive.rightInput];
	int drivePower = sgn(input) * drive.powerCoeff * power(input, drive.powMap) / power(127, drive.powMap-1); //adjust input using powMap and powerCoeff

	if (abs(drivePower) < drive.deadband) drivePower = 0;

	//handle ramping
	if (drive.isRamped) {
		long *lastUpdatePtr = (leftSide ? &drive.lastUpdatedLeft : &drive.lastUpdatedRight);
		long now = nPgmTime;
		int elapsed = now - *lastUpdatePtr;
		int currentPower = motor[ leftSide ? drive.leftMotors[0] : drive.rightMotors[0] ];

		if (elapsed > drive.msPerPowerChange) {
			int maxDiff = elapsed / drive.msPerPowerChange;

			if (abs(currentPower - drivePower) < maxDiff) {
				*lastUpdatePtr = now;
			} else {
				drivePower = (drivePower>currentPower ? currentPower+maxDiff : currentPower-maxDiff);
				*lastUpdatePtr = now - (elapsed % drive.msPerPowerChange);
			}
		}
	}

	//set drive motor power
	if (leftSide) {
		setLeftPower(drive, drivePower);
	} else {
		setRightPower(drive, drivePower);
	}
}

void driveRuntime(parallel_drive &drive) {
	setDriveSide(drive, true);
	setDriveSide(drive, false);
}
