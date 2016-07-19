/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file and coreIncludes.c in the same directory as your code

	2. Include this line near the top of your code:
			| #include "parallelDrive.c"

	3. Change #define statements below to the number of motor ports used by each side of the drive */

#define numLeftMotors 2
#define numRightMotors 2

/*	4. To create drive, include the following lines in your code:
			| parallel_drive driveName;
			| tMotor leftMotors[3] = {left1, left2, left3};
			| tMotor rightMotors[3] = {right1, right2, right3};
			| initializeDrive(driveName, &(leftMotors[0]), &(rightMotors[0]));
	   Any valid variable name can be substituted for driveName
	   Motor names (right1, right2, left1, left2, etc...) should correspond to the names assigned in motor setup
	   The number in brackets after rightMotors and leftMotors should correspond to the number of motor ports used on that side of the drive and the numbers changed in step 3
	   The optional arguments of initializeDrive can be used to further configure the drive
	   	e.g. a user creating a drive with ramping and quadratic input mapping would substitute the fourth line of code with:
	   	| initializeDrive(driveName, &(leftMotors[0]), &(rightMotors[0]), true, 20, 10, 2);

	5. Whenever the drive should be updated (probably once every input cycle) include the following line of code
			| driveRuntime(driveName)
	   Where driveName is the same as in the previous step

	6. To explicitly set drive power, use setDrivePower(driveName, leftPower, rightPower), setRightPower(driveName, power), or setLeftPower(driveName, power)

	7. To attach sensors, call attachGyro(driveName, gyro), attachEncoderL(driveName, leftEncoder), or attachEncoderR(driveName, rightEncoder), where gyro, leftEncoder, and rightEncoder are the names assigned in sensor setup

	8. To explicitly control how encoders are used for distance measurement, call setEncoderConfig(driveName, config) where config is NONE, LEFT, RIGHT, or AVERAGE

	9. To access a sensor value, call encoderVal(driveName), encoderVal_L(driveName), encoderVal_R(driveName), or gyroVal(driveName)

	10. To clear a sensor, call clearEncoders(driveName), clearLeft(driveName), clearRight(driveName), or clearGyro(driveName)
*/

#include "coreIncludes.c"

enum encoderConfig { NONE, LEFT, RIGHT, AVERAGE };

typedef union {
	struct {
		bool isRamped; //whether drive is ramped
		int msPerPowerChange; //if ramping, time between motor power changes, calculated using maxAcc100ms
		int deadband; //range of motor values around 0 for which motors are not engaged
		float powMap; //degree of polynomial to which inputs are mapped (1 for linear)
		float powerCoeff; //factor by which motor speeds are multiplied
		TVexJoysticks leftInput; //id of remote channel used to calculate power of left side of drive (usually Ch3)
		TVexJoysticks rightInput; //id of remote channel used to calculate power of right side of drive (usually Ch2)
		//internal variables for ramping
		long lastUpdatedLeft;
		long lastUpdatedRight;
		//associated sensors
		encoderConfig encoderConfig;
		bool hasGyro, hasEncoderL, hasEncoderR;
		tSensors gyro, leftEncoder, rightEncoder;
	};

	//motor ports used for drive
	tMotor rightMotors[numRightMotors];
	tMotor leftMotors[numLeftMotors];
} parallel_drive;


void initializeDrive(parallel_drive &drive, tMotor *leftMotorsPtr, tMotor *rightMotorsPtr, bool isRamped=false, int maxAcc100ms=20, int deadband=10, float powMap=1, float powerCoeff=1, TVexJoysticks leftInput=Ch3, TVexJoysticks rightInput=Ch2) {
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

	//arrays are stupid in robotc so I have to do this
	for (int i=0; i<numLeftMotors; i++) { //copy motors into drive.leftMotors
		drive.leftMotors[i] = *(leftMotorsPtr + i); //TODO: multipy i by the size of a tMotor (could be resolved by passing motor arrays as tMotor &motorArray?)
	}

	for (int i=0; i<numRightMotors; i++) { //copy motors into drive.rightMotors
		drive.rightMotors[i] = *(rightMotorsPtr + i);
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
	for (int i=0; i<numLeftMotors; i++) {
		motor[ drive.leftMotors[i] ] = power;
	}
}

void setRightPower (parallel_drive &drive, int power) {
	for (int i=0; i<numRightMotors; i++) {
		motor[ drive.rightMotors[i] ] = power;
	}
}

void setDrivePower (parallel_drive &drive, int left, int right) {
	setLeftPower(drive, left);
	setRightPower(drive, right);
}
//end set drive power region


void setDriveSide(parallel_drive &drive, bool leftSide) {
	int drivePower = 127 * drive.powerCoeff * pow(vexRT[leftSide ? drive.leftInput : drive.rightInput]/127, drive.powMap); //adjust input using powMap and powerCoeff

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
