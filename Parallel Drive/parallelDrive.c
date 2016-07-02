/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file in the same directory as your code

	2. Include this line near the top of your code:
			| #include "parallelDrive.c"

	3. Change #define statements on lines 29 and 30 of this file to the number of motor ports used by each side of the drive

	4. To create drive, include the following lines in your code:
			| parallel_drive driveName;
			| tMotor leftMotors[3] = {left1, left2, left3};
			| tMotor rightMotors[3] = {right1, right2, right3};
			| initializeDrive(driveName, &(leftMotors[0]), &(rightMotors[0]));
	   Any valid variable name can be substituted for driveName
	   Motor names (right1, right2, left1, left2, etc...) should correspond to the names assigned in motor setup
	   The number in brackets after rightMotors and leftMotors should correspond to the number of motor ports used on that side of the drive and the numbers changed in step 3
	   The optional arguments of initializeDrive can be used to further configure the drive
	   	e.g. a user creating a drive with ramping and quadratic input mapping would substitute the fourth line of code with:
	   	| initializeDrive(driveName, &leftMotors, &rightMotors, true, 20, 10, 2);

	5. Whenever the drive should be updated (probably once every input cycle) include the following line of code
			| driveRuntime(driveName)
	   Where driveName is the same as in the previous step
*/

#define numLeftMotors 2
#define numRightMotors 2

typedef union {
	struct {
		bool isRamped; //whether drive is ramped
		int maxAcc100ms; //if ramping, maximum change in motor power in 100 ms
		int msPerPowerChange; //calculated using maxAcc100ms
		int deadband; //range of motor values around 0 for which motors are not engaged
		float powMap; //degree of polynomial to which inputs are mapped (1 for linear)
		float powerCoeff; //factor by which motor speeds are multiplied
		TVexJoysticks leftInput; //id of remote channel used to calculate power of left side of drive (usually Ch3)
		TVexJoysticks rightInput; //id of remote channel used to calculate power of right side of drive (usually Ch2)
		//internal variables for ramping
		long lastUpdatedLeft;
		long lastUpdatedRight;
	};

	//motor ports used for drive
	tMotor rightMotors[numRightMotors];
	tMotor leftMotors[numLeftMotors];
} parallel_drive;


void initializeDrive(parallel_drive &drive, tMotor *leftMotorsPtr, tMotor *rightMotorsPtr, bool isRamped=false, int maxAcc100ms=20, int deadband=10, float powMap=1, float powerCoeff=1, TVexJoysticks leftInput=Ch3, TVexJoysticks rightInput=Ch2) {
	//initialize drive variables
	drive.isRamped = isRamped;
	drive.maxAcc100ms;
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
		drive.leftMotors[i] = *(leftMotorsPtr + i); //TODO: multipy i by the size of a tMotor
	}

	for (int i=0; i<numRightMotors; i++) { //copy motors into drive.rightMotors
		drive.rightMotors[i] = *(rightMotorsPtr + i);
	}
}

void setDriveSide(parallel_drive &drive, bool leftSide) {
	int drivePower = 127 * drive.powerCoeff * exp(drive.powMap * log(vexRT[ leftSide ? drive.leftInput : drive.rightInput ] / 127)); //adjust input using powMap and powerCoeff

	if (abs(drivePower) < drive.deadband) drivePower = 0;

	//handle ramping
	if (drive.isRamped) {
		long *lastUpdatePtr = (leftSide ? drive.lastUpdatedLeft : drive.lastUpdatedRight);
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

	//cycle through motors on drive side and set them to drivePower
	for (int i=0; i < (leftSide ? numLeftMotors : numRightMotors); i++) {
		motor[ leftSide ? drive.leftMotors[i] : drive.rightMotors[i] ] = drivePower;
	}
}

void driveRuntime(parallel_drive &drive) {
	setDriveSide(drive, true);
	setDriveSide(drive, false);
}
