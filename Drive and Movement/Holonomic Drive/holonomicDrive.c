#include "coreIncludes.c"
#include "motorGroup.c"

typedef struct {
	tMotor motors[4]; //left front, right front, left back, right back
	TVexJoysticks xInput, yInput, turnInput;
	int deadBand;
} holonomicDrive;

void initializeDrive(holonomicDrive *drive, tMotor lf_motor, tMotor rf_motor, tMotor lb_motor, tMotor rb_motor, int deadBand=15, TVexJoysticks xInput=Ch4, TVexJoysticks yInput=Ch3) {
	tMotor tempMotors[4] = { lf_motor, rf_motor, lb_motor, rb_motor };

	for (int i=0; i<4; i++)
		drive->motors[i] = tempMotors[i];

	drive->xInput = xInput;
	drive->yInput = yInput;
	drive->deadBand = deadBand;
}

void setDrivePower(holonomicDrive *drive, int leftPower, int rightPower, bool diagonal=true) {
	motor[ drive->motors[0] ] = leftPower;
	motor[ drive->motors[1] ] = rightPower;
	motor[ drive->motors[2] ] = (diagonal ? rightPower : leftPower);
	motor[ drive->motors[3] ] = (diagonal ? leftPower : rightPower);
}

void setDrivePowerByVector(holonomicDrive *drive, float x, float y) { //sets motor powers so that drive exerts a force along <x, y> with magnitude proportional to its length
	float squareX, squareY

	if (x != 0) {
		//Input transformed from joystick circle to velocity square. If you want more detail ask Tynan. Sorry.
		squareX = sgn(x) / (abs(y/x) + 1);
		squareY = squareX * y/x;
	}
	else {
		squareX = 0;
		squareY = sgn(y);
	}

	float magnitude = (x*x + y*y) / 127.0;

	setDrivePower(drive, (squareX+squareY)*magnitude, (squareY-squareX)*magnitude);
}

void setDrivePowerByAngle(holonomicDrive *drive, float angle, float magnitude=127, angleType inputType=DEGREES) { //sets motor powers to exert force with a specified direction and magnitude
	angle = convertAngle(angle, RADIANS, inputType);

	setDrivePowerByVector(drive, magnitude*cos(angle), magnitude*sin(angle));
}

void driveRuntime(holonomicDrive *drive) {
	int x = vexRT[ drive->xInput ];
	int y = vexRT[ drive->yInput ];

	if (abs(x) > drive->deadBand || abs(y) > drive->deadBand) {
		setDrivePowerByVector(drive, x, y);
	}
	else {
		int turnPower = vexRT[ drive->turnInput ];

		setDrivePower(drive, -turnPower, turnPower, false);
	}
}
