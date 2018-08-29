#include "coreIncludes.c"
#include "PID.c"
#include "timer.c"

#define DEF_WAIT_TIMEOUT  50
#define DEF_WAIT_LIST_LEN 1

enum controlType { NONE, BUTTON, JOYSTICK };
enum automovementType { NO, TARGET, MANEUVER, DURATION };

typedef struct {
	tMotor motors[10];
	int numMotors;
	int prevPower;
	controlType controlType;
	bool controlActive;
	TVexJoysticks posInput, negInput; //inputs. NegInput used for 2nd input in dual driver
	//button control
	int upPower, downPower, stillSpeed;
		//complex still speeds
	int stillSpeedSwitchPos, stillSpeedType;	//stillSpeedType is 0 for regular, 1 for posDependent, and 2 for buttonDependent	TODO: change to enum?
	bool stillSpeedReversed;
	//joystick control
	int deadband; //range of motor values around 0 for which motors are not engaged
	bool isRamped; //whether group is ramped
	int msPerPowerChange; //if ramping, time between motor power changes, calculated using maxAcc100ms
	float powMap; //degree of polynomial to which inputs are mapped (1 for linear)
	float coeff; //factor by which motor powers are multiplied
	long lastUpdated; //ramping
		//dual driver
	bool usingDualJoystick;
	int dualPowerCutoff;
	//power limits
	int minPow, maxPow;
	bool hasMinPow, hasMaxPow;
	//absolutes
	int absMin, absMax; //extreme  positions of motorGroup
	bool hasAbsMin, hasAbsMax;
	int maxPowerAtAbs, defPowerAtAbs; //maximum power at absolute position (pushing down from minimum or up from maximum) and default power if this is exceeded
	//velocity tracking
	long velocityTimer;
	float currVelocity;
	int v_sampleTime, prevPos;
	bool v_trackingActive;
	//automovement
	automovementType moving;
	int movePower, endPower;	//both maneuver and duration
	long maneuverTimer;
		//execute maneuver
	int targetPos, maneuverTimeout;
	bool forward; //forward: whether target is forwad from initial group position
		//move for duration
	int moveDuration;
		//targeting
	PID posPID;	//PID controller which maintains position
	int waitErrorMargin, autoSSmargin;	//error margin used for
	bool autoStillSpeeding;
		//sensors
	bool hasEncoder, hasPotentiometer;
	bool encoderReversed, potentiometerReversed;
	bool potentiometerDefault; //whether potentiometer (as opposed to encoder) is default sensor for position measurements
	bool encCorrectionActive;	//drift correction
	int encMax, maxDisp;
	tSensors encoder, potentiometer;
} motorGroup;

motorGroup defGroupWaitList[DEF_WAIT_LIST_LEN];

//#region initialization
void configureButtonInput(motorGroup *group, TVexJoysticks posBtn, TVexJoysticks negBtn, int stillSpeed=0, int upPower=127, int downPower=-127) {
	group->controlType = BUTTON;
	group->controlActive = true;
	group->posInput = posBtn;
	group->negInput = negBtn;
	group->stillSpeed = stillSpeed;
	group->stillSpeedType = 0;
	group->stillSpeedReversed = false;
	group->upPower = upPower;
	group->downPower = downPower;
}

void configureJoystickInput(motorGroup *group, TVexJoysticks joystick, int deadband=10, bool isRamped=false, int maxAcc100ms=60, float powMap=1, int maxPow=127) {
	group->controlType = JOYSTICK;
	group->controlActive = true;
	group->posInput = joystick;
	group->deadband = deadband;
	group->isRamped = isRamped;
	group->msPerPowerChange = 100 / maxAcc100ms;
	group->powMap = powMap;
	group->coeff = maxPow /  127.0;
	group->lastUpdated = nPgmTime;
}

void configureDualJoystick(motorGroup *group, TVexJoysticks joystick, int powerCutoff=20) {
	group->negInput = joystick;
	group->dualPowerCutoff = powerCutoff;
	group->usingDualJoystick = true;
}

void configureRamping(motorGroup *group, int maxAcc100ms) {
	group->isRamped = true;
	group->msPerPowerChange = 100 / maxAcc100ms;
}

void initializeGroup(motorGroup *group, int numMotors, tMotor *motors, TVexJoysticks posBtn=Ch1, TVexJoysticks negBtn=Ch1, int stillSpeed=0, int upPower=127, int downPower=-127) {
	group->numMotors = limit(numMotors, 0, 10);

	for (int i=0; i<group->numMotors; i++)
		group->motors[i] = motors[i];

	if (posBtn >= Btn5D)
		configureButtonInput(group, posBtn, negBtn, stillSpeed, upPower, downPower);

	group->moving = NO;
}
//#endregion

//#region sensors
void addSensor(motorGroup *group, tSensors sensor, bool reversed=false, bool setAsDefault=true) {
	switch (SensorType[sensor]) {
		case sensorPotentiometer:
			group->hasPotentiometer = true;
			group->potentiometer = sensor;
			group->potentiometerReversed = reversed;
			if (setAsDefault) group->potentiometerDefault = true;
			break;
		case sensorQuadEncoder:
			group->hasEncoder = true;
			group->encoder = sensor;
			group->encoderReversed = reversed;
			SensorValue[sensor] = 0;
			if (setAsDefault) group->potentiometerDefault = false;
			break;
	}
}

int encoderVal(motorGroup *group, bool useCorrection=true) {
	if (group->hasEncoder) {
		return SensorValue[group->encoder] * (group->encoderReversed ?  -1 : 1)
		        - (group->encCorrectionActive && useCorrection ? group->maxDisp : 0);
	}
	else {
		return 0;
	}
}

int potentiometerVal(motorGroup *group) {
	if (group->hasPotentiometer) {
		return (group->potentiometerReversed ? 4096-SensorValue[group->potentiometer] : SensorValue[group->potentiometer]);
	}
	else {
		return 0;
	}
}

int getPosition(motorGroup *group, bool useEncCorrection=true) {
	if (group->hasPotentiometer && group->hasEncoder) {
		return group->potentiometerDefault ? potentiometerVal(group) : encoderVal(group, useEncCorrection);
	}
	else {
		return (group->hasEncoder ? encoderVal(group, useEncCorrection) : potentiometerVal(group));
	}
}

void resetEncoder(motorGroup *group, int resetVal=0) {
	if (group->hasEncoder)
		SensorValue[group->encoder] = resetVal * (group->encoderReversed ?  -1 : 1);
}

void configureEncoderCorrection(motorGroup *group, int max) {
	group->encMax = max;
	group->encCorrectionActive = true;
	group->maxDisp = 0;
}

void correctEncVal(motorGroup *group) {
	if (group->encCorrectionActive) {
		int encVal = encoderVal(group, false);

		if (encVal <= group->maxDisp) {
			resetEncoder(group);
			group->maxDisp -= encVal;
		}
		else if ((encVal - group->encMax) > group->maxDisp) {
			group->maxDisp = encVal - group->encMax;
		}
	}
}
//#endregion

//#region set and get power
int setPower(motorGroup *group, int power, bool overrideAbsolutes=false) {	//TODO: option to override power limits?
	power = limit(power, (group->hasMinPow ? group->minPow : -128), (group->hasMaxPow ? group->maxPow : 128));

	if (!overrideAbsolutes) {
		if (group->hasAbsMin && (getPosition(group) <= group->absMin) && (power < -group->maxPowerAtAbs))
			power = -group->defPowerAtAbs;

		if (group->hasAbsMax && (getPosition(group) >= group->absMax) && (power > group->maxPowerAtAbs))
			power = group->defPowerAtAbs;
	}

	for (int i=0; i<group->numMotors; i++) //set motors
		motor[group->motors[i]] = power;

	return group->prevPower = power;
}

int getPower(motorGroup *group) {
	return group->prevPower/*motor[ group->motors[0] ]*/;
}
//#endregion

//#region still speeds
void configurePosDependentStillSpeed(motorGroup *group, int switchPos, int stillSpeed=0) {	//motor will have stillSpeed power when below switchPos, -stillSpeed power when above switchPos
	if (stillSpeed!=0) group->stillSpeed = stillSpeed;
	group->stillSpeedType = 1;
	group->stillSpeedSwitchPos = switchPos;
}

void configureBtnDependentStillSpeed(motorGroup *group, int stillSpeed=0) {
	if (stillSpeed!=0) group->stillSpeed = stillSpeed;
	group->stillSpeedType = 2;
}

int calcStillSpeed(motorGroup *group, bool posForBtnSS=true, bool setReversed=true) {
	bool reversed;

	switch (group->stillSpeedType) {
		case 0:
			reversed = false;
			break;
		case 1:
			reversed = (getPosition(group) > group->stillSpeedSwitchPos);
			break;
		case 2:
			reversed = !posForBtnSS;
			break;
	}

	if (setReversed) group->stillSpeedReversed = reversed;

	return group->stillSpeed * (reversed ? -1 : 1);
}

void setToStillSpeed(motorGroup *group, bool posForBtnSS=true) {
	setPower(group, calcStillSpeed(group, posForBtnSS));
}
//#endregion

//#region power limiting
void setMinPow(motorGroup *group, int min) {
	group->minPow = min;
	group->hasMinPow = true;
}

void setMaxPow(motorGroup *group, int max) {
	group->maxPow = max;
	group->hasMaxPow = true;
}

void setPowerLimits(motorGroup *group, int min, int max) {
	setMinPow(group, min);
	setMaxPow(group, max);
}

void stopPowerLimiting(motorGroup* group) {
	group->hasMinPow = false;
	group->hasMaxPow = false;
}
//#endregion

//#region position limiting
void setAbsMax(motorGroup *group, int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMax = max;
	group->hasAbsMax = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}

void setAbsMin(motorGroup *group, int min, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->hasAbsMin = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}

void setAbsolutes(motorGroup *group, int min, int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->absMax = max;
	group->hasAbsMin = true;
	group->hasAbsMax = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}
//#endregion

//#region velocity tracking
void configureVelocityTracking(motorGroup *group, int sampleTime=50) {
	group->v_sampleTime = sampleTime;
	group->currVelocity = 0;
	group->velocityTimer = resetTimer();
	group->v_trackingActive = true;

	if (group->potentiometerDefault)
		group->prevPos = getPosition(group);
	else
		group->prevPos = 0;
}

float getVelocity(motorGroup *group, bool useTimeCorrection=true) {
	long elapsed = time(group->velocityTimer);

	if (elapsed > group->v_sampleTime) {
		group->velocityTimer = resetTimer();
		int currPos = getPosition(group);
		int disp = currPos;

		if (group->potentiometerDefault) {
			disp -= group->prevPos;
			group->prevPos = currPos;
		}
		else {
			resetEncoder(group);
		}

		group->currVelocity = disp / (useTimeCorrection ? elapsed : 1);
	}

	return group->currVelocity;
}
//#endregion

//#region automovement
int executeAutomovement(motorGroup *group, int debugStartCol=-1);	//just a forward declaration

	//#subregion targeting
void initializeTargetingPID(motorGroup *group, float kP, float kI, float kD, int errorMargin=100, int minSampleTime=10, bool useTimeCorrection=true, int integralMax=127) {	//TODO: integralMax DOES NOT WORK with autoStillSpeeding
	initializePID(group->posPID, 0, kP, kI, kD, minSampleTime, useTimeCorrection, integralMax);
	group->waitErrorMargin = errorMargin;
	group->autoStillSpeeding = false;
}

void setTargetingPIDconsts(motorGroup *group, float kP, float kI, float kD) {
	changeGains(group->posPID, kP, kI, kD);
}

void configureAutoStillSpeed(motorGroup *group, int margin) {
	group->autoSSmargin = margin;
	group->autoStillSpeeding = true;
}

void setTargetPosition(motorGroup *group, int position, bool resetIntegral=true) {
	changeTarget(group->posPID, position, resetIntegral);
	group->moving = TARGET;
}

bool errorLessThan(motorGroup *group, int errorMargin) {	//returns true if PID error is less than specified margin
	return fabs(group->posPID.target - getPosition(group)) < errorMargin;
}
	//#endsubregion

	//#subregion maneuvers
void createManeuver(motorGroup *group, int position, bool runConcurrently=true, int movePower=127, int endPower=128, int timeout=10) {	//TODO: auto-determine endPower as in moveForDuration
	group->targetPos = position;
	group->forward = group->targetPos > getPosition(group);
	group->movePower = abs(movePower) * (group->forward ? 1 : -1);
	group->endPower = (endPower>127 ? calcStillSpeed(group, group->forward) : endPower);
	group->maneuverTimeout = timeout;
	group->maneuverTimer = resetTimer();
	group->moving = MANEUVER;

	setPower(group, group->movePower);

	if (!runConcurrently) {
		while (group->moving == MANEUVER) {
			executeAutomovement(group);
			EndTimeSlice();
		}
	}
}
	//#endsubregion

	//#subregion durational movement
void moveForDuration(motorGroup *group, int power, int duration, bool runConcurrently=true, int endPower=128) {	//if endPower>127, will finish with stillSpeed
	group->movePower = power;
	group->moveDuration = duration;
	group->moving = DURATION;
	group->maneuverTimer = resetTimer();
	group->endPower = (endPower>127 ? calcStillSpeed(group, power>0) : endPower);

	setPower(group, group->movePower);

	if (!runConcurrently) {
		wait1Msec(duration);
		setPower(group, group->endPower);
		group->moving = NO;
	}
}
	//#endsubregion

	//#subregion waiting
void waitForMovementToFinish(int timeout=DEF_WAIT_TIMEOUT, int numGroups=DEF_WAIT_LIST_LEN, motorGroup* groups=defGroupWaitList) {	//that's right, nested pointers                                   (help me)
	long movementTimer = resetTimer();

	while (time(movementTimer) < timeout) {	//wait for targeting to stabilize
		for (int i=0; i<numGroups; i++) {
			if (groups[i].moving==TARGET && !errorLessThan(&groups[i], &groups[i]->waitErrorMargin)) {
					movementTimer = resetTimer();
					continue;	//TODO: ??
			}
		}
		EndTimeSlice();
	}

	for (int i=0; i<numGroups; i++)	//wait for maneuvers and durational movement to finish
		while (groups[i].moving!=TARGET && groups[i].moving!=NO)
			EndTimeSlice();
}

void waitForMovementToFinish(bool *waitForGroups, int timeout=DEF_WAIT_TIMEOUT) {
	motorGroup groups[DEF_WAIT_LIST_LEN];
	int j = 0;

	for (int i=0; i<DEF_WAIT_LIST_LEN; i++)
		if (waitForGroups[i])
			groups[j++] = defGroupWaitList[i];

	waitForMovementToFinish(timeout, j, groups);
}

/*void waitForMovementToFinish(motorGroup *group, int timeout=DEF_WAIT_TIMEOUT) {
	waitForMovementToFinish(timeout, 1, group);
}*/
void waitForMovementToFinish(motorGroup *group, int timeout=DEF_WAIT_TIMEOUT) {	//TODO: delete this as soon as possible
	long movementTimer = resetTimer();

	while (time(movementTimer) < timeout) {	//wait for targeting to stabilize
		if (group->moving==TARGET && !errorLessThan(group, group->waitErrorMargin))
				movementTimer = resetTimer();

		EndTimeSlice();
	}

	while (group->moving!=TARGET && group->moving!=NO) EndTimeSlice();
}
	//#endsubregion

int executeAutomovement(motorGroup *group, int debugStartCol) {
	switch (group->moving) {
		case TARGET:
			if (group->posPID.kP != 0) {
				int powerLimit = (group->autoStillSpeeding && errorLessThan(group, group->autoSSmargin) ? group->stillSpeed : 127);	//limit power if autoStillSpeeding conditions are met
				group->posPID.integralMax = powerLimit;
				int PIDpower = PID_runtime(group->posPID, getPosition(group), debugStartCol);
				setPower(group, copysign(PIDpower, limit(fabs(PIDpower), 0, powerLimit)));
			}
			break;

		case MANEUVER:
			if (group->forward == (getPosition(group) < group->targetPos)) {
				group->maneuverTimer = resetTimer();
				setPower(group, group->movePower);
			}
			else if (time(group->maneuverTimer) > group->maneuverTimeout) {
				setPower(group, group->endPower);
				group->moving = NO;
			}
			break;

		case DURATION:
			if (time(group->maneuverTimer) > group->moveDuration) {
				setPower(group, group->endPower);
				group->moving = NO;
			}
			else {
				setPower(group, group->movePower);
			}
			break;
	}

	return getPower(group);	//TODO: be better
}

void stopAutomovement(motorGroup *group) {
	group->moving = NO;
	setPower(group, 0);
}

int moveTowardPosition(motorGroup *group, int position, int power=127) {
	return setPower(group, power * sgn(position - getPosition(group)));
}
//#endregion

//#region user input
int handleButtonInput(motorGroup *group) {
	if (vexRT[group->posInput] == 1) {
		group->moving = NO;

		if (group->stillSpeedType == 2)
			group->stillSpeedReversed = false;

		return group->upPower;
	}
	else if (vexRT[group->negInput] == 1) {
		group->moving = NO;

		if (group->stillSpeedType == 2)
			group->stillSpeedReversed = true;

		return group->downPower;
	}
	else {	//TODO: executeAutomovement?
		if (group->stillSpeedType == 1)
			group->stillSpeedReversed = getPosition(group) > group->stillSpeedSwitchPos;

		return group->stillSpeed * (group->stillSpeedReversed ? -1 : 1);
	}
}

int handleJoystickInput(motorGroup *group) {
	int input = vexRT[group->posInput];

	if (group->usingDualJoystick && abs(input) < group->dualPowerCutoff)
		input = vexRT[group->negInput];

	int power = sgn(input) * group->coeff * fabs(pow(input / 127.0, group->powMap)) * 127;

	if (abs(power) < group->deadband) power = 0;

	//handle ramping
	if (group->isRamped) {
		long now = nPgmTime;
		int elapsed = now - group->lastUpdated;
		int currentPower = motor[ group->motors[0] ];

		if (elapsed >= group->msPerPowerChange) {
			group->lastUpdated = now;

			if (abs(power) > abs(currentPower)) {	//only ramp up in absolute value
				int maxDiff = elapsed / group->msPerPowerChange;

				if (abs(currentPower - power) > maxDiff) {
					group->lastUpdated = now - (elapsed % group->msPerPowerChange);
					return (power>currentPower ? currentPower+maxDiff : currentPower-maxDiff);
				}
			}
		}
		else {
			return currentPower;
		}
	}

	return power;
}

int takeInput(motorGroup *group, bool setMotors=true) {
	int power = 0;

	if (group->controlActive) {
		switch (group->controlType) {
			case BUTTON:
				power = handleButtonInput(group);
				break;
			case JOYSTICK:
				power = handleJoystickInput(group);
				break;
		}
	}

	if (setMotors) setPower(group, power);

	return power;
}
//#endregion
