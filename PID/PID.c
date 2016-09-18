/*///////////////  INSTRUCTIONS  /////////////////


	1. Save this file in the same directory as your code

	2. Include this line near the top of your code:
			| #include "PID.c"

	3. To create PID controller, include the following lines in your code:
			| PID pidName;
			| initializePID(pidName, &input, kP, kI, kD, target);
		 Where pidName can be any legal variable name; input is the variable storing the input value; kP, kI, and kD are tuning constants; and target is the target value
		 To use input as the error value, set target to 0
		 The optional arguments can be used to configure the minimum sample time and initial state of the inputUpdated variable

	4. Whenever the controller should be updated (probably once every input cycle) include the following line of code:
			| PID_runtime(pidName);
	    Where pidName is the same as in the previous step

	5. To change the target value of the PID loop, use
			| pidName.target = newTarget;
*/

typedef union {
	struct {
		float *input;
		float kP, kI, kD; //tuning coefficients
		float target;
		int minSampleTime; //minimum time between sampling input in milliseconds
		bool inputUpdated; //can optionally be used to signal when input has been updated
		float integralMin, integralMax; //minimum and maximum error value which will be added to integral
		float output; //can be used to refer to most recent output
		//internal variables
		long lastUpdated;
		float integral;
		float prevError;
	};
} PID;

void initializePID(PID *pid, float *input, float kP, float kI, float kD, float target, int minSampleTime=30, bool inputUpdated=true, float integralMin=NULL, float integralMax=NULL) {
	pid->input = input;
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->target = target;
	pid->minSampleTime = minSampleTime;
	pid->inputUpdated = inputUpdated;
	pid->integralMin = integralMin;
	pid->integralMax = integralMax;
	pid->integral = 0;
}

float PID_runtime(PID *pid) {
	long now = nPgmTime;
	long elapsed = now - pid->lastUpdated;
	pid->lastUpdated = now;

	if (pid->inputUpdated && elapsed > pid->minSampleTime) {
		float input = *(pid->input);
		float error = (pid->target == 0 ? input : pid->target-input);

		pid->integral += (pid->integralMin==NULL || error>pid->integralMin) && (pid->integralMax==NULL || error>pid->integralMax) ? (error + pid->prevError)*elapsed/2 : 0; //update integral if within bounds of integralMin and integralMax

		pid->output = pid->kP*error + pid->kI*pid->integral + pid->kD*(error - pid->prevError)/elapsed;
		pid->prevError = error;
	}

	return pid->output;
}
