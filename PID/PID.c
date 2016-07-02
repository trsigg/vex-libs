/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file in the same directory as your code

	2. Include this line near the top of your code:
			| #include "PID.c"

	3. To create PID controller, include the following lines in your code:
			| PID pidName;
			| initializePID(pidName, &input, kP, kI, kD, target);
		 Where pidName can be any legal variable name; input is the variable storing the input value; kP, kI, and kD are tuning constants; and target is the target value
		 The optional arguments can be used to configure the minimum sample time and initial state of the inputUpdated variable

	4. Whenever the controller should be updated (probably once every input cycle) include the following line of code:
			| PID_runtime(pidName)
	    Where pidName is the same as in the previous step
*/

typedef union {
	struct {
		float *input;
		float kP, kI, kD; //tuning coefficients
		float target;
		int minSampleTime; //minimum time between sampling input in milliseconds
		bool inputUpdated; //can optionally be used to signal when input has been updated
		float output; //can be used to refer to most recent output
		//internal variables
		long lastUpdated;
		float prevError; 
	};
} PID;

void initiatePID(PID &pid, float *input, float kP, float kI, float kD, float target, int minSampleTime=30, bool inputUpdated=true) {
	pid.input = input;
	pid.kP = kP;
	pid.kI = kI;
	pid.kD = kD;
	pid.target = target;
	pid.minSampleTime = minSampleTime;
	pid.inputUpdated = inputUpdated;
}

float PID_runtime(PID &pid) {
	long now = nPgmTime;
	long elapsed = now - pid.lastUpdated;
	pid.lastUpdated = now;

	if (pid.inputUpdated && elapsed > pid.minSampleTime) {
		float error = pid.target - *(pid.input);
		pid.output = kP*error + kI*elapsed*(error + pid.lastError)/2 + kD*(error - pid.lastError)/elapsed;
		pid.lastError = error;
		return pid.output;
	}
}
