typedef struct {
		float kP, kI, kD; //tuning coefficients
		float target;
		int minSampleTime; //minimum time between sampling input in milliseconds
		float integralMax; //minimum and maximum error value which will be added to integral
		bool hasMax;
		bool useTimeCorrection;
		float output; //can be used to refer to most recent output
		//internal variables
		long lastUpdated;
		float integral;
		float prevError;
} PID;

void initializePID(PID *pid, float target, float kP, float kI, float kD, int minSampleTime=10, bool useTimeCorrection=true, float integralMax=0) {
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->target = target;
	pid->minSampleTime = minSampleTime;
	pid->hasMax = integralMax!=0;
	pid->integralMax = integralMax;
	pid->useTimeCorrection = useTimeCorrection;
	pid->integral = 0;
	pid->lastUpdated = 0;	//indicates that PID hasn't yet been evaluated
}

void changeTarget(PID *pid, float target, int resetIntegral=true) {
	pid->prevError += target - pid->target;
	if (resetIntegral) pid->integral = 0;	//TODO: add more options?
	pid->lastUpdated = nPgmTime;
	pid->target = target;
}

void changeGains(PID *pid, float kP, float kI, float kD) {
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
}

void setIntegralMax(PID *pid, float max) {
	pid->hasMax = true;
	pid->integralMax = max;
}

float PID_runtime(PID *pid, float input, int debugStartCol=-1) {
	long now = nPgmTime;
	long elapsed = now - pid->lastUpdated;

	if (elapsed > pid->minSampleTime) {
		pid->lastUpdated = now;

		float error = pid->target - input;
		int timeCorrectionFctr = (pid->useTimeCorrection ? elapsed : 1);

		float p = pid->kP * error;	//proportional contribution
		float d = 0;								//derivative contribution

		if (pid->lastUpdated != 0) {
			if (!pid->hasMax || fabs(pid->output)<pid->integralMax || sgn(pid->kI*error)!=sgn(pid->output))	//TODO: better limiting
				pid->integral += pid->kI * error * timeCorrectionFctr;	//kI factored in here to avoid problems when changing gain values

			d = pid->kD * (error - pid->prevError) / timeCorrectionFctr;
		}

		pid->output = p + pid->integral + d;

		if (debugStartCol >= 0) {
			datalogAddValueWithTimeStamp(debugStartCol, error);
			datalogAddValueWithTimeStamp(debugStartCol+1, pid->output);
			datalogAddValueWithTimeStamp(debugStartCol+2, p);
			datalogAddValueWithTimeStamp(debugStartCol+3, pid->integral);
			datalogAddValueWithTimeStamp(debugStartCol+4, d);
			datalogAddValueWithTimeStamp(debugStartCol+5, pid->target);
			datalogAddValueWithTimeStamp(debugStartCol+6, input);
		}

		pid->prevError = error;
	}

	return pid->output;
}
