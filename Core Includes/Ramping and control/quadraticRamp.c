typedef struct {
  float a, b, c;
} quadraticRamp;

void initializeQuadraticRamp(quadraticRamp *ramp, float target, float initial, float maximum, float final) { //maximum input and initial, maximum, and final output
  ramp->a = ((final + initial - 2*maximum) - 2*sqrt((final-maximum) * (initial-maximum))) / pow(target, 2);
	ramp->b = ((final-initial)/target - ramp->a*target) * sgn(target);
  ramp->c = initial;
}

float quadraticRampRuntime(quadraticRamp *ramp, float input) {
  return ramp->a*pow(input, 2) + ramp->b*input + ramp->c;
}
