int limit(int input, int min, int max) {
	if (input <= max && input >= min) {
		return input;
	}
	else {
		return (input > max ? max : min);
	}
}

float power(float base, float power) {
	return exp( power * log(abs(base)) );
}
