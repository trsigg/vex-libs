int limit(int input, int min, int max) {
	if (input <= max && input >= min) {
		return input;
	}
	else {
		return (input > max ? max : min);
	}
}

enum angleType { DEGREES, RADIANS, RAW };

float convertAngle(float angle, angleType output, angleType input=RAW) {
	if (input == DEGREES) { //convert input to RAW
		angle *= 10;
	} else if (input == RADIANS) {
		angle *= 1800 / PI;
	}

	if (output == DEGREES) {
		angle /= 10;
	} else if (output == RADIANS) {
		angle *= PI / 1800;
	}

	return angle;
}
