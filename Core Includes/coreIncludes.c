//#region units
enum angleType { DEGREES, RADIANS, RAW_ANGLE };

float convertAngle(float angle, angleType output, angleType input=RAW_ANGLE) {
	if (input != output) {
		//convert input to RAW_ANGLE

		switch (input) {
			case DEGREES:
				angle *= 10;
				break;
			case RADIANS:
				angle *= 1800 / PI;
				break;
		}

		//final conversion
		switch (output) {
			case DEGREES:
				angle /= 10;
				break;
			case RADIANS:
				angle *= PI / 1800;
				break;
		}
	}

	return angle;
}


enum distUnits { INCH, CM, MM, RAW_DIST };

float convertDist(float dist, distUnits output, distUnits input=INCH) {
	if (input != output) {
		//convert input to MM
		switch (input) {
			case CM:
				dist *= 10;
				break;
			case INCH:
				dist *= 25.4;
				break;
		}

		//final conversion
		switch (output) {
			case CM:
				dist /= 10;
				break;
			case INCH:
				dist /= 25.4;
				break;
		}
	}

	return dist;
}
//#endregion

//#region numerical operations
float copysign(float sign, float magnitude) {
	return sgn(sign) * fabs(magnitude);
}

float min(float val1, float val2) {
	return (val1>val2 ? val2 : val1);
}

float max(float val1, float val2) {
	return (val1>val2 ? val1 : val2);
}

int limit(float input, float min, float max) {
	if (input <= max && input >= min) {
		return input;
	}
	else {
		return (input > max ? max : min);
	}
}

/*int limit(float* input, float min, float max) {	doesn't work for some reason
	*input = limit(*input, min, max);
	return *input;
}*/

float tan(float x) {
	return sin(x)/cos(x);
}
//#endregion


void arrayCopy(void* source, void* destination, int elements) {
	for (int i=0; i<elements; i++)
		destination[i] = source[i];
}
