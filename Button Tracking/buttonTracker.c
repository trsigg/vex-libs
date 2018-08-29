#define NUM_TRACKED_BTNS 24

bool tracking[NUM_TRACKED_BTNS];

void updateButtons() {
	for (int i=0; i<NUM_TRACKED_BTNS; i++)
		if (tracking[i] && vexRT[Btn5D + i]==0)
			tracking[i] = false;
}

void startTracking(TVexJoysticks button) {
	tracking[button - Btn5D] = true;
}

bool newlyPressed(TVexJoysticks button, bool startTrackingIfPressed=true) {
	updateButtons();

	if (!tracking[button - Btn5D] && vexRT[button]==1) {
		if (startTrackingIfPressed) startTracking(button);
		return true;
	}
	else {
		return false;
	}
}
