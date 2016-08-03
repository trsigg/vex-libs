/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file in the same directory as your code

	2. Include this line near the top of your code:
			| #include "debouncer.c"

	3. To start tracking the state of a particular button, call startDebounce(buttonName)

	4. To find whether a button has been newly pressed (is being pressed after having been released since the previous press/when tracking was last started), call newlyPressed(buttonName)
		This will automatically update button states and start tracking if button is pushed (optional argument can be used to configure the latter)

	4. Whenever the button states should be updated (probably once every input cycle) include the following line of code:
			| updateButtons();

	5. To change the target value of the PID loop, use
			| pidName.target = newTarget;
*/

TVexJoysticks buttons[12] = { Btn5U, Btn5D, Btn6U, Btn6D, Btn7U, Btn7D, Btn7L, Btn7R, Btn8U, Btn8D, Btn8L, Btn8R };
bool tracking[12] = { false, false, false, false, false, false, false, false, false, false, false, false };

int findBtnIndex(TVexJoysticks button) {
	for (int i=0; i<12; i++) {
		if (buttons[i] == button) return i;
	}

	return 0;
}

void updateButtons() {
	for (int i=0; i<12; i++) {
		if (tracking[i] && vexRT[buttons[i]]==0) tracking[i]=false;
	}
}

void startTracking(TVexJoysticks button) {
	tracking[ findBtnIndex(button) ] = true;
}

bool newlyPressed(TVexJoysticks button, bool startTrackingIfPressed=true) {
	updateButtons();

	int index = findBtnIndex(button);

	if (!tracking[index] && vexRT[buttons[index]]==1) {
		if (startTrackingIfPressed) startTracking(button);
		return true;
	} else {
		return false;
	}
}
