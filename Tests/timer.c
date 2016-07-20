/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file in the same directory as your code

	2. Include this line near the top of your code:
			| #include "timer.c"

	3. To create a new timer, include the following in your code:
			| long timerName;
		If you want the timer to start immediately upon creation, you can substitute the following instead:
			| long timerName = resetTimer();

	4. To reset a timer, use
			| timerName = resetTimer();

	5. To read the value of a timer, call time(timerName)
*/

long resetTimer() {
	return nPgmTime;
}

int time(long timer) {
	return nPgmTime - timer;
}
