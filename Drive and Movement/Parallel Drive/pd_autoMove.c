/*
/////////////////  INSTRUCTIONS  /////////////////


	1. Save this file in the same directory as your code

	2. Include this line near the top of your code:
			| #include "pd_autoMove.c"

	3. Change #define statements on lines 29 and 30 of this file to the number of motor ports used by each side of the drive

	4. To create drive, include the following lines in your code:
			| parallelMover moverName;
			| initializeMover(moverName, &driveName);
	   Any valid variable name can be substituted for moverName
	   driveName should be a parallel_drive object as defined in parallelDrive.c
	   The optional arguments of initializeDrive can be used to further configure the drive
	   	e.g. a user creating a drive with ramping and quadratic input mapping would substitute the fourth line of code with:
	   	| initializeDrive(driveName, &leftMotors, &rightMotors, true, 20, 10, 2);

	5. 