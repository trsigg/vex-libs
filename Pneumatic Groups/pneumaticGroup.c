#include "coreIncludes.c"
#include "buttonTracker.c"
#include "timer.c"

enum pneumaticControlType { P_NONE, TOGGLE, TWO_BTN };

typedef struct {
  tSensors solenoids[10];
  int numSolenoids;
  //movement handling
  int defMoveDuration;
  long moveTimer;
  //button control
  TVexJoysticks openOrToggleBtn, closeBtn;
  pneumaticControlType controlScheme;
  bool isOpen;
} pneumaticGroup;

void initializePneumaticGroup(pneumaticGroup *group, int numSolenoids, tSensors *solenoids, int defMoveDuration=0) {
  group->numSolenoids = limit(numSolenoids, 0, 10);
  group->defMoveDuration = defMoveDuration;

  for (int i=0; i<group->numSolenoids; i++)
    group->solenoids[i] = solenoids[i];

  group->isOpen = false;
}

void initializePneumaticGroup(pneumaticGroup *group, tSensors solenoid, int defMoveDuration=0) {
  group->numSolenoids = 1;
  group->solenoids[0] = solenoid;

  group->defMoveDuration = defMoveDuration;
  group->isOpen = false;
}

void configureTwoBtnInput(pneumaticGroup *group, TVexJoysticks openBtn, TVexJoysticks closeBtn) {
  group->openOrToggleBtn = openBtn;
  group->closeBtn = closeBtn;
  group->controlScheme = TWO_BTN;
}

void configureToggleInput(pneumaticGroup *group, TVexJoysticks toggleBtn) {
  group->openOrToggleBtn = toggleBtn;
  group->controlScheme = TOGGLE;
}

void waitForMovementToFinish(pneumaticGroup *group, int moveDuration=-1) {  //negative moveTime defaults to defMoveDuration
  if (moveDuration < 0)
    moveDuration = group->defMoveDuration;

  while (time(group->moveTimer) < moveDuration) EndTimeSlice();
}

bool setState(pneumaticGroup *group, bool open, bool runConcurrently=true, int moveDuration=-1) {
  group->moveTimer = resetTimer();

  for (int i=0; i<group->numSolenoids; i++)
    SensorValue[ group->solenoids[i] ] = open; //true casts to 1, false to 0

  group->isOpen = open;

  if (!runConcurrently)
    waitForMovementToFinish(group, moveDuration);

  return open;
}

bool takeInput(pneumaticGroup *group) { //returns whether solenoid is open
  switch (group->controlScheme) {
    case TOGGLE:
      if (newlyPressed(group->openOrToggleBtn))
        setState(group, !group->isOpen);
      break;
    case TWO_BTN:
      if (vexRT[group->openOrToggleBtn] == 1)
        setState(group, true);
      else if (vexRT[group->closeBtn] == 1)
        setState(group, false);
      break;
  }

  return group->isOpen;
}
