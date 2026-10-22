#include "vex.h"

using namespace vex;

brain Brain;
controller Controller1(primary);

// Same motor directions as Spartanian in slot 8.
motor LiftLeft(PORT13, true);
motor LiftRight(PORT9, false);
motor Claw(PORT2, false);

constexpr int LIFT_PCT = 30;
constexpr int CLAW_PCT = 20;

void stopMotors() {
  LiftLeft.stop(brake);
  LiftRight.stop(brake);
  Claw.stop(brake);
}

void runMotor(motor& m, int speed) {
  if (speed > 0) m.spin(forward, speed, percent);
  else if (speed < 0) m.spin(reverse, -speed, percent);
  else m.stop(brake);
}

bool buttonsReleased() {
  return !Controller1.ButtonL1.pressing() &&
         !Controller1.ButtonL2.pressing() &&
         !Controller1.ButtonR1.pressing() &&
         !Controller1.ButtonR2.pressing();
}

void showStatus(int state) {
  static int previousState = -1;
  if (state == previousState) return;
  previousState = state;

  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(10, 45, "SPARTANIAN JOG - slot 7");
  if (state == 0) Brain.Screen.printAt(10, 85, "Release buttons, press A to arm");
  else if (state == 1) Brain.Screen.printAt(10, 85, "READY: hold a button to move");
  else if (state == 2) Brain.Screen.printAt(10, 85, "Controller disconnected");
  else if (state == 3) Brain.Screen.printAt(10, 85, "Check motors on ports 13, 9, 2");
  Brain.Screen.printAt(10, 130, "L1 up  L2 down");
  Brain.Screen.printAt(10, 165, "R1 close  R2 open");
  Brain.Screen.printAt(10, 200, "X stop; A arm again");
  Brain.Screen.printAt(10, 235, "NO TRAVEL LIMITS - WATCH MECHANISM");
}

int main() {
  LiftLeft.setMaxTorque(50, percent);
  LiftRight.setMaxTorque(50, percent);
  Claw.setMaxTorque(90, percent);
  stopMotors();

  bool armed = false;
  bool aWasDown = false;
  while (true) {
    const bool connected = Controller1.installed();
    const bool motorsPresent = LiftLeft.installed() &&
                               LiftRight.installed() && Claw.installed();
    const bool aDown = connected && Controller1.ButtonA.pressing();

    if (!connected || !motorsPresent || Controller1.ButtonX.pressing())
      armed = false;
    if (!armed && connected && motorsPresent && aDown && !aWasDown &&
        buttonsReleased() && !Controller1.ButtonX.pressing())
      armed = true;
    aWasDown = aDown;

    if (!armed) {
      stopMotors();
      showStatus(!connected ? 2 : !motorsPresent ? 3 : 0);
      wait(20, msec);
      continue;
    }

    showStatus(1);
    const bool l1 = Controller1.ButtonL1.pressing();
    const bool l2 = Controller1.ButtonL2.pressing();
    const bool r1 = Controller1.ButtonR1.pressing();
    const bool r2 = Controller1.ButtonR2.pressing();

    const int liftSpeed = l1 == l2 ? 0 : l1 ? -LIFT_PCT : LIFT_PCT;
    const int clawSpeed = r1 == r2 ? 0 : r1 ? -CLAW_PCT : CLAW_PCT;
    runMotor(LiftLeft, liftSpeed);
    runMotor(LiftRight, liftSpeed);
    runMotor(Claw, clawSpeed);
    wait(20, msec);
  }
}
