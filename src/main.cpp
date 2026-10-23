#include "vex.h"
#include <atomic>

using namespace vex;

// Competition program for the Spartanian V5 robot.
// Leave controller actions unassigned in the Devices menu: this program reads
// the joysticks and buttons itself.
competition Competition;
brain Brain;
controller Controller1(primary);

// Change a reversed flag only after testing that motor with the robot raised.
motor LeftFront(PORT1, false);
motor LeftRear(PORT11, false);
motor RightFront(PORT12, true);
motor RightRear(PORT10, true);
motor LiftLeft(PORT13, true);
motor LiftRight(PORT9, false);
motor Claw(PORT2, false);
motor WallLifter(PORT3, false);

std::atomic<bool> Armed(false);
std::atomic<bool> LiftBelowZeroFault(false);

// Place the lift at its LOWER reference before the first A after startup.
// This temporary upper limit is a calibration step, not the real top stop.
constexpr double LIFT_CALIBRATION_MAX_DEG = 3335;
constexpr double LIFT_SLOW_ZONE_DEG = 100;
// Place the claw OPEN before A. Closing travel is still a test bound, not the
// measured closed endpoint.
constexpr double CLAW_CALIBRATION_MAX_DEG = 211;
// Place the wall-button lifter at its physical DOWN position before first A.
constexpr double WALL_LIFTER_MAX_DEG = 130;
constexpr double WALL_LIFTER_SLOW_ZONE_DEG = 30;
constexpr double LIMIT_MARGIN_DEG = 5;
constexpr int DRIVE_MAX_PCT = 50;
constexpr int TURN_MAX_PCT = 30;
constexpr int PRECISION_DRIVE_PCT = 15;
constexpr int PRECISION_TURN_PCT = 8;
constexpr int DRIVE_RAMP_STEP_PCT = 2;  // Per 20 ms loop, about 0.5 s to full speed.
constexpr int LIFT_PCT = 30;
constexpr int LIFT_NEAR_LIMIT_PCT = 10;
constexpr int CLAW_PCT = 20;
constexpr int WALL_LIFTER_PCT = 25;
constexpr int WALL_LIFTER_NEAR_LIMIT_PCT = 8;
constexpr int JOYSTICK_DEADZONE = 8;

int axisInput(int value) {
  if (value >= -JOYSTICK_DEADZONE && value <= JOYSTICK_DEADZONE) return 0;
  return value;
}

// In precision mode, use the whole stick range after the deadzone so small
// commands respond more evenly at the lower maximum speed.
int linearAxis(int value) {
  value = axisInput(value);
  if (value == 0) return 0;
  const int magnitude = value < 0 ? -value : value;
  const int scaled = (magnitude - JOYSTICK_DEADZONE) * 100 /
                     (100 - JOYSTICK_DEADZONE);
  return value < 0 ? -scaled : scaled;
}

// Give small stick movements fine control and reserve full speed for the edge.
int shapedAxis(int value) {
  value = axisInput(value);
  if (value == 0) return 0;
  const int magnitude = value < 0 ? -value : value;
  const int scaled = (magnitude - JOYSTICK_DEADZONE) * 100 /
                     (100 - JOYSTICK_DEADZONE);
  const int curved = scaled * scaled / 100;
  const int blended = (7 * scaled + 3 * curved + 5) / 10;
  return value < 0 ? -blended : blended;
}

// Make small steering movements gentle for aligning the claw with a cone.
int shapedSteering(int value) {
  value = axisInput(value);
  if (value == 0) return 0;
  const int magnitude = value < 0 ? -value : value;
  const int scaled = (magnitude - JOYSTICK_DEADZONE) * 100 /
                     (100 - JOYSTICK_DEADZONE);
  const int curved = scaled * scaled / 100;
  const int blended = (4 * scaled + 6 * curved + 5) / 10;
  return value < 0 ? -blended : blended;
}

int rampToward(int current, int target) {
  if (target > current + DRIVE_RAMP_STEP_PCT)
    return current + DRIVE_RAMP_STEP_PCT;
  if (target < current - DRIVE_RAMP_STEP_PCT)
    return current - DRIVE_RAMP_STEP_PCT;
  return target;
}

void runMotor(motor& m, int speed, brakeType stopped) {
  if (speed > 0) m.spin(forward, speed, percent);
  else if (speed < 0) m.spin(reverse, -speed, percent);
  else m.stop(stopped);
}

void stopAll() {
  LeftFront.stop(brake); LeftRear.stop(brake);
  RightFront.stop(brake); RightRear.stop(brake);
  LiftLeft.stop(brake); LiftRight.stop(brake);
  Claw.stop(brake);
  WallLifter.stop(brake);
}

int missingMotorPort() {
  if (!LeftFront.installed()) return 1;
  if (!LeftRear.installed()) return 11;
  if (!RightFront.installed()) return 12;
  if (!RightRear.installed()) return 10;
  if (!LiftLeft.installed()) return 13;
  if (!LiftRight.installed()) return 9;
  if (!Claw.installed()) return 2;
  return 0;
}

// Redraw only when the status changes so the screen does not flicker.
void showStatus(int state, int missingPort) {
  static int previousState = -1;
  static int previousPort = -1;
  if (state == previousState && missingPort == previousPort) return;
  previousState = state;
  previousPort = missingPort;

  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(10, 45, "SPARTANIAN - slot 8");
  if (state == 1) Brain.Screen.printAt(10, 85, "READY: driver control");
  else if (state == 2) Brain.Screen.printAt(10, 85, "Lift low, claw open, wall low: A");
  else if (state == 3) Brain.Screen.printAt(10, 85, "Controller disconnected");
  else if (state == 4) Brain.Screen.printAt(10, 85, "Motor missing on port %d", missingPort);
  else if (state == 7) Brain.Screen.printAt(10, 85, "Lift below zero; check, restart");
  Brain.Screen.printAt(10, 125, "Left drive  Right turn  Hold B: slow");
  Brain.Screen.printAt(10, 165, "L1/L2 lift R1/R2 claw Up/Down wall");
  Brain.Screen.printAt(10, 240, "Lift 0..3330 Claw 0..206 Wall 0..130");
}

bool controlsReleased() {
  return axisInput(Controller1.Axis1.position()) == 0 &&
         axisInput(Controller1.Axis3.position()) == 0 &&
         !Controller1.ButtonL1.pressing() && !Controller1.ButtonL2.pressing() &&
         !Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing() &&
         !Controller1.ButtonUp.pressing() && !Controller1.ButtonDown.pressing();
}

void pre_auton() {
  // Encoder references are set by A after the mechanisms have been placed at
  // their physical starting positions, including before field driver control.
  LiftLeft.setMaxTorque(50, percent);
  LiftRight.setMaxTorque(50, percent);
  Claw.setMaxTorque(90, percent);
  WallLifter.setMaxTorque(100, percent);
  stopAll();
}

void autonomous() {
  stopAll();
}

void usercontrol() {
  int filteredThrottle = 0;
  int filteredSteering = 0;
  int screenTicks = 0;
  while (true) {
    if (!Armed.load()) {
      filteredThrottle = 0;
      filteredSteering = 0;
      stopAll();
      wait(20, msec);
      continue;
    }

    // Teammate's stick mapping: left vertical drives; right horizontal turns.
    const bool precision = Controller1.ButtonB.pressing();
    const int throttle = -(precision ? linearAxis(Controller1.Axis3.position())
                                     : shapedAxis(Controller1.Axis3.position()));
    const int steering = -(precision ? linearAxis(Controller1.Axis1.position())
                                     : shapedSteering(Controller1.Axis1.position()));
    const int driveLimit = precision ? PRECISION_DRIVE_PCT : DRIVE_MAX_PCT;
    const int turnLimit = precision ? PRECISION_TURN_PCT : TURN_MAX_PCT;
    const int targetThrottle = throttle * driveLimit / 100;
    const int targetSteering = steering * turnLimit / 100;
    // Centering either stick removes that component immediately. In particular,
    // releasing the right stick stops turning even while driving forward.
    filteredThrottle = throttle == 0 ? 0 :
                       rampToward(filteredThrottle, targetThrottle);
    filteredSteering = steering == 0 ? 0 :
                       rampToward(filteredSteering, targetSteering);
    if (precision) {
      if (filteredThrottle > driveLimit) filteredThrottle = driveLimit;
      if (filteredThrottle < -driveLimit) filteredThrottle = -driveLimit;
      if (filteredSteering > turnLimit) filteredSteering = turnLimit;
      if (filteredSteering < -turnLimit) filteredSteering = -turnLimit;
    }
    int leftMix = filteredThrottle + filteredSteering;
    int rightMix = filteredThrottle - filteredSteering;
    const int leftMagnitude = leftMix < 0 ? -leftMix : leftMix;
    const int rightMagnitude = rightMix < 0 ? -rightMix : rightMix;
    const int peak = leftMagnitude > rightMagnitude ? leftMagnitude : rightMagnitude;
    if (peak > driveLimit) {
      leftMix = leftMix * driveLimit / peak;
      rightMix = rightMix * driveLimit / peak;
    }
    runMotor(LeftFront, leftMix, brake);
    runMotor(LeftRear, leftMix, brake);
    runMotor(RightFront, rightMix, brake);
    runMotor(RightRear, rightMix, brake);

    const double liftRightPosition = LiftRight.position(degrees);
    const double clawPosition = Claw.position(degrees);
    const double wallPosition = WallLifter.position(degrees);
    if (++screenTicks >= 10) {
      Brain.Screen.printAt(10, 205, "Lift:%5.0f Claw:%5.0f Wall:%5.0f deg  ",
                           -liftRightPosition, -clawPosition, wallPosition);
      screenTicks = 0;
    }

    if (liftRightPosition > LIMIT_MARGIN_DEG) {
      LiftBelowZeroFault.store(true);
      Armed.store(false);
      stopAll();
      wait(20, msec);
      continue;
    }
    const bool l1 = Controller1.ButtonL1.pressing();
    const bool l2 = Controller1.ButtonL2.pressing();
    int liftSpeed = 0;
    const double liftHeight = -liftRightPosition;
    const double upperLimit = LIFT_CALIBRATION_MAX_DEG - LIMIT_MARGIN_DEG;
    if (l1 && !l2 && liftHeight < upperLimit)
      liftSpeed = upperLimit - liftHeight <= LIFT_SLOW_ZONE_DEG
                      ? -LIFT_NEAR_LIMIT_PCT : -LIFT_PCT;
    if (l2 && !l1 && liftHeight > LIMIT_MARGIN_DEG)
      liftSpeed = liftHeight <= LIFT_SLOW_ZONE_DEG
                      ? LIFT_NEAR_LIMIT_PCT : LIFT_PCT;
    runMotor(LiftLeft, liftSpeed, brake);
    runMotor(LiftRight, liftSpeed, brake);

    int clawSpeed = 0;
    const bool r1 = Controller1.ButtonR1.pressing();
    const bool r2 = Controller1.ButtonR2.pressing();
    const double clawClosedDegrees = -clawPosition;
    if (r1 && !r2 && clawClosedDegrees < CLAW_CALIBRATION_MAX_DEG - LIMIT_MARGIN_DEG)
      clawSpeed = -CLAW_PCT;
    if (r2 && !r1 && clawClosedDegrees > LIMIT_MARGIN_DEG)
      clawSpeed = CLAW_PCT;
    runMotor(Claw, clawSpeed, brake);

    // Manual wall-button lifter, bounded by its motor encoder reference.
    const bool wallUp = Controller1.ButtonUp.pressing();
    const bool wallDown = Controller1.ButtonDown.pressing();
    int wallSpeed = 0;
    if (wallUp && !wallDown && wallPosition < WALL_LIFTER_MAX_DEG)
      wallSpeed = WALL_LIFTER_MAX_DEG - wallPosition <= WALL_LIFTER_SLOW_ZONE_DEG
                      ? WALL_LIFTER_NEAR_LIMIT_PCT : WALL_LIFTER_PCT;
    if (wallDown && !wallUp && wallPosition > 0)
      wallSpeed = wallPosition <= WALL_LIFTER_SLOW_ZONE_DEG
                      ? -WALL_LIFTER_NEAR_LIMIT_PCT : -WALL_LIFTER_PCT;
    runMotor(WallLifter, wallSpeed, brake);
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  bool referenceSet = false;
  bool aWasDown = false;
  while (true) {
    const bool connected = Controller1.installed();
    const int missingPort = missingMotorPort();
    const bool aDown = connected && Controller1.ButtonA.pressing();
    const bool xDown = connected && Controller1.ButtonX.pressing();

    if (!connected || missingPort != 0 || xDown) Armed.store(false);
    if (!Armed.load() && !LiftBelowZeroFault.load() && connected &&
        missingPort == 0 && aDown && !aWasDown && !xDown &&
        controlsReleased()) {
      if (!referenceSet) {
        LiftLeft.setPosition(0, degrees);
        LiftRight.setPosition(0, degrees);
        Claw.setPosition(0, degrees);
        WallLifter.setPosition(0, degrees);
        referenceSet = true;
      }
      Armed.store(true);
    }
    aWasDown = aDown;

    if (!connected) showStatus(3, 0);
    else if (missingPort != 0) showStatus(4, missingPort);
    else if (LiftBelowZeroFault.load()) showStatus(7, 0);
    else showStatus(Armed.load() ? 1 : 2, 0);
    wait(20, msec);
  }
}
