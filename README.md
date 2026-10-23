# Spartanian: VEX V5 controller program

This folder is a **VEX VS Code C++** project. The editable source is
`src/main.cpp`; `main.cpp` at the top level points to that same file.
It uses ports 1 and 11 for the left drive, 12 and 10 for the right drive,
13 and 9 for the paired lift, 2 for the claw, and 3 for the wall-button lifter.
It assumes the V5 Robot Radio is on a different Smart Port. Drive and lift
directions were checked on this robot.

The Spartanian competition program uses Brain slot **8**. Slot 1's existing
`testProj` is left alone.

Autonomous is intentionally empty. The driver callback starts when the field
enters driver control. Place the lift at its lower reference, the claw fully
open, and the wall-button lifter at its physical down position before starting;
press A with the controls released before the match.
A sets those three encoder references on its first press after program
startup. Later A presses rearm without changing them. X stops and disarms.

## Manual recovery: slot 7

`recovery/src/main.cpp` builds a separate `SpartanianJog` program for slot 7.
It controls only the two lift motors and claw. Start it, release the mechanism
buttons, then press A to arm. Hold L1 to raise the lift, L2 to lower it, R1 to
close the claw, or R2 to open it. Releasing a button stops that mechanism;
pressing X stops and disarms everything until A is pressed again. Opposing
buttons pressed together stop that mechanism. The drive motors are never
commanded by this program.

This program does not zero encoders and has no software travel endpoints or
stall detection. Use it only to manually put the lift at its lower reference
and the claw fully open. Watch the mechanism throughout: it will keep driving
while a button is held even at a physical stop. Then select slot 8 and press A
with the controls released to set its encoder references. Slot 7 uses 30% lift
speed with 50% torque per lift motor, and 20% claw speed with 90% torque.

The Brain screen asks for the lift low, claw open, and wall lifter low before A.
If it shows `Motor missing on port ...`, check that Smart Cable and the Brain's
Devices screen. `Controller disconnected` means the radio/controller link is
not available to the program. `READY` means the left stick controls speed and
the right stick controls steering.

The stick directions are inverted to match the physical robot. Forward drive
speed is limited to 50%, and turning to 30%. Small steering movements produce
gentler turns, and motor speed ramps up over about half a second. Centering the
right stick removes steering immediately, even if the left stick is still held.
Hold B for cone alignment: forward drive is limited to 15% and turning to 8%.
The precision stick response is more even near the center.
Releasing both sticks or pressing X stops the drive immediately.

## Controls

| Control | Action |
| --- | --- |
| Left stick up/down (Axis 3) | Forward / reverse |
| Right stick left/right (Axis 1) | Turn left / right |
| Hold B | Precision drive and turning for cone alignment |
| Hold L1 / L2 | Paired lift test up / down; release to stop |
| Hold R1 / R2 | Close / open claw; release to stop |
| Hold up / down arrow | Raise / lower the port 3 wall-button lifter |
| X | Disarm and stop |
| A, with sticks centered and mechanism buttons released | Arm or rearm |

The port 3 lifter runs at up to 25% speed with a 100% torque setting while an
arrow is held. Both arrows together stop it. The first A marks its physical
down position as 0 motor degrees; up stops at about 130 motor degrees and down
stops near 0. It slows to 8% within 30 degrees of either bound. These are
encoder bounds, not physical end sensors; release the arrow if it binds. If up
and down move it backward on the robot, reverse its motor direction after a test.

There are no installed limit switches. Single-motor tests showed that the
original port 13 direction was opposite to port 9. This version reverses port
13 and commands both motors together at up to 30% speed, with a 50% torque limit on
each motor. Before the **first** press of A after startup, position and support
the lift at its safe lower reference. A sets both motor encoders to zero. L2
cannot lower below zero, while L1 can raise up to about 3330 motor degrees for
the current calibration step. The Brain displays height from the lower
reference and the claw encoder. Speed drops to 10% within 100
motor degrees of either software limit. If port 9 travels below zero by more
than 5 degrees, the program stops and requires a restart. There is no automatic
lift stall or motor difference shutdown; release the button immediately if the
lift stops moving or
the chain goes taut. The upper calibration bound is not the
physical top endpoint. Stop if the lift binds or the chain is pulled taut.
`brake` does not actively hold the lift.

The claw direction is now confirmed: R1 closes and R2 opens. Before the first
press of A after startup, place the jaws at their safe fully open position;
A sets the claw encoder to zero. R1 and R2 drive at 20% speed with a 90%
torque limit. R1 can close only to about 206 motor degrees from open; R2 stops
near zero. The claw has no automatic stall detection; release the button if it
stops moving or binds. The physical closed endpoint has not been measured yet,
so this remains a calibration range.

## First test

1. Put the robot on a stand with wheels off the floor. Clear hands, hair,
   loose cables, and the lift path. Support the lift so it cannot fall.
2. Pair the controller and Brain, load the project, and check that all four
   drive motors, both lift motors, the claw, and the port 3 motor appear on the
   Brain's Devices screen.
3. With the sticks centered, press A. Test one stick and one direction at a
   time. If a motor on one side fights the other, stop with X and change its
   `true`/`false` reversed flag. If a whole side runs backward, reverse both
   motors on that side.
4. With both motors connected, support and position the lift at its safe lower
   reference. Press A to zero the encoders. Hold L1 briefly and check that the
   lift rises. Release L1, then hold L2 to lower it. L2 stops near zero. Press
   X immediately if the motors fight, bind, or make a harsh noise. If the Brain
   reports a lift fault, inspect the mechanism before restarting.
5. Raise toward the temporary 3330-degree limit only while watching the lift.
   Report the displayed height and how far the lift has risen at that point.
   Measure the physical safe travel before increasing this software range.
6. With the lift safely supported, place the claw fully open before starting
   the program and pressing A. Hold R1 to close it within the temporary
   206-degree range, then R2 to reopen it. Release immediately if it binds or
   makes a harsh noise. Report how far the claw moves at the limit.
7. Place the port 3 lifter at its down position before the first A. Hold the
   up arrow briefly and verify that it rises and the displayed wall angle
   increases. Test down next. Only then approach its 130-degree software bound.

Software encoder bounds only work while the reference position remains valid.
For reliable end protection, fit limit switches at the lift's bottom and top
and suitable claw open/closed feedback, then update the program to use them.
Torque limits reduce force but cannot guarantee that gears, fingers, or the
mechanism will be protected. Support the lift mechanically when working on
the robot.
