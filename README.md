# 687G Override — VEX V5 Project

## Important: hardware is NOT ready to power

This project preserves the original wiring declarations, including a known, unresolved
smart-port conflict. The original project assigns **PORT 1** to both `frontLeft` and `liftMotor1`. A V5
smart port can only host one physical device, so that conflict must be resolved to
match the actual robot wiring before power-up.

**Cascade and lift are one and the same mechanism.** The two physical lift motors,
`liftMotor1` and `liftMotor2`, are grouped as `liftMotors`; every cascade/lift command
operates that one group. There is no separate cascade motor object or separate
mechanism. The `moveLift()` helper is only a compatibility alias for `moveCascade()`.

New mechanisms use clearly marked placeholders in `src/config.cpp`:
- Claw-angle motor: placeholder PORT 11
- Claw-grabber motor: placeholder PORT 12
- Intake motor: placeholder PORT 13

These are **not verified assignments**. The new motors currently use placeholder
`ratio18_1` cartridges and `reversed=false`; verify each actual cartridge and direction.
The existing drivetrain and lift declarations retain their original port, cartridge,
and reversal values, pending verification against the real wiring.

**Before powering the robot:** reconcile all existing conflicts, confirm every device's
port, confirm motor cartridge/gearing and reversal, and update the one hardware section
in `src/config.cpp`. Do not use this software configuration as a wiring diagram.

## Controller mapping

| Control | Function |
|---|---|
| R1 held | Cascade up |
| L1 held | Cascade down |
| R2 held | Claw angle outward/up |
| L2 held | Claw angle inward/down |
| Left arrow held | Grabber and intake forward |
| A held | Grabber and intake reverse (outtake) |
| X press | Nonblocking automatic scoring sequence |
| B press | Toggle arcade / tank drive |

Arcade drive retains Axis 3 forward and Axis 1 turn (turn scaled by 0.5).
Tank drive retains Axis 3 left and Axis 2 right. The original fixed 0.5 drive-speed
multiplier is preserved. A is no longer a drive-mode toggle.

When manual mechanism buttons are released, cascade, claw angle, and grabber use
`hold`; intake uses `brake`. During the X sequence, the state machine owns the claw
angle, cascade, grabber, and intake outputs, preventing manual commands from competing.

## X scoring sequence

1. Reverse grabber and intake for 500 ms.
2. Stop grabber and intake.
3. Command claw angle to `CLAW_MAX_ANGLE_DEG`.
4. Wait for its encoder to reach target (3 degree tolerance).
5. Command both motors in the single cascade/lift group to `CASCADE_ZERO_DEG`.
6. Wait for both encoders (3 degree tolerance), then hold.

Claw-angle timeout is 4 seconds; cascade timeout is 5 seconds. A timeout aborts
and holds/stops the mechanisms. X uses a rising-edge detector; holding X does not
restart the sequence. The sequence is nonblocking relative to the driver loop.

`CLAW_MAX_ANGLE_DEG` and `CASCADE_ZERO_DEG` in `src/config.cpp` are currently **0.0
motor encoder degrees placeholders**, not calibrated mechanism limits. Calibrate on
blocks at low speed and set safe encoder targets before using X. No gear-ratio
conversion is applied.

## Odometry

- Forward tracking wheel: PORT 5
- Lateral tracking wheel: PORT 6
- IMU: PORT 7
- Tracking wheel diameter: 2.00 inches (owner-confirmed)
- Starting forward offset: 2.25 inches
- Starting lateral offset: 3.85 inches

Offsets are in inches and editable in `src/config.cpp`. They are starting values,
not physically validated measurements. A single background task updates odometry
every 20 ms and remains active across autonomous and driver control. Movement
routines read the shared odometry state; they do not create a second updater.

## Autonomous movement API

Include `movement.h` and use the blocking movement calls from `autonomous()`:

```cpp
resetOdometry();
if (!driveDistance(24)) { stopDrive(); return; }
if (!turnToAngle(90)) { stopDrive(); return; }
if (!driveDistance(-12)) { stopDrive(); return; }
if (!turnRelative(-90)) { stopDrive(); return; }
if (!driveToPoint(24, 36)) { stopDrive(); return; }
```

- `driveDistance(inches)`: positive forward, negative backward; odometry distance PID
  with heading hold, slew-limited acceleration, target deceleration, timeout.
- `turnToAngle(degrees)`: absolute IMU/odometry heading, shortest wrapped turn.
- `turnRelative(degrees)`: relative turn from current heading.
- `driveToPoint(x, y)`: turn toward the target and drive the remaining distance.
- Each movement returns `true` on completion and `false` on timeout; drivetrain is
  stopped on either completion or failure.
- `setDrivePID(kP,kI,kD)` and `setTurnPID(kP,kI,kD)` change gains at runtime.
- Mechanism helpers include percent-output commands, stop commands, and position
  moves `moveCascadeToPosition()` / `moveClawAngleToPosition()`.

Coordinate convention follows the existing odometry implementation: heading 0 is
forward along +Y; positive heading rotates toward +X. Coordinates are inches.

## PID and calibration

The current gains in `src/config.cpp` are conservative **untested starting values**:
- Drive: P=2.0, I=0.0, D=0.08
- Turn: P=0.85, I=0.0, D=0.06
- Heading hold P=0.8
- Drive maximum output=65%; turn maximum output=45%
- Drive point tolerance=1.0 inch; acceleration slew=90 percentage points/sec

They have not been physically tuned. Test on blocks / in a clear test area, start with
low speed and short movements, tune one axis at a time, and verify stopping distances.
No successful VEX SDK build or on-robot test is claimed by this delivery.

## Files and build

The original VEX project structure, makefile, SDK make fragments, VEX project settings,
and source files are retained. Added:
- `include/movement.h`
- `src/movement.cpp`
- `README.md`

The build makefile automatically includes all `src/*.cpp` files.
