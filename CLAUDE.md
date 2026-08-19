# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

FRC Team 955 robot code for the 2026 season. Java 17 / WPILib 2026 + AdvantageKit, plus a
Raspberry Pi coprocessor subproject (`gamepiecevision/`) and a CircuitPython operator keypad.

## Commands

Everything goes through Gradle (`./gradlew`). There is no lint/format task — formatting is
enforced by `.editorconfig` (4 spaces, 120 col, LF) via the IDE.

```bash
./gradlew buildReal -x test        # compile for the roboRIO (what CI runs)
./gradlew test                     # JUnit 5 tests (also runs generateBuildConstantsSimTuningMode)
./gradlew runSimTuningMode -x test # physics sim + sim GUI + tuning mode
./gradlew runReal -x test          # build + deploy to the robot
./gradlew runRealTuningMode -x test
./gradlew runRealEvent -x test     # git auto-commit, then deploy (use at competitions)
./gradlew runReplay -x test        # AdvantageKit log replay (set REPLAYENABLED=1 to skip sim GUI)
./gradlew replayWatch              # AdvantageScope replay watch
./gradlew updateVendordeps         # pull newer vendordep JSONs
```

Single test: `./gradlew test --tests 'frc.lib.commands.EagerSequentialCommandGroupTests'`
(add `.methodName` to narrow further). Tests need the JNI natives on the library path — the
`.vscode/settings.json` `WPIlibUnitTests` config does that for IDE-run tests.

IntelliJ run configs for all of the above live in `.run/`.

Shooting regression fitting is Python: `src/main/java/frc/robot/shooting/shooting_regression.py`
(run with cwd `src/main/java/frc/robot`); its output is pasted into `ShootingRegression.java`.

### BuildConstants — the mode switch

`src/main/java/frc/robot/BuildConstants.java` is **generated** by the
`generateBuildConstants{Real,RealTuningMode,SimTuningMode,Replay}` tasks. Never hand-edit it, and
never run a bare `./gradlew build`/`simulateJava` — always use the `build*`/`run*` wrappers so the
right mode is baked in. `BuildConstants.mode` (REAL/SIM/REPLAY) and `BuildConstants.tuningMode`
drive nearly every conditional in the codebase.

`deploy` depends on `deployAlerts` (`src/test/java/DeployAlerts.java`), which pops a Swing
confirmation dialog if tuning mode, `disableDriving`, or `disableGyro` is on.

## Architecture

### Singletons + explicit periodic ordering

Every subsystem and manager is a singleton: `private constructor` + `public static synchronized
X get()` + a `Util.error("Duplicate X created")` guard. `RobotContainer` holds one field per
singleton; other classes grab dependencies via `private static final Foo foo = Foo.get();`.

`Robot` does **not** rely on WPILib's `SubsystemBase.periodic()` — `CommandBasedSubsystem.periodic()`
is `final` and empty. Instead everything implements `frc.lib.subsystem.Periodic`
(`periodicBeforeCommands` / `periodicAfterCommands`) and `Robot` runs an explicitly ordered
`List<Periodic>` around `CommandScheduler.run()`. **Order matters** (hub tracker → controller →
drive → vision → robotState → shooting → dashboard → super* → leaf subsystems → mechanism/LEDs).
Adding a subsystem means adding it to that list in the right slot; nothing runs otherwise.

No references to `RobotContainer`/`RobotState`/subsystems may be made before `new RobotContainer()`
in the `Robot` constructor (logger metadata and sim arena setup happen first).

### AdvantageKit IO layer

Each mechanism is a package: `Foo.java` (logic), `FooConstants.java` (gains, CAN IDs, and a
`static FooIO createIO()` that switches on `BuildConstants.mode` → TalonFX/Spark/… for REAL, a
`*IOSim` for SIM, and a bare `FooIO` no-op for REPLAY), `FooIO.java` (with `@AutoLog` inputs),
plus concrete IO impls. Logic calls `io.updateInputs(inputs)` then
`Logger.processInputs("Inputs/<Path>", inputs)` at the top of `periodicBeforeCommands`. Keep all
hardware access behind the IO interface or replay breaks.

Shared motor IO wrappers live in `frc/lib/motor/` (`MotorIOTalonFX`, `MotorIOSparkMax`,
`MotorIOSim`, `MotorIOArmSim`).

### Goal-based superstructures

`Superintake` and `Superstructure` are the two command-based subsystems; their child mechanisms
(flywheel, hood, feeder, spindexer, intakePivot, intakeRollers) are plain `Periodic` objects owned
by the parent, not WPILib subsystems. Commands only ever set a `Goal` enum
(`superstructure.setGoal(Goal.SHOOT)`); each child reads the parent's goal and its own `Goal`
supplier to compute setpoints. Goal enums carry `DoubleSupplier` setpoints that must be constant
within a loop cycle. Homing has dedicated `setGoalHome*()` sequences — don't pass `HOME_*` goals to
`setGoal`.

### RobotState, vision, shooting

`RobotState` is the single pose estimator: `applyOdometryUpdate` from drive, `addVisionMeasurement`
from `AprilTagVision`, and everything else reads `getPose()`/`getPoseAtTimestamp()`.
`ShootingKinematics` turns robot pose + `ShootingRegression`/`PassingRegression` into flywheel RPM
and hood angle; `HubShiftTracker` corrects the goal's apparent position.

`AllianceFlipUtil` / `AllianceBasedPose2d` handle blue-origin → alliance flipping; field geometry
is in `FieldConstants`.

### Autos

Each auto is a class in `frc/robot/autos/` extending `Auto` (a `Pose2d startingPose` + a
`Command`); the base class prepends the dashboard auto delay and a pose reset. Register it in
`AutoManager`'s constructor to make it selectable. Trajectories come from Choreo
(`src/main/deploy/choreo/`, referenced via `ChoreoTraj`/`ChoreoVars`).

### Tuning & dashboard

`LoggedTunableNumber` / `LoggedTunablePIDF` / `LoggedNetworkNumberExt` publish to NetworkTables only
when `BuildConstants.tuningMode`; otherwise they return the compiled default and `hasChanged()` is
always false. So gain-changing code must be written as `if (gains.hasChanged()) io.setPIDF(...)`.
`OperatorDashboard` holds the operator-facing NT toggles (homing buttons, auto delay, etc.).

### Command helpers

`frc/lib/commands/` adds WPILib-missing pieces used throughout: `CommandsExt.eagerSequence`
(`EagerSequentialCommandGroup` — requirements acquired up front, so a sequence can't be interrupted
mid-way), `startIdle`, `waitUntilRequirements`, `suppliedWaitSeconds`. Prefer these over raw
`Commands.sequence` in superstructure/auto code.

### Subprojects

- `gamepiecevision/` — independent Gradle project deployed as `gamepiecevision.jar` to a
  PhotonVision Pi (`gamepiecevision.service`). Runs DBSCAN + Hungarian multi-object fuel tracking
  and publishes over NT.
- `shared/src/main/java` — sources shared between the robot and the Pi; added to the robot's `main`
  source set via `sourceSets` in `build.gradle` (edit once, used by both).
- `operator_keypad/` — CircuitPython (`code.py`) for the operator button board.
- `visualizer/`, `advantagescope_assets/`, `AdvantageScope.json`, `elastic-layout.json` — dashboard
  assets and layouts.

## Conventions

- Much of `frc/lib` and `Robot.java`/`Constants.java` is derived from FRC 6328's AdvantageKit
  template and keeps their GPLv3 header — preserve it when editing those files.
- Lombok (`@Getter`/`@Setter`/`@RequiredArgsConstructor`) is enabled via the freefair plugin.
- `Util.error(...)` is the project's "this should never happen" reporter; hardware faults surface as
  WPILib `Alert`s, one per failure mode, set every loop.
- `LoggedTracer.record(...)` brackets each periodic for loop-timing telemetry.
- `frc/lib/example/ExampleRollerSubsystem` + `ExampleServoSubsystem` are the reference templates for
  a new mechanism.
- `hs_err_pid*.log` files in the root are JVM crash dumps, not source.
