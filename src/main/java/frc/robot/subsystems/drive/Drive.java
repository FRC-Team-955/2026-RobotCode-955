package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.controller.Controller;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.drive.controllers.FollowTrajectoryController;
import frc.robot.subsystems.drive.controllers.MoveToController;
import lombok.Getter;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;
import java.util.function.IntToDoubleFunction;
import java.util.function.Supplier;

import static frc.lib.HighFrequencySamplingThread.highFrequencyLock;
import static frc.robot.subsystems.drive.DriveConstants.*;

public class Drive extends CommandBasedSubsystem {
    private static final RobotState robotState = RobotState.get();
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final Controller controller = Controller.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private final GyroIO gyroIO = createGyroIO();
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final AccelerometerIO accelerometerIO = createAccelerometerIO();
    private final AccelerometerIOInputsAutoLogged accelerometerInputs = new AccelerometerIOInputsAutoLogged();

    private final LinearFilter accelerationXFilter = LinearFilter.movingAverage(4);
    private final LinearFilter accelerationYFilter = LinearFilter.movingAverage(4);

    private final Debouncer gyroDebouncer = new Debouncer(2.0, Debouncer.DebounceType.kRising);
    private boolean gyroDebounced = false;

    public enum State {
        STOP,
        JOYSTICK_DRIVE,
        MOVE_TO,
        FOLLOW_TRAJECTORY,
        CHASSIS_SPEEDS,
        CHARACTERIZATION,
    }

    private State wantedState = State.STOP;

    private boolean shouldStopWithX = false;

    private @Nullable Supplier<OptionalDouble> headingOverrideSetpointSupplier = null;
    private @Nullable Supplier<OptionalDouble> headingOverrideFeedforwardSupplier = null;
    private final PIDController headingOverrideController = headingOverrideGains
            .toPIDWrapRadians(
                    moveToConfig.angularPositionToleranceRad().get(),
                    moveToConfig.angularVelocityToleranceRadPerSec().get()
            );

    private final Timer joystickDriveHeadingStabilizeTimer = new Timer();
    private double joystickDriveHeadingStabilizeSetpoint;

    private final MoveToController moveToController = new MoveToController();
    private final FollowTrajectoryController followTrajectoryController = new FollowTrajectoryController();
    private @Nullable Supplier<ChassisSpeeds> chassisSpeedsSetpointSupplier = null;

    /**
     * FL, FR, BL, BR
     */
    private final Module[] modules = new Module[4];
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    @Getter
    private Rotation2d rawGyroRotation = new Rotation2d();

    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError);

    private static Drive instance;

    public static synchronized Drive get() {
        if (instance == null) {
            instance = new Drive();
        }

        return instance;
    }

    private Drive() {
        if (instance != null) {
            Util.error("Duplicate Drive created");
        }

        var moduleIO = createModuleIO();
        // Array is currently four nulls, so length works just fine
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(moduleIO[i], i);
        }

        // Usage reporting for swerve template
        HAL.report(FRCNetComm.tResourceType.kResourceType_RobotDrive, FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit);
    }

    /** returns if discarded */
    private boolean processOdometrySample(
            double sampleTimestamp,
            IntToDoubleFunction getDrivePositionRadForModuleIndex,
            IntToDoubleFunction getTurnPositionRadForModuleIndex,
            boolean hasGyroYawPositionRadForSample,
            DoubleSupplier getGyroYawPositionRad
    ) {
        boolean discardSample = false;

        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
        for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
            double positionMeters = getDrivePositionRadForModuleIndex.applyAsDouble(moduleIndex) * driveConfig.wheelRadiusMeters();
            Rotation2d angle = new Rotation2d(getTurnPositionRadForModuleIndex.applyAsDouble(moduleIndex));
            var modulePosition = new SwerveModulePosition(positionMeters, angle);

            modulePositions[moduleIndex] = modulePosition;
            moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePosition.distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    modulePosition.angle
            );
            lastModulePositions[moduleIndex] = modulePosition;

            // We actually don't really care if one of the motors is disconnected, because odometry
            // can handle one wheel position isn't changing. The issue is when one wheel changes drastically
            if (Math.abs(moduleDeltas[moduleIndex].distanceMeters) > odometryPositionDeltaDiscardMeters) {
                discardSample = true;
            }
        }

        // Update gyro angle
        // Sanity check in case gyro is connected but not giving timestamps
        boolean prevGyroDebounced = gyroDebounced;
        gyroDebounced = gyroDebouncer.calculate(gyroInputs.connected);
        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput("Drive/GyroConnectedDebounced", gyroDebounced);
        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroDebounced);
        if (gyroDebounced && !disableGyro && hasGyroYawPositionRadForSample) {
            // Use the real gyro angle
            Rotation2d prevRawGyroRotation = rawGyroRotation;
            rawGyroRotation = new Rotation2d(getGyroYawPositionRad.getAsDouble());
            // If gyro rotation jumps due to a disconnection, power cycle, and reconnection, discard
            if (Math.abs(rawGyroRotation.minus(prevRawGyroRotation).getRadians()) > odometryGyroRotationDeltaDiscardRad || (!prevGyroDebounced && gyroDebounced)) {
                discardSample = true;
            }
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = robotState.getKinematics().toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Apply update
        if (discardSample) {
            // If we need to discard it, apply the update and then revert the pose back to the pose before applying the update
            // This means that the previous wheel positions stored by odometry will be updated to the new wheel positions,
            // but the pose won't change
            Pose2d prevPose = robotState.getPose();
            robotState.applyOdometryUpdate(sampleTimestamp, rawGyroRotation, modulePositions);
            robotState.setPose(prevPose);
            return true; // true for discarded
        } else {
            robotState.applyOdometryUpdate(sampleTimestamp, rawGyroRotation, modulePositions);
            return false; // false for not discarded
        }
    }

    @Override
    public void periodicBeforeCommands() {
        highFrequencyLock.lock();

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Inputs/Drive/Gyro", gyroInputs);

        for (var module : modules) {
            // We have a separate function from periodicBeforeCommands to minimize time spent in lock
            module.updateAndProcessInputs();
        }

        highFrequencyLock.unlock();

        // The accelerometer has no high-frequency inputs, no need to update while in lock
        accelerometerIO.updateInputs(accelerometerInputs);
        Logger.processInputs("Inputs/Drive/Accelerometer", accelerometerInputs);

        for (var module : modules) {
            module.periodicBeforeCommands();
        }

        // Odometry
        if (useHighFrequencyOdometry) {
            // All timestamps will be synced by HighFrequencySamplingThread
            double[] sampleTimestamps = modules[0].getOdometryTimestamps();
            boolean anySampleDiscarded = false;
            for (int sample = 0; sample < sampleTimestamps.length; sample++) {
                double sampleTimestamp = sampleTimestamps[sample];

                // make a copy for use in lambdas
                int finalSample = sample;
                if (processOdometrySample(
                        sampleTimestamp,
                        (moduleIndex) -> modules[moduleIndex].getOdometryDrivePositionsRad()[finalSample],
                        (moduleIndex) -> modules[moduleIndex].getOdometryTurnPositionsRad()[finalSample],
                        gyroInputs.odometryYawTimestamps.length > sample,
                        () -> gyroInputs.odometryYawPositionsRad[finalSample]
                )) {
                    // return true = discarded
                    anySampleDiscarded = true;
                }
            }
            if (BuildConstants.isSimOrReplay)
                Logger.recordOutput("Drive/AnySampleDiscarded", anySampleDiscarded);
        } else {
            boolean discardSample = processOdometrySample(
                    Timer.getTimestamp(),
                    (moduleIndex) -> modules[moduleIndex].getDrivePositionRad(),
                    (moduleIndex) -> modules[moduleIndex].getTurnAngle().getRadians(),
                    true,
                    () -> gyroInputs.yawPositionRad
            );
            if (BuildConstants.isSimOrReplay)
                Logger.recordOutput("Drive/SampleDiscarded", discardSample);
        }

        // Chassis speeds
        ChassisSpeeds measuredChassisSpeeds = robotState.getKinematics().toChassisSpeeds(getMeasuredModuleStates());
        Logger.recordOutput("Drive/ChassisSpeeds/Measured", measuredChassisSpeeds);
        robotState.setMeasuredChassisSpeedsRobotRelative(measuredChassisSpeeds);
        ChassisSpeeds measuredChassisSpeedsFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
                measuredChassisSpeeds,
                robotState.getRotation() // Field is absolute, don't flip
        );
        robotState.setMeasuredChassisSpeedsFieldRelative(measuredChassisSpeedsFieldRelative);

        // Update filtered acceleration
        //Translation2d filteredAccelerationMetersPerSecPerSec = new Translation2d(
        //        accelerationXFilter.calculate(accelerometerInputs.accelerationXMetersPerSecPerSec),
        //        accelerationYFilter.calculate(accelerometerInputs.accelerationYMetersPerSecPerSec)
        //).rotateBy(robotState.getRotation());
        //robotState.setFilteredAccelerationMetersPerSecPerSec(filteredAccelerationMetersPerSecPerSec);
        //Logger.recordOutput("Drive/FilteredAccelerationMetersPerSecPerSec", filteredAccelerationMetersPerSecPerSec);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            for (var module : modules) {
                module.setBrakeMode(!operatorDashboard.coastOverride.get());
            }
        }

        if (moduleConfig.driveGains().hasChanged()) {
            for (var module : modules) {
                module.setDrivePIDF(moduleConfig.driveGains());
            }
        }
        if (moduleConfig.turnRelativeGains().hasChanged()) {
            for (var module : modules) {
                module.setTurnRelativePIDF(moduleConfig.turnRelativeGains());
            }
        }
        if (moduleConfig.turnAbsoluteGains().hasChanged()) {
            for (var module : modules) {
                module.setTurnAbsolutePIDF(moduleConfig.turnAbsoluteGains());
            }
        }

        if (headingOverrideGains.hasChanged()) {
            headingOverrideGains.applyPID(headingOverrideController);
        }

        moveToController.applyNetworkInputs();
        followTrajectoryController.applyNetworkInputs();
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Drive/StopWithXSet", shouldStopWithX);
        Logger.recordOutput("Drive/HeadingOverrideSet", headingOverrideSetpointSupplier != null);

        Logger.recordOutput("Drive/WantedState", wantedState);
        State actualState = evaluateStateMachine(wantedState);
        Logger.recordOutput("Drive/ActualState", actualState);
    }

    private State evaluateStateMachine(State state) {
        // Stop moving when idle or disabled
        if (state == State.STOP || DriverStation.isDisabled()) {
            // Only attempt to stop with X when enabled
            if (DriverStation.isEnabled() && shouldStopWithX) {
                // Create a list of headings where each heading points from the center
                // of the robot to the module. Tell the module to point at this angle.
                // This means that the modules will point towards the center of the
                // robot, forming an X.
                Rotation2d[] headings = new Rotation2d[modules.length];
                for (int i = 0; i < modules.length; i++) {
                    headings[i] = moduleTranslations[i].getAngle();
                    modules[i].runSetpoint(new SwerveModuleState(0.0, headings[i]));
                }
                // We also need to make kinematics aware of the new headings
                robotState.getKinematics().resetHeadings(headings);
            } else {
                for (var module : modules) {
                    module.stop();
                }
            }
        } else if (state == State.CHARACTERIZATION) {
            // Do nothing - commands handle setting voltages
        } else {
            ChassisSpeeds wantedSpeeds = new ChassisSpeeds();

            OptionalDouble headingOverrideSetpoint = headingOverrideSetpointSupplier == null
                    ? OptionalDouble.empty()
                    : headingOverrideSetpointSupplier.get();

            switch (state) {
                case JOYSTICK_DRIVE -> {
                    wantedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(controller.getDriveFieldRelativeSpeeds(), robotState.getRotation());

                    if (headingOverrideSetpoint.isPresent() || controller.getDriveAngularMagnitude() != 0.0) {
                        joystickDriveHeadingStabilizeTimer.restart();
                    } else if (!joystickDriveHeadingStabilizeTimer.hasElapsed(0.3)) {
                        // Before the timer finishes, set the setpoint to current heading
                        joystickDriveHeadingStabilizeSetpoint = robotState.getRotation().getRadians();
                    } else {
                        // After time finishes, run heading stabilize with previously set setpoint
                        headingOverrideSetpoint = OptionalDouble.of(joystickDriveHeadingStabilizeSetpoint);
                    }
                }
                case MOVE_TO -> wantedSpeeds = moveToController.update();
                case FOLLOW_TRAJECTORY -> wantedSpeeds = followTrajectoryController.update();
                case CHASSIS_SPEEDS -> {
                    if (chassisSpeedsSetpointSupplier != null) {
                        wantedSpeeds = chassisSpeedsSetpointSupplier.get();
                    }
                }
            }

            if (headingOverrideSetpoint.isPresent()) {
                wantedSpeeds.omegaRadiansPerSecond = headingOverrideController.calculate(
                        robotState.getRotation().getRadians(),
                        headingOverrideSetpoint.getAsDouble()
                );

                boolean atSetpoint = headingOverrideController.atSetpoint();
                if (BuildConstants.isSimOrReplay)
                    Logger.recordOutput("Drive/HeadingOverrideAtSetpoint", atSetpoint);
                if (atSetpoint) {
                    wantedSpeeds.omegaRadiansPerSecond = 0.0;
                }

                OptionalDouble feedforward = headingOverrideFeedforwardSupplier == null
                        ? OptionalDouble.empty()
                        : headingOverrideFeedforwardSupplier.get();
                if (feedforward.isPresent()) {
                    wantedSpeeds.omegaRadiansPerSecond += feedforward.getAsDouble();
                }
            } else {
                headingOverrideController.reset();
            }

            if (wantedSpeeds.vxMetersPerSecond == 0.0 && wantedSpeeds.vyMetersPerSecond == 0.0 && wantedSpeeds.omegaRadiansPerSecond == 0.0) {
                // Re-evaluate state machine to stop - this handles stopping with X in a nice way
                return evaluateStateMachine(State.STOP);
            }

            Logger.recordOutput("Drive/ModuleStates/Setpoints", robotState.getKinematics().toSwerveModuleStates(wantedSpeeds));

            // Discretize - use larger dt than actual to reduce translational skew when rotating and translating at the same time
            // See https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
            // and https://github.com/frc1678/C2024-Public/blob/main/src/main/java/com/team1678/frc2024/subsystems/Drive.java#L406
            ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(wantedSpeeds, Constants.loopPeriod * 4.0);

            // Convert to module states and desaturate
            SwerveModuleState[] setpointStates = robotState.getKinematics().toSwerveModuleStates(discreteSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, driveConfig.maxVelocityMetersPerSec());

            // Send setpoints to modules
            for (int i = 0; i < modules.length; i++) {
                var currentAngle = modules[i].getTurnAngle();

                // Optimize angle to reduce azimuth/steering/turn rotation
                setpointStates[i].optimize(currentAngle);

                // Note that cosine scaling MUST come AFTER angle optimization
                // because cosine scaling should be based on the final angle setpoint
                setpointStates[i].cosineScale(currentAngle);

                modules[i].runSetpoint(setpointStates[i]);
            }

            // Log setpoint states
            Logger.recordOutput("Drive/ModuleStates/SetpointsOptimized", setpointStates);
            Logger.recordOutput("Drive/ChassisSpeeds/SetpointOptimized", robotState.getKinematics().toChassisSpeeds(setpointStates));
        }

        // No re-evaluation, so state stayed the same. Return it
        return state;
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "Drive/ModuleStates/Measured")
    private SwerveModuleState[] getMeasuredModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    public SwerveModulePosition[] getMeasuredModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Doesn't require the Drive subsystem. This is intended to be used in conjunction with another state command */
    public Command setHeadingOverride(Supplier<OptionalDouble> targetRad) {
        return setHeadingOverride(targetRad, OptionalDouble::empty);
    }

    /** Used to ensure that only one heading override setting is active at one time. */
    private final CommandBasedSubsystem headingOverrideSetting = new CommandBasedSubsystem() {};

    /** Doesn't require the Drive subsystem. This is intended to be used in conjunction with another state command */
    public Command setHeadingOverride(Supplier<OptionalDouble> targetRad, Supplier<OptionalDouble> feedforwardRadPerSec) {
        return headingOverrideSetting.startEnd(
                () -> {
                    headingOverrideSetpointSupplier = targetRad;
                    headingOverrideFeedforwardSupplier = feedforwardRadPerSec;
                },
                () -> {
                    headingOverrideSetpointSupplier = null;
                    headingOverrideFeedforwardSupplier = null;
                }
        );
    }

    /** Used to ensure that only one stop with X setting is active at one time. */
    private final CommandBasedSubsystem stopWithXSetting = new CommandBasedSubsystem() {};

    /** Doesn't require the Drive subsystem. This is intended to be used in conjunction with another state command */
    public Command setStopWithX() {
        return stopWithXSetting.startEnd(
                () -> shouldStopWithX = true,
                () -> shouldStopWithX = false
        );
    }

    public Command setAim() {
        return Commands.parallel(
                setHeadingOverride(
                        () -> operatorDashboard.manualAiming.get()
                                ? OptionalDouble.empty()
                                : OptionalDouble.of(shootingKinematics.getShootingParameters().headingRad()),
                        () -> operatorDashboard.manualAiming.get()
                                ? OptionalDouble.empty()
                                : OptionalDouble.of(shootingKinematics.rotationAboutHubRadiansPerSecForDrivebase(controller.getDriveFieldRelativeSpeeds()))
                ),
                setStopWithX()
        );
    }

    public Command stop() {
        return startIdle(() -> wantedState = State.STOP);
    }

    public Command joystickDrive() {
        return startIdle(() -> {
            wantedState = State.JOYSTICK_DRIVE;
            joystickDriveHeadingStabilizeTimer.restart();
        });
    }

    public Command moveTo(Supplier<Pose2d> goalPoseSupplier) {
        return moveTo(goalPoseSupplier, defaultMoveToConstraints);
    }

    public Command moveTo(Supplier<Pose2d> goalPoseSupplier, MoveToConstraints constraints) {
        return startEnd(
                () -> {
                    wantedState = State.MOVE_TO;
                    moveToController.start(goalPoseSupplier, constraints);
                },
                moveToController::stop
        );
    }

    public Command followTrajectory(Trajectory<SwerveSample> trajectory) {
        return startEndWaitUntil(
                () -> {
                    wantedState = State.FOLLOW_TRAJECTORY;
                    followTrajectoryController.start(trajectory);
                },
                followTrajectoryController::stop,
                followTrajectoryController::isDone
        );
    }

    public Command chassisSpeeds(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        return startIdle(() -> {
            wantedState = State.CHASSIS_SPEEDS;
            chassisSpeedsSetpointSupplier = chassisSpeedsSupplier;
        });
    }

    private void runCharacterization(double volts) {
        for (var module : modules) {
            module.runCharacterization(volts);
        }
    }

    public Command fullSpeedCharacterization() {
        Timer timer = new Timer();
        return startRun(
                () -> {
                    wantedState = State.CHARACTERIZATION;
                    timer.restart();
                },
                () -> {
                    if (!timer.hasElapsed(2.0)) {
                        runCharacterization(2.0);
                    } else {
                        runCharacterization(12.0);
                    }
                }
        );
    }

    public Command wheelRadiusCharacterization() {
        double speedRadPerSec = 1.0;
        int direction = 1; // or -1

        SlewRateLimiter omegaLimiter = new SlewRateLimiter(0.2);
        var state = new Object() {
            double[] startWheelPositions = new double[4];
            double lastGyroYawRad = 0.0;
            double accumGyroYawRad = 0.0;
        };

        Supplier<double[]> wheelPositionsSupplier = () -> Arrays.stream(modules).mapToDouble(Module::getDrivePositionRad).toArray();

        return CommandsExt.eagerSequence(
                runOnce(() -> {
                    omegaLimiter.reset(0.0);
                    state.startWheelPositions = wheelPositionsSupplier.get();
                    state.lastGyroYawRad = rawGyroRotation.getRadians();
                    state.accumGyroYawRad = 0.0;
                }),
                chassisSpeeds(() -> {
                    var omega = omegaLimiter.calculate(direction * speedRadPerSec);

                    // Get yaw and wheel positions
                    state.accumGyroYawRad += MathUtil.angleModulus(rawGyroRotation.getRadians() - state.lastGyroYawRad);
                    state.lastGyroYawRad = getRawGyroRotation().getRadians();
                    double averageWheelPosition = 0.0;
                    double[] wheelPositions = wheelPositionsSupplier.get();
                    for (int i = 0; i < 4; i++) {
                        averageWheelPosition += Math.abs(wheelPositions[i] - state.startWheelPositions[i]);
                    }
                    averageWheelPosition /= 4.0;

                    double currentEffectiveWheelRadius = (state.accumGyroYawRad * drivebaseRadiusMeters) / averageWheelPosition;
                    Logger.recordOutput("Drive/WheelRadiusCharacterization/DrivePosition", averageWheelPosition);
                    Logger.recordOutput("Drive/WheelRadiusCharacterization/AccumGyroYawRad", state.accumGyroYawRad);
                    Logger.recordOutput("Drive/WheelRadiusCharacterization/CurrentWheelRadiusInches", Units.metersToInches(currentEffectiveWheelRadius));
                    Logger.recordOutput("Drive/WheelRadiusCharacterization/HasEnoughData", Math.abs(state.accumGyroYawRad) > Math.PI * 2.0);

                    return new ChassisSpeeds(0, 0, omega);
                })
        );
    }

    public Command slipCurrentCharacterization() {
        double minVelocityRadPerSec = 0.1;

        Timer timer = new Timer();
        var state = new Object() {
            double[] lastCurrents = new double[4];
            double[] currentNeededForMovement = new double[4];
            boolean done = false;
        };

        Supplier<double[]> velocitiesSupplier = () -> Arrays.stream(modules).mapToDouble(Module::getDriveVelocityRadPerSec).toArray();
        Supplier<double[]> currentsSupplier = () -> Arrays.stream(modules).mapToDouble(Module::getDriveCurrentAmps).toArray();

        return startRun(
                () -> {
                    timer.restart();
                    state.lastCurrents = new double[4];
                    state.currentNeededForMovement = new double[4];
                    state.done = false;
                },
                () -> {
                    if (!state.done) {
                        double[] velocities = velocitiesSupplier.get();
                        double[] currents = currentsSupplier.get();

                        if (timer.hasElapsed(0.5)) {
                            double total = 0.0;
                            state.done = true;
                            for (int i = 0; i < 4; i++) {
                                if (velocities[i] > minVelocityRadPerSec && state.currentNeededForMovement[i] == 0) {
                                    state.currentNeededForMovement[i] = state.lastCurrents[i];
                                }

                                if (state.done && state.currentNeededForMovement[i] != 0) {
                                    total += state.currentNeededForMovement[i];
                                } else {
                                    state.done = false;
                                }
                            }

                            if (state.done) {
                                double avg = total / 4.0;
                                System.out.println("Average current for each module to move: " + avg);
                                for (int i = 0; i < 4; i++) {
                                    System.out.println("\tCurrent for module " + i + " to move: " + state.currentNeededForMovement[i]);
                                }
                                // Don't stop moving to allow for manual line graph based analysis
                            }
                        }

                        state.lastCurrents = currents;
                    }

                    // Increase voltage at 0.1 V/s
                    runCharacterization(timer.get() * 0.1);
                }
        );
    }
}
