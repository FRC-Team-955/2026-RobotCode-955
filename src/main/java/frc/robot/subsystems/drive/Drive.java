package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Util;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.Constants;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.goals.*;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.IntToDoubleFunction;
import java.util.function.Supplier;

import static frc.lib.HighFrequencySamplingThread.highFrequencyLock;
import static frc.robot.subsystems.drive.DriveConstants.*;

public class Drive extends CommandBasedSubsystem {
    private static final RobotState robotState = RobotState.get();
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final GyroIO gyroIO = createGyroIO();
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final AccelerometerIO accelerometerIO = createAccelerometerIO();
    private final AccelerometerIOInputsAutoLogged accelerometerInputs = new AccelerometerIOInputsAutoLogged();

    private final LinearFilter accelerationXFilter = LinearFilter.movingAverage(4);
    private final LinearFilter accelerationYFilter = LinearFilter.movingAverage(4);

    @Getter
    private DriveGoal goal = new IdleGoal();

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
        if (gyroInputs.connected && !disableGyro && hasGyroYawPositionRadForSample) {
            // Use the real gyro angle
            rawGyroRotation = new Rotation2d(getGyroYawPositionRad.getAsDouble());
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

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected);

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
            //Logger.recordOutput("Drive/AnySampleDiscarded", anySampleDiscarded);
        } else {
            boolean discardSample = processOdometrySample(
                    Timer.getTimestamp(),
                    (moduleIndex) -> modules[moduleIndex].getDrivePositionRad(),
                    (moduleIndex) -> modules[moduleIndex].getTurnAngle().getRadians(),
                    true,
                    () -> gyroInputs.yawPositionRad
            );

            //Logger.recordOutput("Drive/SampleDiscarded", discardSample);
        }

        // Chassis speeds
        ChassisSpeeds measuredChassisSpeeds = robotState.getKinematics().toChassisSpeeds(getMeasuredModuleStates());
        Logger.recordOutput("Drive/ChassisSpeeds/Measured", measuredChassisSpeeds);
        ChassisSpeeds measuredChassisSpeedsFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
                measuredChassisSpeeds,
                robotState.getRotation() // Field is absolute, don't flip
        );
        robotState.setMeasuredChassisSpeeds(measuredChassisSpeedsFieldRelative);

        // Update filtered acceleration
        Translation2d filteredAccelerationMetersPerSecPerSec = new Translation2d(
                accelerationXFilter.calculate(accelerometerInputs.accelerationXMetersPerSecPerSec),
                accelerationYFilter.calculate(accelerometerInputs.accelerationYMetersPerSecPerSec)
        ).rotateBy(robotState.getRotation());
        robotState.setFilteredAccelerationMetersPerSecPerSec(filteredAccelerationMetersPerSecPerSec);
        Logger.recordOutput("Drive/FilteredAccelerationMetersPerSecPerSec", filteredAccelerationMetersPerSecPerSec);

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
        if (moduleConfig.turnGains().hasChanged()) {
            for (var module : modules) {
                module.setTurnPIDF(moduleConfig.turnGains());
            }
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Drive/Goal", goal.loggableName);
        DriveRequest request = goal.getRequest();
        Logger.recordOutput("Drive/RequestType", request.type());
        Logger.recordOutput("Drive/RequestValue", request.value());

        // Stop moving when idle or disabled
        if (request.type() == DriveRequest.Type.STOP || DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        } else if (request.type() == DriveRequest.Type.STOP_WITH_X) {
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
        }
        // Closed loop control
        else if (request.type() == DriveRequest.Type.CHASSIS_SPEEDS) {
            ChassisSpeeds rawSpeeds = request.value();
            Logger.recordOutput("Drive/ModuleStates/Setpoints", robotState.getKinematics().toSwerveModuleStates(rawSpeeds));

            // Discretize - use larger dt than actual to reduce translational skew when rotating and translating at the same time
            // See https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
            // and https://github.com/frc1678/C2024-Public/blob/main/src/main/java/com/team1678/frc2024/subsystems/Drive.java#L406
            ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(rawSpeeds, Constants.loopPeriod * 4.0);

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
        } else if (request.type() == DriveRequest.Type.CHARACTERIZATION) {
            for (var module : modules) {
                module.runCharacterization(request.value().vxMetersPerSecond);
            }
        } else {
            Util.error("Unknown request type: " + request.type());
        }
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

    public double[] getWheelRadiusCharacterizationPositions() {
        return Arrays.stream(modules).mapToDouble(Module::getDrivePositionRad).toArray();
    }

    public double[] getSlipCurrentCharacterizationVelocities() {
        return Arrays.stream(modules).mapToDouble(Module::getDriveVelocityRadPerSec).toArray();
    }

    public double[] getSlipCurrentCharacterizationCurrents() {
        return Arrays.stream(modules).mapToDouble(Module::getDriveCurrentAmps).toArray();
    }

    public Command followTrajectory(Trajectory<SwerveSample> trajectory) {
        return startIdle(() -> goal = new FollowTrajectoryGoal(trajectory));
    }

    public Command moveTo(Supplier<Pose2d> poseSupplier) {
        return startIdle(() -> goal = new MoveToGoal(poseSupplier, defaultMoveToConstraints));
    }

    public Command moveTo(Supplier<Pose2d> poseSupplier, DriveConstants.MoveToConstraints moveToConstraints) {
        return startIdle(() -> goal = new MoveToGoal(poseSupplier, moveToConstraints));
    }

    public Command driveJoystick(DriveJoystickGoal.Mode mode) {
        return startIdle(() -> goal = new DriveJoystickGoal(mode));
    }

    public Command runRobotRelative(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        return startIdle(() -> goal = new VelocityRobotRelativeGoal(chassisSpeedsSupplier));
    }

    public Command fullSpeedCharacterization() {
        return startIdle(() -> goal = new FullSpeedCharacterizationGoal());
    }

    public Command idle() {
        return run(() -> goal.getRequest());
    }

    public Command wheelRadiusCharacterization(WheelRadiusCharacterizationGoal.Direction direction) {
        return startIdle(() -> goal = new WheelRadiusCharacterizationGoal(direction));
    }

    public Command slipCurrentCharacterization() {
        return startIdle(() -> goal = new SlipCurrentCharacterizationGoal());
    }
}
