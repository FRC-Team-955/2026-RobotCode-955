package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.lib.swerve.SwerveSetpoint;
import frc.lib.swerve.SwerveSetpointGenerator;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.goals.*;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.function.Supplier;

import static frc.lib.HighFrequencySamplingThread.highFrequencyLock;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.drive.DriveTuning.moduleDriveGainsTunable;
import static frc.robot.subsystems.drive.DriveTuning.moduleTurnGainsTunable;

public class Drive extends CommandBasedSubsystem {
    private final RobotState robotState = RobotState.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final GyroIO gyroIO = createGyroIO();
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

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
    private ChassisSpeeds measuredChassisSpeeds = new ChassisSpeeds();

    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(robotState.getKinematics());
    /** If null, it will be set to the measured ChassisSpeeds and module states when the setpoint generator starts to be used */
    private SwerveSetpoint prevSetpoint = null;

    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError);

    private static Drive instance;

    public static Drive get() {
        if (instance == null)
            synchronized (Drive.class) {
                instance = new Drive();
            }

        return instance;
    }

    private Drive() {
        var moduleIO = createModuleIO();
        // Array is currently four nulls, so length works just fine
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(moduleIO[i], i);
        }

        // Usage reporting for swerve template
        HAL.report(FRCNetComm.tResourceType.kResourceType_RobotDrive, FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit);
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
                boolean discardSample = false;

                // Read wheel positions and deltas from each module
                SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
                SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
                for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                    double positionMeters = modules[moduleIndex].getOdometryDrivePositionsRad()[sample] * driveConfig.wheelRadiusMeters();
                    Rotation2d angle = new Rotation2d(modules[moduleIndex].getOdometryTurnPositionsRad()[sample]);
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
                if (gyroInputs.connected && !disableGyro && gyroInputs.odometryYawTimestamps.length > sample) {
                    // Use the real gyro angle
                    rawGyroRotation = new Rotation2d(gyroInputs.odometryYawPositionsRad[sample]);
                } else {
                    // Use the angle delta from the kinematics and module deltas
                    Twist2d twist = robotState.getKinematics().toTwist2d(moduleDeltas);
                    rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
                }

                // Apply update
                if (discardSample) {
                    anySampleDiscarded = true;
                    // If we need to discard it, apply the update and then revert the pose back to the pose before applying the update
                    // This means that the previous wheel positions stored by odometry will be updated to the new wheel positions,
                    // but the pose won't change
                    Pose2d prevPose = robotState.getPose();
                    robotState.applyOdometryUpdate(sampleTimestamp, rawGyroRotation, modulePositions);
                    robotState.setPose(prevPose);
                } else {
                    robotState.applyOdometryUpdate(sampleTimestamp, rawGyroRotation, modulePositions);
                }
            }
            Logger.recordOutput("Drive/SampleDiscarded", anySampleDiscarded);
        } else {
            boolean discardSample = false;

            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                double positionMeters = modules[moduleIndex].getDrivePositionRad() * driveConfig.wheelRadiusMeters();
                Rotation2d angle = modules[moduleIndex].getTurnAngle();
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
            if (gyroInputs.connected && !disableGyro) {
                // Use the real gyro angle
                rawGyroRotation = new Rotation2d(gyroInputs.yawPositionRad);
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
                robotState.applyOdometryUpdate(Timer.getTimestamp(), rawGyroRotation, modulePositions);
                robotState.setPose(prevPose);
            } else {
                robotState.applyOdometryUpdate(Timer.getTimestamp(), rawGyroRotation, modulePositions);
            }
            Logger.recordOutput("Drive/SampleDiscarded", discardSample);
        }

        // Chassis speeds
        measuredChassisSpeeds = robotState.getKinematics().toChassisSpeeds(getMeasuredModuleStates());
        Logger.recordOutput("Drive/ChassisSpeeds/Measured", measuredChassisSpeeds);
        ChassisSpeeds measuredChassisSpeedsFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
                measuredChassisSpeeds,
                robotState.getRotation() // Field is absolute, don't flip
        );
        robotState.setMeasuredChassisSpeeds(measuredChassisSpeedsFieldRelative);

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            for (var module : modules) {
                module.setBrakeMode(!operatorDashboard.coastOverride.get());
            }
        }

        moduleDriveGainsTunable.ifChanged(gains -> {
            for (var module : modules) {
                module.setDrivePIDF(gains);
            }
        });
        moduleTurnGainsTunable.ifChanged(gains -> {
            for (var module : modules) {
                module.setTurnPIDF(gains);
            }
        });
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Drive/Goal", goal.loggableName);
        DriveRequest request = goal.getRequest();
        Logger.recordOutput("Drive/RequestType", request.type());
        Logger.recordOutput("Drive/RequestValue", request.value());

        // Stop moving when idle or disabled
        if (request.type() == DriveRequest.Type.STOP || DriverStation.isDisabled()) {
            prevSetpoint = null;

            for (var module : modules) {
                module.stop();
            }
        }
        // Closed loop control
        else if (request.type() == DriveRequest.Type.CHASSIS_SPEEDS_DIRECT || request.type() == DriveRequest.Type.CHASSIS_SPEEDS_OPTIMIZED) {
            if (useSetpointGenerator && !disableDriving && request.type() == DriveRequest.Type.CHASSIS_SPEEDS_OPTIMIZED) {
                Logger.recordOutput("Drive/SetpointGenerator", true);

                Logger.recordOutput(
                        "Drive/ModuleStates/Setpoints",
                        // DON'T DO ANYTHING WITH THIS. SETPOINT GENERATOR SHOULD NOT GET A DISCRETIZED SETPOINT
                        // Only for logging
                        robotState.getKinematics().toSwerveModuleStates(ChassisSpeeds.discretize(request.value(), 0.02))
                );

                if (prevSetpoint == null) {
                    // Reset to current chassis speeds and module states
                    prevSetpoint = new SwerveSetpoint(
                            measuredChassisSpeeds,
                            getMeasuredModuleStates()
                    );
                }

                prevSetpoint = setpointGenerator.generateSetpoint(
                        goal.getModuleLimits(),
                        prevSetpoint,
                        request.value(), // THIS SHOULD NOT BE DISCRETIZED
                        0.02
                );
                var setpointStates = prevSetpoint.moduleStates();

                // Send setpoints to modules
                for (int i = 0; i < modules.length; i++) {
                    // Optimize velocity setpoint
                    var currentAngle = modules[i].getTurnAngle();
                    setpointStates[i].cosineScale(currentAngle);
                    // Setpoint generator already optimizes the setpoints
                    modules[i].runSetpoint(setpointStates[i]);
                }

                // Log setpoint states
                Logger.recordOutput("Drive/ModuleStates/SetpointsOptimized", setpointStates);
                Logger.recordOutput("Drive/ChassisSpeeds/SetpointOptimized", prevSetpoint.chassisSpeeds());
            } else {
                Logger.recordOutput("Drive/SetpointGenerator", false);
                prevSetpoint = null;

                // Calculate module setpoints
                ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(request.value(), 0.02);
                SwerveModuleState[] setpointStates = robotState.getKinematics().toSwerveModuleStates(discreteSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, driveConfig.moduleLimits().maxDriveVelocityMetersPerSec());

                Logger.recordOutput("Drive/ModuleStates/Setpoints", setpointStates);

                // Send setpoints to modules
                for (int i = 0; i < modules.length; i++) {
                    // Optimize velocity setpoint
                    var currentAngle = modules[i].getTurnAngle();
                    setpointStates[i].cosineScale(currentAngle);
                    setpointStates[i].optimize(currentAngle);
                    modules[i].runSetpoint(setpointStates[i]);
                }

                // Log setpoint states
                Logger.recordOutput("Drive/ModuleStates/SetpointsOptimized", setpointStates);
                Logger.recordOutput("Drive/ChassisSpeeds/SetpointOptimized", robotState.getKinematics().toChassisSpeeds(setpointStates));
            }
        } else if (request.type() == DriveRequest.Type.CHARACTERIZATION) {
            prevSetpoint = null;

            for (var module : modules) {
                module.runCharacterization(request.value().vxMetersPerSecond);
            }
        } else {
            Util.error("Unknown request type: " + request.type());
        }
    }

    // TODO goal or something
//    /**
//     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
//     * return to their normal orientations the next time a nonzero velocity is requested.
//     */
//    private void stopWithX() {
//        Rotation2d[] headings = new Rotation2d[modules.length];
//        for (int i = 0; i < modules.length; i++) {
//            headings[i] = moduleTranslations[i].getAngle();
//        }
//        // Why does this work? See SwerveDriveKinematics.toModuleStates
//        robotState.getKinematics().resetHeadings(headings);
//        closedLoopSetpoint = new ChassisSpeeds();
//    }

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

    public Command followTrajectory(Trajectory<SwerveSample> trajectory) {
        return startIdle(() -> goal = new FollowTrajectoryGoal(trajectory));
    }

    public Command moveTo(Supplier<Pose2d> poseSupplier, boolean mergeJoystickDrive) {
        return startIdle(() -> goal = new MoveToGoal(poseSupplier, mergeJoystickDrive));
    }

    public Command driveJoystick() {
        return startIdle(() -> goal = new DriveJoystickGoal());
    }

    public Command runRobotRelative(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        return startIdle(() -> goal = new VelocityRobotRelativeGoal(chassisSpeedsSupplier));
    }

    public Command fullSpeedCharacterization() {
        return startIdle(() -> goal = new FullSpeedCharacterizationGoal());
    }

    public Command wheelRadiusCharacterization(WheelRadiusCharacterizationGoal.Direction direction) {
        return startIdle(() -> goal = new WheelRadiusCharacterizationGoal(direction));
    }
}
