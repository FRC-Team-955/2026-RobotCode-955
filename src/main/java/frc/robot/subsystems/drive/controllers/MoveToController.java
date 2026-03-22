package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.SlewRateLimiter2d;
import frc.lib.Util;
import frc.lib.wpilib.SlewRateLimiter;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.moveToConfig;

public class MoveToController {
    private static final RobotState robotState = RobotState.get();

    private final PIDController linearController = moveToConfig.linearGains()
            .toPID(
                    moveToConfig.linearPositionToleranceMeters().get(),
                    moveToConfig.linearVelocityToleranceMetersPerSec().get()
            );
    private final SlewRateLimiter2d linearAccelLimiter = new SlewRateLimiter2d(0.0);

    private final PIDController angularController = moveToConfig.angularGains()
            .toPIDWrapRadians(
                    moveToConfig.angularPositionToleranceRad().get(),
                    moveToConfig.angularVelocityToleranceRadPerSec().get()
            );
    private final SlewRateLimiter angularAccelLimiter = new SlewRateLimiter(0.0);

    public void applyNetworkInputs() {
        if (moveToConfig.linearGains().hasChanged()) {
            moveToConfig.linearGains().applyPID(linearController);
        }

        if (moveToConfig.angularGains().hasChanged()) {
            moveToConfig.angularGains().applyPID(angularController);
        }
    }

    private @Nullable Supplier<Pose2d> goalPoseSupplier = null;
    private @Nullable DriveConstants.MoveToConstraints constraints = null;

    public void start(Supplier<Pose2d> goalPoseSupplier, DriveConstants.MoveToConstraints constraints) {
        this.goalPoseSupplier = goalPoseSupplier;
        this.constraints = constraints;

        linearController.reset();
        angularController.reset();

        ChassisSpeeds speeds = robotState.getMeasuredChassisSpeedsFieldRelative();
        linearAccelLimiter.reset(speeds);
        angularAccelLimiter.reset(speeds.omegaRadiansPerSecond);

        if (linearAccelLimiter.getLimit() != constraints.maxLinearAccelerationMetersPerSecPerSec().get()) {
            linearAccelLimiter.setLimit(constraints.maxLinearAccelerationMetersPerSecPerSec().get());
        }
        if (angularAccelLimiter.getLimit() != constraints.maxAngularAccelerationRadPerSecPerSec().get()) {
            angularAccelLimiter.setLimit(constraints.maxAngularAccelerationRadPerSecPerSec().get());
        }
    }

    public ChassisSpeeds update() {
        if (goalPoseSupplier == null || constraints == null) {
            Util.error("Goal pose or constraints are null");
            return new ChassisSpeeds();
        }

        if (constraints.maxLinearAccelerationMetersPerSecPerSec().hasChanged()) {
            linearAccelLimiter.setLimit(constraints.maxLinearAccelerationMetersPerSecPerSec().get());
        }
        if (constraints.maxAngularAccelerationRadPerSecPerSec().hasChanged()) {
            angularAccelLimiter.setLimit(constraints.maxAngularAccelerationRadPerSecPerSec().get());
        }

        Pose2d currentPose = robotState.getPose();

        Pose2d goalPose = goalPoseSupplier.get();
        robotState.setMoveToGoal(Optional.of(goalPose));
        Logger.recordOutput("Drive/MoveTo/Goal", goalPose);

        double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        //Logger.recordOutput("Drive/MoveTo/DistanceToGoalMeters", distanceToGoal);

        double linearVelocityMetersPerSec = calculateLinearVelocityMetersPerSec(distanceToGoal, constraints);
        Rotation2d angleToGoalRad = goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
        Translation2d linearXYVelocityMetersPerSec = new Translation2d(linearVelocityMetersPerSec, angleToGoalRad);

        double angularVelocityRadPerSec = calculateAngularVelocityRadPerSec(currentPose, goalPose, constraints);

        // Scale linear speed by angular speed so that angular change is prioritized
        // When going max angular speed, reduce linear to 50%
        double linearScalar = MathUtil.clamp(1 - angularVelocityRadPerSec / DriveConstants.maxAngularVelocityRadPerSec, 0.50, 1);

        linearXYVelocityMetersPerSec = linearAccelLimiter.calculate(linearXYVelocityMetersPerSec);
        //Logger.recordOutput("Drive/MoveTo/LinearSetpoint", linearXYVelocityMetersPerSec.getNorm());

        angularVelocityRadPerSec = angularAccelLimiter.calculate(angularVelocityRadPerSec);
        //Logger.recordOutput("Drive/MoveTo/LinearSetpoint", angularVelocityRadPerSec);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                linearXYVelocityMetersPerSec.getX() * linearScalar,
                linearXYVelocityMetersPerSec.getY() * linearScalar,
                angularVelocityRadPerSec,
                currentPose.getRotation() // Move to is absolute, don't flip
        );
    }

    private double calculateLinearVelocityMetersPerSec(double distanceToGoal, DriveConstants.MoveToConstraints constraints) {
        double linearVelocityMetersPerSec = linearController.calculate(
                -distanceToGoal,
                0.0
        );
        // linear velocity *shouldn't* ever be negative because it is a magnitude
        // but eh you never know
        linearVelocityMetersPerSec = MathUtil.clamp(
                linearVelocityMetersPerSec,
                -constraints.maxLinearVelocityMetersPerSec().get(),
                constraints.maxLinearVelocityMetersPerSec().get()
        );
        //Logger.recordOutput("Drive/MoveTo/LinearSetpointUnlimited", linearVelocityMetersPerSec);

        boolean linearAtSetpoint = linearController.atSetpoint();
        //Logger.recordOutput("Drive/MoveTo/LinearAtSetpoint", linearAtSetpoint);
        if (linearAtSetpoint) {
            linearVelocityMetersPerSec = 0.0;
        }

        return linearVelocityMetersPerSec;
    }

    private double calculateAngularVelocityRadPerSec(Pose2d currentPose, Pose2d goalPose, DriveConstants.MoveToConstraints constraints) {
        double angularVelocityRadPerSec = angularController.calculate(
                MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                MathUtil.angleModulus(goalPose.getRotation().getRadians())
        );
        angularVelocityRadPerSec = MathUtil.clamp(
                angularVelocityRadPerSec,
                -constraints.maxAngularVelocityRadPerSec().get(),
                constraints.maxAngularVelocityRadPerSec().get()
        );
        //Logger.recordOutput("Drive/MoveTo/AngularSetpointUnlimited", angularVelocityRadPerSec);

        boolean angularAtSetpoint = angularController.atSetpoint();
        //Logger.recordOutput("Drive/MoveTo/AngularAtSetpoint", angularAtSetpoint);
        if (angularAtSetpoint) {
            angularVelocityRadPerSec = 0.0;
        }

        return angularVelocityRadPerSec;
    }

    public void stop() {
        robotState.setMoveToGoal(Optional.empty());
    }
}
