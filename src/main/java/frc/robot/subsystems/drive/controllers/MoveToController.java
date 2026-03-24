package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.Util;
import frc.robot.BuildConstants;
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

    private final PIDController angularController = moveToConfig.angularGains()
            .toPIDWrapRadians(
                    moveToConfig.angularPositionToleranceRad().get(),
                    moveToConfig.angularVelocityToleranceRadPerSec().get()
            );

    public void applyNetworkInputs() {
        if (moveToConfig.linearGains().hasChanged()) {
            moveToConfig.linearGains().applyPID(linearController);
        }

        if (moveToConfig.angularGains().hasChanged()) {
            moveToConfig.angularGains().applyPID(angularController);
        }
    }

    private @Nullable Supplier<Pose2d> goalPoseSupplier = null;

    public void start(Supplier<Pose2d> goalPoseSupplier) {
        this.goalPoseSupplier = goalPoseSupplier;

        linearController.reset();
        angularController.reset();
    }

    public ChassisSpeeds update() {
        if (goalPoseSupplier == null) {
            Util.error("Goal pose is null");
            return new ChassisSpeeds();
        }

        Pose2d currentPose = robotState.getPose();

        Pose2d goalPose = goalPoseSupplier.get();
        robotState.setMoveToGoal(Optional.of(goalPose));
        Logger.recordOutput("Drive/MoveTo/Goal", goalPose);

        double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        if (BuildConstants.isSimOrReplay) Logger.recordOutput("Drive/MoveTo/DistanceToGoalMeters", distanceToGoal);

        double linearVelocityMetersPerSec = calculateLinearVelocityMetersPerSec(distanceToGoal);
        Rotation2d angleToGoalRad = goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
        Translation2d linearXYVelocityMetersPerSec = new Translation2d(linearVelocityMetersPerSec, angleToGoalRad);

        double angularVelocityRadPerSec = calculateAngularVelocityRadPerSec(currentPose, goalPose);

        // Scale linear speed by angular speed so that angular change is prioritized
        // When going max angular speed, reduce linear to 50%
        double linearScalar = MathUtil.clamp(1 - angularVelocityRadPerSec / DriveConstants.maxAngularVelocityRadPerSec, 0.50, 1);

        return new ChassisSpeeds(
                linearXYVelocityMetersPerSec.getX() * linearScalar,
                linearXYVelocityMetersPerSec.getY() * linearScalar,
                angularVelocityRadPerSec
        );
    }

    private double calculateLinearVelocityMetersPerSec(double distanceToGoal) {
        double linearVelocityMetersPerSec = linearController.calculate(
                -distanceToGoal,
                0.0
        );
        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput("Drive/MoveTo/LinearSetpointUnlimited", linearVelocityMetersPerSec);

        boolean linearAtSetpoint = linearController.atSetpoint();
        if (BuildConstants.isSimOrReplay) Logger.recordOutput("Drive/MoveTo/LinearAtSetpoint", linearAtSetpoint);
        if (linearAtSetpoint) {
            linearVelocityMetersPerSec = 0.0;
        }

        return linearVelocityMetersPerSec;
    }

    private double calculateAngularVelocityRadPerSec(Pose2d currentPose, Pose2d goalPose) {
        double angularVelocityRadPerSec = angularController.calculate(
                MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                MathUtil.angleModulus(goalPose.getRotation().getRadians())
        );
        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput("Drive/MoveTo/AngularSetpointUnlimited", angularVelocityRadPerSec);

        boolean angularAtSetpoint = angularController.atSetpoint();
        if (BuildConstants.isSimOrReplay) Logger.recordOutput("Drive/MoveTo/AngularAtSetpoint", angularAtSetpoint);
        if (angularAtSetpoint) {
            angularVelocityRadPerSec = 0.0;
        }

        return angularVelocityRadPerSec;
    }

    public void stop() {
        goalPoseSupplier = null;

        robotState.setMoveToGoal(Optional.empty());
    }
}
