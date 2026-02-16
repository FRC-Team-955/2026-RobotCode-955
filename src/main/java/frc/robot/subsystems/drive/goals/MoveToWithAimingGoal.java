package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.PIDF;
import frc.lib.SlewRateLimiter2d;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.ShootingKinematics;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.moveToConfig;
import static frc.robot.subsystems.drive.DriveTuning.headingOverrideGainsTunable;

@RequiredArgsConstructor
public class MoveToWithAimingGoal extends DriveGoal {
    private static final PIDF.Tunable moveToLinearTunable = moveToConfig.linear().tunable("Drive/MoveToWithAiming/Linear");
    private static final LoggedTunableNumber maxAccelerationMetersPerSecPerSec = new LoggedTunableNumber("Drive/MoveToWithAiming/MaxAccelerationMetersPerSecPerSec", moveToConfig.maxAccelerationMetersPerSecPerSec());
    private static final LoggedTunableNumber maxVelocityMetersPerSec = new LoggedTunableNumber("Drive/MoveToWithAiming/MaxVelocityMetersPerSec", 1.0);

    private static final RobotState robotState = RobotState.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private final Supplier<Pose2d> poseSupplier;

    private final PIDController moveToLinear = moveToLinearTunable.getOrOriginal()
            .toPID(
                    moveToConfig.linearPositionToleranceMeters(),
                    moveToConfig.linearVelocityToleranceMetersPerSec()
            );
    private final SlewRateLimiter2d moveToLinearAccelerationLimiter = new SlewRateLimiter2d(maxAccelerationMetersPerSecPerSec.get(), robotState.getMeasuredChassisSpeeds());
    private final PIDController headingOverride = headingOverrideGainsTunable.getOrOriginal().toPIDWrapRadians();

    @Override
    public DriveRequest getRequest() {
        moveToLinearTunable.ifChanged(gains -> gains.applyPID(moveToLinear));
        headingOverrideGainsTunable.ifChanged(gains -> gains.applyPID(headingOverride));
        if (maxAccelerationMetersPerSecPerSec.hasChanged()) {
            moveToLinearAccelerationLimiter.setLimit(maxAccelerationMetersPerSecPerSec.get());
        }

        Logger.recordOutput("Drive/MoveToWithAiming/Active", true);

        Pose2d currentPose = robotState.getPose();
        Pose2d goalPose = poseSupplier.get();
        Logger.recordOutput("Drive/MoveToWithAiming/Goal", goalPose);
        Logger.recordOutput("Drive/MoveToWithAiming/CurrentPose", currentPose);

        double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        Logger.recordOutput("Drive/MoveToWithAiming/DistanceToGoalMeters", distanceToGoal);

        double linearVelocityMetersPerSec = moveToLinear.calculate(distanceToGoal, 0.0);

        linearVelocityMetersPerSec = Math.min(Math.abs(linearVelocityMetersPerSec), maxVelocityMetersPerSec.get());
        linearVelocityMetersPerSec = -linearVelocityMetersPerSec;

        boolean linearAtSetpoint = moveToLinear.atSetpoint();
        Logger.recordOutput("Drive/MoveToWithAiming/LinearAtSetpoint", linearAtSetpoint);
        if (linearAtSetpoint) {
            linearVelocityMetersPerSec = 0.0;
        }

        double targetHeading = shootingKinematics.getShootingParameters().headingRad();
        double angularVelocityRadPerSec = headingOverride.calculate(
                currentPose.getRotation().getRadians(),
                targetHeading
        );
        Logger.recordOutput("Drive/MoveToWithAiming/TargetHeading", targetHeading);
        Logger.recordOutput("Drive/MoveToWithAiming/AngularVelocity", angularVelocityRadPerSec);

        Rotation2d angleToGoalRad = currentPose.getTranslation().minus(goalPose.getTranslation()).getAngle();
        Translation2d linearXYVelocityMetersPerSec = new Translation2d(linearVelocityMetersPerSec, angleToGoalRad);
        linearXYVelocityMetersPerSec = moveToLinearAccelerationLimiter.calculate(linearXYVelocityMetersPerSec);
        Logger.recordOutput("Drive/MoveToWithAiming/LinearSetpoint", linearXYVelocityMetersPerSec.getNorm());
        Logger.recordOutput("Drive/MoveToWithAiming/AngleToGoal", angleToGoalRad);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearXYVelocityMetersPerSec.getX(),
                linearXYVelocityMetersPerSec.getY(),
                angularVelocityRadPerSec,
                currentPose.getRotation()
        );
        Logger.recordOutput("Drive/MoveToWithAiming/OutputChassisSpeeds", chassisSpeeds);

        return DriveRequest.chassisSpeeds(chassisSpeeds);
    }
}



