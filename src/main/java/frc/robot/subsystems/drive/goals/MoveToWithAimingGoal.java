package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.AllianceFlipUtil;
import frc.lib.SlewRateLimiter2d;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.ShootingKinematics;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.maxAngularVelocityRadPerSec;
import static frc.robot.subsystems.drive.DriveConstants.moveToConfig;

@RequiredArgsConstructor
public class MoveToWithAimingGoal extends DriveGoal {
    private static final LoggedTunableNumber maxAccelerationMetersPerSecPerSec = new LoggedTunableNumber("Drive/MoveToWithAiming/MaxAccelerationMetersPerSecPerSec", moveToConfig.maxAccelerationMetersPerSecPerSec());

    private static final RobotState robotState = RobotState.get();

    private final Supplier<Pose2d> poseSupplier;
    private final double maxVelocityMetersPerSec;

    private final PIDController moveToLinear = moveToConfig.linear()
            .toPID(
                    moveToConfig.linearPositionToleranceMeters(),
                    moveToConfig.linearVelocityToleranceMetersPerSec()
            );
    private final SlewRateLimiter2d moveToLinearAccelerationLimiter = new SlewRateLimiter2d(maxAccelerationMetersPerSecPerSec.get(), robotState.getMeasuredChassisSpeeds());

    private final PIDController headingOverride = moveToConfig.angular()
            .toPIDWrapRadians(
                    moveToConfig.angularPositionToleranceRad(),
                    moveToConfig.angularVelocityToleranceRadPerSec()
            );

    @Override
    public DriveRequest getRequest() {
        Logger.recordOutput("Drive/MoveToWithAiming/Active", true);

        if (moveToConfig.linear().hasChanged()) {
            moveToConfig.linear().applyPID(moveToLinear);
        }
        if (moveToConfig.angular().hasChanged()) {
            moveToConfig.angular().applyPID(headingOverride);
        }
        if (maxAccelerationMetersPerSecPerSec.hasChanged()) {
            moveToLinearAccelerationLimiter.setLimit(maxAccelerationMetersPerSecPerSec.get());
        }

        //////////////////////////////////////////////////////////////////////

        Pose2d currentPose = robotState.getPose();

        Pose2d goalPose = poseSupplier.get();
        Logger.recordOutput("Drive/MoveToWithAiming/Goal", goalPose);

        double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());

        double linearVelocityMetersPerSec;
        {
            linearVelocityMetersPerSec = moveToLinear.calculate(
                    distanceToGoal,
                    0.0
            );

            // Clamp to max velocity
            linearVelocityMetersPerSec = MathUtil.clamp(linearVelocityMetersPerSec, -maxVelocityMetersPerSec, maxVelocityMetersPerSec);

            boolean linearAtSetpoint = moveToLinear.atSetpoint();
            if (linearAtSetpoint) {
                linearVelocityMetersPerSec = 0.0;
            }
        }

        double angularVelocityRadPerSec;
        {
            // Calculate heading to hub directly
            // The fuel exit is at the back of the robot (180 deg offset), so we need to point
            // the back of the robot at the hub
            Translation2d hubCenter = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
            Translation2d robotToHub = hubCenter.minus(currentPose.getTranslation());
            // Add 180 degrees because we want the back of the robot (where fuel exits) to face the hub
            double targetHeading = robotToHub.getAngle().plus(ShootingKinematics.fuelExitRotation).getRadians();

            double currentHeading = currentPose.getRotation().getRadians();
            Logger.recordOutput("Drive/MoveToWithAiming/HubCenter", hubCenter);
            Logger.recordOutput("Drive/MoveToWithAiming/TargetHeading", targetHeading);
            Logger.recordOutput("Drive/MoveToWithAiming/CurrentHeading", currentHeading);
            Logger.recordOutput("Drive/MoveToWithAiming/HeadingError", MathUtil.angleModulus(targetHeading - currentHeading));

            angularVelocityRadPerSec = headingOverride.calculate(
                    MathUtil.angleModulus(currentHeading),
                    MathUtil.angleModulus(targetHeading)
            );
            Logger.recordOutput("Drive/MoveToWithAiming/AngularVelocityRadPerSec", angularVelocityRadPerSec);
        }

        // Use absolute value for linear scaling
        double linearScalar = MathUtil.clamp(1 - Math.abs(angularVelocityRadPerSec) / maxAngularVelocityRadPerSec, 0.50, 1);

        Rotation2d angleToGoalRad = currentPose.getTranslation().minus(goalPose.getTranslation()).getAngle();
        Translation2d linearXYVelocityMetersPerSec = new Translation2d(linearVelocityMetersPerSec, angleToGoalRad);
        linearXYVelocityMetersPerSec = moveToLinearAccelerationLimiter.calculate(linearXYVelocityMetersPerSec);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearXYVelocityMetersPerSec.getX() * linearScalar,
                linearXYVelocityMetersPerSec.getY() * linearScalar,
                angularVelocityRadPerSec,
                currentPose.getRotation()
        );

        return DriveRequest.chassisSpeeds(chassisSpeeds);
    }
}
