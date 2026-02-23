package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState;
import frc.robot.controller.Controller;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;

import java.util.function.Supplier;

@RequiredArgsConstructor
public class MoveToGoal extends DriveGoal {
    private static final LoggedTunableNumber maxAccelerationMetersPerSecPerSec = new LoggedTunableNumber("Drive/MoveTo/MaxAccelerationMetersPerSecPerSec", defaultMoveToConfig.maxAccelerationMetersPerSecPerSec());

    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();

    private final Supplier<Pose2d> poseSupplier;
    private final DriveConstants.MoveToConstraints config;

    private final PIDController moveToLinear = defaultMoveToConfig.linear()
            .toPID(
                    defaultMoveToConfig.linearPositionToleranceMeters(),
                    defaultMoveToConfig.linearVelocityToleranceMetersPerSec()
            );
    private final SlewRateLimiter2d moveToLinearAccelerationLimiter = new SlewRateLimiter2d(maxAccelerationMetersPerSecPerSec.get(), robotState.getMeasuredChassisSpeeds());

    private final PIDController moveToAngular = defaultMoveToConfig.angular()
            .toPIDWrapRadians(
                    defaultMoveToConfig.angularPositionToleranceRad(),
                    defaultMoveToConfig.angularVelocityToleranceRadPerSec()
            );

    @Override
    public DriveRequest getRequest() {
        if (defaultMoveToConfig.linear().hasChanged()) {
            defaultMoveToConfig.linear().applyPID(moveToLinear);
        }
        if (defaultMoveToConfig.angular().hasChanged()) {
            defaultMoveToConfig.angular().applyPID(moveToAngular);
        }
        if (maxAccelerationMetersPerSecPerSec.hasChanged()) {
            moveToLinearAccelerationLimiter.setLimit(maxAccelerationMetersPerSecPerSec.get());
        }

        //////////////////////////////////////////////////////////////////////

        Pose2d currentPose = robotState.getPose();

        Pose2d goalPose = poseSupplier.get();
        Logger.recordOutput("Drive/MoveTo/Goal", goalPose);

        double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
//        Logger.recordOutput("Drive/MoveTo/DistanceToGoalMeters", distanceToGoal);

        double linearVelocityMetersPerSec;
        {
            linearVelocityMetersPerSec = moveToLinear.calculate(
                    distanceToGoal,
                    0.0
            );
            linearVelocityMetersPerSec = MathUtil.clamp(linearVelocityMetersPerSec, -2, 2);
//            Logger.recordOutput("Drive/MoveTo/LinearSetpointUnlimited", linearVelocityMetersPerSec);

            boolean linearAtSetpoint = moveToLinear.atSetpoint();
//            Logger.recordOutput("Drive/MoveTo/LinearAtSetpoint", linearAtSetpoint);
            if (linearAtSetpoint) {
                linearVelocityMetersPerSec = 0.0;
            }
        }

        double angularVelocityRadPerSec;
        {
            angularVelocityRadPerSec = moveToAngular.calculate(
                    MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                    MathUtil.angleModulus(goalPose.getRotation().getRadians())
            );
//            Logger.recordOutput("Drive/MoveTo/AngularSetpoint", angularVelocityRadPerSec);

            boolean angularAtSetpoint = moveToAngular.atSetpoint();
//            Logger.recordOutput("Drive/MoveTo/AngularAtSetpoint", angularAtSetpoint);
            if (angularAtSetpoint) {
                angularVelocityRadPerSec = 0.0;
            }
        }

        // Scale linear speed by angular speed so that angular change is prioritized
        // When going max angular speed, reduce linear to 50%
        double linearScalar = MathUtil.clamp(1 - angularVelocityRadPerSec / maxAngularVelocityRadPerSec, 0.50, 1);

        Rotation2d angleToGoalRad = currentPose.getTranslation().minus(goalPose.getTranslation()).getAngle();
        Translation2d linearXYVelocityMetersPerSec = new Translation2d(linearVelocityMetersPerSec, angleToGoalRad);
        linearXYVelocityMetersPerSec = moveToLinearAccelerationLimiter.calculate(linearXYVelocityMetersPerSec);
//        Logger.recordOutput("Drive/MoveTo/LinearSetpoint", linearXYVelocityMetersPerSec.getNorm());

        return DriveRequest.chassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                linearXYVelocityMetersPerSec.getX() * linearScalar,
                linearXYVelocityMetersPerSec.getY() * linearScalar,
                angularVelocityRadPerSec,
                currentPose.getRotation() // Move to is absolute, don't flip
        ));
        return DriveRequest.stop();
    }
}
