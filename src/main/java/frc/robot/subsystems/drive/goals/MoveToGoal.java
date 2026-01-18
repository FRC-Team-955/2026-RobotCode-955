package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.PIDF;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.maxAngularVelocityRadPerSec;
import static frc.robot.subsystems.drive.DriveConstants.moveToConfig;

@RequiredArgsConstructor
public class MoveToGoal extends DriveGoal {
    private static final PIDF.Tunable moveToLinearTunable = moveToConfig.linear().tunable("Drive/MoveTo/Linear");
    private static final PIDF.Tunable moveToAngularTunable = moveToConfig.angular().tunable("Drive/MoveTo/Angular");

    private static final LoggedTunableNumber maxAccelerationMetersPerSecSquared = new LoggedTunableNumber("Drive/MoveTo/MaxAccelerationMetersPerSecSquared", moveToConfig.maxAccelerationMetersPerSecSquared());

    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();

    private final Supplier<Pose2d> poseSupplier;
    private final boolean mergeJoystickDrive;

    private final PIDController moveToLinear = moveToLinearTunable.getOrOriginal()
            .toPID(
                    moveToConfig.linearPositionToleranceMeters(),
                    moveToConfig.linearVelocityToleranceMetersPerSec()
            );
    private SlewRateLimiter moveToLinearAccelerationLimiter = new SlewRateLimiter(maxAccelerationMetersPerSecSquared.get());

    private final PIDController moveToAngular = moveToAngularTunable.getOrOriginal()
            .toPIDWrapRadians(
                    moveToConfig.angularPositionToleranceRad(),
                    moveToConfig.angularVelocityToleranceRadPerSec()
            );

    @Override
    public DriveRequest getRequest() {
        moveToLinearTunable.ifChanged(gains -> gains.applyPID(moveToLinear));
        moveToAngularTunable.ifChanged(gains -> gains.applyPID(moveToAngular));
        if (maxAccelerationMetersPerSecSquared.hasChanged()) {
            double lastVal = moveToLinearAccelerationLimiter.lastValue();
            moveToLinearAccelerationLimiter = new SlewRateLimiter(maxAccelerationMetersPerSecSquared.get());
            moveToLinearAccelerationLimiter.reset(lastVal);
        }

        //////////////////////////////////////////////////////////////////////

        Pose2d currentPose = robotState.getPose();

        Pose2d goalPose = poseSupplier.get();
        Logger.recordOutput("Drive/MoveTo/Goal", goalPose);

        double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        Logger.recordOutput("Drive/MoveTo/DistanceToGoalMeters", distanceToGoal);

        double linearVelocityMetersPerSec;
        {
            linearVelocityMetersPerSec = moveToLinear.calculate(
                    distanceToGoal,
                    0.0
            );
            Logger.recordOutput("Drive/MoveTo/LinearSetpointUnlimited", linearVelocityMetersPerSec);

            linearVelocityMetersPerSec = moveToLinearAccelerationLimiter.calculate(linearVelocityMetersPerSec);
            Logger.recordOutput("Drive/MoveTo/LinearSetpoint", linearVelocityMetersPerSec);

            boolean linearAtSetpoint = moveToLinear.atSetpoint();
            Logger.recordOutput("Drive/MoveTo/LinearAtSetpoint", linearAtSetpoint);
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
            Logger.recordOutput("Drive/MoveTo/AngularSetpoint", angularVelocityRadPerSec);

            boolean angularAtSetpoint = moveToAngular.atSetpoint();
            Logger.recordOutput("Drive/MoveTo/AngularAtSetpoint", angularAtSetpoint);
            if (angularAtSetpoint) {
                angularVelocityRadPerSec = 0.0;
            }
        }

        // Scale linear speed by angular speed so that angular change is prioritized
        // When going max angular speed, reduce linear to 50%
        double linearScalar = MathUtil.clamp(1 - angularVelocityRadPerSec / maxAngularVelocityRadPerSec, 0.50, 1);

        Rotation2d angleToGoalRad = currentPose.getTranslation().minus(goalPose.getTranslation()).getAngle();
        Translation2d linearXYVelocityMetersPerSec = new Translation2d(linearVelocityMetersPerSec, angleToGoalRad);
        ChassisSpeeds moveToSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearXYVelocityMetersPerSec.getX() * linearScalar,
                linearXYVelocityMetersPerSec.getY() * linearScalar,
                angularVelocityRadPerSec,
                currentPose.getRotation() // Move to is absolute, don't flip
        );

        Logger.recordOutput("Drive/MoveTo/MergeJoystickDrive", mergeJoystickDrive);
        if (mergeJoystickDrive) {
            ChassisSpeeds joystickDriveSpeeds = controller.getDriveSetpointRobotRelative(robotState.getRotation());
            return DriveRequest.chassisSpeeds(moveToSpeeds.plus(joystickDriveSpeeds.times(0.3)));
        } else {
            return DriveRequest.chassisSpeeds(moveToSpeeds);
        }
    }
}
