package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.PIDF;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.maxAngularVelocityRadPerSec;
import static frc.robot.subsystems.drive.DriveConstants.moveToConfig;
import static frc.robot.subsystems.drive.DriveTuning.moveToAngularTunable;
import static frc.robot.subsystems.drive.DriveTuning.moveToLinearTunable;

@RequiredArgsConstructor
public class MoveToGoal extends DriveGoal {
    private static final PIDF.Tunable moveToLinearTunable = moveToConfig.linear().tunable("Drive/MoveTo/Linear");
    private static final PIDF.Tunable moveToAngularTunable = moveToConfig.angular().tunable("Drive/MoveTo/Angular");

    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();

    private final Supplier<Pose2d> poseSupplier;
    private final boolean mergeJoystickDrive;

    private final PIDController moveToLinear = moveToLinearTunable.getOrOriginal()
            .toPID(
                    moveToConfig.linearPositionToleranceMeters(),
                    moveToConfig.linearVelocityToleranceMetersPerSec()
            );
    private final PIDController moveToAngular = moveToAngularTunable.getOrOriginal()
            .toPIDWrapRadians(
                    moveToConfig.angularPositionToleranceRad(),
                    moveToConfig.angularVelocityToleranceRadPerSec()
            );

    @Override
    public DriveRequest getRequest() {
        moveToLinearTunable.ifChanged(gains -> gains.applyPID(moveToLinear));
        moveToAngularTunable.ifChanged(gains -> gains.applyPID(moveToAngular));

        //////////////////////////////////////////////////////////////////////

        Pose2d currentPose = robotState.getPose();

        Pose2d goalPose = poseSupplier.get();
        Logger.recordOutput("Drive/MoveTo/Goal", goalPose);

        double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        Logger.recordOutput("Drive/MoveTo/DistanceToGoalMeters", distanceToGoal);

        double linearVelocityMetersPerSec = moveToLinear.calculate(
                distanceToGoal,
                0.0
        );
        boolean linearAtSetpoint = moveToLinear.atSetpoint();
        Logger.recordOutput("Drive/MoveTo/LinearAtSetpoint", linearAtSetpoint);
        if (linearAtSetpoint) {
            linearVelocityMetersPerSec = 0.0;
        }

        double angularVelocityRadPerSec = moveToAngular.calculate(
                MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                MathUtil.angleModulus(goalPose.getRotation().getRadians())
        );
        boolean angularAtSetpoint = moveToAngular.atSetpoint();
        Logger.recordOutput("Drive/MoveTo/AngularAtSetpoint", angularAtSetpoint);
        if (angularAtSetpoint) {
            angularVelocityRadPerSec = 0.0;
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
