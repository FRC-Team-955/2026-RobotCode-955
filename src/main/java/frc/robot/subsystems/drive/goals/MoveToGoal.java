package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.swerve.ModuleLimits;
import frc.robot.Controller;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.drive.DriveTuning.moveToAngularTunable;
import static frc.robot.subsystems.drive.DriveTuning.moveToLinearTunable;

@RequiredArgsConstructor
public class MoveToGoal extends DriveGoal {
    private static final RobotState robotState = RobotState.get();
    private static final Controller controller = Controller.get();

    private final Supplier<Pose2d> poseSupplier;
    private final boolean mergeJoystickDrive;

    private final PIDController moveToLinearX = moveToLinearTunable.getOrOriginal()
            .toPID(
                    moveToConfig.linearPositionToleranceMeters(),
                    moveToConfig.linearVelocityToleranceMetersPerSec()
            );
    private final PIDController moveToLinearY = moveToLinearTunable.getOrOriginal()
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
        moveToLinearTunable.ifChanged(gains -> {
            gains.applyPID(moveToLinearX);
            gains.applyPID(moveToLinearY);
        });
        moveToAngularTunable.ifChanged(gains -> gains.applyPID(moveToAngular));

        //////////////////////////////////////////////////////////////////////

        Pose2d currentPose = robotState.getPose();

        Pose2d goalPose = poseSupplier.get();
        Logger.recordOutput("Drive/MoveTo/Goal", goalPose);

        double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        Logger.recordOutput("Drive/MoveTo/DistanceToGoalMeters", distanceToGoal);

        double angleToGoalRad = goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle().getRadians();
        Logger.recordOutput("Drive/MoveTo/AngleToGoalRad", angleToGoalRad);

        double linearXVelocityMetersPerSec = moveToLinearX.calculate(
                currentPose.getX(),
                goalPose.getX()
        );
        boolean linearXAtSetpoint = moveToLinearX.atSetpoint();
        Logger.recordOutput("Drive/MoveTo/LinearXAtSetpoint", linearXAtSetpoint);
        if (linearXAtSetpoint) {
            linearXVelocityMetersPerSec = 0.0;
        }

        double linearYVelocityMetersPerSec = moveToLinearY.calculate(
                currentPose.getY(),
                goalPose.getY()
        );
        boolean linearYAtSetpoint = moveToLinearY.atSetpoint();
        Logger.recordOutput("Drive/MoveTo/LinearYAtSetpoint", linearYAtSetpoint);
        if (linearYAtSetpoint) {
            linearYVelocityMetersPerSec = 0.0;
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

        ChassisSpeeds moveToSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearXVelocityMetersPerSec * linearScalar,
                linearYVelocityMetersPerSec * linearScalar,
                angularVelocityRadPerSec,
                currentPose.getRotation() // Move to is absolute, don't flip
        );

        Logger.recordOutput("Drive/MoveTo/MergeJoystickDrive", mergeJoystickDrive);
        if (mergeJoystickDrive) {
            ChassisSpeeds joystickDriveSpeeds = controller.getDriveSetpointRobotRelative(robotState.getRotation());
            return DriveRequest.chassisSpeedsOptimized(moveToSpeeds.plus(joystickDriveSpeeds.times(0.3)));
        } else {
            return DriveRequest.chassisSpeedsOptimized(moveToSpeeds);
        }
    }

    @Override
    public ModuleLimits getModuleLimits() {
        return moveToModuleLimits;
    }
}
