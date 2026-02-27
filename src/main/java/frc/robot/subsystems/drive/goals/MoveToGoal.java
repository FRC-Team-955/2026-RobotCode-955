package frc.robot.subsystems.drive.goals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.SlewRateLimiter2d;
import frc.lib.wpilib.SlewRateLimiter;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.drive.DriveConstants.moveToConfig;

public class MoveToGoal extends DriveGoal {
    private static final RobotState robotState = RobotState.get();
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private final Supplier<Pose2d> poseSupplier;
    private final DriveConstants.MoveToConstraints constraints;

    private final PIDController moveToLinear = moveToConfig.linearGains()
            .toPID(
                    moveToConfig.linearPositionToleranceMeters().get(),
                    moveToConfig.linearVelocityToleranceMetersPerSec().get()
            );
    private final SlewRateLimiter2d moveToLinearAccelerationLimiter;

    private final PIDController moveToAngular = moveToConfig.angularGains()
            .toPIDWrapRadians(
                    moveToConfig.angularPositionToleranceRad().get(),
                    moveToConfig.angularVelocityToleranceRadPerSec().get()
            );
    private final SlewRateLimiter moveToAngularAccelerationLimiter;

    public MoveToGoal(Supplier<Pose2d> poseSupplier, DriveConstants.MoveToConstraints constraints) {
        this.poseSupplier = poseSupplier;
        this.constraints = constraints;

        moveToAngularAccelerationLimiter = new SlewRateLimiter(
                constraints.maxAngularAccelerationRadPerSecPerSec().get(),
                robotState.getMeasuredChassisSpeeds().omegaRadiansPerSecond
        );
        moveToLinearAccelerationLimiter = new SlewRateLimiter2d(
                constraints.maxLinearAccelerationMetersPerSecPerSec().get(),
                robotState.getMeasuredChassisSpeeds()
        );
    }

    @Override
    public DriveRequest getRequest() {
        if (moveToConfig.linearGains().hasChanged()) {
            moveToConfig.linearGains().applyPID(moveToLinear);
        }
        if (constraints.maxLinearAccelerationMetersPerSecPerSec().hasChanged()) {
            moveToLinearAccelerationLimiter.setLimit(constraints.maxLinearAccelerationMetersPerSecPerSec().get());
        }

        if (moveToConfig.angularGains().hasChanged()) {
            moveToConfig.angularGains().applyPID(moveToAngular);
        }
        if (constraints.maxAngularAccelerationRadPerSecPerSec().hasChanged()) {
            moveToAngularAccelerationLimiter.setLimit(constraints.maxAngularAccelerationRadPerSecPerSec().get());
        }

        //////////////////////////////////////////////////////////////////////

        Pose2d currentPose = robotState.getPose();

        Pose2d goalPose = poseSupplier.get();
        Logger.recordOutput("Drive/MoveTo/Goal", goalPose);

        double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        //Logger.recordOutput("Drive/MoveTo/DistanceToGoalMeters", distanceToGoal);

        double linearVelocityMetersPerSec = calculateLinearVelocityMetersPerSec(distanceToGoal);
        Rotation2d angleToGoalRad = goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
        Translation2d linearXYVelocityMetersPerSec = new Translation2d(linearVelocityMetersPerSec, angleToGoalRad);

        double angularVelocityRadPerSec = calculateAngularVelocityRadPerSec(currentPose, goalPose, linearXYVelocityMetersPerSec);

        // Scale linear speed by angular speed so that angular change is prioritized
        // When going max angular speed, reduce linear to 50%
        double linearScalar = MathUtil.clamp(1 - angularVelocityRadPerSec / DriveConstants.maxAngularVelocityRadPerSec, 0.50, 1);

        linearXYVelocityMetersPerSec = moveToLinearAccelerationLimiter.calculate(linearXYVelocityMetersPerSec);
        //Logger.recordOutput("Drive/MoveTo/LinearSetpoint", linearXYVelocityMetersPerSec.getNorm());

        angularVelocityRadPerSec = moveToAngularAccelerationLimiter.calculate(angularVelocityRadPerSec);
        //Logger.recordOutput("Drive/MoveTo/LinearSetpoint", angularVelocityRadPerSec);

        return DriveRequest.chassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                linearXYVelocityMetersPerSec.getX() * linearScalar,
                linearXYVelocityMetersPerSec.getY() * linearScalar,
                angularVelocityRadPerSec,
                currentPose.getRotation() // Move to is absolute, don't flip
        ));
    }

    private double calculateLinearVelocityMetersPerSec(double distanceToGoal) {
        if (!constraints.fullSpeed()) {
            double linearVelocityMetersPerSec = moveToLinear.calculate(
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

            boolean linearAtSetpoint = moveToLinear.atSetpoint();
            //Logger.recordOutput("Drive/MoveTo/LinearAtSetpoint", linearAtSetpoint);
            if (linearAtSetpoint) {
                linearVelocityMetersPerSec = 0.0;
            }

            return linearVelocityMetersPerSec;
        } else {
            return constraints.maxLinearVelocityMetersPerSec().get();
        }
    }

    private double calculateAngularVelocityRadPerSec(Pose2d currentPose, Pose2d goalPose, Translation2d linearSetpoint) {
        double angularVelocityRadPerSec = moveToAngular.calculate(
                MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                MathUtil.angleModulus(goalPose.getRotation().getRadians())
        );
        angularVelocityRadPerSec = MathUtil.clamp(
                angularVelocityRadPerSec,
                -constraints.maxAngularVelocityRadPerSec().get(),
                constraints.maxAngularVelocityRadPerSec().get()
        );
        //Logger.recordOutput("Drive/MoveTo/AngularSetpointUnlimited", angularVelocityRadPerSec);

        boolean angularAtSetpoint = moveToAngular.atSetpoint();
        //Logger.recordOutput("Drive/MoveTo/AngularAtSetpoint", angularAtSetpoint);
        if (angularAtSetpoint) {
            angularVelocityRadPerSec = 0.0;
        }

        if (constraints.applyAimingFeedforward()) {
            angularVelocityRadPerSec += shootingKinematics.rotationAboutHubRadiansPerSec(linearSetpoint);
        }

        return angularVelocityRadPerSec;
    }
}
