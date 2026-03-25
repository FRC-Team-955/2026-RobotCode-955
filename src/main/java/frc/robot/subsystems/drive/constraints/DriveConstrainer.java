package frc.robot.subsystems.drive.constraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.SlewRateLimiter2d;
import frc.lib.wpilib.SlewRateLimiter;
import frc.robot.RobotState;
import org.jetbrains.annotations.Nullable;

public class DriveConstrainer {
    private static final RobotState robotState = RobotState.get();

    private final SlewRateLimiter2d linearAccelLimiter = new SlewRateLimiter2d(0.0);
    private final SlewRateLimiter angularAccelLimiter = new SlewRateLimiter(0.0);

    private @Nullable DriveConstraints constraints = null;

    public void start(DriveConstraints constraints) {
        this.constraints = constraints;

        ChassisSpeeds speeds = robotState.getMeasuredChassisSpeedsFieldRelative();
        linearAccelLimiter.reset(speeds);
        angularAccelLimiter.reset(speeds.omegaRadiansPerSecond);

        if (constraints.maxLinearAccelerationMetersPerSecPerSec() != null &&
                linearAccelLimiter.getLimit() != constraints.maxLinearAccelerationMetersPerSecPerSec().get()) {
            linearAccelLimiter.setLimit(constraints.maxLinearAccelerationMetersPerSecPerSec().get());
        }
        if (constraints.maxAngularAccelerationRadPerSecPerSec() != null &&
                angularAccelLimiter.getLimit() != constraints.maxAngularAccelerationRadPerSecPerSec().get()) {
            angularAccelLimiter.setLimit(constraints.maxAngularAccelerationRadPerSecPerSec().get());
        }
    }

    public void constrainFieldRelativeSpeedsLinear(ChassisSpeeds wantedSpeeds) {
        if (constraints == null) {
            return;
        }

        if (constraints.maxLinearAccelerationMetersPerSecPerSec() != null &&
                constraints.maxLinearAccelerationMetersPerSecPerSec().hasChanged()) {
            linearAccelLimiter.setLimit(constraints.maxLinearAccelerationMetersPerSecPerSec().get());
        }

        Translation2d wantedLinearSpeed = new Translation2d(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);
        // Limit max vel
        if (constraints.maxLinearVelocityMetersPerSec() != null &&
                wantedLinearSpeed.getNorm() > constraints.maxLinearVelocityMetersPerSec().get()) {
            // Convert to unit vector by dividing by norm and then multiply by max
            wantedLinearSpeed = wantedLinearSpeed.div(wantedLinearSpeed.getNorm() / constraints.maxLinearVelocityMetersPerSec().get());
        }
        // Limit max accel
        if (constraints.maxLinearAccelerationMetersPerSecPerSec() != null) {
            wantedLinearSpeed = linearAccelLimiter.calculate(wantedLinearSpeed);
        }
        wantedSpeeds.vxMetersPerSecond = wantedLinearSpeed.getX();
        wantedSpeeds.vyMetersPerSecond = wantedLinearSpeed.getY();
    }

    public void constrainFieldRelativeSpeedsAngular(ChassisSpeeds wantedSpeeds) {
        if (constraints == null) {
            return;
        }

        if (constraints.maxAngularAccelerationRadPerSecPerSec() != null &&
                constraints.maxAngularAccelerationRadPerSecPerSec().hasChanged()) {
            angularAccelLimiter.setLimit(constraints.maxAngularAccelerationRadPerSecPerSec().get());
        }

        double wantedAngularSpeed = wantedSpeeds.omegaRadiansPerSecond;
        // Limit max vel
        if (constraints.maxAngularVelocityRadPerSec() != null &&
                Math.abs(wantedAngularSpeed) > constraints.maxAngularVelocityRadPerSec().get()) {
            wantedAngularSpeed = Math.copySign(constraints.maxAngularVelocityRadPerSec().get(), wantedAngularSpeed);
        }
        // Limit max accel
        if (constraints.maxAngularAccelerationRadPerSecPerSec() != null) {
            wantedAngularSpeed = angularAccelLimiter.calculate(wantedAngularSpeed);
        }
        wantedSpeeds.omegaRadiansPerSecond = wantedAngularSpeed;
    }

    public void stop() {
        constraints = null;
    }
}
