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

        // NOTE: robot relative! not field relative
        ChassisSpeeds speeds = robotState.getMeasuredChassisSpeedsRobotRelative();
        linearAccelLimiter.reset(speeds);
        angularAccelLimiter.reset(speeds.omegaRadiansPerSecond);

        if (linearAccelLimiter.getLimit() != constraints.maxLinearAccelerationMetersPerSecPerSec().get()) {
            linearAccelLimiter.setLimit(constraints.maxLinearAccelerationMetersPerSecPerSec().get());
        }
        if (angularAccelLimiter.getLimit() != constraints.maxAngularAccelerationRadPerSecPerSec().get()) {
            angularAccelLimiter.setLimit(constraints.maxAngularAccelerationRadPerSecPerSec().get());
        }
    }

    public ChassisSpeeds constrainRobotRelativeSpeeds(ChassisSpeeds wantedSpeeds) {
        if (constraints == null) {
            return wantedSpeeds;
        }

        if (constraints.maxLinearAccelerationMetersPerSecPerSec().hasChanged()) {
            linearAccelLimiter.setLimit(constraints.maxLinearAccelerationMetersPerSecPerSec().get());
        }
        if (constraints.maxAngularAccelerationRadPerSecPerSec().hasChanged()) {
            angularAccelLimiter.setLimit(constraints.maxAngularAccelerationRadPerSecPerSec().get());
        }

        Translation2d wantedLinearSpeed = new Translation2d(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);
        // Limit max vel
        if (wantedLinearSpeed.getNorm() > constraints.maxLinearVelocityMetersPerSec().get()) {
            // Convert to unit vector by dividing by norm and then multiply by max
            wantedLinearSpeed = wantedLinearSpeed.div(wantedLinearSpeed.getNorm() / constraints.maxLinearVelocityMetersPerSec().get());
        }
        // Limit max accel
        wantedLinearSpeed = linearAccelLimiter.calculate(wantedLinearSpeed);

        double wantedAngularSpeed = wantedSpeeds.omegaRadiansPerSecond;
        // Limit max vel
        if (Math.abs(wantedAngularSpeed) > constraints.maxAngularVelocityRadPerSec().get()) {
            wantedAngularSpeed = Math.copySign(constraints.maxAngularVelocityRadPerSec().get(), wantedAngularSpeed);
        }
        // Limit max accel
        wantedAngularSpeed = angularAccelLimiter.calculate(wantedAngularSpeed);

        return new ChassisSpeeds(
                wantedLinearSpeed.getX(),
                wantedLinearSpeed.getY(),
                wantedAngularSpeed
        );
    }

    public void stop() {
        constraints = null;
    }
}
