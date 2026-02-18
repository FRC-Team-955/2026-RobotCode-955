package frc.lib.motor.follower;

import org.littletonrobotics.junction.AutoLog;

public class MotorIOFollower {
    @AutoLog
    public static class MotorIOFollowerInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public void updateInputs(MotorIOFollowerInputs inputs) {
    }

    /**
     * Enable or disable brake mode on the follower motor.
     */
    public void setBrakeMode(boolean enable) {
    }
}
