package frc.lib.motor;

import frc.lib.PIDF;
import org.littletonrobotics.junction.AutoLog;

public class MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public void updateInputs(MotorIOInputs inputs) {
    }

    /**
     * Change PIDF gains. IO layers should not rely on this method being called, and
     * should default to the gains in the constants
     */
    public void setPositionPIDF(PIDF newGains) {
    }

    /**
     * Change PIDF gains. IO layers should not rely on this method being called, and
     * should default to the gains in the constants
     */
    public void setVelocityPIDF(PIDF newGains) {
    }

    /**
     * Enable or disable brake mode on the motor.
     */
    public void setBrakeMode(boolean enable) {
    }

    public void setRequest(RequestType type, double value) {
    }
}
