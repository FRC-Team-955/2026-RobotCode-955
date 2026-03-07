package frc.lib.motor;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.network.LoggedTunablePIDF;
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
    public void setPositionPIDF(LoggedTunablePIDF newGains) {
    }

    /**
     * Change PIDF gains. IO layers should not rely on this method being called, and
     * should default to the gains in the constants
     */
    public void setVelocityPIDF(LoggedTunablePIDF newGains) {
    }

    /**
     * Enable or disable brake mode on the motor.
     */
    public void setNeutralMode(NeutralModeValue neutralMode) {
    }

    public void setRequest(RequestType type, double value) {
    }

    /**
     * NOTE: The position will not instantly change!! Keep this in mind!
     * You may want to add a delay before returning to closed loop control
     * so that the motor does not attempt to move to an invalid position
     */
    public void setEncoderPosition(double positionRad) {
    }
}
