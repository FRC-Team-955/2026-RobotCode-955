package frc.robot.subsystems.superstructure.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.network.LoggedTunablePIDF;

public class HoodIO {
    public enum HoodCurrentLimitMode {
        NORMAL,
        HOMING,
    }

    public void updateInputs(MotorIOInputs inputs) {
    }

    /**
     * Change PIDF gains. IO layers should not rely on this method being called, and
     * should default to the gains in the constants
     */
    public void setPositionPIDF(LoggedTunablePIDF newGains) {
    }

    public void setPositionRequest(double positionRad) {
    }

    public void setVoltageRequest(double volts) {
    }

    /**
     * Enable or disable brake mode on the motor.
     */
    public void setNeutralMode(NeutralModeValue neutralMode) {
    }

    public void setCurrentLimit(HoodCurrentLimitMode mode) {
    }

    public void setEncoderPositionToInitial() {
    }
}
