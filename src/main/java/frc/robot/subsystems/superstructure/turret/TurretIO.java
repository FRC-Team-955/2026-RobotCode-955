package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.network.LoggedTunablePIDF;

public class TurretIO {
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
     * Enable or disable brake mode on the motor. The turret must be in coast to be hand
     * pushed into the hard stops for homing.
     */
    public void setNeutralMode(NeutralModeValue neutralMode) {
    }

    /**
     * Unlike the other homed mechanisms, the turret is homed off of two points, so the
     * seeded position isn't known ahead of time.
     */
    public void setEncoderPosition(double positionRad) {
    }
}
