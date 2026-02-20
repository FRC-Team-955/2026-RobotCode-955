package frc.robot.subsystems.superintake.intakepivot;

import frc.lib.motor.MotorIO.MotorIOInputs;
import frc.lib.network.LoggedTunablePIDF;

public class IntakePivotIO {
    public enum IntakePivotCurrentLimitMode {
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

    public void setStopRequest() {
    }

    public void setCurrentLimit(IntakePivotCurrentLimitMode mode) {
    }
}
