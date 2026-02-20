package frc.robot.subsystems.superstructure.flywheel;

import frc.lib.PIDF;
import frc.lib.motor.MotorIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public class FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public MotorIOInputsAutoLogged leader = new MotorIOInputsAutoLogged();
        public MotorIOInputsAutoLogged follower = new MotorIOInputsAutoLogged();
    }

    public void updateInputs(FlywheelIOInputs inputs) {
    }

    /**
     * Change PIDF gains. IO layers should not rely on this method being called, and
     * should default to the gains in the constants
     */
    public void setVelocityPIDF(PIDF newGains) {
    }

    public void setVelocityRequest(double setpointRadPerSec) {
    }

    public void setStopRequest() {
    }
}
