package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public class SuperstructureIO {
    @AutoLog
    public static class SuperstructureIOInputs {
        public boolean funnelBeamBreakTriggered = false;

        /** If triggered is true, the beam is broken */
        public boolean endEffectorBeamBreakTriggered = false;
    }

    public void updateInputs(SuperstructureIOInputs inputs) {
    }
}