package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DigitalInput;

public class SuperstructureIOReal extends SuperstructureIO {
    private final DigitalInput endEffectorBeamBreak = new DigitalInput(1);
    private final DigitalInput funnelBeamBreak = new DigitalInput(9);

    public SuperstructureIOReal() {
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        inputs.endEffectorBeamBreakTriggered = !endEffectorBeamBreak.get();
        inputs.funnelBeamBreakTriggered = !funnelBeamBreak.get();
    }
}
