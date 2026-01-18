package frc.robot.subsystems.gamepiecevision;

import frc.robot.subsystems.superstructure.SuperstructureIOSim;

public class GamePieceVisionIOSim extends GamePieceVisionIO {
    private boolean ledsOn = false;

    public GamePieceVisionIOSim() {
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        inputs.connected = true;
        inputs.ledsOn = ledsOn;
        inputs.visible = ledsOn && false;
    }

    @Override
    public void setLEDs(boolean on) {
        ledsOn = on;
    }
}
