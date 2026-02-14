package frc.robot.subsystems.gamepiecevision;

public class GamePieceVisionIOSim extends GamePieceVisionIO {
    private boolean ledsOn = false;

    public GamePieceVisionIOSim() {
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        inputs.connected = true;
        inputs.ledsOn = ledsOn;
        inputs.visible = false;
    }

    @Override
    public void setLEDs(boolean on) {
        ledsOn = on;
    }
}
