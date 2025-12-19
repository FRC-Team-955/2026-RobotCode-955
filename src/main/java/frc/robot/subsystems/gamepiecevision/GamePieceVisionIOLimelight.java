package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class GamePieceVisionIOLimelight extends GamePieceVisionIO {
    private final DoubleSubscriber latencySubscriber;
    private final IntegerSubscriber tvSubscriber;
    private final DoublePublisher ledModePublisher;

    private boolean ledsOn = false;

    public GamePieceVisionIOLimelight(String name) {
        var table = NetworkTableInstance.getDefault().getTable(name);

        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        tvSubscriber = table.getIntegerTopic("tv").subscribe(0);
        ledModePublisher = table.getDoubleTopic("ledMode").publish();
        setLEDs(ledsOn);
    }


    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        inputs.ledsOn = ledsOn;
        inputs.visible = tvSubscriber.get() == 1;
    }

    @Override
    public void setLEDs(boolean on) {
        ledsOn = on;
        ledModePublisher.accept(on ? 3 : 1);

        // Ensures NT changes are propagated ASAP
        // Increases network traffic but recommended by Limelight
        NetworkTableInstance.getDefault().flush();
    }
}
