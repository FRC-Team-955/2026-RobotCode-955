package frc955.gamepiecevision;

import edu.wpi.first.wpilibj.TimedRobot;
import org.photonvision.PhotonCamera;

public class Robot extends TimedRobot {
    public Robot() {
    }

    private final GamePieceVision gamePieceVision = new GamePieceVision();

    @Override
    public void robotPeriodic() {
        Logger.periodicBeforeCode();

        gamePieceVision.periodic();

        Logger.periodicAfterCode();
    }
}