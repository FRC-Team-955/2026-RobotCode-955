package frc955.gamepiecevision;

import edu.wpi.first.wpilibj.TimedRobot;
import frc955.gamepiecevision.logging.Logger;

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