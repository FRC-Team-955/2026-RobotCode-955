package frc.robot.subsystems.drive.goals;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;

public class FullSpeedCharacterizationGoal extends DriveGoal {
    private final Timer timer = new Timer();

    @Override
    public DriveRequest getRequest() {
        if (!timer.isRunning()) {
            timer.restart();
        }

        if (!timer.hasElapsed(2.0)) {
            return DriveRequest.characterization(2.0);
        } else {
            return DriveRequest.characterization(12.0);
        }
    }
}
