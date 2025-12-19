package frc.robot.subsystems.drive.goals;

import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;

public class IdleGoal extends DriveGoal {
    @Override
    public DriveRequest getRequest() {
        return DriveRequest.stop();
    }
}
