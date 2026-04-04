package frc955.gamepiecevision;

import edu.wpi.first.wpilibj.TimedRobot;
import org.photonvision.PhotonCamera;

public class Robot extends TimedRobot {
    public Robot() {
    }

    PhotonCamera c = new PhotonCamera("IntakeCam");

    @Override
    public void robotPeriodic() {
        System.out.println(c.isConnected());
    }
}