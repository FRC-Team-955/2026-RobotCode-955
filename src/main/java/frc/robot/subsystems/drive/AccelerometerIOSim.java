package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.SimManager;

public class AccelerometerIOSim extends AccelerometerIO {
    private final SimManager simManager = SimManager.get();

    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    public AccelerometerIOSim() {
    }

    @Override
    public void updateInputs(AccelerometerIOInputs inputs) {
        ChassisSpeeds currentSpeeds = simManager.driveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative();
        ChassisSpeeds dSpeeds = currentSpeeds.minus(lastSpeeds).div(Constants.loopPeriod);
        lastSpeeds = currentSpeeds;

        inputs.accelerationXMetersPerSecPerSec = dSpeeds.vxMetersPerSecond;
        inputs.accelerationYMetersPerSecPerSec = dSpeeds.vyMetersPerSecond;
        inputs.accelerationZMetersPerSecPerSec = 0.0;
    }
}
