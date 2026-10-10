package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.SimManager;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import java.util.Arrays;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class GyroIOSim extends GyroIO {
    private final GyroSimulation gyroSimulation = SimManager.get().driveSimulation.getGyroSimulation();

    public GyroIOSim() {
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;

        inputs.yawPositionRad = gyroSimulation.getGyroReading().getRadians();
        inputs.orientation = new Rotation3d(gyroSimulation.getGyroReading());
        // If you want to test shooting on the bump:
        //.rotateBy(new Rotation3d(Units.degreesToRadians(15), Units.degreesToRadians(-15), 0.0));

        inputs.angularVelocityZRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);

        inputs.odometryYawTimestamps = ModuleIOSim.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositionsRad = Arrays.stream(gyroSimulation.getCachedGyroReadings())
                .mapToDouble(Rotation2d::getRadians)
                .toArray();
    }
}
