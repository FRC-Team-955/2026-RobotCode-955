package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.util.Units;
import frc.lib.HighFrequencySamplingThread;

import java.util.Queue;

/**
 * IO implementation for Canandgyro
 */
public class GyroIORedux extends GyroIO {
    private final Canandgyro canandgyro;

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIORedux(int canID) {
        canandgyro = new Canandgyro(canID);
        yawTimestampQueue = HighFrequencySamplingThread.get().makeTimestampQueue();
        yawPositionQueue = HighFrequencySamplingThread.get().registerGenericSignal(canandgyro::getYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = canandgyro.isConnected();
        inputs.temperatureCelsius = canandgyro.getTemperature();

        inputs.yawPositionRad = canandgyro.getYaw();
        inputs.orientation = canandgyro.getRotation3d();

        inputs.angularVelocityXRadPerSec = Units.rotationsToRadians(canandgyro.getAngularVelocityRoll());
        inputs.angularVelocityYRadPerSec = Units.rotationsToRadians(canandgyro.getAngularVelocityPitch());
        inputs.angularVelocityZRadPerSec = Units.rotationsToRadians(canandgyro.getAngularVelocityYaw());

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositionsRad =
                yawPositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
