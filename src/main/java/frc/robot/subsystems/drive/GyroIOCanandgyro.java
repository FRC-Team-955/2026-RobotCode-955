package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.lib.HighFrequencySamplingThread;

import java.util.Queue;

/**
 * IO implementation for Canandgyro
 */
public class GyroIOCanandgyro extends GyroIO {
    private final Canandgyro canandgyro;

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    private final double yawFramePeriodSeconds = 0.004;
    private final double otherFramePeriodSeconds = 0.02;

    private final Debouncer connectedDebouncer = new Debouncer(0.5);


    public GyroIOCanandgyro(int canID) {
        canandgyro = new Canandgyro(canID);

        var canandgyroSettings = new CanandgyroSettings();

        canandgyroSettings.setYawFramePeriod(yawFramePeriodSeconds);
        canandgyroSettings.setAngularVelocityFramePeriod(otherFramePeriodSeconds);
        canandgyroSettings.setStatusFramePeriod(otherFramePeriodSeconds);
        canandgyroSettings.setAccelerationFramePeriod(otherFramePeriodSeconds);
        canandgyroSettings.setAngularPositionFramePeriod(otherFramePeriodSeconds);

        canandgyro.setSettings(canandgyroSettings, 0.25, 5);
        canandgyro.setYaw(0.0, 0.25, 5);

        yawTimestampQueue = HighFrequencySamplingThread.get().makeTimestampQueue();
        yawPositionQueue = HighFrequencySamplingThread.get().registerGenericSignal(canandgyro::getYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = connectedDebouncer.calculate(canandgyro.isConnected());
        inputs.temperatureCelsius = canandgyro.getTemperature();

        inputs.yawPositionRad = Units.rotationsToRadians(canandgyro.getMultiturnYaw());
        inputs.orientation = canandgyro.getRotation3d();

        inputs.angularVelocityXRadPerSec = Units.rotationsToRadians(canandgyro.getAngularVelocityRoll());
        inputs.angularVelocityYRadPerSec = Units.rotationsToRadians(canandgyro.getAngularVelocityPitch());
        inputs.angularVelocityZRadPerSec = Units.rotationsToRadians(canandgyro.getAngularVelocityYaw());

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositionsRad =
                yawPositionQueue.stream()
                        .mapToDouble(Units::rotationsToRadians)
                        .toArray();

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
