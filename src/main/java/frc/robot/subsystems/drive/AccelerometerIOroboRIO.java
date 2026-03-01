package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;

public class AccelerometerIOroboRIO extends AccelerometerIO {
    private static final double g = 9.81;

    private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer(BuiltInAccelerometer.Range.k8G);

    public AccelerometerIOroboRIO() {
    }

    @Override
    public void updateInputs(AccelerometerIOInputs inputs) {
        inputs.accelerationXMetersPerSecPerSec = accelerometer.getX() / g;
        inputs.accelerationYMetersPerSecPerSec = accelerometer.getY() / g;
        inputs.accelerationZMetersPerSecPerSec = accelerometer.getZ() / g;
    }
}
