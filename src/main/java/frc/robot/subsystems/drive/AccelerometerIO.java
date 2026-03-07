package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public class AccelerometerIO {
    @AutoLog
    public static class AccelerometerIOInputs {
        public double accelerationXMetersPerSecPerSec = 0.0;
        public double accelerationYMetersPerSecPerSec = 0.0;
        public double accelerationZMetersPerSecPerSec = 0.0;
    }

    public void updateInputs(AccelerometerIOInputs inputs) {
    }
}
