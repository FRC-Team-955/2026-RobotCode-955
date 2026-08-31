package frc.lib.devices.distancesensor;

import com.ctre.phoenix6.signals.MeasurementHealthValue;
import frc.lib.devices.device.DeviceIO;
import org.littletonrobotics.junction.AutoLog;

public abstract class DistanceSensorIO extends DeviceIO<DistanceSensorIOInputsAutoLogged> {
    @AutoLog
    public static class DistanceSensorIOInputs {
        public boolean connected = false;
        public double distanceMeters = 0.0;
        public double distanceStdDevMeters = 0.0;
        public double signalStrength = 0.0;
        public double ambientSignal = 0.0;
        public MeasurementHealthValue measurementHealth = MeasurementHealthValue.Bad;
        public double measurementTime = 0.0;
    }
}
