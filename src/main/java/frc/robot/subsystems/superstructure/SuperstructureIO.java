package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.MeasurementHealthValue;
import org.littletonrobotics.junction.AutoLog;

public class SuperstructureIO {
    @AutoLog
    public static class SuperstructureIOInputs {
        public boolean canrangeConnected = false;
        public double canrangeDistanceMeters = 0.0;
        public double canrangeDistanceStddevMeters = 0.0;
        public double canrangeSignalStrength = 0.0;
        public double canrangeAmbientSignal = 0.0;
        public MeasurementHealthValue canrangeMeasurementHealth = MeasurementHealthValue.Bad;
        public double canrangeMeasurementTime = 0.0;
    }

    public void updateInputs(SuperstructureIOInputs inputs) {
    }
}