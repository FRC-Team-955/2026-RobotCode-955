package frc.lib.devices.distancesensor;

import com.ctre.phoenix6.signals.MeasurementHealthValue;
import frc.lib.devices.device.Device;
import frc.robot.BuildConstants;

import java.util.function.DoubleSupplier;

public class DistanceSensor extends Device<DistanceSensorIO, DistanceSensorIOInputsAutoLogged> {
    private DistanceSensor(String name, DistanceSensorIO io) {
        super(name, io, new DistanceSensorIOInputsAutoLogged());
    }

    public static DistanceSensor createCANRange(String name, int canID, DoubleSupplier simulatedDistanceSupplier) {
        return new DistanceSensor(name, switch (BuildConstants.mode) {
            case REAL -> new DistanceSensorIOCANrange(canID);
            case SIM -> new DistanceSensorIOCANrangeSim(simulatedDistanceSupplier);
            case REPLAY -> new DistanceSensorIOReplay();
        });
    }

    @Override
    public boolean isConnected() {
        return inputs.connected;
    }

    public double getDistanceMeters() {
        return inputs.distanceMeters;
    }

    public double getDistanceStdDevMeters() {
        return inputs.distanceStdDevMeters;
    }

    public double getSignalStrength() {
        return inputs.signalStrength;
    }

    public double getAmbientSignal() {
        return inputs.ambientSignal;
    }

    public MeasurementHealthValue getMeasurementHealth() {
        return inputs.measurementHealth;
    }

    public double getMeasurementTime() {
        return inputs.measurementTime;
    }
}
