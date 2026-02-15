package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.PhoenixUtil;
import frc.robot.Constants;

public class SuperstructureIOReal extends SuperstructureIO {
    private final CANrange canrange = new CANrange(-1, Constants.canivoreBus);

    private final StatusSignal<Distance> distance;
    private final StatusSignal<Distance> distanceStddev;
    private final StatusSignal<Double> signalStrength;
    private final StatusSignal<Double> ambientSignal;
    private final StatusSignal<MeasurementHealthValue> measurementHealth;
    private final StatusSignal<Time> measurementTime;

    private final Debouncer canrangeConnectedDebouncer = new Debouncer(0.5);

    public SuperstructureIOReal() {
        var canrangeConfig = new CANrangeConfiguration();
        canrangeConfig.FovParams.FOVRangeX = 8.0;
        canrangeConfig.FovParams.FOVRangeY = 8.0;
        canrangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
        PhoenixUtil.tryUntilOk(5, () -> canrange.getConfigurator().apply(canrangeConfig, 0.25));

        distance = canrange.getDistance();
        distanceStddev = canrange.getDistanceStdDev();
        signalStrength = canrange.getSignalStrength();
        ambientSignal = canrange.getAmbientSignal();
        measurementHealth = canrange.getMeasurementHealth();
        measurementTime = canrange.getMeasurementTime();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                distance,
                distanceStddev,
                signalStrength,
                ambientSignal,
                measurementHealth,
                measurementTime
        );
        ParentDevice.optimizeBusUtilizationForAll(canrange);
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        var canrangeStatus = BaseStatusSignal.refreshAll(
                distance,
                distanceStddev,
                signalStrength,
                ambientSignal,
                measurementHealth,
                measurementTime
        );
        inputs.canrangeConnected = canrangeConnectedDebouncer.calculate(canrangeStatus.isOK());
        inputs.canrangeDistanceMeters = distance.getValueAsDouble();
        inputs.canrangeDistanceStddevMeters = distanceStddev.getValueAsDouble();
        inputs.canrangeSignalStrength = signalStrength.getValueAsDouble();
        inputs.canrangeAmbientSignal = ambientSignal.getValueAsDouble();
        inputs.canrangeMeasurementHealth = measurementHealth.getValue();
        inputs.canrangeMeasurementTime = measurementTime.getValueAsDouble();
    }
}
