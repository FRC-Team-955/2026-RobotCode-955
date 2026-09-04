package frc.lib.devices.distancesensor;

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

public class DistanceSensorIOCANrange extends DistanceSensorIO {
    protected final CANrange canRange;

    private final StatusSignal<Distance> distance;
    private final StatusSignal<Distance> distanceStdDev;
    private final StatusSignal<Double> signalStrength;
    private final StatusSignal<Double> ambientSignal;
    private final StatusSignal<MeasurementHealthValue> measurementHealth;
    private final StatusSignal<Time> measurementTime;

    private final Debouncer canrangeConnectedDebouncer = new Debouncer(0.5);

    public DistanceSensorIOCANrange(int canID) {
        canRange = new CANrange(canID, Constants.canivoreBus);

        // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/canrange/tuning-canrange.html
        var canrangeConfig = new CANrangeConfiguration();
        canrangeConfig.FovParams.FOVRangeX = 8.0;
        canrangeConfig.FovParams.FOVRangeY = 8.0;
        canrangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
        PhoenixUtil.tryUntilOk(5, () -> canRange.getConfigurator().apply(canrangeConfig, 0.25));

        distance = canRange.getDistance();
        distanceStdDev = canRange.getDistanceStdDev();
        signalStrength = canRange.getSignalStrength();
        ambientSignal = canRange.getAmbientSignal();
        measurementHealth = canRange.getMeasurementHealth();
        measurementTime = canRange.getMeasurementTime();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                distance,
                distanceStdDev,
                signalStrength,
                ambientSignal,
                measurementHealth,
                measurementTime
        );
        ParentDevice.optimizeBusUtilizationForAll(canRange);
    }

    @Override
    public void updateInputs(DistanceSensorIOInputsAutoLogged inputs) {
        var canRangeStatus = BaseStatusSignal.refreshAll(
                distance,
                distanceStdDev,
                signalStrength,
                ambientSignal,
                measurementHealth,
                measurementTime
        );
        inputs.connected = canrangeConnectedDebouncer.calculate(canRangeStatus.isOK());
        inputs.distanceMeters = distance.getValueAsDouble();
        inputs.distanceStdDevMeters = distanceStdDev.getValueAsDouble();
        inputs.signalStrength = signalStrength.getValueAsDouble();
        inputs.ambientSignal = ambientSignal.getValueAsDouble();
        inputs.measurementHealth = measurementHealth.getValue();
        inputs.measurementTime = measurementTime.getValueAsDouble();
    }
}
