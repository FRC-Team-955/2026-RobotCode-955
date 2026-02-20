package frc.lib.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.Constants;

import static frc.lib.PhoenixUtil.tryUntilOk;
import static frc.lib.PhoenixUtil.tryUntilOkAsync;

public class MotorIOTalonFX extends MotorIO {
    // Hardware objects
    private final TalonFX talon;
    private final TalonFXConfiguration config;

    // Request objects
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(1);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> temperatureCelsius;

    // Connection debouncers
    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public MotorIOTalonFX(
            int canID,
            boolean inverted,
            NeutralModeValue neutralMode,
            int currentLimitAmps,
            double gearRatio,
            LoggedTunablePIDF positionGains,
            LoggedTunablePIDF velocityGains
    ) {
        talon = new TalonFX(canID, Constants.canivoreBus);

        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.TorqueCurrent.PeakForwardTorqueCurrent = currentLimitAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -currentLimitAmps;
        config.Feedback.SensorToMechanismRatio = gearRatio;
        if (positionGains != null) config.Slot0 = Slot0Configs.from(positionGains.toPhoenix());
        if (velocityGains != null) config.Slot1 = Slot1Configs.from(velocityGains.toPhoenix());
        tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
        tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

        position = talon.getPosition();
        velocity = talon.getVelocity();
        appliedVolts = talon.getMotorVoltage();
        currentAmps = talon.getStatorCurrent();
        temperatureCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVolts,
                currentAmps,
                temperatureCelsius
        );
        ParentDevice.optimizeBusUtilizationForAll(talon);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps, temperatureCelsius);
        inputs.connected = connectedDebounce.calculate(status.isOK());
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
    }

    @Override
    public void setPositionPIDF(LoggedTunablePIDF newGains) {
        System.out.println("Setting motor position gains");
        config.Slot0 = Slot0Configs.from(newGains.toPhoenix());
        tryUntilOkAsync(5, () -> talon.getConfigurator().apply(config, 0.25));
    }

    @Override
    public void setVelocityPIDF(LoggedTunablePIDF newGains) {
        System.out.println("Setting motor velocity gains");
        config.Slot1 = Slot1Configs.from(newGains.toPhoenix());
        tryUntilOkAsync(5, () -> talon.getConfigurator().apply(config, 0.25));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting motor brake mode to " + enable);
        config.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        tryUntilOkAsync(5, () -> talon.getConfigurator().apply(config, 0.25));
    }

    // This is not included in MotorIO because it should not be used by subsystem code directly
    // If you need to use this, make a custom IO layer that either subclasses or nests this IO layer
    // and take an enum as the argument instead of raw current limit
    // We do this because wanted current limit varies based on which motor we are using
    public void setCurrentLimit(double currentLimitAmps) {
        System.out.println("Setting motor current limit to " + currentLimitAmps);
        config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
        config.TorqueCurrent.PeakForwardTorqueCurrent = currentLimitAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -currentLimitAmps;
        tryUntilOkAsync(5, () -> talon.getConfigurator().apply(config, 0.25));
    }

    @Override
    public void setRequest(RequestType type, double value) {
        talon.setControl(switch (type) {
            case VoltageVolts -> voltageRequest.withOutput(value);
            case PositionRad -> positionRequest.withPosition(Units.radiansToRotations(value));
            case VelocityRadPerSec -> velocityRequest.withVelocity(Units.radiansToRotations(value));
        });
    }

    public void setFollow(MotorIOTalonFX leader, MotorAlignmentValue alignment) {
        talon.setControl(new Follower(leader.talon.getDeviceID(), alignment));
    }
}
