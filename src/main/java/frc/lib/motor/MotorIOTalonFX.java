package frc.lib.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.lib.PIDF;
import frc.lib.PhoenixUtil;

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
            CANBus canBus,
            boolean inverted,
            boolean brakeMode,
            int currentLimitAmps,
            double gearRatio,
            PIDF positionGains,
            PIDF velocityGains
    ) {
        talon = new TalonFX(canID, canBus);

        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = gearRatio;
        config.Slot0 = Slot0Configs.from(positionGains.toPhoenixWithoutFeedforward());
        config.Slot1 = Slot1Configs.from(velocityGains.toPhoenix());

        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

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
        talon.optimizeBusUtilization();
    }

    public MotorIOTalonFX(
            int canID,
            boolean inverted,
            boolean brakeMode,
            int currentLimitAmps,
            double gearRatio,
            PIDF positionGains,
            PIDF velocityGains
    ) {
        this(canID, new CANBus(""), inverted, brakeMode, currentLimitAmps, gearRatio, positionGains, velocityGains);
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
    public void setPositionPIDF(PIDF newGains) {
        System.out.println("Setting motor position gains");
        var slot0 = Slot0Configs.from(newGains.toPhoenixWithoutFeedforward());
        PhoenixUtil.tryUntilOkAsync(5, () -> talon.getConfigurator().apply(slot0, 0.25));
    }

    @Override
    public void setVelocityPIDF(PIDF newGains) {
        System.out.println("Setting motor velocity gains");
        var slot1 = Slot1Configs.from(newGains.toPhoenix());
        PhoenixUtil.tryUntilOkAsync(5, () -> talon.getConfigurator().apply(slot1, 0.25));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting motor brake mode to " + enable);
        var newConfig = new TalonFXConfiguration();
        newConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        PhoenixUtil.tryUntilOkAsync(5, () -> talon.getConfigurator().apply(newConfig, 0.25));
    }

    @Override
    public void setRequest(RequestType type, double value) {
        switch (type) {
            case VoltageVolts -> talon.setControl(voltageRequest.withOutput(value));
            case PositionRad -> talon.setControl(positionRequest.withPosition(Units.radiansToRotations(value)));
            case VelocityRadPerSec -> talon.setControl(velocityRequest.withVelocity(Units.radiansToRotations(value)));
        }
    }
}
