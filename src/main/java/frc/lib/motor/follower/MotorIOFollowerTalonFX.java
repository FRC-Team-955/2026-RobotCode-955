package frc.lib.motor.follower;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.lib.PhoenixUtil;

public class MotorIOFollowerTalonFX extends MotorIOFollower {
    // Hardware objects
    private final TalonFX talon;
    private final TalonFXConfiguration config;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> temperatureCelsius;

    // Connection debouncers
    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public MotorIOFollowerTalonFX(
            int canID,
            CANBus canBus,
            int leaderCanID,
            boolean opposeMasterDirection,
            boolean brakeMode,
            int currentLimitAmps,
            double gearRatio
    ) {
        talon = new TalonFX(canID, canBus);

        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = gearRatio;

        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> talon.setPosition(0.0, 0.25));

        Follower followerRequest = new Follower(leaderCanID,
                opposeMasterDirection ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned);
        talon.setControl(followerRequest);

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

    public MotorIOFollowerTalonFX(
            int canID,
            int leaderCanID,
            boolean opposeMasterDirection,
            boolean brakeMode,
            int currentLimitAmps,
            double gearRatio
    ) {
        this(canID, new CANBus(""), leaderCanID, opposeMasterDirection, brakeMode, currentLimitAmps, gearRatio);
    }

    @Override
    public void updateInputs(MotorIOFollowerInputs inputs) {
        var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps, temperatureCelsius);
        inputs.connected = connectedDebounce.calculate(status.isOK());
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting follower motor brake mode to " + enable);
        var newConfig = new TalonFXConfiguration();
        newConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        PhoenixUtil.tryUntilOkAsync(5, () -> talon.getConfigurator().apply(newConfig, 0.25));
    }
}



