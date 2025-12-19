package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.lib.PIDF;
import frc.robot.Constants;

import static frc.lib.PhoenixUtil.tryUntilOk;
import static frc.lib.PhoenixUtil.tryUntilOkAsync;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains;
import static frc.robot.subsystems.elevator.ElevatorConstants.gearRatio;

public class ElevatorIOTalonFX extends ElevatorIO {
    private static final double currentLimitAmps = 120;

    // Hardware objects
    private final TalonFX leaderTalon;
    private final TalonFX followerTalon;
    private final TalonFXConfiguration leaderConfig;
    private final TalonFXConfiguration followerConfig;

    // Control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

    // Status signals
    private final StatusSignal<Angle> leaderPosition;
    private final StatusSignal<AngularVelocity> leaderVelocity;
    private final StatusSignal<Voltage> leaderAppliedVolts;
    private final StatusSignal<Current> leaderCurrentAmps;
    private final StatusSignal<Temperature> leaderTemperatureCelsius;

    private final StatusSignal<Angle> followerPosition;
    private final StatusSignal<AngularVelocity> followerVelocity;
    private final StatusSignal<Voltage> followerAppliedVolts;
    private final StatusSignal<Current> followerCurrentAmps;
    private final StatusSignal<Temperature> followerTemperatureCelsius;

    // Closed loop controllers
    private ElevatorFeedforward ff = gains.toElevatorFF();

    // Connection debouncers
    private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
    private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

    private boolean emergencyStopped = false;

    /** Used when calculating feedforward */
    private double lastVelocitySetpointRadPerSec = 0;

    public ElevatorIOTalonFX(
            int leaderCanID,
            int followerCanID,
            boolean leaderInverted,
            boolean followerInvertedRelativeToLeader
    ) {
        leaderTalon = new TalonFX(leaderCanID, Constants.CANivore.busName);
        followerTalon = new TalonFX(followerCanID, Constants.CANivore.busName);

        // Configure leader motor
        leaderConfig = new TalonFXConfiguration();
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfig.Slot0 = Slot0Configs.from(gains.toPhoenixWithoutFeedforward());
        leaderConfig.Feedback.SensorToMechanismRatio = gearRatio;
        leaderConfig.TorqueCurrent.PeakForwardTorqueCurrent = currentLimitAmps;
        leaderConfig.TorqueCurrent.PeakReverseTorqueCurrent = -currentLimitAmps;
        leaderConfig.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leaderConfig.MotorOutput.Inverted =
                leaderInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> leaderTalon.getConfigurator().apply(leaderConfig, 0.25));
        tryUntilOk(5, () -> leaderTalon.setPosition(0.0, 0.25));

        // Configure follower motor
        // Follower shouldn't have closed loop or any current limits
        // It should only have neutral mode, gear ratio, and inversion
        followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.Feedback.SensorToMechanismRatio = gearRatio;
        // Inversion not included for now since we don't need it and it gets complicated with following and "relative to leader"
        tryUntilOk(5, () -> followerTalon.getConfigurator().apply(followerConfig, 0.25));
        tryUntilOk(5, () -> followerTalon.setPosition(0.0, 0.25));

        followerTalon.setControl(new Follower(leaderTalon.getDeviceID(), followerInvertedRelativeToLeader));

        // Status signals
        leaderPosition = leaderTalon.getPosition();
        leaderVelocity = leaderTalon.getVelocity();
        leaderAppliedVolts = leaderTalon.getMotorVoltage();
        leaderCurrentAmps = leaderTalon.getStatorCurrent();
        leaderTemperatureCelsius = leaderTalon.getDeviceTemp();

        followerPosition = followerTalon.getPosition();
        followerVelocity = followerTalon.getVelocity();
        followerAppliedVolts = followerTalon.getMotorVoltage();
        followerCurrentAmps = followerTalon.getStatorCurrent();
        followerTemperatureCelsius = followerTalon.getDeviceTemp();

        // Configure status signals
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                leaderPosition,
                leaderVelocity,
                leaderAppliedVolts,
                leaderCurrentAmps,
                leaderTemperatureCelsius,
                followerPosition,
                followerVelocity,
                followerAppliedVolts,
                followerCurrentAmps,
                followerTemperatureCelsius
        );
        ParentDevice.optimizeBusUtilizationForAll(leaderTalon, followerTalon);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Refresh all signals
        var leaderStatus = BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrentAmps, leaderTemperatureCelsius);
        var followerStatus = BaseStatusSignal.refreshAll(followerPosition, followerVelocity, followerAppliedVolts, followerCurrentAmps, followerTemperatureCelsius);

        // Update leader inputs
        inputs.leaderConnected = leaderConnectedDebounce.calculate(leaderStatus.isOK());
        inputs.leaderPositionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble());
        inputs.leaderVelocityRadPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
        inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
        inputs.leaderCurrentAmps = leaderCurrentAmps.getValueAsDouble();
        inputs.leaderTemperatureCelsius = leaderTemperatureCelsius.getValueAsDouble();

        // Update follower inputs
        inputs.followerConnected = followerConnectedDebounce.calculate(followerStatus.isOK());
        inputs.followerPositionRad = Units.rotationsToRadians(followerPosition.getValueAsDouble());
        inputs.followerVelocityRadPerSec = Units.rotationsToRadians(followerVelocity.getValueAsDouble());
        inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
        inputs.followerCurrentAmps = followerCurrentAmps.getValueAsDouble();
        inputs.followerTemperatureCelsius = followerTemperatureCelsius.getValueAsDouble();
    }

    @Override
    public void setPIDF(PIDF newGains) {
        System.out.println("Setting elevator gains");
        ff = newGains.toElevatorFF();
        leaderConfig.Slot0 = Slot0Configs.from(newGains.toPhoenixWithoutFeedforward());
        tryUntilOkAsync(5, () -> leaderTalon.getConfigurator().apply(leaderConfig, 0.25));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        leaderConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        followerConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        tryUntilOkAsync(5, () -> leaderTalon.getConfigurator().apply(leaderConfig, 0.25));
        tryUntilOkAsync(5, () -> followerTalon.getConfigurator().apply(followerConfig, 0.25));
    }

    @Override
    public void setEmergencyStopped(boolean emergencyStopped) {
        this.emergencyStopped = emergencyStopped;
        if (emergencyStopped) {
            lastVelocitySetpointRadPerSec = 0;
            leaderTalon.setControl(voltageRequest.withOutput(0));
        }
    }

    @Override
    public void setVoltage(double output) {
        if (!emergencyStopped) {
            lastVelocitySetpointRadPerSec = 0;
            leaderTalon.setControl(voltageRequest.withOutput(output));
        }
    }

    @Override
    public void setMotionProfile(double positionRad, double velocityRadPerSec) {
        if (!emergencyStopped) {
            var ffVolts = ff.calculateWithVelocities(lastVelocitySetpointRadPerSec, velocityRadPerSec);
            lastVelocitySetpointRadPerSec = velocityRadPerSec;

            double positionRot = Units.radiansToRotations(positionRad);
            leaderTalon.setControl(
                    positionVoltageRequest
                            .withPosition(positionRot)
                            .withFeedForward(ffVolts)
            );
        }
    }

    @Override
    public void setEncoder(double positionRad) {
        double positionRot = Units.radiansToRotations(positionRad);
        tryUntilOkAsync(5, () -> leaderTalon.setPosition(positionRot, 0.25));
        tryUntilOkAsync(5, () -> followerTalon.setPosition(positionRot, 0.25));
    }

    @Override
    public void setManualCurrentLimit(boolean manualCurrentLimit) {
        double newCurrentLimit = manualCurrentLimit ? 60 : currentLimitAmps;
        leaderConfig.TorqueCurrent.PeakForwardTorqueCurrent = newCurrentLimit;
        leaderConfig.TorqueCurrent.PeakReverseTorqueCurrent = -newCurrentLimit;
        leaderConfig.CurrentLimits.StatorCurrentLimit = newCurrentLimit;
        tryUntilOkAsync(5, () -> leaderTalon.getConfigurator().apply(leaderConfig, 0.25));
    }
}
