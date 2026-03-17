package frc.lib.device;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.lib.Util;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.Constants;

import static frc.lib.PhoenixUtil.tryUntilOk;
import static frc.lib.PhoenixUtil.tryUntilOkAsync;

public class MotorIOTalonFX extends MotorIO {
    // Hardware objects
    protected final TalonFX talon;
    private final TalonFXConfiguration config;

    // Request objects
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorCurrentAmps;
    private final StatusSignal<Current> supplyCurrentAmps;
    private final StatusSignal<Temperature> temperatureCelsius;

    // Connection debouncers
    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public MotorIOTalonFX(int canID, TalonFXConfiguration config, double initialPositionRad) {
        talon = new TalonFX(canID, Constants.canivoreBus);

        this.config = config;
        tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
        tryUntilOk(5, () -> talon.setPosition(Units.radiansToRotations(initialPositionRad), 0.25));

        position = talon.getPosition();
        velocity = talon.getVelocity();
        appliedVolts = talon.getMotorVoltage();
        statorCurrentAmps = talon.getStatorCurrent();
        supplyCurrentAmps = talon.getSupplyCurrent();
        temperatureCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVolts,
                statorCurrentAmps,
                supplyCurrentAmps,
                temperatureCelsius
        );
        ParentDevice.optimizeBusUtilizationForAll(talon);
    }

    @Override
    public void updateInputs(MotorIOInputsAutoLogged inputs) {
        var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, statorCurrentAmps, supplyCurrentAmps, temperatureCelsius);
        inputs.connected = connectedDebounce.calculate(status.isOK());
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
    }

    @Override
    public void setVoltageRequest(double volts) {
        talon.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setPositionRequest(double setpointRad) {
        talon.setControl(positionRequest.withPosition(Units.radiansToRotations(setpointRad)));
    }

    @Override
    public void setVelocityRequest(double setpointRadPerSec) {
        talon.setControl(velocityRequest.withVelocity(Units.radiansToRotations(setpointRadPerSec)));
    }

    @Override
    public void setFollowRequest(MotorIO leaderIO, MotorAlignmentValue alignment) {
        if (leaderIO instanceof MotorIOTalonFX talonLeaderIO) {
            talon.setControl(new Follower(talonLeaderIO.talon.getDeviceID(), alignment));
        } else {
            Util.error("Unable to follow a MotorIO of type other than MotorIOTalonFX");
        }
    }

    @Override
    public void setGains(LoggedTunablePIDF newGains) {
        config.Slot0 = newGains.toPhoenix();
        tryUntilOkAsync(5, () -> talon.getConfigurator().apply(config, 0.25));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        config.MotorOutput.NeutralMode = neutralMode;
        tryUntilOkAsync(5, () -> talon.getConfigurator().apply(config, 0.25));
    }

    @Override
    public void setEncoderPosition(double positionRad) {
        tryUntilOkAsync(5, () -> talon.setPosition(Units.radiansToRotations(positionRad), 0.25));
    }
}
