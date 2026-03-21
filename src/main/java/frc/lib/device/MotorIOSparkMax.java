package frc.lib.device;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.lib.Util;
import frc.lib.network.LoggedTunablePIDF;

import java.util.function.DoubleSupplier;

import static frc.lib.SparkUtil.*;

public class MotorIOSparkMax extends MotorIO {
    // Hardware objects
    protected final SparkMax spark;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig config;

    // Closed loop controllers
    private final SparkClosedLoopController controller;

    // Connection debouncers
    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public MotorIOSparkMax(int canID, CtrlSparkMaxConfig config, double initialPositionRad) {
        spark = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless);
        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        // Configure motor
        this.config = config.getInner();
        tryUntilOk(5, () -> spark.configure(
                this.config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        ));
        tryUntilOk(5, () -> encoder.setPosition(initialPositionRad));
    }

    @Override
    public void updateInputs(MotorIOInputsAutoLogged inputs) {
        sparkStickyFault = hasFault(spark);
        ifOk(spark, encoder::getPosition, (value) -> inputs.positionRad = value);
        ifOk(spark, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
                spark,
                new DoubleSupplier[]{spark::getAppliedOutput, spark::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]
        );
        ifOk(spark, spark::getOutputCurrent, (value) -> {
            inputs.statorCurrentAmps = value;
            inputs.supplyCurrentAmps = value;
        });
        ifOk(spark, spark::getMotorTemperature, (value) -> inputs.temperatureCelsius = value);
        inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void setVoltageRequest(double volts) {
        spark.setVoltage(volts);
    }

    @Override
    public void setPositionRequest(double setpointRad) {
        controller.setSetpoint(
                setpointRad,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0
        );
    }

    @Override
    public void setVelocityRequest(double setpointRadPerSec) {
        controller.setSetpoint(
                setpointRadPerSec,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0
        );
    }

    @Override
    public void setFollowRequest(MotorIO leaderIO, MotorAlignmentValue alignment) {
        if (leaderIO instanceof MotorIOSparkMax sparkLeaderIO) {
            config.follow(sparkLeaderIO.spark, alignment == MotorAlignmentValue.Opposed);
            tryUntilOk(5, () -> spark.configure(
                    config,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            ));
        } else {
            Util.error("Unable to follow a MotorIO of type other than MotorIOSparkMax");
        }
    }

    @Override
    public void setGains(LoggedTunablePIDF newGains) {
        var newConfig = new SparkMaxConfig();
        newGains.applySpark(newConfig.closedLoop);
        tryUntilOkAsync(5, () -> spark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        var newConfig = new SparkMaxConfig()
                .idleMode(switch (neutralMode) {
                    case Coast -> SparkBaseConfig.IdleMode.kCoast;
                    case Brake -> SparkBaseConfig.IdleMode.kBrake;
                });
        tryUntilOkAsync(5, () -> spark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setEncoderPosition(double positionRad) {
        tryUntilOkAsync(5, () -> encoder.setPosition(positionRad));
    }
}
