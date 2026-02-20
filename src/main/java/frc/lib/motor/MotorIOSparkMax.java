package frc.lib.motor;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.lib.network.LoggedTunablePIDF;

import java.util.function.DoubleSupplier;

import static frc.lib.SparkUtil.*;

public class MotorIOSparkMax extends MotorIO {
    // Hardware objects
    private final SparkMax spark;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig config;

    // Closed loop controllers
    private final SparkClosedLoopController controller;

    // Connection debouncers
    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public MotorIOSparkMax(
            int canID,
            boolean inverted,
            SparkBaseConfig.IdleMode idleMode,
            int currentLimitAmps,
            double gearRatio,
            LoggedTunablePIDF positionGains,
            LoggedTunablePIDF velocityGains
    ) {
        spark = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless);
        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        // Configure drive motor
        config = new SparkMaxConfig();
        config
                .inverted(inverted)
                .idleMode(idleMode)
                .smartCurrentLimit(currentLimitAmps)
                .voltageCompensation(12.0);
        config
                .encoder
                .positionConversionFactor(2 * Math.PI / gearRatio) // Rotor Rotations -> Wheel Radians
                .velocityConversionFactor((2 * Math.PI) / 60.0 / gearRatio) // Rotor RPM -> Wheel Rad/Sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        config
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        if (positionGains != null)
            positionGains.applySpark(config.closedLoop, ClosedLoopSlot.kSlot0); // position = slot0
        if (velocityGains != null)
            velocityGains.applySpark(config.closedLoop, ClosedLoopSlot.kSlot1); // velocity = slot1
        config
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(5, () -> spark.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        ));
        tryUntilOk(5, () -> encoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        sparkStickyFault = hasFault(spark);
        ifOk(spark, encoder::getPosition, (value) -> inputs.positionRad = value);
        ifOk(spark, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
                spark,
                new DoubleSupplier[]{spark::getAppliedOutput, spark::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]
        );
        ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
        ifOk(spark, spark::getMotorTemperature, (value) -> inputs.temperatureCelsius = value);
        inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void setPositionPIDF(LoggedTunablePIDF newGains) {
        System.out.println("Setting motor position gains");
        var newConfig = new SparkMaxConfig();
        newGains.applySpark(newConfig.closedLoop, ClosedLoopSlot.kSlot0); // position = slot0
        tryUntilOkAsync(5, () -> spark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setVelocityPIDF(LoggedTunablePIDF newGains) {
        System.out.println("Setting motor velocity gains");
        var newConfig = new SparkMaxConfig();
        newGains.applySpark(newConfig.closedLoop, ClosedLoopSlot.kSlot1); // velocity = slot1
        tryUntilOkAsync(5, () -> spark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting motor brake mode to " + enable);
        var newConfig = new SparkMaxConfig()
                .idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        tryUntilOkAsync(5, () -> spark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    // This is not included in MotorIO because it should not be used by subsystem code directly
    // If you need to use this, make a custom IO layer that either subclasses or nests this IO layer
    // and take an enum as the argument instead of raw current limit
    // We do this because wanted current limit varies based on which motor we are using
    public void setCurrentLimit(int currentLimitAmps) {
        System.out.println("Setting motor current limit to " + currentLimitAmps);
        var newConfig = new SparkMaxConfig()
                .smartCurrentLimit(currentLimitAmps);
        tryUntilOkAsync(5, () -> spark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setRequest(RequestType type, double value) {
        switch (type) {
            case VoltageVolts -> spark.setVoltage(value);
            case PositionRad -> controller.setSetpoint(
                    value,
                    SparkBase.ControlType.kPosition,
                    ClosedLoopSlot.kSlot0 // position = slot0
            );
            case VelocityRadPerSec -> controller.setSetpoint(
                    value,
                    SparkBase.ControlType.kVelocity,
                    ClosedLoopSlot.kSlot1  // velocity = slot1
            );
        }
    }

    /**
     * NOTE: BLOCKS THE MAIN THREAD!!! ONLY CALL ON STARTUP!!!!
     */
    public void setFollow(MotorIOSparkMax leader, MotorAlignmentValue alignment) {
        config.follow(leader.spark, alignment == MotorAlignmentValue.Opposed);
        tryUntilOk(5, () -> spark.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }
}
