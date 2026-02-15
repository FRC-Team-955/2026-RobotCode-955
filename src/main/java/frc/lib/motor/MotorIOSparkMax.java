package frc.lib.motor;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import frc.lib.PIDF;

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

    private SimpleMotorFeedforward velocityFeedforward;

    public MotorIOSparkMax(
            int canID,
            boolean inverted,
            boolean brakeMode,
            int currentLimitAmps,
            double gearRatio,
            PIDF positionGains,
            PIDF velocityGains
    ) {
        spark = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless);
        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        velocityFeedforward = velocityGains.toSimpleFF();

        // Configure drive motor
        config = new SparkMaxConfig();
        config
                .inverted(inverted)
                .idleMode(brakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
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
        positionGains.applySparkWithoutFeedforward(config.closedLoop, ClosedLoopSlot.kSlot0); // position = slot0
        velocityGains.applySparkWithoutFeedforward(config.closedLoop, ClosedLoopSlot.kSlot1); // velocity = slot1
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
    public void setPositionPIDF(PIDF newGains) {
        System.out.println("Setting motor position gains");
        var newConfig = new SparkMaxConfig();
        newGains.applySparkWithoutFeedforward(newConfig.closedLoop, ClosedLoopSlot.kSlot0);
        tryUntilOkAsync(5, () -> spark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setVelocityPIDF(PIDF newGains) {
        System.out.println("Setting motor velocity gains");
        velocityFeedforward = newGains.toSimpleFF();
        var newConfig = new SparkMaxConfig();
        newGains.applySparkWithoutFeedforward(newConfig.closedLoop, ClosedLoopSlot.kSlot0);
        tryUntilOkAsync(5, () -> spark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting motor brake mode to " + enable);
        var newConfig = new SparkMaxConfig().idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
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
            case VelocityRadPerSec -> {
                var ffVolts = velocityFeedforward.calculate(value);
                controller.setSetpoint(
                        value,
                        SparkBase.ControlType.kVelocity,
                        ClosedLoopSlot.kSlot1,  // velocity = slot1
                        ffVolts,
                        SparkClosedLoopController.ArbFFUnits.kVoltage
                );
            }
        }
    }
}
