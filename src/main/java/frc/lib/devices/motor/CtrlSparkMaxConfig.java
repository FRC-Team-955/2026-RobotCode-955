package frc.lib.devices.motor;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import lombok.Getter;

/**
 * Spark MAX's have bad defaults and a messy config, so we use this class to clean up setting things like gear ratio and to provide sensible defaults.
 */
public class CtrlSparkMaxConfig {
    private final SparkMaxConfig config;

    public SparkMaxConfig getInner() {
        return config;
    }

    @Getter
    private double gearRatio = 1;

    private double calculatePositionConversionFactor() {
        // Rotor Rotations -> Wheel Radians
        return 2 * Math.PI / gearRatio;
    }

    private double calculateVelocityConversionFactor() {
        // Rotor RPM -> Wheel Rad/Sec
        return (2 * Math.PI) / 60.0 / gearRatio;
    }

    public CtrlSparkMaxConfig() {
        config = new SparkMaxConfig();
        config
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        config
                .encoder
                .positionConversionFactor(calculatePositionConversionFactor())
                .velocityConversionFactor(calculateVelocityConversionFactor())
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        config
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
    }

    public CtrlSparkMaxConfig withInverted(boolean inverted) {
        config.inverted(inverted);
        return this;
    }

    public CtrlSparkMaxConfig withNeutralMode(NeutralModeValue neutralMode) {
        config.idleMode(switch (neutralMode) {
            case Coast -> SparkBaseConfig.IdleMode.kCoast;
            case Brake -> SparkBaseConfig.IdleMode.kBrake;
        });
        return this;
    }

    public CtrlSparkMaxConfig withCurrentLimit(int currentLimit) {
        config.smartCurrentLimit(currentLimit);
        return this;
    }

    public CtrlSparkMaxConfig withGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        config.encoder.positionConversionFactor(calculatePositionConversionFactor());
        config.encoder.velocityConversionFactor(calculateVelocityConversionFactor());
        return this;
    }
}
