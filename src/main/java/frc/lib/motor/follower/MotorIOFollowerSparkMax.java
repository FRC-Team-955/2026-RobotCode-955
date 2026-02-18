package frc.lib.motor.follower;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.math.filter.Debouncer;

import java.util.function.DoubleSupplier;

import static frc.lib.SparkUtil.*;

public class MotorIOFollowerSparkMax extends MotorIOFollower {
    // Hardware objects
    private final SparkMax spark;
    private final RelativeEncoder encoder;

    // Connection debouncers
    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public MotorIOFollowerSparkMax(
            int canID,
            int leaderCanID,
            boolean inverted,
            boolean brakeMode,
            int currentLimitAmps,
            double gearRatio
    ) {
        spark = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless);
        encoder = spark.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(brakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(currentLimitAmps)
                .voltageCompensation(12.0)
                .follow(leaderCanID, inverted);
        config
                .encoder
                .positionConversionFactor(2 * Math.PI / gearRatio)
                .velocityConversionFactor((2 * Math.PI) / 60.0 / gearRatio)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
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
    public void updateInputs(MotorIOFollowerInputs inputs) {
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
    public void setBrakeMode(boolean enable) {
        System.out.println("Setting follower motor brake mode to " + enable);
        var newConfig = new SparkMaxConfig().idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        tryUntilOkAsync(5, () -> spark.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters
        ));
    }
}


