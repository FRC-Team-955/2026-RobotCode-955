package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import frc.lib.PIDF;

import java.util.function.DoubleSupplier;

import static frc.lib.SparkUtil.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains;
import static frc.robot.subsystems.elevator.ElevatorConstants.gearRatio;

public class ElevatorIOSparkMax extends ElevatorIO {
    private static final int currentLimitAmps = 60;

    // Hardware objects
    private final SparkMax leaderSpark;
    private final SparkMax followerSpark;
    private final RelativeEncoder leaderEncoder;
    private final RelativeEncoder followerEncoder;
    private final SparkMaxConfig leaderConfig;
    private final SparkMaxConfig followerConfig;

    // Closed loop controllers
    private final SparkClosedLoopController pid;
    private ElevatorFeedforward ff = gains.toElevatorFF();

    // Connection debouncers
    private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
    private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

    private boolean emergencyStopped = false;

    /** Used when calculating feedforward */
    private double lastVelocitySetpointRadPerSec = 0;

    public ElevatorIOSparkMax(
            int leaderCanID,
            int followerCanID,
            boolean leaderInverted,
            boolean followerInvertedRelativeToLeader
    ) {
        leaderSpark = new SparkMax(leaderCanID, SparkLowLevel.MotorType.kBrushless);
        followerSpark = new SparkMax(followerCanID, SparkLowLevel.MotorType.kBrushless);
        leaderEncoder = leaderSpark.getEncoder();
        followerEncoder = followerSpark.getEncoder();
        pid = leaderSpark.getClosedLoopController();

        // Configure motors
        leaderConfig = new SparkMaxConfig();
        followerConfig = new SparkMaxConfig();

        leaderConfig
                .inverted(leaderInverted)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(currentLimitAmps)
                .voltageCompensation(12.0);
        leaderConfig
                .encoder
                .positionConversionFactor(2 * Math.PI / gearRatio) // Rotor Rotations -> Drum Radians
                .velocityConversionFactor((2 * Math.PI) / 60.0 / gearRatio) // Rotor RPM -> Drum Rad/Sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        leaderConfig
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        gains.applySparkWithoutFeedforward(leaderConfig.closedLoop, ClosedLoopSlot.kSlot0);
        leaderConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        followerConfig.apply(leaderConfig).follow(leaderSpark, followerInvertedRelativeToLeader);

        tryUntilOk(5, () -> leaderSpark.configure(
                leaderConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOk(5, () -> followerSpark.configure(
                followerConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOk(5, () -> leaderEncoder.setPosition(0.0));
        tryUntilOk(5, () -> followerEncoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Update leader inputs
        sparkStickyFault = hasFault(leaderSpark);
        ifOk(leaderSpark, leaderEncoder::getPosition, (value) -> inputs.leaderPositionRad = value);
        ifOk(leaderSpark, leaderEncoder::getVelocity, (value) -> inputs.leaderVelocityRadPerSec = value);
        ifOk(
                leaderSpark,
                new DoubleSupplier[]{leaderSpark::getAppliedOutput, leaderSpark::getBusVoltage},
                (values) -> inputs.leaderAppliedVolts = values[0] * values[1]
        );
        ifOk(leaderSpark, leaderSpark::getOutputCurrent, (value) -> inputs.leaderCurrentAmps = value);
        ifOk(leaderSpark, leaderSpark::getMotorTemperature, (value) -> inputs.leaderTemperatureCelsius = value);
        inputs.leaderConnected = leaderConnectedDebounce.calculate(!sparkStickyFault);

        // Update follower inputs
        sparkStickyFault = hasFault(followerSpark);
        ifOk(followerSpark, followerEncoder::getPosition, (value) -> inputs.followerPositionRad = value);
        ifOk(followerSpark, followerEncoder::getVelocity, (value) -> inputs.followerVelocityRadPerSec = value);
        ifOk(
                followerSpark,
                new DoubleSupplier[]{followerSpark::getAppliedOutput, followerSpark::getBusVoltage},
                (values) -> inputs.followerAppliedVolts = values[0] * values[1]
        );
        ifOk(followerSpark, followerSpark::getOutputCurrent, (value) -> inputs.followerCurrentAmps = value);
        ifOk(followerSpark, followerSpark::getMotorTemperature, (value) -> inputs.followerTemperatureCelsius = value);
        inputs.followerConnected = followerConnectedDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void setPIDF(PIDF newGains) {
        System.out.println("Setting elevator gains");
        ff = newGains.toElevatorFF();
        var newConfig = new SparkMaxConfig();
        newGains.applySparkWithoutFeedforward(newConfig.closedLoop, ClosedLoopSlot.kSlot0);
        tryUntilOkAsync(5, () -> leaderSpark.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOkAsync(5, () -> followerSpark.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        var newConfig = new SparkMaxConfig().idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        tryUntilOkAsync(5, () -> leaderSpark.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOkAsync(5, () -> followerSpark.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setEmergencyStopped(boolean emergencyStopped) {
        this.emergencyStopped = emergencyStopped;
        if (emergencyStopped) {
            lastVelocitySetpointRadPerSec = 0;
            leaderSpark.setVoltage(0);
        }
    }

    @Override
    public void setVoltage(double output) {
        if (!emergencyStopped) {
            lastVelocitySetpointRadPerSec = 0;
            leaderSpark.setVoltage(output);
        }
    }

    @Override
    public void setMotionProfile(double positionRad, double velocityRadPerSec) {
        if (!emergencyStopped) {
            var ffVolts = ff.calculateWithVelocities(lastVelocitySetpointRadPerSec, velocityRadPerSec);
            lastVelocitySetpointRadPerSec = velocityRadPerSec;

            pid.setReference(
                    positionRad,
                    SparkBase.ControlType.kPosition,
                    ClosedLoopSlot.kSlot0,
                    ffVolts,
                    SparkClosedLoopController.ArbFFUnits.kVoltage
            );
        }
    }

    @Override
    public void setEncoder(double positionRad) {
        tryUntilOkAsync(5, () -> leaderEncoder.setPosition(positionRad));
        tryUntilOkAsync(5, () -> followerEncoder.setPosition(positionRad));
    }

    @Override
    public void setManualCurrentLimit(boolean manualCurrentLimit) {
        var newConfig = new SparkMaxConfig().smartCurrentLimit(manualCurrentLimit ? 30 : currentLimitAmps);
        tryUntilOkAsync(5, () -> leaderSpark.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
        tryUntilOkAsync(5, () -> followerSpark.configure(
                newConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        ));
    }
}
