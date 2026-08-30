package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.EnergyLogger;
import frc.lib.Util;
import frc.lib.devices.motor.MechanismSim;
import frc.lib.devices.motor.Motor;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.network.LoggedTunablePIDF;
import frc.lib.subsystem.Periodic;
import frc.robot.BuildConstants;
import frc.robot.shooting.ShootingKinematics;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Flywheel implements Periodic {
    public static final double flywheelRadiusMeters = Units.inchesToMeters(2.0);

    private static final LoggedTunableNumber ejectRPM = new LoggedTunableNumber("Superstructure/Flywheel/Goal/EjectRPM", -300);

    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    private static final EnergyLogger energyLogger = EnergyLogger.get();

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(140)
                    .withSupplyCurrentLimit(30))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1));
    private final MechanismSim.Builder mechanismSimBuilder = MechanismSim.roller(0.01);
    private final Motor leaderMotor = Motor
            .createTalonFX(
                    "Superstructure/Flywheel/Leader",
                    16,
                    motorConfig,
                    0.0,
                    mechanismSimBuilder
            )
            .withVelocityGains(switch (BuildConstants.mode) {
                case REAL, REPLAY -> new LoggedTunablePIDF("Superstructure/Flywheel/Gains")
                        .withS(0.27, StaticFeedforwardSignValue.UseVelocitySign)
                        .withV(0.019)
                        .withP(0.02)
                        .withI(0.001);
                case SIM -> new LoggedTunablePIDF("Superstructure/Flywheel/Gains")
                        .withS(0.2, StaticFeedforwardSignValue.UseVelocitySign)
                        .withV(0.019)
                        .withP(0.01);
            });
    private final Motor followerMotor = Motor
            .createTalonFX(
                    "Superstructure/Flywheel/Follower",
                    19,
                    motorConfig,
                    0.0,
                    mechanismSimBuilder
            )
            .withFollowRequest(leaderMotor, MotorAlignmentValue.Opposed);

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0),
        SHOOT(() -> shootingKinematics.getShootingParameters().velocityRPM()),
        EJECT(ejectRPM::get),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointRPM;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    private static Flywheel instance;

    public static synchronized Flywheel get() {
        if (instance == null) {
            instance = new Flywheel();
        }

        return instance;
    }

    private Flywheel() {
        if (instance != null) {
            Util.error("Duplicate Flywheel created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        energyLogger.reportPowerUsage("Flywheel",
                leaderMotor.isConnected() ? leaderMotor.getAppliedVolts() * leaderMotor.getSupplyCurrentAmps() : 0.0,
                followerMotor.isConnected() ? followerMotor.getAppliedVolts() * followerMotor.getSupplyCurrentAmps() : 0.0);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Flywheel/Goal", goal);
        if (DriverStation.isDisabled() || goal == Goal.IDLE) {
            leaderMotor.setVoltageRequest(0.0);
        } else {
            double setpointRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(goal.setpointRPM.getAsDouble());
            leaderMotor.setVelocityRequest(setpointRadPerSec);
            if (BuildConstants.isSimOrReplay)
                Logger.recordOutput("Superstructure/Flywheel/SetpointRadPerSec", setpointRadPerSec);
        }
    }

    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(leaderMotor.getVelocityRadPerSec());
    }

    public boolean isDisconnected() {
        return !leaderMotor.isConnected() || !followerMotor.isConnected();
    }

    public double getSetpointRPM() {
        return goal.setpointRPM.getAsDouble();
    }

    public Transform3d transform() {
        return new Transform3d(
                new Translation3d(Units.inchesToMeters(-6.910046), Units.inchesToMeters(-9.109744), Units.inchesToMeters(12.861381)),
                new Rotation3d(0.0, 0.0, 0.0)
        ).plus(new Transform3d(
                new Translation3d(),
                new Rotation3d(0.0, leaderMotor.getPositionRad(), 0.0)
        ));
    }
}
