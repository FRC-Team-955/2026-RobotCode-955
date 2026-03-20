package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Util;
import frc.lib.device.*;
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

    public static final double gearRatio = 1;

    public static final LoggedTunablePIDF velocityGains = switch (BuildConstants.mode) {
        case REAL, REPLAY -> new LoggedTunablePIDF("Flywheel/Gains")
                .withS(0.27, StaticFeedforwardSignValue.UseVelocitySign)
                .withV(0.02)
                .withP(0.01);
        case SIM -> new LoggedTunablePIDF("Flywheel/Gains")
                .withS(0.2, StaticFeedforwardSignValue.UseVelocitySign)
                .withV(0.019)
                .withP(0.01);
    };

    private static final LoggedTunableNumber ejectRPM = new LoggedTunableNumber("Flywheel/Goal/EjectRPM", -300);
    private static final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    private static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(30)
            )
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(gearRatio)
            );

    private final Motor leader = new Motor("FlywheelLeader", velocityGains, switch (BuildConstants.mode) {
        case REAL -> new MotorIOTalonFX(16, motorConfig, 0.0);
        case SIM -> new MotorIOTalonFXSim(motorConfig, 0.0, MechanismSim.roller(gearRatio, 0.001));
        case REPLAY -> new MotorIOReplay();
    });

    private final Motor follower = new Motor("FlywheelFollower", null, switch (BuildConstants.mode) {
        case REAL -> new MotorIOTalonFX(19, motorConfig, 0.0);
        case SIM -> new MotorIOTalonFXSim(motorConfig, 0.0, MechanismSim.roller(gearRatio, 0.001));
        case REPLAY -> new MotorIOReplay();
    });

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

        follower.setFollowRequest(leader, MotorAlignmentValue.Opposed);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Flywheel/Goal", goal);
        if (DriverStation.isDisabled() || goal == Goal.IDLE) {
            leader.setVoltageRequest(0);
        } else {
            double value = Units.rotationsPerMinuteToRadiansPerSecond(goal.setpointRPM.getAsDouble());
            leader.setVelocityRequest(value);
        }
    }

    public double getPositionRad() {
        return leader.getPositionRad();
    }

    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(leader.getVelocityRadPerSec());
    }
}

