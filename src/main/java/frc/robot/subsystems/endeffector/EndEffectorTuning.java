package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.util.Units;
import frc.lib.PIDF;
import frc.lib.network.LoggedTunableNumber;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.positionGains;
import static frc.robot.subsystems.endeffector.EndEffectorConstants.velocityGains;


public class EndEffectorTuning {
    public static final PIDF.Tunable positionGainsTunable = positionGains.tunable("EndEffector/Position");
    public static final PIDF.Tunable velocityGainsTunable = velocityGains.tunable("EndEffector/Velocity");

    public static final LoggedTunableNumber funnelIntakeGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/FunnelIntake", Units.rotationsPerMinuteToRadiansPerSecond(500));
    public static final LoggedTunableNumber scoreCoralGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/ScoreCoral", Units.rotationsPerMinuteToRadiansPerSecond(300));
    public static final LoggedTunableNumber scoreCoralL1GoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/ScoreCoralL1", Units.rotationsPerMinuteToRadiansPerSecond(600));
    public static final LoggedTunableNumber descoreAlgaeGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/DescoreAlgae", Units.rotationsPerMinuteToRadiansPerSecond(-600));
    public static final LoggedTunableNumber ejectGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/Eject", Units.rotationsPerMinuteToRadiansPerSecond(450));
    public static final LoggedTunableNumber zeroCoralGoalSetpoint =
            new LoggedTunableNumber("EndEffector/Goal/ZeroCoral", Units.rotationsPerMinuteToRadiansPerSecond(-600));

    public static final LoggedTunableNumber homeInitialMeters =
            new LoggedTunableNumber("EndEffector/HomeInitialMeters", Units.inchesToMeters(1.5));
    public static final LoggedTunableNumber homeFinalMeters =
            new LoggedTunableNumber("EndEffector/HomeFinalMeters", Units.inchesToMeters(2.5));
}