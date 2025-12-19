package frc.robot.subsystems.elevator;

import frc.lib.PIDF;
import frc.lib.network.LoggedTunableNumber;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class ElevatorTuning {
    public static final PIDF.Tunable gainsTunable = gains.tunable("Elevator/Gains");

    public static final LoggedTunableNumber maxVelocityMetersPerSecondTunable =
            new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSecond", maxVelocityMetersPerSecond);
    public static final LoggedTunableNumber maxAccelerationMetersPerSecondSquaredTunable =
            new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSecondSquared", maxAccelerationMetersPerSecondSquared);

    public static final LoggedTunableNumber stowGoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/Stow", 0.02);
    public static final LoggedTunableNumber scoreL1GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/ScoreL1", 0.33);
    public static final LoggedTunableNumber scoreL2GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/ScoreL2", 0.69);
    public static final LoggedTunableNumber scoreL3GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/ScoreL3", 1.10);
    public static final LoggedTunableNumber scoreL4GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/ScoreL4", maxHeightMeters);
    public static final LoggedTunableNumber descoreL2GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/DescoreL2", 0.48);
    public static final LoggedTunableNumber descoreL3GoalSetpoint =
            new LoggedTunableNumber("Elevator/Goal/DescoreL3", 0.88);
}