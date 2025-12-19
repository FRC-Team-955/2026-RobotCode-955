package frc.robot.subsystems.funnel;

import edu.wpi.first.math.util.Units;
import frc.lib.PIDF;
import frc.lib.network.LoggedTunableNumber;

import static frc.robot.subsystems.funnel.FunnelConstants.velocityGains;

public class FunnelTuning {
    public static final PIDF.Tunable velocityGainsTunable = velocityGains.tunable("Funnel/Belt/Velocity");

    public static final LoggedTunableNumber intakeGoalSetpoint =
            new LoggedTunableNumber("Funnel/Belt/Goal/Intake", Units.rotationsPerMinuteToRadiansPerSecond(900));
    public static final LoggedTunableNumber ejectGoalSetpoint =
            new LoggedTunableNumber("Funnel/Belt/Goal/Eject", Units.rotationsPerMinuteToRadiansPerSecond(1000));
}
