package frc.robot.subsystems.drive;

import frc.lib.PIDF;
import frc.lib.network.LoggedTunableNumber;

import static frc.robot.subsystems.drive.DriveConstants.moduleConfig;
import static frc.robot.subsystems.drive.DriveConstants.moveToConfig;

public class DriveTuning {
    public static final LoggedTunableNumber wheelRadiusCharacterizationSpeedRadPerSec = new LoggedTunableNumber("Drive/WheelRadiusCharacterizationSpeedRadPerSecond", 1.0);

    public static final PIDF.Tunable moduleDriveGainsTunable = moduleConfig.driveGains().tunable("Drive/ModuleDrive");
    public static final PIDF.Tunable moduleTurnGainsTunable = moduleConfig.turnGains().tunable("Drive/ModuleTurn");

    public static final PIDF.Tunable moveToLinearTunable = moveToConfig.linear().tunable("Drive/MoveToLinear");
    public static final PIDF.Tunable moveToAngularTunable = moveToConfig.angular().tunable("Drive/MoveToAngular");
}
