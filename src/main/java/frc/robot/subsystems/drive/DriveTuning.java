package frc.robot.subsystems.drive;

import frc.lib.PIDF;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class DriveTuning {
    public static final PIDF.Tunable headingOverrideGainsTunable = driveConfig.headingOverrideGains().tunable("Drive/HeadingOverride");
}
