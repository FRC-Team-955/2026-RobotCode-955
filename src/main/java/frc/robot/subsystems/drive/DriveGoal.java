package frc.robot.subsystems.drive;

import frc.lib.Util;
import frc.lib.swerve.ModuleLimits;
import frc.robot.OperatorDashboard;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public abstract class DriveGoal {
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    public final String loggableName = Util.camelCaseToSnakeCase(getClass().getSimpleName().replace("Goal", ""));

    public abstract DriveRequest getRequest();

    public ModuleLimits getModuleLimits() {
        return driveConfig.moduleLimits();
    }
}
