package frc.robot.subsystems.drive;

import frc.lib.Util;

public abstract class DriveGoal {
    public final String loggableName = Util.camelCaseToSnakeCase(getClass().getSimpleName().replace("Goal", ""));

    public abstract DriveRequest getRequest();
}
