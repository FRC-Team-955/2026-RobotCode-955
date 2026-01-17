package frc.robot.subsystems.drive;

import frc.lib.Util;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.elevator.Elevator;

public abstract class DriveGoal {
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final Elevator elevator = Elevator.get();

    public final String loggableName = Util.camelCaseToSnakeCase(getClass().getSimpleName().replace("Goal", ""));

    public abstract DriveRequest getRequest();
}
