package frc.robot.subsystems.drive;

import frc.lib.Util;
import frc.lib.swerve.ModuleLimits;
import frc.robot.OperatorDashboard;
import frc.robot.subsystems.elevator.Elevator;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public abstract class DriveGoal {
    private static final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private static final Elevator elevator = Elevator.get();

    public final String loggableName = Util.camelCaseToSnakeCase(getClass().getSimpleName().replace("Goal", ""));

    public abstract DriveRequest getRequest();

    public ModuleLimits getModuleLimits() {
        if (operatorDashboard.coralStuckInRobotMode.get()) {
            return driveConfig.moduleLimits();
        }

        return driveConfig.moduleLimits().times(elevator.getDriveConstraintScalar());
    }
}
