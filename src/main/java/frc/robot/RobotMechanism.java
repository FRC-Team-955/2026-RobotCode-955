package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

public class RobotMechanism implements Periodic {
    // All transforms are relative to center of robot at the bottom of the frame rail


    private static final RobotState robotState = RobotState.get();
    private static final Superintake superintake = Superintake.get();
    private static final Superstructure superstructure = Superstructure.get();

    private static RobotMechanism instance;

    public static synchronized RobotMechanism get() {
        if (instance == null) {
            instance = new RobotMechanism();
        }

        return instance;
    }

    private RobotMechanism() {
        if (instance != null) {
            Util.error("Duplicate RobotMechanism created");
        }
    }

    @Override
    public void periodicAfterCommands() {
        Pose3d robotPose = robotState.robotPoseMec();

        Transform3d intakeRollersTransform = superintake.intakeRollers.intakeRollersTransform();

        Transform3d intakePivotTransform = superintake.intakePivot.intakePivotTransform();

        Transform3d spindexerTransform = superstructure.spindexer.spindexerTransform();

        Transform3d feederTransform = superstructure.feeder.feederTransform();

        Transform3d flywheelTransform = superstructure.flywheel.flywheelTransform();

        Transform3d hoodTransform = superstructure.hood.hoodTransform();

        Logger.recordOutput("RobotMechanism/Pose", robotPose);
        Logger.recordOutput(
                "RobotMechanism/Components",
                intakeRollersTransform,
                spindexerTransform,
                flywheelTransform,
                feederTransform,
                intakePivotTransform,
                hoodTransform
        );
    }
}