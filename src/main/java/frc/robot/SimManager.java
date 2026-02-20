package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.Util;
import frc.robot.subsystems.drive.DriveConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import static edu.wpi.first.units.Units.*;
import static frc.robot.FieldConstants.aprilTagLayout;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;
import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.fuelDiameterMeters;

public class SimManager {
    private static final int hopperCapacity = 50;

    private static final RobotState robotState = RobotState.get();

    public final SwerveDriveSimulation driveSimulation = new SwerveDriveSimulation(
            // Specify Configuration
            DriveTrainSimulationConfig.Default()
                    // Specify gyro type (for realistic gyro drifting and error simulation)
                    .withGyro(COTS.ofPigeon2())
                    // Specify swerve module (for realistic swerve dynamics)
                    .withSwerveModule(COTS.ofMark4i(
                            DCMotor.getKrakenX60(1),
                            DCMotor.getNEO(1),
                            COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                            2 // L2 Gear ratio
                    ))
                    // Configures the track length and track width (spacing between swerve modules)
                    .withTrackLengthTrackWidth(Meters.of(driveConfig.trackLengthMeters()), Meters.of(driveConfig.trackWidthMeters()))
                    // Configures the bumper size (dimensions of the robot bumper)
                    .withBumperSize(Meters.of(driveConfig.bumperLengthMeters()), Meters.of(driveConfig.bumperWidthMeters()))
                    .withRobotMass(Pounds.of(125)),
            // Specify starting pose
            new Pose2d(1, 1, new Rotation2d())
    );

    public final IntakeSimulation intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSimulation,
            // Width of the intake
            Meters.of(DriveConstants.driveConfig.trackWidthMeters()),
            // The extension length of the intake beyond the robot's bumper (when activated)
            Inches.of(5.625),
            IntakeSimulation.IntakeSide.FRONT,
            hopperCapacity
    );

    public final VisionSystemSim visionSystem = new VisionSystemSim("photonvision");

    private static SimManager instance;

    public static synchronized SimManager get() {
        if (instance == null) {
            instance = new SimManager();
        }

        return instance;
    }

    private SimManager() {
        if (instance != null) {
            Util.error("Duplicate SimManager created");
        }

        if (BuildConstants.mode != BuildConstants.Mode.SIM) {
            Util.error("SimManager created when not in sim");
        } else {
            visionSystem.addAprilTags(aprilTagLayout);
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

            SimulatedArena.getInstance().resetFieldForAuto();
            RobotModeTriggers.autonomous().onTrue(Commands.runOnce(SimulatedArena.getInstance()::resetFieldForAuto));
            robotState.setPose(driveSimulation.getSimulatedDriveTrainPose());
        }
    }

    private boolean lastInNeutralZone = false;

    public void periodic() {
        SimulatedArena.getInstance().simulationPeriodic();

        // Bump sim
        Pose2d pose = driveSimulation.getSimulatedDriveTrainPose();
        boolean inNeutralZone =
                pose.getX() > FieldConstants.LinesVertical.hubCenter &&
                        pose.getX() < FieldConstants.LinesVertical.oppHubCenter;
        if (
            // If we exited the neutral zone
                inNeutralZone != lastInNeutralZone &&
                        // If we are not going through the trench
                        pose.getY() > FieldConstants.LinesHorizontal.rightTrenchOpenStart &&
                        pose.getY() < FieldConstants.LinesHorizontal.leftTrenchOpenEnd
        ) {
            // We went over the bump
            driveSimulation.setSimulationWorldPose(new Pose2d(
                    pose.getX() + Math.random() * Math.signum(
                            pose.getX() < FieldConstants.LinesVertical.center
                                    ? pose.getX() - FieldConstants.LinesVertical.hubCenter
                                    : pose.getX() - FieldConstants.LinesVertical.oppHubCenter
                    ),
                    pose.getY() + 2.0 * Math.random() - 1.0,
                    pose.getRotation()
            ));
        }
        lastInNeutralZone = inNeutralZone;

        // Vision sim
        Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
        TargetModel targetModel = new TargetModel(fuelDiameterMeters);
        // Only remove old fuel - don't remove AprilTags
        visionSystem.removeVisionTargets("targets");
        for (Pose3d fuelPose : fuelPoses) {
            VisionTargetSim visionTarget = new VisionTargetSim(fuelPose, targetModel);
            visionSystem.addVisionTargets(visionTarget);
        }
        visionSystem.update(driveSimulation.getSimulatedDriveTrainPose());

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")
        );
    }
}
