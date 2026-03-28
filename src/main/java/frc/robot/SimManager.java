package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.Util;
import frc.robot.subsystems.drive.DriveConstants;
import org.dyn4j.dynamics.Settings;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import static edu.wpi.first.units.Units.*;
import static frc.robot.FieldConstants.aprilTagLayout;
import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class SimManager {
    private static final int hopperCapacity = 50;

    private static final RobotState robotState = RobotState.get();

    private static SwerveModuleSimulationConfig createConfig(double turnGearRatio) {
        return new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(1),
                DCMotor.getNEO(1),
                (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0),
                turnGearRatio,
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof
        );
    }

    @SuppressWarnings("unchecked")
    public final SwerveDriveSimulation driveSimulation = new SwerveDriveSimulation(
            // Specify Configuration
            DriveTrainSimulationConfig.Default()
                    // Specify gyro type (for realistic gyro drifting and error simulation)
                    .withGyro(COTS.ofPigeon2())
                    // Specify swerve module (for realistic swerve dynamics)
                    .withSwerveModules(
                            createConfig(150.0 / 7.0),
                            createConfig(150.0 / 7.0),
                            createConfig(18.75),
                            createConfig(18.75)
                    )
                    // Configures the track length and track width (spacing between swerve modules)
                    .withTrackLengthTrackWidth(Meters.of(driveConfig.trackLengthMeters()), Meters.of(driveConfig.trackWidthMeters()))
                    // Configures the bumper size (dimensions of the robot bumper)
                    .withBumperSize(Meters.of(driveConfig.bumperLengthMeters()), Meters.of(driveConfig.bumperWidthMeters()))
                    .withRobotMass(Pounds.of(125)),
            // Specify starting pose
            // Real starting pose is specified in periodic
            new Pose2d()
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

    public final VisionSystemSim aprilTagVisionSystem = new VisionSystemSim("apriltag");
    public final VisionSystemSim gamePieceVisionSystem = new VisionSystemSim("gamepiece");

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

        if (!BuildConstants.isSim) {
            Util.error("SimManager created when not in sim");
        } else {
            aprilTagVisionSystem.addAprilTags(aprilTagLayout);

            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

            SimulatedArena.getInstance().resetFieldForAuto();
            RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
                // maple-sim currently has a bug where the two outposts are swapped.
                // So when the alliance is set to blue, only the red outpost will have
                // fuel in it. Therefore, we have to swap the alliance when resetting the field
                DriverStation.Alliance initial = DriverStation.getAlliance().orElseThrow();
                DriverStationSim.setAllianceStationId(initial == DriverStation.Alliance.Blue ? AllianceStationID.Red1 : AllianceStationID.Blue1);
                DriverStationSim.notifyNewData();
                SimulatedArena.getInstance().resetFieldForAuto();
                DriverStationSim.setAllianceStationId(initial == DriverStation.Alliance.Blue ? AllianceStationID.Blue1 : AllianceStationID.Red1);
                DriverStationSim.notifyNewData();

                intakeSimulation.setGamePiecesCount(8);
            }));

            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
        }
    }

    private boolean setInitialPose = false;
    private boolean lastInNeutralZone = false;

    public void periodicBeforeNormalCode() {
        if (!setInitialPose) {
            robotState.setPose(new Pose2d(1, 1, new Rotation2d()));
            setInitialPose = true;
        }

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

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")
        );

        aprilTagVisionSystemUpdated = false;
        gamePieceVisionSystemUpdated = false;
    }

    private boolean aprilTagVisionSystemUpdated = false;

    public void ensureAprilTagVisionSystemUpdated() {
        // Avoid updating the vision system more than once per cycle (multiple cameras
        // would cause multiple updates if we don't check this)
        if (!aprilTagVisionSystemUpdated) {
            aprilTagVisionSystem.update(driveSimulation.getSimulatedDriveTrainPose());
            aprilTagVisionSystemUpdated = true;
        }
    }

    private boolean gamePieceVisionSystemUpdated = false;

    public void ensureGamePieceVisionSystemUpdated() {
        // Avoid updating the vision system more than once per cycle (multiple cameras
        // would cause multiple updates if we don't check this)
        if (!gamePieceVisionSystemUpdated) {
            Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
            TargetModel targetModel = new TargetModel(FieldConstants.fuelDiameter);
            gamePieceVisionSystem.clearVisionTargets();
            for (Pose3d fuelPose : fuelPoses) {
                VisionTargetSim visionTarget = new VisionTargetSim(fuelPose, targetModel);
                gamePieceVisionSystem.addVisionTargets(visionTarget);
            }

            gamePieceVisionSystem.update(driveSimulation.getSimulatedDriveTrainPose());
            gamePieceVisionSystemUpdated = true;
        }
    }

    public static class CustomArena extends SimulatedArena {
        protected boolean isInEfficiencyMode = false;

        protected static Translation2d centerPieceBottomRightCorner = new Translation2d(7.35737, 1.724406);
        protected static Translation2d redDepotBottomRightCorner = new Translation2d(0.02, 5.53);
        protected static Translation2d blueDepotBottomRightCorner = new Translation2d(16.0274, 1.646936);

        /** the obstacles on the 2026 competition field */
        public static final class RebuiltFieldObstaclesMap extends FieldMap {
            private static final double FIELD_WIDTH = 16.54;

            public RebuiltFieldObstaclesMap(boolean AddRampCollider) {
                super.addBorderLine(new Translation2d(0, 0), new Translation2d(0, 8.052));

                // red wall
                super.addBorderLine(new Translation2d(16.540988, 0), new Translation2d(16.540988, 8.052));

                // upper walls
                super.addBorderLine(new Translation2d(16.540988, 8.052), new Translation2d(0, 8.052));

                // lower walls
                super.addBorderLine(new Translation2d(0, 0), new Translation2d(16.540988, 0));

                // Trench Walls (47 inch height, 12 inch width)
                double trenchWallDistX =
                        Inches.of(120.0).in(Meters) + Inches.of(47.0 / 2).in(Meters);

                double trenchWallDistY = Inches.of(73.0).in(Meters)
                        + Inches.of(47.0 / 2).in(Meters)
                        + Inches.of(6).in(Meters);

                super.addRectangularObstacle(
                        Inches.of(53).in(Meters),
                        Inches.of(12).in(Meters),
                        new Pose2d(8.27 - trenchWallDistX, 4.035 - trenchWallDistY, Rotation2d.kZero));
                super.addRectangularObstacle(
                        Inches.of(53).in(Meters),
                        Inches.of(12).in(Meters),
                        new Pose2d(8.27 + trenchWallDistX, 4.035 - trenchWallDistY, Rotation2d.kZero));
                super.addRectangularObstacle(
                        Inches.of(53).in(Meters),
                        Inches.of(12).in(Meters),
                        new Pose2d(8.27 - trenchWallDistX, 4.035 + trenchWallDistY, Rotation2d.kZero));
                super.addRectangularObstacle(
                        Inches.of(53).in(Meters),
                        Inches.of(12).in(Meters),
                        new Pose2d(8.27 - trenchWallDistX, 4.035 - trenchWallDistY, Rotation2d.kZero));

                //    // poles of the tower
                //    super.addRectangularObstacle(
                //            Inches.of(2).in(Meters),
                //            Inches.of(47).in(Meters),
                //            new Pose2d(new Translation2d(Inches.of(42), Inches.of(159)), new Rotation2d()));
                //
                //    super.addRectangularObstacle(
                //            Inches.of(2).in(Meters),
                //            Inches.of(47).in(Meters),
                //            new Pose2d(new Translation2d(Inches.of(651 - 42), Inches.of(170)), new Rotation2d()));
                //
                //    // Colliders to describe the hub plus ramps
                //    if (AddRampCollider) {
                //        super.addRectangularObstacle(
                //                Inches.of(47).in(Meters),
                //                Inches.of(217).in(Meters),
                //                new Pose2d(RebuiltHub.blueHubPose.toTranslation2d(), new Rotation2d()));
                //
                //        super.addRectangularObstacle(
                //                Inches.of(47).in(Meters),
                //                Inches.of(217).in(Meters),
                //                new Pose2d(RebuiltHub.redHubPose.toTranslation2d(), new Rotation2d()));
                //    }
                //
                //    // Colliders to describe just the hub
                //    else {
                //        super.addRectangularObstacle(
                //                Inches.of(47).in(Meters),
                //                Inches.of(47).in(Meters),
                //                new Pose2d(RebuiltHub.blueHubPose.toTranslation2d(), new Rotation2d()));
                //
                //        super.addRectangularObstacle(
                //                Inches.of(47).in(Meters),
                //                Inches.of(47).in(Meters),
                //                new Pose2d(RebuiltHub.redHubPose.toTranslation2d(), new Rotation2d()));
                //    }
            }
        }

        /**
         *
         *
         * <h2>Creates an Arena for the 2026 FRC game rebuilt </h2>
         *
         * <p>This will create an Arena with the ramp areas marked as inaccessible. If you would like to change that use
         * {@link #CustomArena(boolean)}. Additionally due to performance issues the arena will not spawn all fuel by
         * default. If you would like to change this use {@link #setEfficiencyMode(boolean)}
         */
        public CustomArena() {
            this(true);
        }

        /**
         *
         *
         * <h2>Creates an Arena for the 2026 FRC game rebuilt </h2>
         *
         * <p>Due to the nature of maple sim they can not be fully simulated and so either must be non existent or treated
         * as full colliders. This behavior can be changed with the AddRampCollider variable. Additionally due to
         * performance issues the arena will not spawn all fuel by default. If you would like to change this use
         * {@link #setEfficiencyMode(boolean)}
         *
         * @param AddRampCollider Whether or not the ramps should be added as colliders.
         */
        public CustomArena(boolean AddRampCollider) {
            super(new RebuiltFieldObstaclesMap(AddRampCollider));

            Settings settings = physicsWorld.getSettings();

            // settings.setVelocityConstraintSolverIterations(3);
            // settings.setPositionConstraintSolverIterations(2);
            settings.setMinimumAtRestTime(0.02);

            physicsWorld.setSettings(settings);
        }

        @Override
        public void placeGamePiecesOnField() {
            for (int x = 0; x < 12; x += 1) {
                for (int y = 0; y < 30; y += isInEfficiencyMode ? 3 : 1) {
                    addGamePiece(new RebuiltFuelOnField(centerPieceBottomRightCorner.plus(
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                }
            }

            boolean isOnBlue = !DriverStation.getAlliance().isEmpty()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

            if (isOnBlue || !isInEfficiencyMode) {
                for (int x = 0; x < 4; x++) {
                    for (int y = 0; y < 6; y++) {
                        addGamePiece(new RebuiltFuelOnField(blueDepotBottomRightCorner.plus(
                                new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                    }
                }
            }

            if (!isOnBlue || !isInEfficiencyMode) {
                for (int x = 0; x < 4; x++) {
                    for (int y = 0; y < 6; y++) {
                        addGamePiece(new RebuiltFuelOnField(redDepotBottomRightCorner.plus(
                                new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                    }
                }
            }

            setupValueForMatchBreakdown("CurrentFuelInOutpost");
            setupValueForMatchBreakdown("TotalFuelInOutpost");
            setupValueForMatchBreakdown("TotalFuelInHub");
            setupValueForMatchBreakdown("WastedFuel");
        }

        @Override
        public void simulationSubTick(int tickNum) {
            super.simulationSubTick(tickNum);
        }

        /**
         *
         *
         * <h2>Determines wether the arena is in efficiency mode </h2>
         *
         * <h3>For changes too take effect call {@link #resetFieldForAuto()}. </h3>
         *
         * <p>Efficiency mode reduces the amount of game pieces on the field too increase performance. MapleSim was not
         * designed with 400 game pieces in mind and so can struggle with the large number of game pieces present in
         * rebuilt.
         *
         * @param efficiencyMode Wether efficiency mode should be on or off.
         */
        public void setEfficiencyMode(boolean efficiencyMode) {
            isInEfficiencyMode = efficiencyMode;
        }

        /**
         *
         *
         * <h2>Returns wether or not the arena is in Efficiency mode </h2>
         * <p>
         * For more information see {@link #setEfficiencyMode(boolean)}
         *
         * @return Wether or not the Arena is in efficiency mode.
         */
        public boolean getEfficiencyMode() {
            return isInEfficiencyMode;
        }
    }
}
