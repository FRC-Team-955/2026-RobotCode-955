package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.CANLogger;
import frc.lib.commands.CommandsExt;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.goals.WheelRadiusCharacterizationGoal;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final LoggedDashboardChooser<Command> characterizationChooser = new LoggedDashboardChooser<>("Characterization Choices");

    public final RobotState robotState = RobotState.get();
    public final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    public final Controller controller = Controller.get();
    public final CANLogger canLogger = CANLogger.get();
    public final RobotMechanism robotMechanism = RobotMechanism.get();
    public final ShootingKinematics shootingKinematics = ShootingKinematics.get();

    /* Subsystems */
    // Note: order does matter
    public final Drive drive = Drive.get();
    public final AprilTagVision aprilTagVision = AprilTagVision.get();
    public final GamePieceVision gamePieceVision = GamePieceVision.get();
    public final Superstructure superstructure = Superstructure.get();
    public final Superintake superintake = Superintake.get();

    public RobotContainer() {
        addAutos();
        addCharacterizations();
        setDefaultCommands();
        configureBindings();

        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() < 30)
                .onTrue(controller.rumble(0.5, 2.0));
    }

    private void addAutos() {
        autoChooser.addOption("None", Commands.none());

        autoChooser.addOption(
                "Characterization",
                // We need to require the superstructure during characterization so that the default command doesn't get run
                Commands.deferredProxy(() -> CommandsExt.eagerSequence(
                        superstructure.cancel(),
                        superintake.cancel(),
                        characterizationChooser.get()
                ))
        );
    }

    private void addCharacterizations() {
        ////////////////////// DRIVE //////////////////////

        // TODO
        characterizationChooser.addOption("Drive 1 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(1.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 2 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(2.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 3 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(3.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 4 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(4.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive Full Speed Characterization", drive.fullSpeedCharacterization());
        characterizationChooser.addOption("Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization(WheelRadiusCharacterizationGoal.Direction.CLOCKWISE));
        characterizationChooser.addOption("Drive Slip Current Characterization", drive.slipCurrentCharacterization());
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(drive.driveJoystick());
        superstructure.setDefaultCommand(superstructure.cancel());
        superintake.setDefaultCommand(superintake.cancel());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        // NOTE: if you are binding a trigger to a command returned by a subsystem, you must wrap it in CommandsExt.eagerSequence(superstructure.cancel(), <your command>)
        // You must do this because if you don't, superstructure's default command will cancel your command

        controller.y().onTrue(robotState.resetRotation());

        controller.leftBumper().onTrue(Commands.parallel(
                superstructure.cancel(),
                superintake.cancel()
        ));
        controller.b().toggleOnTrue(drive.driveJoystickWithAiming());
        controller.leftTrigger().whileTrue(Commands.repeatingSequence(
                Commands.runOnce(() -> {
                    if (!shootingKinematics.isValidShootingParameters()) return;
                    SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                            ModuleIOSim.driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                            ShootingKinematics.fuelExitTransform.getTranslation().toTranslation2d(),
                            ModuleIOSim.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            //Rotation2d.fromRadians(shootingKinematics.getShootingParameters().headingRad()),
                            ModuleIOSim.driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                            Units.Meters.of(ShootingKinematics.fuelExitTransform.getTranslation().getZ()),
                            Units.MetersPerSecond.of(shootingKinematics.getShootingParameters().velocityMetersPerSec()),
                            Units.Radians.of(shootingKinematics.getShootingParameters().hoodAngleRad())
                    ).disableBecomesGamePieceOnFieldAfterTouchGround());
                }),
                Commands.waitSeconds(0.1)
        ));

        if (BuildConstants.mode == BuildConstants.Mode.SIM) {
            var ref = new Object() {
                Pose2d setpoint;
            };
            controller.a().toggleOnTrue(CommandsExt.eagerSequence(
                    Commands.runOnce(() -> ref.setpoint = new Pose2d(
                            robotState.getPose().getX() + 1,
                            robotState.getPose().getY() + 1,
                            robotState.getPose().getRotation().plus(Rotation2d.kPi)
                    )),
                    drive.moveTo(() -> ref.setpoint, false)
            ));
        }

        // NOTE: if you are binding a trigger to a command returned by a subsystem, you must wrap it in CommandsExt.eagerSequence(superstructure.cancel(), <your command>)
        // You must do this because if you don't, superstructure's default command will cancel your command
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
