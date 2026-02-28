package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.CANLogger;
import frc.robot.autos.AutoManager;
import frc.robot.controller.Controller;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.goals.DriveJoystickGoal;
import frc.robot.subsystems.drive.goals.WheelRadiusCharacterizationGoal;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> characterizationChooser = new LoggedDashboardChooser<>("Characterization Choices");

    public final RobotState robotState = RobotState.get();
    public final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    public final Controller controller = Controller.get();
    public final CANLogger canLogger = CANLogger.get();
    public final RobotMechanism robotMechanism = RobotMechanism.get();
    public final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    public final AutoManager autoManager = AutoManager.get();
    public final HubShiftTracker hubShiftTracker = HubShiftTracker.get();

    /* Subsystems */
    public final Drive drive = Drive.get();
    public final AprilTagVision aprilTagVision = AprilTagVision.get();
    public final GamePieceVision gamePieceVision = GamePieceVision.get();
    public final LEDs leds = LEDs.get();

    public final Superintake superintake = Superintake.get();
    public final Superstructure superstructure = Superstructure.get();

    public RobotContainer() {
        autoManager.autoChooser.addOption("Characterization", Commands.deferredProxy(characterizationChooser::get));
        addCharacterizations();
        setDefaultCommands();
        configureBindings();

        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() < 30)
                .onTrue(controller.rumble(0.5, 2.0));
    }


    private void addCharacterizations() {
        characterizationChooser.addOption("Drive 1 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(1.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 2 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(2.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 3 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(3.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 4 m/s Characterization", drive.runRobotRelative(() -> new ChassisSpeeds(4.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive Full Speed Characterization", drive.fullSpeedCharacterization());
        characterizationChooser.addOption("Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization(WheelRadiusCharacterizationGoal.Direction.CLOCKWISE));
        characterizationChooser.addOption("Drive Slip Current Characterization", drive.slipCurrentCharacterization());
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(drive.driveJoystick(() -> DriveJoystickGoal.Mode.Normal));
        superintake.setDefaultCommand(superintake.setGoal(Superintake.Goal.IDLE).ignoringDisable(true));
        superstructure.setDefaultCommand(superstructure.setGoal(Superstructure.Goal.IDLE).ignoringDisable(true));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        controller.y().onTrue(robotState.resetRotation());

        controller.leftTrigger()
                .and(controller.rightTrigger().negate())
                .whileTrue(Commands.parallel(
                        drive.driveJoystick(() -> operatorDashboard.manualAiming.get() ? DriveJoystickGoal.Mode.StopWithX : DriveJoystickGoal.Mode.Aim),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                ));

        controller.rightTrigger()
                .and(controller.leftTrigger().negate())
                .whileTrue(Commands.parallel(
                        drive.driveJoystick(() -> operatorDashboard.disableAssist.get() ? DriveJoystickGoal.Mode.Normal : DriveJoystickGoal.Mode.Assist),
                        superintake.setGoal(Superintake.Goal.INTAKE)
                ));

        controller.leftTrigger()
                .and(controller.rightTrigger())
                .whileTrue(Commands.parallel(
                        drive.driveJoystick(() -> {
                            if (operatorDashboard.manualAiming.get() && operatorDashboard.disableAssist.get()) {
                                return DriveJoystickGoal.Mode.StopWithX;
                            } else if (operatorDashboard.manualAiming.get()) {
                                return DriveJoystickGoal.Mode.Assist;
                            } else if (operatorDashboard.disableAssist.get()) {
                                return DriveJoystickGoal.Mode.Aim;
                            } else {
                                return DriveJoystickGoal.Mode.AimAndAssist;
                            }
                        }),
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        superstructure.setGoal(Superstructure.Goal.SHOOT)
                ));
        //                                .alongWith(drive.driveJoystickWithAssist(() -> {
        //                                            Pose2d robotPose = robotState.getPose();
        //                                            Pose2d closestFuel = null;
        //                                            double closestDist = Double.MAX_VALUE;
        //
        //                                            for (var fuel : gamePieceVision.getFreshCoral()) {
        //                                                Pose2d fuelPose = fuel.toPose2d();
        //                                                double dist = fuelPose.getTranslation().getDistance(robotPose.getTranslation());
        //
        //                                                if (dist < closestDist) {
        //                                                    closestDist = dist;
        //                                                    closestFuel = fuelPose;
        //                                                }
        //                                            }
        //
        //                                            return Optional.ofNullable(closestFuel);
        //                                        }
        //
        //                                ))

        controller.a()
                .whileTrue(Commands.parallel(
                        superintake.setGoal(Superintake.Goal.EJECT),
                        superstructure.setGoal(Superstructure.Goal.EJECT)
                ));

        new Trigger(operatorDashboard.homeIntakePivot::get)
                .onTrue(Commands.runOnce(() -> operatorDashboard.homeIntakePivot.set(false)).ignoringDisable(true));
        new Trigger(operatorDashboard.homeIntakePivot::get)
                .and(DriverStation::isEnabled)
                .onTrue(superintake.setGoal(Superintake.Goal.HOME_INTAKE_PIVOT));
        new Trigger(operatorDashboard.homeIntakePivot::get)
                .and(DriverStation::isDisabled)
                .onTrue(Commands.runOnce(superintake.intakePivot::finishHoming).ignoringDisable(true));

        new Trigger(operatorDashboard.homeHood::get)
                .onTrue(Commands.runOnce(() -> operatorDashboard.homeHood.set(false)).ignoringDisable(true));
        new Trigger(operatorDashboard.homeHood::get)
                .and(DriverStation::isEnabled)
                .onTrue(superstructure.setGoal(Superstructure.Goal.HOME_HOOD));
        new Trigger(operatorDashboard.homeHood::get)
                .and(DriverStation::isDisabled)
                .onTrue(Commands.runOnce(superstructure.hood::finishHoming).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoManager.getSelectedAuto();
    }
}
