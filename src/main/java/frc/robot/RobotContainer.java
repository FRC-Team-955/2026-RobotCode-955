package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.CANLogger;
import frc.lib.EnergyLogger;
import frc.robot.autos.AutoManager;
import frc.robot.controller.Controller;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.superintake.Superintake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

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
    /* Subsystems */
    public final Drive drive = Drive.get();
    public final AprilTagVision aprilTagVision = AprilTagVision.get();
    public final GamePieceVision gamePieceVision = GamePieceVision.get();
    public final LEDs leds = LEDs.get();

    public final Superintake superintake = Superintake.get();
    public final Superstructure superstructure = Superstructure.get();

    /* Other stuff */
    public final Controller controller = Controller.get();
    public final CANLogger canLogger = CANLogger.get();
    public final RobotMechanism robotMechanism = RobotMechanism.get();
    public final ShootingKinematics shootingKinematics = ShootingKinematics.get();
    public final AutoManager autoManager = AutoManager.get();
    public final HubShiftTracker hubShiftTracker = HubShiftTracker.get();
    public final EnergyLogger energyLogger = EnergyLogger.get();
    public final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    public RobotContainer() {
        addCharacterizations();
        setDefaultCommands();
        configureBindings();
    }

    private void addCharacterizations() {
        characterizationChooser.addOption("Drive 1 m/s Characterization", drive.chassisSpeeds(() -> new ChassisSpeeds(1.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 2 m/s Characterization", drive.chassisSpeeds(() -> new ChassisSpeeds(2.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 3 m/s Characterization", drive.chassisSpeeds(() -> new ChassisSpeeds(3.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive 4 m/s Characterization", drive.chassisSpeeds(() -> new ChassisSpeeds(4.0, 0.0, 0.0)));
        characterizationChooser.addOption("Drive Full Speed Characterization", drive.fullSpeedCharacterization());
        characterizationChooser.addOption("Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization());
        characterizationChooser.addOption("Drive Slip Current Characterization", drive.slipCurrentCharacterization());
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(drive.joystickDrive());
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

        Trigger shoot = controller.leftTrigger();
        Trigger shootForce = controller.leftBumper();
        Trigger anyShoot = shoot.or(shootForce);

        BooleanSupplier shouldNotAssist = () -> operatorDashboard.disableAssist.get() || robotState.isInTrench(robotState.getTranslation());
        controller.rightTrigger()
                .or(controller.rightBumper().and(anyShoot))
                .whileTrue(superintake.setGoal(Superintake.Goal.INTAKE));
        controller.rightBumper()
                .and(anyShoot.negate())
                .whileTrue(Commands.parallel(
                        superintake.setGoal(Superintake.Goal.INTAKE),
                        // Citrus mode: always point in direction of travel
                        drive.joystickDrive().withHeadingOverride(() -> OptionalDouble.of(controller.getDriveLinearDirection().getRadians()))
                ));

        shoot.whileTrue(Commands.parallel(
                drive.joystickDrive().withAiming(),
                superstructure.setGoal(Superstructure.Goal.SHOOT)
        ));
        shootForce.whileTrue(Commands.parallel(
                drive.joystickDrive().withAiming(),
                superstructure.setGoal(Superstructure.Goal.SHOOT_FORCE)
        ));

        controller.x()
                .whileTrue(Commands.parallel(
                        superintake.setGoal(Superintake.Goal.EJECT),
                        superstructure.setGoal(Superstructure.Goal.EJECT)
                ));

        new Trigger(operatorDashboard.homeIntakePivot::get)
                .onTrue(Commands.runOnce(() -> operatorDashboard.homeIntakePivot.set(false)).ignoringDisable(true));
        new Trigger(operatorDashboard.homeIntakePivot::get)
                .and(DriverStation::isEnabled)
                .onTrue(superintake.setGoalHomeIntakePivot());
        new Trigger(operatorDashboard.homeIntakePivot::get)
                .and(DriverStation::isDisabled)
                .onTrue(Commands.runOnce(superintake.intakePivot::finishHoming).ignoringDisable(true));

        new Trigger(operatorDashboard.homeHood::get)
                .onTrue(Commands.runOnce(() -> operatorDashboard.homeHood.set(false)).ignoringDisable(true));
        new Trigger(operatorDashboard.homeHood::get)
                .and(DriverStation::isEnabled)
                .onTrue(superstructure.setGoalHomeHood());
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
        return autoManager.getSelectedAutoCommand();
    }

    public Command getTestCommand() {
        return characterizationChooser.get();
    }
}
