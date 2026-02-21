package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.MeasurementHealthValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.ShootingKinematics;
import frc.robot.SimManager;
import frc.robot.subsystems.superintake.intakepivot.IntakePivot;
import frc.robot.subsystems.superintake.intakerollers.IntakeRollers;
import frc.robot.subsystems.superstructure.feeder.Feeder;
import frc.robot.subsystems.superstructure.flywheel.Flywheel;
import frc.robot.subsystems.superstructure.flywheel.FlywheelConstants;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.spindexer.Spindexer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SuperstructureIOSim extends SuperstructureIO {
    private static final double shootingBallsPerSec = 4.0;
    private static final double ballShootDelay = 1.0 / shootingBallsPerSec;

    private static final IntakePivot intakePivot = IntakePivot.get();
    private static final IntakeRollers intakeRollers = IntakeRollers.get();
    private static final Feeder feeder = Feeder.get();
    private static final Flywheel flywheel = Flywheel.get();
    private static final Hood hood = Hood.get();
    private static final Spindexer spindexer = Spindexer.get();

    private final SimManager simManager = SimManager.get();

    private double lastShotTimestamp = 0.0;

    public SuperstructureIOSim() {
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        if (
                intakeRollers.getGoal() == IntakeRollers.Goal.INTAKE &&
                        intakePivot.getGoal() == IntakePivot.Goal.DEPLOY
        ) {
            if (!simManager.intakeSimulation.isRunning()) {
                simManager.intakeSimulation.startIntake();
            }
        } else if (simManager.intakeSimulation.isRunning()) {
            simManager.intakeSimulation.stopIntake();
        }

        // get this value before potentially subtracting 1 when shooting
        int gamePiecesInHopper = simManager.intakeSimulation.getGamePiecesAmount();

        if (
                feeder.getGoal() == Feeder.Goal.FEED &&
                        spindexer.getGoal() == Spindexer.Goal.FEED
        ) {
            if (
                    Timer.getTimestamp() - lastShotTimestamp > ballShootDelay &&
                            simManager.intakeSimulation.obtainGamePieceFromIntake()
            ) {
                lastShotTimestamp = Timer.getTimestamp();

                Pose2d robotPose = simManager.driveSimulation.getSimulatedDriveTrainPose();
                var gamePiece = new RebuiltFuelOnFly(
                        robotPose.getTranslation(),
                        ShootingKinematics.fuelExitTranslation.toTranslation2d(),
                        simManager.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        robotPose.getRotation(),
                        Meters.of(ShootingKinematics.fuelExitTranslation.getZ()),
                        MetersPerSecond.of(Units.rotationsPerMinuteToRadiansPerSecond(flywheel.getVelocityRPM()) * FlywheelConstants.flywheelRadiusMeters),
                        // Applying the shooter facing direction to the maple-sim parameter
                        // causes issues because it causes the shooter position to be rotated
                        // which puts it in the opposite corner of the robot. Instead, just
                        // reverse the hood
                        Radians.of(Math.PI - hood.getPositionRad())
                ).disableBecomesGamePieceOnFieldAfterTouchGround();
                SimulatedArena.getInstance().addGamePieceProjectile(gamePiece);
            }
        }

        inputs.canrangeConnected = true;
        if (gamePiecesInHopper > 0) {
            inputs.canrangeDistanceMeters = Timer.getTimestamp() - lastShotTimestamp < ballShootDelay * 0.5
                    ? Units.inchesToMeters(5.0)
                    : Units.inchesToMeters(15.0);
            inputs.canrangeMeasurementHealth = MeasurementHealthValue.Good;
        } else {
            inputs.canrangeDistanceMeters = Units.inchesToMeters(20.0);
            inputs.canrangeMeasurementHealth = MeasurementHealthValue.Bad;
        }

        Logger.recordOutput("FieldSimulation/NumberOfFuelInHopper", simManager.intakeSimulation.getGamePiecesAmount());
    }
}
