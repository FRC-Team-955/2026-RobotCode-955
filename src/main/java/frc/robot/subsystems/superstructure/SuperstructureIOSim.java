package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.MeasurementHealthValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.SimManager;
import frc.robot.shooting.ShootingKinematics;
import frc.robot.subsystems.superintake.IntakePivot;
import frc.robot.subsystems.superintake.IntakeRollers;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SuperstructureIOSim extends SuperstructureIO {
    private static final double shootingBallsPerSec = 8.0;
    private static final double ballShootDelay = 1.0 / shootingBallsPerSec;

    private double lastShotTimestamp = 0.0;

    public SuperstructureIOSim() {
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        if (
                IntakeRollers.getInstance().getGoal() == IntakeRollers.Goal.INTAKE &&
                        IntakePivot.getInstance().getGoal() == IntakePivot.Goal.DEPLOY
        ) {
            if (!SimManager.getInstance().intakeSimulation.isRunning()) {
                SimManager.getInstance().intakeSimulation.startIntake();
            }
        } else if (SimManager.getInstance().intakeSimulation.isRunning()) {
            SimManager.getInstance().intakeSimulation.stopIntake();
        }

        // get this value before potentially subtracting 1 when shooting
        int gamePiecesInHopper = SimManager.getInstance().intakeSimulation.getGamePiecesAmount();

        if (
                Feeder.getInstance().getGoal() == Feeder.Goal.FEED &&
                        Spindexer.getInstance().getGoal() == Spindexer.Goal.FEED
        ) {
            if (
                    Timer.getTimestamp() - lastShotTimestamp > ballShootDelay &&
                            (SimManager.getInstance().intakeSimulation.obtainGamePieceFromIntake() || DriverStation.isTeleopEnabled())
            ) {
                lastShotTimestamp = Timer.getTimestamp();

                Pose2d robotPose = SimManager.getInstance().driveSimulation.getSimulatedDriveTrainPose();
                var gamePiece = new RebuiltFuelOnFly(
                        robotPose.getTranslation(),
                        ShootingKinematics.getInstance().getFuelExitTranslation().toTranslation2d(),
                        SimManager.getInstance().driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        robotPose.getRotation(),
                        Meters.of(ShootingKinematics.getInstance().getFuelExitTranslation().getZ()),
                        MetersPerSecond.of(Flywheel.getInstance().getVelocityRadPerSec() * Flywheel.radiusMeters),
                        // Applying the shooter facing direction to the maple-sim parameter
                        // causes issues because it causes the shooter position to be rotated
                        // which puts it in the opposite corner of the robot. Instead, just
                        // reverse the hood
                        Radians.of(Math.PI - Hood.getInstance().getShotAngleRad())
                );
                if (!ShootingKinematics.getInstance().getShootingParameters().isPass()) {
                    gamePiece.disableBecomesGamePieceOnFieldAfterTouchGround();
                }
                Logger.recordOutput("ShootingKinematics/ProjectileVelocity", gamePiece.getVelocity3dMPS());
                Logger.recordOutput("ShootingKinematics/ProjectileSpeedRobotRelative", Flywheel.getInstance().getVelocityRadPerSec() * Flywheel.radiusMeters);
                SimulatedArena.getInstance().addGamePieceProjectile(gamePiece);
            }
        }

        inputs.canrangeConnected = true;
        if (gamePiecesInHopper > 0) {
            inputs.canrangeDistanceMeters = Timer.getTimestamp() - lastShotTimestamp < ballShootDelay * 0.5
                    ? Units.inchesToMeters(5.0)
                    : 0.3;
            inputs.canrangeMeasurementHealth = MeasurementHealthValue.Good;
        } else {
            inputs.canrangeDistanceMeters = 0.4;
            inputs.canrangeMeasurementHealth = MeasurementHealthValue.Bad;
        }

        Logger.recordOutput("FieldSimulation/NumberOfFuelInHopper", SimManager.getInstance().intakeSimulation.getGamePiecesAmount());
    }
}
