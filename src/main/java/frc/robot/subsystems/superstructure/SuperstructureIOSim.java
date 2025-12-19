package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;

public class SuperstructureIOSim extends SuperstructureIO {
    private static final Translation2d[] stationLocations = {
            new Translation2d(1, 1),
            new Translation2d(1, 7),
            new Translation2d(16.5, 7),
            new Translation2d(16.5, 1)
    };

    private final RobotState robotState = RobotState.get();
    private final EndEffector endEffector = EndEffector.get();
    private final Elevator elevator = Elevator.get();
    private final Funnel funnel = Funnel.get();

    public static boolean gamePieceVisible = false;

    private final Timer sinceAtStation = new Timer();
    private static final double stationGamePieceVisibleTime = 0.25;
    private static final double stationIntakeTime = 0.3;

    private final Timer sinceCoralIntaked = new Timer();
    private static final double indexTime = 0.5;
    private CoralState coralState = CoralState.IN_END_EFFECTOR; // preload

    private enum CoralState {
        NO_CORAL,
        INTAKING,
        IN_END_EFFECTOR
    }

    public SuperstructureIOSim() {
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> coralState = CoralState.IN_END_EFFECTOR));
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        var pose = robotState.getPose();
        switch (coralState) {
            case NO_CORAL -> {
                var current = robotState.getPose().getTranslation();

                if (
                        Arrays.stream(stationLocations)
                                .anyMatch(t -> t.getDistance(current) < 1.5)
                                // TODO update
//                                && (funnel.getGoal() == Funnel.Goal.INTAKE_ALTERNATE || funnel.getGoal() == Funnel.Goal.INTAKE_BACKWARDS)
                                && endEffector.getGoal() == EndEffector.Goal.FUNNEL_INTAKE
                ) {
                    if (!sinceAtStation.isRunning()) sinceAtStation.restart();

                    if (sinceAtStation.hasElapsed(stationIntakeTime)) {
                        coralState = CoralState.INTAKING;
                        sinceCoralIntaked.restart();
                    }
                } else {
                    sinceAtStation.stop();
                    sinceAtStation.reset();
                }

                gamePieceVisible = sinceAtStation.hasElapsed(stationGamePieceVisibleTime);
            }
            case INTAKING -> {
                if (
                    // TODO update
//                        (funnel.getGoal() == Funnel.Goal.INTAKE_ALTERNATE || funnel.getGoal() == Funnel.Goal.INTAKE_BACKWARDS)
                    //                &&
                        endEffector.getGoal() == EndEffector.Goal.FUNNEL_INTAKE
                ) {
                    if (!sinceCoralIntaked.isRunning()) sinceCoralIntaked.restart();
                    if (sinceCoralIntaked.hasElapsed(indexTime)) {
                        coralState = CoralState.IN_END_EFFECTOR;
                    }
                } else {
                    sinceCoralIntaked.stop();
                    sinceCoralIntaked.reset();
                }

                gamePieceVisible = !sinceCoralIntaked.hasElapsed(0.25);
            }
            case IN_END_EFFECTOR -> {
                gamePieceVisible = false;

                if (endEffector.getGoal() == EndEffector.Goal.SCORE_CORAL
                        || endEffector.getGoal() == EndEffector.Goal.SCORE_CORAL_L1
                        || endEffector.getGoal() == EndEffector.Goal.EJECT_ALTERNATE) {
                    coralState = CoralState.NO_CORAL;

                    var endEffectorAngleRad = endEffector.getAngleRad();
                    var coralInEndEffector = SuperstructureConstants.coralInEndEffector(elevator.getPositionMeters(), endEffectorAngleRad);
                    SimulatedArena.getInstance()
                            .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                    pose.getTranslation(),
                                    new Translation2d(coralInEndEffector.getX() - Units.inchesToMeters(2), 0),
                                    ModuleIOSim.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                    pose.getRotation(),
                                    // The height at which the coral is ejected
                                    Meters.of(coralInEndEffector.getZ()),
                                    // The initial speed of the coral
                                    MetersPerSecond.of(-1.5),
                                    elevator.getGoal() == Elevator.Goal.SCORE_L4
                                            ? Degrees.of(65)
                                            : (
                                            elevator.getGoal() == Elevator.Goal.SCORE_L1
                                                    ? Degrees.of(0)
                                                    : Radians.of(endEffectorAngleRad)
                                    )
                            ));
                }
            }
        }
        Logger.recordOutput("FieldSimulation/CoralState", coralState);

        switch (coralState) {
            case INTAKING -> {
                inputs.funnelBeamBreakTriggered = true;
                inputs.endEffectorBeamBreakTriggered = false;
            }
            case IN_END_EFFECTOR -> {
                inputs.funnelBeamBreakTriggered = false;
                inputs.endEffectorBeamBreakTriggered = true;
            }
            case NO_CORAL -> {
                inputs.funnelBeamBreakTriggered = false;
                inputs.endEffectorBeamBreakTriggered = false;
            }
        }
    }
}
