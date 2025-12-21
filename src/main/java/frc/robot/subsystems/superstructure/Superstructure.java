package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.CommandBasedSubsystem;
import frc.robot.OperatorDashboard;
import frc.robot.OperatorDashboard.CoralScoringLevel;
import frc.robot.RobotState;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.superstructure.ReefAlign.LocalReefSide;
import frc.robot.subsystems.superstructure.ReefAlign.ReefZoneSide;
import frc.robot.subsystems.superstructure.StationAlign.Station;
import frc.robot.subsystems.superstructure.commands.*;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

public class Superstructure extends CommandBasedSubsystem {
    private final RobotState robotState = RobotState.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final AprilTagVision aprilTagVision = AprilTagVision.get();
    private final Elevator elevator = Elevator.get();
    private final EndEffector endEffector = EndEffector.get();
    private final Funnel funnel = Funnel.get();
    private final GamePieceVision gamePieceVision = GamePieceVision.get();

    private final SuperstructureIO io = createIO();
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE,

        MANUAL_SCORE_CORAL_WAIT_FOR_ELEVATOR,
        MANUAL_SCORE_CORAL_WAIT_FOR_CONFIRM,
        MANUAL_SCORE_CORAL_SCORING,

        AUTO_SCORE_CORAL_WAIT_UNTIL_CAN_RAISE,
        AUTO_SCORE_CORAL_WAIT_FOR_ALIGN,
        AUTO_SCORE_CORAL_WAIT_FOR_ELEVATOR,
        AUTO_SCORE_CORAL_WAIT_BEFORE_SCORING,
        AUTO_SCORE_CORAL_SCORING,

        DESCORE_ALGAE_WAIT_FOR_ELEVATOR,
        DESCORE_ALGAE_DESCORING,

        AUTO_DESCORE_ALGAE_WAIT_UNTIL_CAN_RAISE,
        AUTO_DESCORE_ALGAE_WAIT_FOR_ALIGN,
        AUTO_DESCORE_ALGAE_WAIT_FOR_AMPERAGE,
        AUTO_DESCORE_ALGAE_MOVE_BACK,

        HANDOFF,
        HOME_STEP_1,
        HOME_STEP_2,
        HOME_STEP_3,
        HOME_STEP_4,

        FUNNEL_INTAKE_WAITING,

        AUTO_FUNNEL_INTAKE_WAITING_ALIGN,
        AUTO_FUNNEL_INTAKE_WAITING_SHAKE,

        EJECT,

        ZERO_ELEVATOR, // TODO SEE ELEVATOR JOYSTICK CONTROL
    }

    private Goal goal = Goal.IDLE;

    public Command setGoal(Goal superstructureGoal, Supplier<Elevator.Goal> elevatorGoal, Supplier<EndEffector.Goal> endEffectorGoal, Funnel.Goal funnelGoal) {
        return runOnce(() -> {
            goal = superstructureGoal;
            elevator.setGoal(elevatorGoal.get());
            endEffector.setGoal(endEffectorGoal.get());
            funnel.setGoal(funnelGoal);
        });
    }

    private static Superstructure instance;

    public static Superstructure get() {
        if (instance == null)
            synchronized (Superstructure.class) {
                instance = new Superstructure();
            }

        return instance;
    }

    private Superstructure() {
    }

    private final Debouncer endEffectorBeamBreakDebouncer = new Debouncer(3 * 0.02);
    @AutoLogOutput(key = "Superstructure/EndEffectorTriggered")
    @Getter
    private boolean endEffectorTriggered = false;

    private final Debouncer funnelBeamBreakDebouncer = new Debouncer(3 * 0.02);
    @AutoLogOutput(key = "Superstructure/FunnelTriggered")
    @Getter
    private boolean funnelTriggered = false;

    private final Debouncer hasCoralDebouncer = new Debouncer(1, Debouncer.DebounceType.kFalling);
    @AutoLogOutput(key = "Superstructure/HasCoral")
    @Getter
    private boolean hasCoral = false;

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure", inputs);

        endEffectorTriggered = endEffectorBeamBreakDebouncer.calculate(inputs.endEffectorBeamBreakTriggered);
        funnelTriggered = funnelBeamBreakDebouncer.calculate(inputs.funnelBeamBreakTriggered);
        hasCoral = hasCoralDebouncer.calculate(endEffectorTriggered || funnelTriggered || gamePieceVision.visibleDebounced());

        // OperatorDashboard periodicBeforeCommands runs after superstructure
//        throw new RuntimeException("TODO just get the closest side at the start of the command");
//        operatorDashboard.setIgnoreClosestReefSideChanges(switch (goal) {
//            case AUTO_SCORE_CORAL_WAIT_ALIGN, AUTO_SCORE_CORAL_WAIT_ELEVATOR, AUTO_SCORE_CORAL_SCORING,
//                 AUTO_DESCORE_ALGAE_WAIT_FOR_ALIGN, AUTO_DESCORE_ALGAE_WAIT_FOR_AMPERAGE,
//                 AUTO_DESCORE_ALGAE_MOVE_BACK -> true;
//
//            // Allow reef side changes before elevator raises during auto align sequences
//            case AUTO_SCORE_CORAL_WAIT_RAISE, AUTO_DESCORE_ALGAE_WAIT_UNTIL_CAN_RAISE,
//                 // All goals that don't involve auto choose side
//                 IDLE,
//                 MANUAL_SCORE_CORAL_WAIT_FOR_ELEVATOR, MANUAL_SCORE_CORAL_WAIT_FOR_CONFIRM, MANUAL_SCORE_CORAL_SCORING,
//                 DESCORE_ALGAE_WAIT_FOR_ELEVATOR, DESCORE_ALGAE_DESCORING,
//                 HANDOFF, HOME,
//                 FUNNEL_INTAKE_WAITING,
//                 AUTO_FUNNEL_INTAKE_WAITING_ALIGN, AUTO_FUNNEL_INTAKE_WAITING_SHAKE,
//                 EJECT,
//                 ZERO_ELEVATOR -> false;
//        });
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Superstructure/Goal", goal);

        Pose3d robotPose = new Pose3d(robotState.getPose());

        if (gamePieceVision.visibleNotDebounced()) {
            Pose3d coral = robotPose.transformBy(coralAboveFunnel);
            Logger.recordOutput("Superstructure/CoralAboveFunnel", new Pose3d[]{coral});
        } else {
            Logger.recordOutput("Superstructure/CoralAboveFunnel", new Pose3d[]{});
        }

        if (inputs.funnelBeamBreakTriggered) {
            Pose3d coral = robotPose.transformBy(coralInFunnel);
            Logger.recordOutput("Superstructure/CoralInFunnel", new Pose3d[]{coral});
        } else {
            Logger.recordOutput("Superstructure/CoralInFunnel", new Pose3d[]{});
        }

        if (inputs.endEffectorBeamBreakTriggered) {
            Pose3d coral = robotPose.transformBy(coralInEndEffector(elevator.getPositionMeters(), endEffector.getAngleRad()));
            Logger.recordOutput("Superstructure/CoralInEndEffector", new Pose3d[]{coral});
        } else {
            Logger.recordOutput("Superstructure/CoralInEndEffector", new Pose3d[]{});
        }

        Logger.recordOutput("Superstructure/ReefAlign/FinalAlign", ReefAlign.getFinalAlignPose(operatorDashboard.getSelectedReefZoneSide(), operatorDashboard.getSelectedLocalReefSide()));
    }

    public Command cancel() {
        return CommandsExt.eagerSequence(
                setGoal(
                        Goal.IDLE,
                        () -> Elevator.Goal.STOW,
                        () -> EndEffector.Goal.IDLE,
                        Funnel.Goal.IDLE
                ),
                aprilTagVision.setTagIdFilter(new int[0])
        ).ignoringDisable(true);
    }

    public Command home() {
        return CommandsExt.eagerSequence(
                CommandsExt.onlyIf(
                        () -> !endEffectorTriggered || operatorDashboard.ignoreEndEffectorBeamBreak.get(),
                        CommandsExt.eagerSequence(
                                setGoal(
                                        Goal.HOME_STEP_1,
                                        () -> Elevator.Goal.ZERO_CORAL,
                                        () -> EndEffector.Goal.HOME_INITIAL,
                                        Funnel.Goal.IDLE
                                ),
                                endEffector.waitUntilAtGoal()
                        )
                ),

                setGoal(
                        Goal.HOME_STEP_2,
                        () -> Elevator.Goal.ZERO_CORAL,
                        () -> EndEffector.Goal.ZERO_CORAL,
                        Funnel.Goal.IDLE
                ),
                Commands.waitSeconds(0.15),

                setGoal(
                        Goal.HOME_STEP_3,
                        () -> Elevator.Goal.STOW,
                        () -> EndEffector.Goal.IDLE,
                        Funnel.Goal.IDLE
                ),
                Commands.waitSeconds(0.05),

                setGoal(
                        Goal.HOME_STEP_4,
                        () -> Elevator.Goal.STOW,
                        () -> EndEffector.Goal.HOME_FINAL,
                        Funnel.Goal.IDLE
                ),
                endEffector.waitUntilAtGoal()
        );
    }

    public Command zeroElevator() {
        throw new RuntimeException("TODO joystick zeroing see goal and elevator");
//        return wrapExposedCommand(Commands.parallel(
//                setGoal(Goal.ZERO_ELEVATOR),
//                elevator.zeroElevator()
//        ));
    }

    public Command eject() {
        return setGoal(
                Goal.EJECT,
                () -> Elevator.Goal.STOW,
                () -> EndEffector.Goal.EJECT_ALTERNATE,
                Funnel.Goal.EJECT_ALTERNATE
        );
    }

    public Command scoreCoralManual(
            BooleanSupplier forwardCondition,
            Supplier<CoralScoringLevel> coralScoringLevelSupplier
    ) {
        return new ScoreCoralManual(forwardCondition, coralScoringLevelSupplier).create();
    }

    public Command descoreAlgaeManual(Supplier<ReefZoneSide> reefZoneSideSupplier) {
        return new DescoreAlgaeManual(reefZoneSideSupplier).create();
    }

    public Command funnelIntake() {
        return new FunnelIntake().create();
    }

    public Command autoFunnelIntake(Station station) {
        return new AutoFunnelIntake(station).create();
    }

    public Command autoScoreCoral(
            Supplier<ReefZoneSide> reefSideSupplier,
            Supplier<LocalReefSide> sideSupplier,
            Supplier<CoralScoringLevel> coralScoringLevelSupplier,
            BooleanSupplier forceCondition
    ) {
        return new AutoScoreCoral(reefSideSupplier, sideSupplier, coralScoringLevelSupplier, forceCondition).create();
    }

    public Command autoDescoreAlgae(
            Supplier<ReefZoneSide> reefSideSupplier,
            BooleanSupplier forceCondition
    ) {
        return new AutoDescoreAlgae(reefSideSupplier, forceCondition).create();
    }
}