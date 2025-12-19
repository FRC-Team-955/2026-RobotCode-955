package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.robot.subsystems.elevator.ElevatorConstants.hardstopMeters;
import static frc.robot.subsystems.elevator.ElevatorConstants.hardstopSlowdownMeters;

/** Holds the Mechanism2d and all roots and ligaments that visualizes the robot state */
public class RobotMechanism {
    private static RobotMechanism instance;

    public static RobotMechanism get() {
        if (instance == null)
            synchronized (RobotMechanism.class) {
                instance = new RobotMechanism();
            }

        return instance;
    }

    private RobotMechanism() {
        addBumpers();
    }

    /** Middle of the robot in the mechanism */
    public static final double middleOfRobot = 0.75;

    @AutoLogOutput(key = "RobotState/Mechanism")
    public final LoggedMechanism2d mechanism = new LoggedMechanism2d(middleOfRobot * 2, 2.1, new Color8Bit(Color.kBlack));

    public final ElevatorMechanism elevator = new ElevatorMechanism();
    public final EndEffectorMechanism endEffector = new EndEffectorMechanism();
    public final FunnelMechanism funnel = new FunnelMechanism();

    private void addBumpers() {
        double bumperThickness = Units.inchesToMeters(3.375);

        LoggedMechanismRoot2d frontBumperRoot = mechanism.getRoot("bumpers_front", middleOfRobot + (DriveConstants.driveConfig.bumperLengthMeters() / 2) - bumperThickness, -0.25);
        frontBumperRoot.append(new LoggedMechanismLigament2d(
                "bumpers_front",
                bumperThickness - 0.0025,
                0,
                90,
                new Color8Bit(Color.kBlue)
        ));

        LoggedMechanismRoot2d backBumperRoot = mechanism.getRoot("bumpers_back", middleOfRobot - (DriveConstants.driveConfig.bumperLengthMeters() / 2), -0.25);
        backBumperRoot.append(new LoggedMechanismLigament2d(
                "bumpers_back",
                bumperThickness,
                0,
                90,
                new Color8Bit(Color.kBlue)
        ));
    }

    public class FunnelMechanism {
        private static final double x = middleOfRobot + Units.inchesToMeters(12.2);
        private static final double y = Units.inchesToMeters(6);
        private static final double angle = 90;

        public final LoggedMechanismRoot2d root = mechanism.getRoot("funnel", x, y + 0.065);

        public final LoggedMechanismRoot2d beltRoot = mechanism.getRoot(
                "funnel_belt",
                x - Units.inchesToMeters(5),
                y + Units.inchesToMeters(7)
        );
        public final LoggedMechanismLigament2d beltLigament = beltRoot.append(new LoggedMechanismLigament2d(
                "funnel_belt",
                Units.inchesToMeters(1),
                0,
                12,
                new Color8Bit(Color.kOrange)
        ));

        private FunnelMechanism() {
            root.append(new LoggedMechanismLigament2d(
                    "ligament",
                    Units.inchesToMeters(14.75),
                    angle,
                    10,
                    new Color8Bit(Color.kBlue)
            ));
        }
    }

    public class ElevatorMechanism {
        public final LoggedMechanismRoot2d stage1Root = mechanism.getRoot("elevator_stage1", 0, 0);
        public final LoggedMechanismRoot2d stage2Root = mechanism.getRoot("elevator_stage2", 0, 0);
        public final LoggedMechanismRoot2d stage3Root = mechanism.getRoot("elevator_stage3", 0, 0);
        private final LoggedMechanismRoot2d hardstopSlowdownRoot = mechanism.getRoot("elevator_hardstopSlowdown", 0, 0);

        public void updateHardstopSlowdownPosition() {
            hardstopSlowdownRoot.setPosition(middleOfRobot - Units.inchesToMeters(15), Units.inchesToMeters(2.85) + hardstopSlowdownMeters);
        }

        private ElevatorMechanism() {
            var baseRoot = mechanism.getRoot(
                    "elevatorBase",
                    middleOfRobot - Units.inchesToMeters(7) + 0.06,
                    Units.inchesToMeters(1.85)
            );
            baseRoot.append(new LoggedMechanismLigament2d(
                    "base",
                    Units.inchesToMeters(33.2),
                    90,
                    13,
                    new Color8Bit(new Color(0.2, 0.2, 0.2))
            ));

            stage1Root.append(new LoggedMechanismLigament2d(
                    "stage1",
                    Units.inchesToMeters(32.5),
                    90,
                    12,
                    new Color8Bit(new Color(0.3, 0.3, 0.3))
            ));

            stage2Root.append(new LoggedMechanismLigament2d(
                    "stage2",
                    Units.inchesToMeters(32),
                    90,
                    11,
                    new Color8Bit(new Color(0.4, 0.4, 0.4))
            ));

            stage3Root.append(new LoggedMechanismLigament2d(
                    "stage3",
                    Units.inchesToMeters(7),
                    90,
                    10,
                    new Color8Bit(new Color(0.5, 0.5, 0.5))
            ));

            var hardstopRoot = mechanism.getRoot("elevator_hardstop", middleOfRobot - Units.inchesToMeters(15), Units.inchesToMeters(2.85) + hardstopMeters);
            hardstopRoot.append(new LoggedMechanismLigament2d(
                    "hardstop",
                    Units.inchesToMeters(1),
                    90,
                    11,
                    new Color8Bit(Color.kGray)
            ));

            updateHardstopSlowdownPosition();
            hardstopSlowdownRoot.append(new LoggedMechanismLigament2d(
                    "hardstopSlowdown",
                    Units.inchesToMeters(1),
                    90,
                    11,
                    new Color8Bit(Color.kYellow)
            ));
        }
    }

    public class EndEffectorMechanism {
        public final LoggedMechanismRoot2d root = mechanism.getRoot("endEffector", 0, 0);
        public final LoggedMechanismLigament2d ligament = root.append(new LoggedMechanismLigament2d(
                "ligament",
                Units.inchesToMeters(10),
                90,
                10,
                new Color8Bit(Color.kPurple)
        ));

        public final LoggedMechanismRoot2d topRollersRoot = mechanism.getRoot("endEffector_topRollers", 0, 0);
        public final LoggedMechanismLigament2d topRollersLigament = topRollersRoot.append(new LoggedMechanismLigament2d(
                "endEffector_topRollers",
                Units.inchesToMeters(1),
                0,
                12,
                new Color8Bit(Color.kOrange)
        ));

        private EndEffectorMechanism() {
        }
    }
}
