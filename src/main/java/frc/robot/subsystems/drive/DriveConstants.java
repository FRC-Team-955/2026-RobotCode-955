package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.PIDF;
import frc.robot.BuildConstants;

public class DriveConstants {
    public static final double assistDirectionToleranceRad = Units.degreesToRadians(50);
    public static final double assistMaximumDistanceMeters = Units.feetToMeters(5);

    public static final MoveToConfig moveToConfig = new MoveToConfig(
            PIDF.ofPD(4.5, 0.05),
            PIDF.ofPD(4.5, 0.05),
            0.02,
            0.1,
            Units.degreesToRadians(2),
            Units.degreesToRadians(10),
            15
    );

    public static final boolean disableDriving = false;
    public static final boolean disableGyro = false;
    static final boolean useHighFrequencyOdometry = true;

    static final double odometryPositionDeltaDiscardMeters = Units.inchesToMeters(8);

    public static final DriveConfig driveConfig = new DriveConfig(
            // TO TUNE WHEEL RADIUS: Place robot on carpet and use wheel radius characterization auto.
            // Output will be in console in AdvantageScope.
            Units.inchesToMeters(1.935948620917915),
            Units.inchesToMeters(19.25),
            Units.inchesToMeters(24.25),
            Units.inchesToMeters(36.5),
            Units.inchesToMeters(31.5),
            Units.inchesToMeters(-0.247776),
            PIDF.ofPD(3.5, 0),
            PIDF.ofPD(3, 0),
            PIDF.ofP(6),
            4.58
    );

    /**
     * FL, FR, BL, BR
     */
    public static final Translation2d[] moduleTranslations = new Translation2d[]{
            new Translation2d(driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0),
            new Translation2d(driveConfig.trackWidthMeters / 2.0, -driveConfig.trackLengthMeters / 2.0),
            new Translation2d(-driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0),
            new Translation2d(-driveConfig.trackWidthMeters / 2.0, -driveConfig.trackLengthMeters / 2.0)
    };

    public static final double drivebaseRadiusMeters = Math.hypot(driveConfig.trackWidthMeters / 2.0, driveConfig.trackLengthMeters / 2.0);

    /** Maximum angular velocity of the whole drivetrain if all drive motors/wheels are going at full speed. */
    public static final double maxAngularVelocityRadPerSec = driveConfig.maxVelocityMetersPerSec() / drivebaseRadiusMeters;

    public static final double joystickMaxAngularSpeedRadPerSec = Math.min(Units.degreesToRadians(500), maxAngularVelocityRadPerSec);
    public static final double joystickDriveDeadband = 0.05;

    static final ModuleConfig moduleConfig = switch (BuildConstants.mode) {
        // TO TUNE DRIVE GAINS: Use go forward at 1 m/s characterization auto and tune until you get 1 m/s.
        // After that, move to 2 m/s and make sure you get 2 m/s, and do the same thing for 3 m/s and 4 m/s.
        // TO TUNE TURN GAINS: Put robot on ground. Disable driving with the constant. Tune the PID with teleop.
        // TO TUNE DRIVE CURRENT LIMIT: Put the robot against a wall while having it on carpet and run the slip
        // current characterization auto. Output will be in console in AdvantageScope.
        case REAL, REPLAY -> new ModuleConfig(
                PIDF.ofPDSVA(
                        0.0, 0.0,
                        0.19, 0.125, 0.005
                ),
                PIDF.ofPD(5, 0.04),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN,
                true,
                false,
                false,
                120,
                60
        );
        case SIM -> new ModuleConfig(
                PIDF.ofPDSV(0.05, 0.0, 0.04075, 0.14117),
                PIDF.ofPD(10.0, 0.07),
                Mk4iGearRatios.L2,
                Mk4iGearRatios.TURN,
                true,
                false,
                false,
                77,
                60
        );
    };

    static ModuleIO[] createModuleIO() {
        return switch (BuildConstants.mode) {
            // TO CALIBRATE ABSOLUTE ENCODER OFFSETS:
            // 1. Put robot on robot cart
            // 2. Turn on coast override
            // 3. Open inputs in AdvantageScope. For each module (go in FL, FR, BL, BR order),
            // turn it forward and move the wheel so that the robot would be propelled forward
            // (the wheel itself will be moving backwards). If the drive position is getting
            // more NEGATIVE, rotate the wheel 180° and make sure that it is getting more
            // POSITIVE when you move the wheel as previously mentioned.
            // 4. Use a long aluminum tube to press up against the sides of the wheels and ensure
            // that they are pointing perfectly straight.
            // 5. For each module, copy the value in "/Inputs/Drive/Module<INDEX>/TurnAbsolutePositionRad"
            // to the absolute encoder offset parameter in the IO layer constructor.
            // Module order: FL, FR, BL, BR
            case REAL -> new ModuleIO[]{
                    new ModuleIOTalonFXSparkMaxCANcoder(1, 1, 5, 1.577),
                    new ModuleIOTalonFXSparkMaxCANcoder(2, 2, 6, 1.770),
                    new ModuleIOTalonFXSparkMaxCANcoder(3, 3, 7, 3.105),
                    new ModuleIOTalonFXSparkMaxCANcoder(4, 4, 8, -2.817),
            };
            case SIM -> new ModuleIO[]{
                    new ModuleIOSim(0),
                    new ModuleIOSim(1),
                    new ModuleIOSim(2),
                    new ModuleIOSim(3)
            };
            case REPLAY -> new ModuleIO[]{new ModuleIO(), new ModuleIO(), new ModuleIO(), new ModuleIO()};
        };
    }

    static GyroIO createGyroIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new GyroIOPigeon2(9);
            case SIM -> new GyroIOSim();
            case REPLAY -> new GyroIO();
        };
    }

    static AccelerometerIO createAccelerometerIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new AccelerometerIOroboRIO();
            case SIM -> new AccelerometerIOSim();
            case REPLAY -> new AccelerometerIO();
        };
    }

    public record MoveToConfig(
            PIDF linear,
            PIDF angular,
            double linearPositionToleranceMeters,
            double linearVelocityToleranceMetersPerSec,
            double angularPositionToleranceRad,
            double angularVelocityToleranceRadPerSec,
            double maxAccelerationMetersPerSecPerSec // Maximum acceleration of the robot during move to
    ) {
    }

    public record DriveConfig(
            double wheelRadiusMeters,
            double trackWidthMeters, // Measured from the center of the swerve wheels
            double trackLengthMeters,
            double bumperWidthMeters,
            double bumperLengthMeters,
            // Measured from bottom of frame rails (2x1s) to center of swerve wheels
            double bottomOfFrameRailsToCenterOfWheelsMeters,
            PIDF choreoFeedbackXY,
            PIDF choreoFeedbackOmega,
            PIDF headingOverrideGains,
            double maxVelocityMetersPerSec // Maximum velocity of the robot
    ) {
    }

    record ModuleConfig(
            PIDF driveGains,
            PIDF turnGains,
            double driveGearRatio,
            double turnGearRatio,
            boolean turnInverted,
            boolean driveInverted,
            boolean encoderInverted,
            int driveCurrentLimit, // AKA current that causes wheel slip
            int turnCurrentLimit
    ) {
    }

    private static class Mk4iGearRatios {
        public static final double L2 = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        public static final double L3 = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

        public static final double TURN = (150.0 / 7.0);
    }
}
