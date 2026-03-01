package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.network.LoggedTunableNumber;
import frc.lib.network.LoggedTunablePIDF;
import frc.robot.BuildConstants;
import lombok.With;

public class DriveConstants {
    public static final double assistDirectionToleranceRad = Units.degreesToRadians(50);
    public static final double assistMaximumDistanceMeters = Units.feetToMeters(5);

    public static final LoggedTunablePIDF choreoFeedbackXY = new LoggedTunablePIDF("Drive/ChoreoFeedbackXY").withP(3.5);
    public static final LoggedTunablePIDF choreoFeedbackOmega = new LoggedTunablePIDF("Drive/ChoreoFeedbackOmega").withP(3);
    public static final LoggedTunablePIDF headingOverrideGains = new LoggedTunablePIDF("Drive/HeadingOverrideGains").withP(4);
    public static final LoggedTunablePIDF assistGains = new LoggedTunablePIDF("Drive/AssistY").withP(2.0);

    public static final boolean disableDriving = false;
    public static final boolean disableGyro = false;
    static final boolean useHighFrequencyOdometry = true;

    static final double odometryPositionDeltaDiscardMeters = Units.inchesToMeters(8);

    public static final DriveConfig driveConfig = new DriveConfig(
            // TO TUNE WHEEL RADIUS: Place robot on carpet and use wheel radius characterization auto.
            // Output will be in console in AdvantageScope.
            // KEEP SYNCED WITH shooting_regression.py
            Units.inchesToMeters(1.91443),
            Units.inchesToMeters(19.25),
            Units.inchesToMeters(24.25),
            Units.inchesToMeters(36.5),
            Units.inchesToMeters(31.5),
            // KEEP SYNCED WITH shooting_regression.py
            Units.inchesToMeters(-0.247776),
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

    public static final MoveToConstraints defaultMoveToConstraints = new MoveToConstraints(
            new LoggedTunableNumber("Drive/MoveTo/MaxLinearVelocity", driveConfig.maxVelocityMetersPerSec()),
            new LoggedTunableNumber("Drive/MoveTo/MaxLinearAcceleration", 15.0),
            new LoggedTunableNumber("Drive/MoveTo/MaxAngularVelocity", maxAngularVelocityRadPerSec),
            new LoggedTunableNumber("Drive/MoveTo/MaxAngularAcceleration", 30.0),
            false,
            false
    );

    public static final MoveToConfig moveToConfig = new MoveToConfig(
            new LoggedTunablePIDF("Drive/MoveTo/Linear").withP(4.5).withD(0.05),
            new LoggedTunableNumber("Drive/MoveTo/LinearPositionTolerance", 0.02),
            new LoggedTunableNumber("Drive/MoveTo/LinearVelocityToleranceMeters", 0.1),
            new LoggedTunablePIDF("Drive/MoveTo/Angular").withP(4.5).withD(0.05),
            new LoggedTunableNumber("Drive/MoveTo/AngularPositionTolerance", Units.degreesToRadians(2)),
            new LoggedTunableNumber("Drive/MoveTo/AngularVelocityTolerance", Units.degreesToRadians(10))
    );

    public static final double joystickMaxAngularSpeedRadPerSec = Math.min(Units.degreesToRadians(500), maxAngularVelocityRadPerSec);
    public static final double joystickDriveDeadband = 0.05;

    static final ModuleConfig moduleConfig = switch (BuildConstants.mode) {
        // TO TUNE DRIVE GAINS: Use go forward at 1 m/s characterization auto and tune until you get 1 m/s.
        // After that, move to 2 m/s and make sure you get 2 m/s, and do the same thing for 3 m/s and 4 m/s.
        // TO TUNE TURN GAINS: Put robot on ground. Disable driving with the constant. Tune the PID with teleop.
        // TO TUNE DRIVE CURRENT LIMIT: Put the robot against a wall while having it on carpet and run the slip
        // current characterization auto. Output will be in console in AdvantageScope.
        case REAL, REPLAY -> new ModuleConfig(
                new LoggedTunablePIDF("Drive/DriveGains")
                        .withS(0.19, StaticFeedforwardSignValue.UseVelocitySign)
                        .withV(0.125)
                        .withA(0.005),
                new LoggedTunablePIDF("Drive/TurnGains")
                        .withP(5)
                        .withD(0.04),
                true,
                false,
                false,
                75,
                60
        );
        case SIM -> new ModuleConfig(
                new LoggedTunablePIDF("Drive/DriveGains")
                        .withS(0.04075, StaticFeedforwardSignValue.UseClosedLoopSign)
                        .withV(0.14117),
                new LoggedTunablePIDF("Drive/TurnGains")
                        .withP(10.0)
                        .withD(0.07),
                true,
                false,
                false,
                77,
                60
        );
    };

    static final GearRatioConfig[] gearRatioConfigs = new GearRatioConfig[]{
            // Module order: FL, FR, BL, BR
            new GearRatioConfig(Mk4GearRatios.L2_PLUS, Mk4GearRatios.MK4I_TURN),
            new GearRatioConfig(Mk4GearRatios.L2_PLUS, Mk4GearRatios.MK4I_TURN),
            new GearRatioConfig(Mk4GearRatios.L2_PLUS, Mk4GearRatios.MK4N_TURN),
            new GearRatioConfig(Mk4GearRatios.L2_PLUS, Mk4GearRatios.MK4N_TURN),
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
                    new ModuleIOTalonFXSparkMaxCANcoder(0, 2, 1, 9, -1.609),
                    new ModuleIOTalonFXSparkMaxCANcoder(1, 4, 7, 10, 3.12),
                    new ModuleIOTalonFXSparkMaxCANcoder(2, 6, 5, 11, -0.589),
                    new ModuleIOTalonFXSparkMaxCANcoder(3, 8, 3, 12, 2.755),
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
            case REAL -> new GyroIOPigeon2(18);
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

    @With
    public record MoveToConstraints(
            LoggedTunableNumber maxLinearVelocityMetersPerSec,
            LoggedTunableNumber maxLinearAccelerationMetersPerSecPerSec,
            LoggedTunableNumber maxAngularVelocityRadPerSec,
            LoggedTunableNumber maxAngularAccelerationRadPerSecPerSec,
            boolean applyAimingFeedforward,
            boolean fullSpeed
    ) {
    }

    public record MoveToConfig(
            LoggedTunablePIDF linearGains,
            LoggedTunableNumber linearPositionToleranceMeters,
            LoggedTunableNumber linearVelocityToleranceMetersPerSec,
            LoggedTunablePIDF angularGains,
            LoggedTunableNumber angularPositionToleranceRad,
            LoggedTunableNumber angularVelocityToleranceRadPerSec
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
            double maxVelocityMetersPerSec // Maximum velocity of module
    ) {
    }

    record ModuleConfig(
            LoggedTunablePIDF driveGains,
            LoggedTunablePIDF turnGains,
            boolean turnInverted,
            boolean driveInverted,
            boolean encoderInverted,
            int driveCurrentLimit, // AKA current that causes wheel slip
            int turnCurrentLimit
    ) {
    }

    record GearRatioConfig(
            double driveGearRatio,
            double turnGearRatio
    ) {}

    private static class Mk4GearRatios {
        public static final double L2 = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        public static final double L2_PLUS = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);

        public static final double MK4I_TURN = (150.0 / 7.0);
        public static final double MK4N_TURN = 18.75;
    }
}
