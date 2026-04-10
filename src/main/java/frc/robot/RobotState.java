package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.subsystems.drive.DriveConstants;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class RobotState implements Periodic {
    @Getter
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[]{
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            },
            new Pose2d()
    );

    private final Field2d field2d = new Field2d();

    @Setter
    private Supplier<Optional<Pose2d>> autoStartPoseSupplier = Optional::empty;
    private final FieldObject2d autoStartPoseObject = field2d.getObject("AutoStart");

    @Setter
    private Optional<Pose2d> moveToGoal = Optional.empty();
    private final FieldObject2d moveToGoalObject = field2d.getObject("MoveTo");

    @Setter
    private Optional<Pose2d[]> trajectory = Optional.empty();
    private final FieldObject2d trajectoryObject = field2d.getObject("Trajectory");

    @Setter
    private Optional<Pose2d> trajectorySample = Optional.empty();
    private final FieldObject2d trajectorySampleObject = field2d.getObject("TrajectorySample");

    @Setter
    private List<Pose3d> acceptedPoses = List.of();
    private final FieldObject2d[] acceptedPoseObjects = IntStream.range(0, 3)
            .mapToObj(n -> field2d.getObject("AcceptedPose" + n))
            .toArray(FieldObject2d[]::new);

    @Setter
    private List<Pose3d> rejectedPoses = List.of();
    private final FieldObject2d[] rejectedPoseObjects = IntStream.range(0, 3)
            .mapToObj(n -> field2d.getObject("RejectedPose" + n))
            .toArray(FieldObject2d[]::new);

    @Setter
    private Pose2d[] fuel = new Pose2d[0];
    private final FieldObject2d[] fuelObjects = IntStream.range(0, 3)
            .mapToObj(n -> field2d.getObject("Fuel" + n))
            .toArray(FieldObject2d[]::new);

    private final FieldObject2d[] uncertaintyRangeObjects = IntStream.range(0, 5)
            .mapToObj(n -> field2d.getObject("UncertaintyRange" + n))
            .toArray(FieldObject2d[]::new);

    @Getter
    @Setter
    private ChassisSpeeds measuredChassisSpeedsRobotRelative = new ChassisSpeeds();
    @Getter
    @Setter
    private ChassisSpeeds measuredChassisSpeedsFieldRelative = new ChassisSpeeds();

    /**
     * Field relative
     */
    @Getter
    @Setter
    private Translation2d filteredAccelerationMetersPerSecPerSec = new Translation2d();

    // The missile knows where it is at all times. It knows this because it
    // knows where it isn't. By subtracting where it is from where it isn't,
    // or where it isn't from where it is (whichever is greater), it obtains
    // a difference, or deviation. The guidance subsystem uses deviations to
    // generate corrective commands to drive the missile from a position where
    // it is to a position where it isn't, and arriving at a position where it
    // wasn't, it now is. Consequently, the position where it is, is now the
    // position that it wasn't, and it follows that the position that it was,
    // is now the position that it isn't. In the event that the position that
    // it is in is not the position that it wasn't, the system has acquired a
    // variation, the variation being the difference between where the missile
    // is, and where it wasn't. If variation is considered to be a significant
    // factor, it too may be corrected by the GEA. However, the missile must also
    // know where it was. The missile guidance computer scenario works as follows.
    // Because a variation has modified some of the information the missile has
    // obtained, it is not sure just where it is. However, it is sure where it
    // isn't, within reason, and it knows where it was. It now subtracts where
    // it should be from where it wasn't, or vice-versa, and by differentiating
    // this from the algebraic sum of where it shouldn't be, and where it was,
    // it is able to obtain the deviation and its variation, which is called error.
    private double poseUncertaintyLinearXMeters = 0.0;
    private double poseUncertaintyLinearYMeters = 0.0;
    @Getter
    private double poseUncertaintyAngularRad = 0.0;

    public double getPoseUncertaintyLinearMeters() {
        return Math.hypot(poseUncertaintyLinearXMeters, poseUncertaintyLinearYMeters);
    }

    /*
    private boolean lastInNeutralZone = false;
    private double lastIncreasedUncertaintyDueToBump = 0.0;
     */

    private static RobotState instance;

    public static synchronized RobotState get() {
        if (instance == null) {
            instance = new RobotState();
        }

        return instance;
    }

    private RobotState() {
        if (instance != null) {
            Util.error("Duplicate RobotState created");
        }
    }

    @Override
    public void periodicBeforeCommands() {
        // Increase uncertainty if we are moving
        final double chassisSpeedsLinearFactor = 0.06;
        poseUncertaintyLinearXMeters += Constants.loopPeriod * chassisSpeedsLinearFactor * Math.abs(measuredChassisSpeedsFieldRelative.vxMetersPerSecond);
        poseUncertaintyLinearYMeters += Constants.loopPeriod * chassisSpeedsLinearFactor * Math.abs(measuredChassisSpeedsFieldRelative.vyMetersPerSecond);
        // Gyro is generally pretty trustworthy
        final double chassisSpeedsAngularFactor = 0.0005;
        poseUncertaintyAngularRad += Constants.loopPeriod * chassisSpeedsAngularFactor * Math.abs(measuredChassisSpeedsFieldRelative.omegaRadiansPerSecond);

        // Increase uncertainty if we went over the bump
        Translation2d t = getTranslation();
        /*
        boolean inNeutralZone =
                t.getX() > FieldConstants.LinesVertical.hubCenter &&
                        t.getX() < FieldConstants.LinesVertical.oppHubCenter;
        if (
            // If we exited the neutral zone
                inNeutralZone != lastInNeutralZone &&
                        // If we are not going through the trench
                        t.getY() > FieldConstants.LinesHorizontal.rightTrenchOpenStart &&
                        t.getY() < FieldConstants.LinesHorizontal.leftTrenchOpenEnd &&
                        // If it has been a while since we increased uncertainty
                        Timer.getTimestamp() - lastIncreasedUncertaintyDueToBump > 10.0
        ) {
            // We went over the bump
            lastIncreasedUncertaintyDueToBump = Timer.getTimestamp();
            poseUncertaintyLinearXMeters += 0.25;
            poseUncertaintyLinearYMeters += 0.1;
            // Gyro is pretty trustworthy
        }
        lastInNeutralZone = inNeutralZone;
        Logger.recordOutput("RobotState/InNeutralZone", inNeutralZone);
        Logger.recordOutput("RobotState/LastIncreasedUncertaintyDueToBump", lastIncreasedUncertaintyDueToBump);
        */

        // Increase uncertainty if there is a large impact
        final double accelerationFactor = 0.001;
        // No abs needed because we are squaring acceleration
        poseUncertaintyLinearXMeters += Constants.loopPeriod * accelerationFactor * filteredAccelerationMetersPerSecPerSec.getX() * filteredAccelerationMetersPerSecPerSec.getX();
        poseUncertaintyLinearYMeters += Constants.loopPeriod * accelerationFactor * filteredAccelerationMetersPerSecPerSec.getY() * filteredAccelerationMetersPerSecPerSec.getY();

        // Prevent uncertainty from going negative
        if (poseUncertaintyLinearXMeters < 0.0) poseUncertaintyLinearXMeters = 0.0;
        if (poseUncertaintyLinearYMeters < 0.0) poseUncertaintyLinearYMeters = 0.0;
        if (poseUncertaintyAngularRad < 0.0) poseUncertaintyAngularRad = 0.0;

        // Log uncertainty
        if (BuildConstants.isSimOrReplay) {
            Logger.recordOutput("RobotState/PoseUncertainty/LinearX", poseUncertaintyLinearXMeters);
            Logger.recordOutput("RobotState/PoseUncertainty/LinearY", poseUncertaintyLinearYMeters);
            Logger.recordOutput("RobotState/PoseUncertainty/Linear", getPoseUncertaintyLinearMeters());
            Logger.recordOutput("RobotState/PoseUncertainty/Angular", poseUncertaintyAngularRad);
        }

        // Log uncertainty range
        // The Periodic execution order defined in Robot ensures that AprilTagVision will have already reduced uncertainty if there were any vision measurements
        Rotation2d r = getRotation();
        Pose2d[] uncertaintyRange = new Pose2d[]{
                new Pose2d(t.getX() + poseUncertaintyLinearXMeters, t.getY() + poseUncertaintyLinearYMeters, r),
                new Pose2d(t.getX() + poseUncertaintyLinearXMeters, t.getY() - poseUncertaintyLinearYMeters, r),
                new Pose2d(t.getX() - poseUncertaintyLinearXMeters, t.getY() - poseUncertaintyLinearYMeters, r),
                new Pose2d(t.getX() - poseUncertaintyLinearXMeters, t.getY() + poseUncertaintyLinearYMeters, r),
                new Pose2d(t, r.plus(new Rotation2d(poseUncertaintyAngularRad))),
                new Pose2d(t, r.minus(new Rotation2d(poseUncertaintyAngularRad)))
        };
        for (int i = 0; i < uncertaintyRangeObjects.length; i++) {
            if (i < uncertaintyRange.length) {
                uncertaintyRangeObjects[i].setPose(uncertaintyRange[i]);
            } else {
                Util.error("Number of uncertainty range poses and field objects don't match up");
            }
        }
        if (BuildConstants.isSimOrReplay) {
            Logger.recordOutput("RobotState/PoseUncertainty/Range", uncertaintyRange);
        }

        // Reset field objects for drive controllers so they only show up if they are explicitly set
        moveToGoal = Optional.empty();
        trajectory = Optional.empty();
        trajectorySample = Optional.empty();
    }

    @Override
    public void periodicAfterCommands() {
        field2d.setRobotPose(getPose());
        autoStartPoseSupplier.get().ifPresentOrElse(
                autoStartPoseObject::setPose,
                autoStartPoseObject::setPoses
        );
        moveToGoal.ifPresentOrElse(
                moveToGoalObject::setPose,
                moveToGoalObject::setPoses
        );
        trajectory.ifPresentOrElse(
                trajectoryObject::setPoses,
                trajectoryObject::setPoses
        );
        trajectorySample.ifPresentOrElse(
                trajectorySampleObject::setPose,
                trajectorySampleObject::setPoses
        );
        for (int i = 0; i < acceptedPoseObjects.length; i++) {
            if (i < acceptedPoses.size()) {
                acceptedPoseObjects[i].setPose(acceptedPoses.get(i).toPose2d());
            } else {
                acceptedPoseObjects[i].setPoses();
            }
        }
        for (int i = 0; i < rejectedPoseObjects.length; i++) {
            if (i < rejectedPoses.size()) {
                rejectedPoseObjects[i].setPose(rejectedPoses.get(i).toPose2d());
            } else {
                rejectedPoseObjects[i].setPoses();
            }
        }
        for (int i = 0; i < fuelObjects.length; i++) {
            if (i < fuel.length) {
                fuelObjects[i].setPose(fuel[i]);
            } else {
                fuelObjects[i].setPoses();
            }
        }
        SmartDashboard.putData("Field2d", field2d);
    }

    public void applyOdometryUpdate(
            double currentTimeSeconds,
            Rotation2d gyroAngle,
            SwerveModulePosition[] wheelPositions
    ) {
        poseEstimator.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            double linearStdDev,
            double angularStdDev
    ) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        // Decrease uncertainty based on std devs
        final double stdDevLinearFactor = 0.1;
        // Make X and Y converge at roughly the same rate by making decrease proportional to uncertainty
        poseUncertaintyLinearXMeters -= poseUncertaintyLinearXMeters * stdDevLinearFactor / linearStdDev;
        poseUncertaintyLinearYMeters -= poseUncertaintyLinearYMeters * stdDevLinearFactor / linearStdDev;
        // Gyro is generally pretty trustworthy
        poseUncertaintyAngularRad -= 0.0005 / angularStdDev;
    }

    public Optional<Pose2d> getPoseAtTimestamp(double timestampSeconds) {
        return poseEstimator.sampleAt(timestampSeconds);
    }

    @AutoLogOutput(key = "RobotState/Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
        if (BuildConstants.isSim) {
            SimManager.get().driveSimulation.setSimulationWorldPose(pose);
        }
    }

    public Command setPose(Supplier<Pose2d> pose) {
        return Commands
                .runOnce(() -> setPose(pose.get()))
                .ignoringDisable(true);
    }

    public Command resetRotation() {
        return setPose(() -> new Pose2d(getTranslation(), new Rotation2d()));
    }

    public boolean isAtPoseWithTolerance(Pose2d desiredPose, double linearToleranceMeters, double angularToleranceRad) {
        Transform2d relative = new Transform2d(desiredPose, getPose());
        return Math.abs(relative.getTranslation().getNorm()) < linearToleranceMeters
                && Math.abs(MathUtil.angleModulus(relative.getRotation().getRadians())) < angularToleranceRad;
    }

    public boolean isMeasuredChassisSpeedsBelowTolerance(double linearToleranceMetersPerSec, double angularToleranceRadPerSec) {
        return Math.hypot(getMeasuredChassisSpeedsFieldRelative().vxMetersPerSecond, getMeasuredChassisSpeedsFieldRelative().vyMetersPerSecond) < linearToleranceMetersPerSec
                && Math.abs(getMeasuredChassisSpeedsFieldRelative().omegaRadiansPerSecond) < angularToleranceRadPerSec;
    }

    public boolean isInTrench(Translation2d t) {
        // make each bounding box larger so that the hood has time to move down before going under the trench
        double adjustmentMetersPositiveX = 0.55 + Math.max(0.0, getMeasuredChassisSpeedsFieldRelative().vxMetersPerSecond) * 0.5;
        double adjustmentMetersNegativeX = 0.55 + Math.min(0.0, getMeasuredChassisSpeedsFieldRelative().vxMetersPerSecond) * -0.5;
        if (BuildConstants.isSimOrReplay)
            Logger.recordOutput(
                    "RobotState/TrenchChecks",
                    new Pose2d(new Translation2d(FieldConstants.LinesVertical.neutralZoneNear + adjustmentMetersNegativeX, 0.6), new Rotation2d()),
                    new Pose2d(new Translation2d(FieldConstants.LinesVertical.neutralZoneFar - adjustmentMetersPositiveX, 0.6), new Rotation2d()),
                    new Pose2d(new Translation2d(FieldConstants.LinesVertical.allianceZone - adjustmentMetersPositiveX, 0.6), new Rotation2d()),
                    new Pose2d(new Translation2d(FieldConstants.LinesVertical.oppAllianceZone + adjustmentMetersNegativeX, 0.6), new Rotation2d())
            );
        boolean inNeutralZone = t.getX() > FieldConstants.LinesVertical.neutralZoneNear + adjustmentMetersNegativeX &&
                t.getX() < FieldConstants.LinesVertical.neutralZoneFar - adjustmentMetersPositiveX;
        boolean inAllianceZone = t.getX() < FieldConstants.LinesVertical.allianceZone - adjustmentMetersPositiveX ||
                t.getX() > FieldConstants.LinesVertical.oppAllianceZone + adjustmentMetersNegativeX;
        boolean inLeftTrench = t.getY() > FieldConstants.LinesHorizontal.leftTrenchOpenEnd;
        boolean inRightTrench = t.getY() < FieldConstants.LinesHorizontal.rightTrenchOpenStart;
        return !inNeutralZone && !inAllianceZone && (inLeftTrench || inRightTrench);
    }

    public Pose3d robotPoseMec() {
        return new Pose3d(getPose())
                .transformBy(new Transform3d(
                        new Translation3d(0.0, 0.0, driveConfig.bottomOfFrameRailsToCenterOfWheelsMeters() + driveConfig.wheelRadiusMeters()),
                        new Rotation3d()
                ));
    }
}
