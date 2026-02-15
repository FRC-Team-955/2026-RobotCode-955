package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.Periodic;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.ModuleIOSim;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

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

    /**
     * Field relative
     */
    @Getter
    @Setter
    private ChassisSpeeds measuredChassisSpeeds = new ChassisSpeeds();

    /**
     * Field relative
     */
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
    private double poseUncertaintyAngularRad = 0.0;

    private boolean lastInNeutralZone = false;
    private double lastIncreasedUncertaintyDueToBump = 0.0;

    private static RobotState instance;

    public static RobotState get() {
        synchronized (RobotState.class) {
            if (instance == null) {
                instance = new RobotState();
            }
        }

        return instance;
    }

    private RobotState() {
    }

    @Override
    public void periodicBeforeCommands() {
        // Increase uncertainty if we are moving
        final double chassisSpeedsLinearFactor = 0.06;
        poseUncertaintyLinearXMeters += Constants.loopPeriod * chassisSpeedsLinearFactor * Math.abs(measuredChassisSpeeds.vxMetersPerSecond);
        poseUncertaintyLinearYMeters += Constants.loopPeriod * chassisSpeedsLinearFactor * Math.abs(measuredChassisSpeeds.vyMetersPerSecond);
        // Gyro is generally pretty trustworthy
        final double chassisSpeedsAngularFactor = 0.0005;
        poseUncertaintyAngularRad += Constants.loopPeriod * chassisSpeedsAngularFactor * Math.abs(measuredChassisSpeeds.omegaRadiansPerSecond);

        // Increase uncertainty if we went over the bump
        Translation2d t = getTranslation();
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

        // Increase uncertainty if there is a large impact
        final double accelerationFactor = 0.001;
        // No abs needed because we are squaring acceleration
        poseUncertaintyLinearXMeters += Constants.loopPeriod * accelerationFactor * filteredAccelerationMetersPerSecPerSec.getX() * filteredAccelerationMetersPerSecPerSec.getX();
        poseUncertaintyLinearYMeters += Constants.loopPeriod * accelerationFactor * filteredAccelerationMetersPerSecPerSec.getY() * filteredAccelerationMetersPerSecPerSec.getY();

        // Prevent uncertainty from going negative
        if (poseUncertaintyLinearXMeters < 0.0) poseUncertaintyLinearXMeters = 0.0;
        if (poseUncertaintyLinearYMeters < 0.0) poseUncertaintyLinearYMeters = 0.0;
        if (poseUncertaintyAngularRad < 0.0) poseUncertaintyAngularRad = 0.0;

        // Log uncertainty range
        // The Periodic execution order defined in Robot ensures that AprilTagVision will have already reduced uncertainty if there were any vision measurements
        Rotation2d r = getRotation();
        Logger.recordOutput(
                "RobotState/PoseUncertaintyRange",
                new Pose2d(t.getX() + poseUncertaintyLinearXMeters, t.getY() + poseUncertaintyLinearYMeters, r),
                new Pose2d(t.getX() + poseUncertaintyLinearXMeters, t.getY() - poseUncertaintyLinearYMeters, r),
                new Pose2d(t.getX() - poseUncertaintyLinearXMeters, t.getY() - poseUncertaintyLinearYMeters, r),
                new Pose2d(t.getX() - poseUncertaintyLinearXMeters, t.getY() + poseUncertaintyLinearYMeters, r),
                new Pose2d(t, r.plus(new Rotation2d(poseUncertaintyAngularRad))),
                new Pose2d(t, r.minus(new Rotation2d(poseUncertaintyAngularRad)))
        );
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
        if (BuildConstants.mode == BuildConstants.Mode.SIM) {
            ModuleIOSim.driveSimulation.setSimulationWorldPose(pose);
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
        return Math.hypot(getMeasuredChassisSpeeds().vxMetersPerSecond, getMeasuredChassisSpeeds().vyMetersPerSecond) < linearToleranceMetersPerSec
                && Math.abs(getMeasuredChassisSpeeds().omegaRadiansPerSecond) < angularToleranceRadPerSec;
    }
}
