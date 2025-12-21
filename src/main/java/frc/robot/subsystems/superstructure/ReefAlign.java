package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.lib.AllianceBasedPose2d;
import frc.lib.Util;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.subsystems.elevator.Elevator;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Comparator;

public class ReefAlign {
    private static final RobotState robotState = RobotState.get();

    private static final double distanceCenterOfReefToBranchMeters = Units.inchesToMeters(6.5);

    private static final Transform2d initialAlignStartOffset = new Transform2d(1.5, 0, new Rotation2d());
    private static final Transform2d initialAlignEndOffset = new Transform2d(0.5, 0, new Rotation2d());
    private static final double initialAlignDistYForStartMeters = 1.5;
    private static final double initialAlignDistYOffset = 0.5;
    private static final double initialAlignDistXForFullAngle = 0.5;

    private static final double finalAlignAngularDiffForInitialRad = Units.degreesToRadians(30);
    private static final double finalAlignElevatorPercentageMultiplier = 1.25;

    // Distance at which to start raising the elevator
    public static final double elevatorRaiseDistanceMeters = 1.5;
    // Distance at which elevator cannot be raised
    public static final double elevatorStowDistanceXMeters = initialAlignEndOffset.getX() / 2.0;
    public static final double elevatorStowDistanceYMeters = 0.3;
    public static final double elevatorRaiseAngularToleranceRad = Units.degreesToRadians(60);

    public static final double alignLinearToleranceMeters = 0.04;
    public static final double alignAngularToleranceRad = Units.degreesToRadians(4);
    public static final double alignLinearToleranceMetersPerSecond = 0.02;
    public static final double alignAngularToleranceRadPerSecond = Units.degreesToRadians(8);

    public static boolean canRaiseElevator(ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        Pose2d finalAlign = getFinalAlignPose(reefZoneSide, localReefSide);
        Transform2d relative = new Transform2d(finalAlign, robotState.getPose());

        boolean distanceMet = relative.getTranslation().getNorm() < elevatorRaiseDistanceMeters
                && relative.getX() > elevatorStowDistanceXMeters
                && Math.abs(relative.getY()) < elevatorStowDistanceYMeters;

        boolean rotationMet = Math.abs(MathUtil.angleModulus(relative.getRotation().getRadians())) < elevatorRaiseAngularToleranceRad;

        Logger.recordOutput("Superstructure/ReefAlign/ElevatorDistanceMet", distanceMet);
        Logger.recordOutput("Superstructure/ReefAlign/ElevatorRotationMet", rotationMet);

        return distanceMet && rotationMet;
    }

    public static boolean atFinalAlign(ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        Pose2d finalAlign = getFinalAlignPose(reefZoneSide, localReefSide);
        boolean positionMet = robotState.isAtPoseWithTolerance(finalAlign, alignLinearToleranceMeters, alignAngularToleranceRad);
        boolean velocityMet = robotState.isMeasuredChassisSpeedsBelowTolerance(alignLinearToleranceMetersPerSecond, alignAngularToleranceRadPerSecond);
        Logger.recordOutput("Superstructure/ReefAlign/PositionMet", positionMet);
        Logger.recordOutput("Superstructure/ReefAlign/VelocityMet", velocityMet);
        return positionMet && velocityMet;
    }

    /**
     * Checks whether moving to the side will intersect with the reef, and refuses to do so if it does.
     * A bit rudimentary and imperfect, but definitely plenty good
     */
    public static boolean isAlignable(Pose2d currentPose, ReefZoneSide reefZoneSide) {
        return currentPose.relativeTo(reefZoneSide.getAdjustedAprilTagPose()).getX() > -0.15;
    }

    public static Pose2d getFinalAlignPose(ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        return reefZoneSide.getAdjustedAprilTagPose().plus(localReefSide.adjust);
    }

    public static Pose2d getAlignPose(double elevatorPercentage, ReefZoneSide reefZoneSide, LocalReefSide localReefSide) {
        Pose2d finalAlign = getFinalAlignPose(reefZoneSide, localReefSide);

        // Start by calculating the initial align
        Pose2d initialStart = finalAlign.plus(initialAlignStartOffset);
        // We want to transform the end offset towards the robot to prevent
        // it from going straight forward into the reef if it doesn't need to
        // The angle from current to final gets unstable when we get really close to the point; at this point
        // we are close enough that we should just use the normal final align rotation
        // We also want to use the normal final align rotation if we haven't raised the elevator because
        // it will get stuck otherwise
//        Rotation2d angleForTransformation = finalAlign.getRotation().interpolate(
//                robotState.getPose().getTranslation().minus(finalAlign.getTranslation()).getAngle(),
//                Math.abs(new Transform2d(finalAlign, robotState.getPose()).getX()) / initialAlignDistXForFullAngle
//        );
//        Pose2d initialEnd = new Pose2d(
//                new Pose2d(finalAlign.getTranslation(), angleForTransformation)
//                        .transformBy(initialAlignEndOffset)
//                        .getTranslation(),
//                finalAlign.getRotation()
//        );
        Pose2d initialEnd = finalAlign.plus(initialAlignEndOffset);

        // Interpolate to initialBase based on y distance (left/right distance)
        double initialDistY = Math.abs(new Transform2d(initialEnd, robotState.getPose()).getY()) - initialAlignDistYOffset;
        // No clamping needed, Pose2d.interpolate will handle it
        double initialInterp = 1.0 - (initialDistY / initialAlignDistYForStartMeters);
        Pose2d initial = initialStart.interpolate(initialEnd, initialInterp);

        // Now interpolate from initial to final end based on elevator percentage
        // Fully at final when 100% raised, fully at initial when 0% raised
        if (elevatorPercentage >= 0.95) {
            elevatorPercentage = 1.0;
        }
        elevatorPercentage *= finalAlignElevatorPercentageMultiplier;
        elevatorPercentage = MathUtil.clamp(elevatorPercentage, 0.0, 1.0);

        return initial.interpolate(finalAlign, elevatorPercentage);
    }

    public static final int[] reefTagIds = {
            // Blue
            17,
            18,
            19,
            20,
            21,
            22,

            // Red
            6,
            7,
            8,
            9,
            10,
            11
    };

    private static AllianceBasedPose2d getAdjustedReefAprilTagPose(int aprilTagOffset) {
        return new AllianceBasedPose2d(
                // Blue
                AlignHelpers.getAprilTagPose(22 - ((aprilTagOffset + 3) % 6)).plus(AlignHelpers.bumperOffset),

                // Red
                AlignHelpers.getAprilTagPose(aprilTagOffset + 6).plus(AlignHelpers.bumperOffset)
        );
    }

    private static final LoggedTunableNumber velocityLookaheadSeconds = new LoggedTunableNumber("ReefAlign/VelocityLookaheadSeconds", 0.4);
    private static final Transform2d reefSideAngleOffset = new Transform2d(0.25, 0, new Rotation2d());

    private static ReefZoneSide closestReefSideToPose(Pose2d currentPose) {
        return Arrays.stream(ReefZoneSide.values())
                .min(Comparator.comparing(
                        side -> currentPose.getTranslation().getDistance(side.getAdjustedAprilTagPose().getTranslation())
                ))
                .orElse(ReefZoneSide.LeftFront);
    }

    public static ReefZoneSide determineClosestReefSide(ChassisSpeeds joystickSetpointFieldRelative) {
        Pose2d currentPose = robotState.getPose();

        ReefZoneSide closestSide = closestReefSideToPose(currentPose);
        boolean lookahead = false;

        if (joystickSetpointFieldRelative.vxMetersPerSecond != 0 || joystickSetpointFieldRelative.vyMetersPerSecond != 0) {
            // Get poses of sides left and right of closest, with an offset
            Pose2d leftTagPose = ReefZoneSide.fromOrdinal(closestSide.ordinal() - 1).getAdjustedAprilTagPose()
                    .plus(reefSideAngleOffset);
//        Logger.recordOutput("Superstructure/ClosestReefSide/LeftTagPose", leftTagPose);
            Pose2d rightTagPose = ReefZoneSide.fromOrdinal(closestSide.ordinal() + 1).getAdjustedAprilTagPose()
                    .plus(reefSideAngleOffset);
//        Logger.recordOutput("Superstructure/ClosestReefSide/RightTagPose", rightTagPose);

            // Get angles to left and right
            Rotation2d robotToLeft = leftTagPose.relativeTo(currentPose).getTranslation().getAngle();
//            Logger.recordOutput("Superstructure/ClosestReefSide/RobotToLeft", robotToLeft);
            Rotation2d robotToRight = rightTagPose.relativeTo(currentPose).getTranslation().getAngle();
//            Logger.recordOutput("Superstructure/ClosestReefSide/RobotToRight", robotToRight);

            // Get the joystick angle relative to the closest tag
            Rotation2d joystickAngle = new Rotation2d(joystickSetpointFieldRelative.vxMetersPerSecond, joystickSetpointFieldRelative.vyMetersPerSecond);
//            Logger.recordOutput("Superstructure/ClosestReefSide/JoystickAngle", joystickAngle);

            // Lookahead if the angle of the joystick setpoint angle is more CCW (positive) than the left pose or more CW (negative) than the right pose
            // We have to make all rotations relative to the center tag so that CCW and CW are actually positive and negative and the inequalities work out
            Pose2d centerTagPose = closestSide.getAdjustedAprilTagPose();
            Rotation2d relativeToCenterTag = centerTagPose.getRotation().unaryMinus().plus(Rotation2d.k180deg);
            lookahead = joystickAngle.rotateBy(relativeToCenterTag).getRadians() > robotToLeft.rotateBy(relativeToCenterTag).getRadians() ||
                    joystickAngle.rotateBy(relativeToCenterTag).getRadians() < robotToRight.rotateBy(relativeToCenterTag).getRadians();
        }
        Pose2d currentPoseWithLookahead = currentPose.exp(ChassisSpeeds.fromFieldRelativeSpeeds(joystickSetpointFieldRelative, currentPose.getRotation()).toTwist2d(velocityLookaheadSeconds.get()));
//        Logger.recordOutput("Superstructure/ClosestReefSide/Lookahead", currentPoseWithLookahead);
        if (lookahead) {
            return closestReefSideToPose(currentPoseWithLookahead);
        } else {
            return closestSide;
        }
    }

    // Distance at which to raise the elevator when descoring
    public static final double descoreElevatorRaiseDistanceMeters = 1.0;
    private static final double descoreLinearToleranceMeters = 0.1;
    private static final double descoreAngularToleranceRad = Units.degreesToRadians(10);

    public static Pose2d getDescoreAlignPose(ReefZoneSide reefZoneSide) {
        return ReefAlign.getFinalAlignPose(reefZoneSide, LocalReefSide.Middle);
    }

    public static boolean descoreCanRaiseElevator(ReefZoneSide reefZoneSide) {
        Pose2d alignPose = getDescoreAlignPose(reefZoneSide);
        return robotState.isAtPoseWithTolerance(
                alignPose,
                descoreElevatorRaiseDistanceMeters,
                descoreAngularToleranceRad
        );
    }

    public static boolean descoreIsAligned(ReefZoneSide reefZoneSide) {
        Pose2d alignPose = getDescoreAlignPose(reefZoneSide);
        return robotState.isAtPoseWithTolerance(
                alignPose,
                descoreLinearToleranceMeters,
                descoreAngularToleranceRad
        );
    }

    @RequiredArgsConstructor
    public enum ReefZoneSide {
        LeftFront(getAdjustedReefAprilTagPose(0), Elevator.Goal.DESCORE_L2),
        MiddleFront(getAdjustedReefAprilTagPose(1), Elevator.Goal.DESCORE_L3),
        RightFront(getAdjustedReefAprilTagPose(2), Elevator.Goal.DESCORE_L2),
        RightBack(getAdjustedReefAprilTagPose(3), Elevator.Goal.DESCORE_L3),
        MiddleBack(getAdjustedReefAprilTagPose(4), Elevator.Goal.DESCORE_L2),
        LeftBack(getAdjustedReefAprilTagPose(5), Elevator.Goal.DESCORE_L3);

        private final AllianceBasedPose2d adjustedAprilTagPoses;
        public final Elevator.Goal algaeDescoringElevatorGoal;

        public static ReefZoneSide fromOrdinal(int ordinal) {
            var values = ReefZoneSide.values();
            return values[Util.positiveModulus(ordinal, values.length)];
        }

        public Pose2d getAdjustedAprilTagPose() {
            return adjustedAprilTagPoses.get();
        }
    }

    @RequiredArgsConstructor
    public enum LocalReefSide {
        Left(new Transform2d(0, -distanceCenterOfReefToBranchMeters, new Rotation2d())),
        Right(new Transform2d(0, distanceCenterOfReefToBranchMeters, new Rotation2d())),
        Middle(new Transform2d()),
        ;

        public final Transform2d adjust;
    }
}
