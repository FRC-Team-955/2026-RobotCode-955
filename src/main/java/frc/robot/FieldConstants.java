// MIT License
//
// Copyright (c) 2025-2026 Littleton Robotics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/FieldConstants.java
 * <p>
 * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/192
 * <p>
 * Contains information for location of field element and other useful reference points.
 * <p>
 * NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station
 */
@SuppressWarnings("SuspiciousNameCombination")
public class FieldConstants {
    // AprilTag related constants
    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Field dimensions
    public static final double fieldLength = aprilTagLayout.getFieldLength();
    public static final double fieldWidth = aprilTagLayout.getFieldWidth();

    /**
     * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
     */
    public static class LinesVertical {
        public static final double center = fieldLength / 2.0;
        public static final double starting = aprilTagLayout.getTagPose(26).orElseThrow().getX();
        public static final double allianceZone = starting;
        /**
         * Same as center of bump
         */
        public static final double hubCenter = aprilTagLayout.getTagPose(18).orElseThrow().getX();
        public static final double neutralZoneNear = center - Units.inchesToMeters(120);
        public static final double neutralZoneFar = center + Units.inchesToMeters(120);
        public static final double oppHubCenter = aprilTagLayout.getTagPose(5).orElseThrow().getX();
        public static final double oppAllianceZone = aprilTagLayout.getTagPose(10).orElseThrow().getX();
    }

    /**
     * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
     *
     * <p>NOTE: The field element start and end are always left to right from the perspective of the
     * alliance station
     */
    public static class LinesHorizontal {
        public static final double center = fieldWidth / 2.0;

        // Right of hub
        public static final double rightBumpStart = Hub.nearRightCorner.getY();
        public static final double rightBumpEnd = rightBumpStart - RightBump.width;
        public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
        public static final double rightTrenchOpenEnd = 0;

        // Left of hub
        public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
        public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
        public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
        public static final double leftTrenchOpenStart = fieldWidth;
    }

    /** Hub related constants */
    public static class Hub {
        // Dimensions
        public static final double width = Units.inchesToMeters(47.0);
        public static final double height =
                Units.inchesToMeters(72.0); // includes the catcher at the top
        public static final double innerWidth = Units.inchesToMeters(41.7);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        // Relevant reference points on alliance side
        public static final Translation3d topCenterPoint = new Translation3d(
                aprilTagLayout.getTagPose(18).orElseThrow().getX(),
                fieldWidth / 2.0,
                height
        );
        public static final Translation3d innerCenterPoint = new Translation3d(
                aprilTagLayout.getTagPose(18).orElseThrow().getX(),
                fieldWidth / 2.0,
                innerHeight
        );

        public static final Translation2d nearLeftCorner =
                new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner =
                new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d farLeftCorner =
                new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d farRightCorner =
                new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Relevant reference points on the opposite side
        public static final Translation3d oppTopCenterPoint = new Translation3d(
                aprilTagLayout.getTagPose(5).orElseThrow().getX(),
                fieldWidth / 2.0,
                height
        );
        public static final Translation2d oppNearLeftCorner =
                new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppNearRightCorner =
                new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d oppFarLeftCorner =
                new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppFarRightCorner =
                new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Hub faces
        public static final Pose2d nearFace = aprilTagLayout.getTagPose(26).orElseThrow().toPose2d();
        public static final Pose2d farFace = aprilTagLayout.getTagPose(20).orElseThrow().toPose2d();
        public static final Pose2d rightFace = aprilTagLayout.getTagPose(18).orElseThrow().toPose2d();
        public static final Pose2d leftFace = aprilTagLayout.getTagPose(21).orElseThrow().toPose2d();
    }

    /** Left Bump related constants */
    public static class LeftBump {
        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side
        public static final Translation2d oppNearLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
        public static final Translation2d oppFarLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Right Bump related constants */
    public static class RightBump {
        // Dimensions
        public static final double width = LeftBump.width;
        public static final double height = LeftBump.height;
        public static final double depth = LeftBump.depth;

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side
        public static final Translation2d oppNearLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
        public static final Translation2d oppFarLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Left Trench related constants */
    public static class LeftTrench {
        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft =
                new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
        public static final Translation3d openingTopRight =
                new Translation3d(LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

        // Relevant reference points on opposing side
        public static final Translation3d oppOpeningTopLeft =
                new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
        public static final Translation3d oppOpeningTopRight =
                new Translation3d(LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
    }

    public static class RightTrench {
        // Dimensions
        public static final double width = LeftTrench.width;
        public static final double depth = LeftTrench.depth;
        public static final double height = LeftTrench.height;
        public static final double openingWidth = LeftTrench.openingWidth;
        public static final double openingHeight = LeftTrench.openingHeight;

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft =
                new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
        public static final Translation3d openingTopRight =
                new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

        // Relevant reference points on opposing side
        public static final Translation3d oppOpeningTopLeft =
                new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
        public static final Translation3d oppOpeningTopRight =
                new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
    }

    /** Tower related constants */
    public static class Tower {
        // Dimensions
        public static final double width = Units.inchesToMeters(49.25);
        public static final double depth = Units.inchesToMeters(45.0);
        public static final double height = Units.inchesToMeters(78.25);
        public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
        public static final double frontFaceX = Units.inchesToMeters(43.51);

        public static final double uprightHeight = Units.inchesToMeters(72.1);

        // Rung heights from the floor
        public static final double lowRungHeight = Units.inchesToMeters(27.0);
        public static final double midRungHeight = Units.inchesToMeters(45.0);
        public static final double highRungHeight = Units.inchesToMeters(63.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint =
                new Translation2d(
                        frontFaceX, aprilTagLayout.getTagPose(31).orElseThrow().getY());
        public static final Translation2d leftUpright =
                new Translation2d(
                        frontFaceX,
                        (aprilTagLayout.getTagPose(31).orElseThrow().getY())
                                + innerOpeningWidth / 2
                                + Units.inchesToMeters(0.75));
        public static final Translation2d rightUpright =
                new Translation2d(
                        frontFaceX,
                        (aprilTagLayout.getTagPose(31).orElseThrow().getY())
                                - innerOpeningWidth / 2
                                - Units.inchesToMeters(0.75));

        // Relevant reference points on opposing side
        public static final Translation2d oppCenterPoint =
                new Translation2d(
                        fieldLength - frontFaceX,
                        aprilTagLayout.getTagPose(15).orElseThrow().getY());
        public static final Translation2d oppLeftUpright =
                new Translation2d(
                        fieldLength - frontFaceX,
                        (aprilTagLayout.getTagPose(15).orElseThrow().getY())
                                + innerOpeningWidth / 2
                                + Units.inchesToMeters(0.75));
        public static final Translation2d oppRightUpright =
                new Translation2d(
                        fieldLength - frontFaceX,
                        (aprilTagLayout.getTagPose(15).orElseThrow().getY())
                                - innerOpeningWidth / 2
                                - Units.inchesToMeters(0.75));
    }

    public static class Depot {
        // Dimensions
        public static final double width = Units.inchesToMeters(42.0);
        public static final double depth = Units.inchesToMeters(27.0);
        public static final double height = Units.inchesToMeters(1.125);
        public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

        // Relevant reference points on alliance side
        public static final Translation3d depotCenter =
                new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
        public static final Translation3d leftCorner =
                new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
        public static final Translation3d rightCorner =
                new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
    }

    public static class Outpost {
        // Dimensions
        public static final double width = Units.inchesToMeters(31.8);
        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
        public static final double height = Units.inchesToMeters(7.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint =
                new Translation2d(0, aprilTagLayout.getTagPose(29).orElseThrow().getY());
    }
}
