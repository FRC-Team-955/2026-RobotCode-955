package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.Optional;

public record DriveRequest(Type type, ChassisSpeeds value, Optional<Translation2d> centerOfRotation) {
    public enum Type {
        CHASSIS_SPEEDS,
        /** vxMetersPerSecond will be treated as the characterization voltage and given to all modules */
        CHARACTERIZATION,
        STOP,
        STOP_WITH_X
    }

    public static DriveRequest chassisSpeeds(ChassisSpeeds speeds) {
        return new DriveRequest(Type.CHASSIS_SPEEDS, speeds, Optional.empty());
    }

    public static DriveRequest chassisSpeeds(ChassisSpeeds speeds, Optional<Translation2d> centerOfRotation) {
        return new DriveRequest(Type.CHASSIS_SPEEDS, speeds, centerOfRotation);
    }

    public static DriveRequest characterization(double voltage) {
        return new DriveRequest(Type.CHARACTERIZATION, new ChassisSpeeds(voltage, 0.0, 0.0), Optional.empty());
    }

    public static DriveRequest stop() {
        return new DriveRequest(Type.STOP, new ChassisSpeeds(), Optional.empty());
    }

    public static DriveRequest stopWithX() {
        return new DriveRequest(Type.STOP_WITH_X, new ChassisSpeeds(), Optional.empty());
    }

}
