package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record DriveRequest(Type type, ChassisSpeeds value) {
    public enum Type {
        CHASSIS_SPEEDS,
        /** vxMetersPerSecond will be treated as the characterization voltage and given to all modules */
        CHARACTERIZATION,
        STOP,
    }

    public static DriveRequest chassisSpeeds(ChassisSpeeds speeds) {
        return new DriveRequest(Type.CHASSIS_SPEEDS, speeds);
    }

    public static DriveRequest characterization(double voltage) {
        return new DriveRequest(Type.CHARACTERIZATION, new ChassisSpeeds(voltage, 0.0, 0.0));
    }

    public static DriveRequest stop() {
        return new DriveRequest(Type.STOP, new ChassisSpeeds());
    }
}
