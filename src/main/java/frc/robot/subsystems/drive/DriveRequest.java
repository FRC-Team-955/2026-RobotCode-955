package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record DriveRequest(Type type, ChassisSpeeds value) {
    public enum Type {
        /** ChassisSpeeds will be optimized with the setpoint generator (unless disabled) before being fed to modules */
        CHASSIS_SPEEDS_OPTIMIZED,
        /** ChassisSpeeds will be directly fed to modules */
        CHASSIS_SPEEDS_DIRECT,
        /** vxMetersPerSecond will be treated as the characterization voltage and given to all modules */
        CHARACTERIZATION,
        STOP,
    }

    public static DriveRequest chassisSpeedsOptimized(ChassisSpeeds speeds) {
        return new DriveRequest(Type.CHASSIS_SPEEDS_OPTIMIZED, speeds);
    }

    public static DriveRequest chassisSpeedsDirect(ChassisSpeeds speeds) {
        return new DriveRequest(Type.CHASSIS_SPEEDS_DIRECT, speeds);
    }

    public static DriveRequest characterization(double voltage) {
        return new DriveRequest(Type.CHARACTERIZATION, new ChassisSpeeds(voltage, 0.0, 0.0));
    }

    public static DriveRequest stop() {
        return new DriveRequest(Type.STOP, new ChassisSpeeds());
    }
}
