package frc.robot.subsystems.superstructure.flywheel;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.motor.MotorIOTalonFX;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

public class FlywheelIOTalonFX extends FlywheelIO {
    private static final int shootCurrentLimitAmps = 120;
    private static final int spinupCurrentLimitAmps = 20;

    private final MotorIOTalonFX leader;
    private final MotorIOTalonFX follower;

    public FlywheelIOTalonFX(
            int leaderCanID,
            int followerCanID,
            boolean leaderInverted,
            MotorAlignmentValue motorAlignment
    ) {
        leader = new MotorIOTalonFX(
                leaderCanID,
                leaderInverted,
                NeutralModeValue.Coast,
                shootCurrentLimitAmps,
                FlywheelConstants.gearRatio,
                null,
                FlywheelConstants.velocityGains
        );

        follower = new MotorIOTalonFX(
                followerCanID,
                false,
                NeutralModeValue.Coast,
                shootCurrentLimitAmps,
                FlywheelConstants.gearRatio,
                null,
                null
        );
        follower.setFollow(leader, motorAlignment);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        leader.updateInputs(inputs.leader);
        follower.updateInputs(inputs.follower);
    }

    @Override
    public void setVelocityPIDF(LoggedTunablePIDF newGains) {
        leader.setVelocityPIDF(newGains);
    }

    @Override
    public void setVelocityRequest(double setpointRadPerSec) {
        leader.setRequest(RequestType.VelocityRadPerSec, setpointRadPerSec);
    }

    @Override
    public void setStopRequest() {
        leader.setRequest(RequestType.VoltageVolts, 0);
    }

    @Override
    public void setCurrentLimit(FlywheelCurrentLimitMode mode) {
        System.out.println("Setting flywheel current limit to " + mode);
        double currentLimitAmps = switch (mode) {
            case SHOOT -> shootCurrentLimitAmps;
            case SPINUP -> spinupCurrentLimitAmps;
        };
        leader.setCurrentLimit(currentLimitAmps);
        follower.setCurrentLimit(currentLimitAmps);
    }
}
