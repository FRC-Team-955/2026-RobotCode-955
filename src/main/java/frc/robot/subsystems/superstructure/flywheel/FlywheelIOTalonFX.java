package frc.robot.subsystems.superstructure.flywheel;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.motor.MotorIOTalonFX;
import frc.lib.motor.RequestType;
import frc.lib.network.LoggedTunablePIDF;

import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.gearRatio;
import static frc.robot.subsystems.superstructure.flywheel.FlywheelConstants.velocityGains;

public class FlywheelIOTalonFX extends FlywheelIO {
    private static final int currentLimitAmps = 120;

    private final MotorIOTalonFX leader;
    private final MotorIOTalonFX follower;

    public FlywheelIOTalonFX(
            int leaderCanID,
            int followerCanID,
            boolean leaderInverted,
            MotorAlignmentValue followerAlignment
    ) {
        leader = new MotorIOTalonFX(
                leaderCanID,
                leaderInverted,
                NeutralModeValue.Coast,
                currentLimitAmps,
                gearRatio,
                null,
                velocityGains,
                0.0
        );

        follower = new MotorIOTalonFX(
                followerCanID,
                false,
                NeutralModeValue.Coast,
                currentLimitAmps,
                gearRatio,
                null,
                null,
                0.0
        );
        follower.setFollow(leader, followerAlignment);
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
}
