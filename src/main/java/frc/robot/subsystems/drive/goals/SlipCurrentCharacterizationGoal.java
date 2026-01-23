package frc.robot.subsystems.drive.goals;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.network.LoggedTunableNumber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveGoal;
import frc.robot.subsystems.drive.DriveRequest;

public class SlipCurrentCharacterizationGoal extends DriveGoal {
    private static final LoggedTunableNumber minVelocityRadPerSec = new LoggedTunableNumber("Drive/SlipCurrentCharacterization/MinVelocityRadPerSec", 0.01);

    private static final Drive drive = Drive.get();

    private final Timer timer = new Timer();

    private double[] lastCurrents = new double[4];
    private final double[] currentNeededForMovement = new double[4];
    private boolean done = false;

    @Override
    public DriveRequest getRequest() {
        if (!timer.isRunning()) {
            timer.restart();
        }

        if (done) {
            return DriveRequest.stop();
        }

        double[] velocities = drive.getSlipCurrentCharacterizationVelocities();
        double[] currents = drive.getSlipCurrentCharacterizationCurrents();

        if (lastCurrents[0] != 0) {
            double total = 0.0;
            done = true;
            for (int i = 0; i < 4; i++) {
                if (velocities[i] > minVelocityRadPerSec.get() && currentNeededForMovement[i] == 0) {
                    currentNeededForMovement[i] = lastCurrents[i];
                }

                if (done && currentNeededForMovement[i] != 0) {
                    total += currentNeededForMovement[i];
                } else {
                    done = false;
                }
            }

            if (done) {
                double avg = total / 4.0;
                System.out.println("Average current for each module to move: " + avg);
                for (int i = 0; i < 4; i++) {
                    System.out.println("\tCurrent for module " + i + " to move: " + currentNeededForMovement[i]);
                }
                return DriveRequest.stop();
            }
        }

        lastCurrents = currents;

        // Increase voltage at 0.5 V/s
        return DriveRequest.characterization(timer.get() * 0.5);
    }
}
