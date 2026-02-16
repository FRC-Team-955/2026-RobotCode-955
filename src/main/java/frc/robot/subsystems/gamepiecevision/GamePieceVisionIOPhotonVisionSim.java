package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import java.util.function.Supplier;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.diagFOVRad;
import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.fuelDiameterMeters;

public class GamePieceVisionIOPhotonVisionSim extends GamePieceVisionIOPhotonVision {
    public static VisionSystemSim visionSim;


    private static final Supplier<Pose2d> poseSupplier = ModuleIOSim.driveSimulation::getSimulatedDriveTrainPose;
    private final PhotonCameraSim cameraSim;


    public GamePieceVisionIOPhotonVisionSim(String name, Transform3d robotToCamera) {
        super(name);

        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
        }


        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(640, 480, Rotation2d.fromRadians(diagFOVRad));
        cameraProperties.setCalibError(0, 0);
        cameraProperties.setFPS(120);
        cameraProperties.setAvgLatencyMs(45);
        cameraProperties.setLatencyStdDevMs(10);
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(GamePieceVisionIOInputs inputs) {

        Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
        TargetModel targetModel = new TargetModel(fuelDiameterMeters);
        visionSim.clearVisionTargets();
        for (Pose3d fuelPose : fuelPoses) {


            VisionTargetSim visionTarget = new VisionTargetSim(fuelPose, targetModel);

            visionSim.addVisionTargets(visionTarget);
        }


        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
