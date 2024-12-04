package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagCameraSim extends AprilTagCamera {

  public AprilTagCameraSim(
    String cameraName,
    Transform3d cameraTranslation,
    VisionSystemSim visionSimField
  ) {
    super(cameraName, cameraTranslation);
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(30);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);
    var cameraSim = new PhotonCameraSim(super.camera, cameraProp);

    visionSimField.addCamera(cameraSim, cameraTranslation);

    cameraSim.enableRawStream(true);
    cameraSim.enableDrawWireframe(true);
  }
}
