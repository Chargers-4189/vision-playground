// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

  /** Creates a new vision. */
  public VisionSystemSim visionSimField = new VisionSystemSim("main");
  public PhotonCamera camera1 = new PhotonCamera("Camera 1");
  public PhotonCamera camera2 = new PhotonCamera("Camera 2");
  public PhotonPoseEstimator poseEstimator1;
  public PhotonPoseEstimator poseEstimator2;
  public Pose2d estimatedAvgPose;
  public Pose3d robotPose = new Pose3d();
  public Pose2d robotPosition = new Pose2d();
  public boolean estimateAvailable = false;

  public Vision() {
    Rotation3d robotToCam1Rot = new Rotation3d(0, Math.toRadians(-15), 0);
    Transform3d robotToCam1 = new Transform3d(
      new Translation3d(0.25, -0.25, 0),
      robotToCam1Rot
    );
    Rotation3d robotToCam2Rot = new Rotation3d(0, Math.toRadians(-15), 0);
    Transform3d robotToCam2 = new Transform3d(
      new Translation3d(0.25, 0.25, 0),
      robotToCam2Rot
    );
    try {
      AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(
        AprilTagFields.k2024Crescendo.m_resourceFile
      );
      visionSimField.addAprilTags(tagLayout);
      poseEstimator1 =
        new PhotonPoseEstimator(
          tagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCam1
        );
      poseEstimator2 =
        new PhotonPoseEstimator(
          tagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCam2
        );

      poseEstimator1.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
      );
      poseEstimator2.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
      );
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }

    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(30);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);
    var camera1Sim = new PhotonCameraSim(camera1, cameraProp);
    var camera2Sim = new PhotonCameraSim(camera2, cameraProp);
    // Add the simulated camera to view the targets on this simulated field.

    visionSimField.addCamera(camera1Sim, robotToCam1);
    visionSimField.addCamera(camera2Sim, robotToCam2);

    camera1Sim.enableRawStream(true);
    camera1Sim.enableDrawWireframe(true);
    camera2Sim.enableRawStream(true);
    camera2Sim.enableDrawWireframe(true);
  }

  public void update() {
    var results1 = camera1.getAllUnreadResults();
    var results2 = camera2.getAllUnreadResults();
    var estimatedResult1 = targetFilter(results1);
    var estimatedResult2 = targetFilter(results2);
    
    if (estimatedResult1 != null && estimatedResult2 != null) {
      estimatedAvgPose = estimatedResult1.estimatedPose.toPose2d();
      estimateAvailable = true;
      //AVG distance between two estimates.
      /*
      estimatedAvgPose = new Pose2d(
        ((
          estimatedResult1.estimatedPose.toPose2d().getX() +
          estimatedResult2.estimatedPose.toPose2d().getX()
        ) / 2),
        ((
          estimatedResult1.estimatedPose.toPose2d().getY() +
          estimatedResult2.estimatedPose.toPose2d().getY()
        ) / 2),
        new Rotation2d((((
          estimatedResult1.estimatedPose.toPose2d().getRotation().getDegrees() +
          estimatedResult2.estimatedPose.toPose2d().getRotation().getDegrees()
        ) / 2) * Math.PI) / 180)
      );*/
    }
  }

  private EstimatedRobotPose targetFilter(List<PhotonPipelineResult> results) {
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      result.targets.removeIf(tag -> {
        double maxDistance = 6.0;
        Transform3d transform = tag.getBestCameraToTarget();
        return (
          transform.getX() > maxDistance || transform.getY() > maxDistance
        );
      });

      if (
        result.hasTargets() &&
        result.getTargets().size() < 16 &&
        result.getTargets().size() > 0
      ) {
        for (var i : result.getTargets()) {
          System.out.println(i.getPoseAmbiguity());
        }
        //System.out.println(result.getTargets().size());
        // TODO helper class for different camera's
        var estimatedResult = poseEstimator1.update(result);
        if (estimatedResult.isPresent()) {
          return estimatedResult.get();
        }
      }
    }
    return null;
  }

  public Pose2d getEstimatedRobotPose() {
    estimateAvailable = false;
    return estimatedAvgPose;
  }

  public void updateRobotPosition(Pose2d position) {
    robotPosition = position;
  }

  @Override
  public void periodic() {
    visionSimField.update(robotPosition);
    this.update();
  }
}
