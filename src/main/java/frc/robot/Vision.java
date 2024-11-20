// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
  /** Creates a new vision. */
  public VisionSystemSim visionSimField = new VisionSystemSim("main");
  public PhotonCamera camera = new PhotonCamera("HelloCam");
  public PhotonPoseEstimator poseEstimator;
  public EstimatedRobotPose estimatedPose;
  public Pose3d robotPose = new Pose3d();
  public Pose2d robotPosition = new Pose2d();
  
  public Vision() {
    Rotation3d robotToCamerRot = new Rotation3d(0, Math.toRadians(-15), 0);
    Transform3d robotToCamera = new Transform3d(new Translation3d(0,0,0),robotToCamerRot);
    try{
      AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      visionSimField.addAprilTags(tagLayout);
      poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamera);
    } catch(Exception e){
      System.out.println(e.getMessage());
    }

    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);
    var cameraSim = new PhotonCameraSim(camera, cameraProp);
    // Add the simulated camera to view the targets on this simulated field.

    visionSimField.addCamera(cameraSim, robotToCamera);

    cameraSim.enableRawStream(true);
    cameraSim.enableDrawWireframe(true);
  
  }

  public void update() {
    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) { 
          var estimatedResult = poseEstimator.update(result);
          if(estimatedResult.isPresent()){
            estimatedPose = estimatedResult.get();
            //System.out.println(estimatedPose.estimatedPose);
          }
        }
          //return robotPose;
      }
    }

  public Pose3d getEstimatedRobotPose(){
    if (estimatedPose != null) {
      robotPose = estimatedPose.estimatedPose;
    }
    return robotPose;
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