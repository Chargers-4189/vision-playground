// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.simulation.VisionSystemSim;

public class Vision extends SubsystemBase {

  private Pose2d robotPosition = new Pose2d();
  private Pose2d avgEstimatedRobotPosition = new Pose2d();
  private boolean avgEstimateAvailable = false;

  /** Creates a new vision. */
  public VisionSystemSim visionSimField = new VisionSystemSim("main");
  private AprilTagCameraSim[] cameras;

  public Vision() {
    try {
      AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(
        AprilTagFields.k2025Reefscape.m_resourceFile
        // Filesystem.getDeployDirectory() + "\\2025-reefscape.json"
      );
      visionSimField.addAprilTags(tagLayout);
    } catch (Exception e) {
      System.err.println("Unable to load April Tag Field Layout.");
      System.out.println(e.getMessage());
    }

    Rotation3d robotToCamRot = new Rotation3d(0, Math.toRadians(-15), 0);
    Transform3d robotToCam = new Transform3d(
      new Translation3d(0.25, -0.25, 0),
      robotToCamRot
    );
    Rotation3d robotToCam2Rot = new Rotation3d(0, Math.toRadians(-15), 0);
    Transform3d robotToCam2 = new Transform3d(
      new Translation3d(0.25, 0.25, 0),
      robotToCam2Rot
    );

    cameras =
      new AprilTagCameraSim[] {
        new AprilTagCameraSim("Camera 1", robotToCam, visionSimField),
        new AprilTagCameraSim("Camera 2", robotToCam2, visionSimField),
      };
  }

  public Pose2d getEstimatedRobotPosition() {
    if (avgEstimateAvailable) {
      return avgEstimatedRobotPosition;
    }
    return null;
  }

  public void AvgEstimatedRobotPosition() {
    int count = 0;
    double x = 0;
    double y = 0;
    double degrees = 0;
    for (AprilTagCameraSim camera : cameras) {
      if (camera.isEstimateReady()) {
        x += camera.getEstimatedRobotPose().getX();
        y += camera.getEstimatedRobotPose().getY();
        degrees += camera.getEstimatedRobotPose().getRotation().getDegrees();
        count++;
      }
    }
    x /= count;
    y /= count;
    degrees = ((degrees / count) * Math.PI) / 180;
    avgEstimatedRobotPosition = new Pose2d(x, y, new Rotation2d(degrees));
    avgEstimateAvailable = true;
    if (count == 0) {
      avgEstimatedRobotPosition = null;
      avgEstimateAvailable = false;
    }
  }

  public void updateSimRobotPosition(Pose2d position) {
    robotPosition = position;
  }

  @Override
  public void periodic() {
    for (AprilTagCameraSim camera : cameras) {
      camera.update();
    }
    AvgEstimatedRobotPosition();
    visionSimField.update(robotPosition);
  }
}
