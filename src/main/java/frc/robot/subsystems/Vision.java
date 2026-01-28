// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*
  Bryan's simple code
  PhotonCamera camera = new PhotonCamera(cameraName);
  Transform3d robotToCam = new Transform3d(new Translation3d(
      cameraForwardOffset, cameraLeftOffset, cameraUpOffset
  ), new Rotation3d(
      cameraRollRadians, cameraPitchRadians, cameraYawRadians 
  ));

  aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  PhotonPoseEstimator ppe = new PhotonPoseEstimator(
      aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      robotToCam
  );
  */
}
