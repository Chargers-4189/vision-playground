// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class Robot extends TimedRobot {

  private final Joystick m_controller = new Joystick(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Drivetrain m_drive = new Drivetrain();
  private final Vision vision = new Vision();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;

  @Override
  public void robotInit() {
    m_trajectory =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(2, 2, new Rotation2d()),
        List.of(),
        new Pose2d(6, 4, new Rotation2d()),
        new TrajectoryConfig(2, 2)
      );
  }

  @Override
  public void robotPeriodic() {
    vision.periodic();
    Pose2d estimatedPose = vision.getEstimatedRobotPosition();
    if (estimatedPose != null) {
      m_drive.updateVison(estimatedPose, Timer.getFPGATimestamp());
      try (
        BufferedWriter writer = new BufferedWriter(
          new FileWriter("robot.logs", true)
        )
      ) {
        writer.write(
          estimatedPose.getX() + ""
        );
        writer.newLine();
      } catch (IOException e) {
        System.err.println(e);
      }
    }
    m_drive.periodic();
    vision.updateSimRobotPosition(m_drive.getPose());
  }

  @Override
  public void autonomousInit() {
    m_timer.restart();
    m_drive.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = m_timer.get();
    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed =
      -m_speedLimiter.calculate(m_controller.getRawAxis(1)) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot =
      -m_rotLimiter.calculate(m_controller.getRawAxis(0)) *
      Drivetrain.kMaxAngularSpeed;
    m_drive.drive(xSpeed, rot);
  }

  @Override
  public void simulationPeriodic() {
    m_drive.simulationPeriodic();
  }
}
