// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private Drivetrain drivetrain;

  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  // returns estimated pose
  public Pose2d getPose() {
    return drivetrain.swerveOdometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
