// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivot;

public class PivotCoralIntake extends Command {
  private CoralPivot pivot;
  private double speed;
  public PivotCoralIntake(CoralPivot pivot, double speed) {
    this.pivot = pivot;
    this.speed = speed;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    pivot.setPivotPower(speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    pivot.setPivotPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
