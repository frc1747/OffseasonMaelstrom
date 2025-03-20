// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralPivot;

public class AngleCoral extends Command {
  private double position;
  private CoralPivot pivot;
  public AngleCoral(CoralPivot pivot, double position) {
    this.pivot = pivot;
    this.position = position;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivot.setPosition(position);
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setPosition(pivot.getPosition());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
