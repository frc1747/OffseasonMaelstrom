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
  private int counter;
  public AngleCoral(CoralPivot pivot, double position) {
    this.pivot = pivot;
    this.position = position;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    pivot.setPosition(position);
    counter = 0;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if ((pivot.getPosition() < position + Constants.CoralPivot.POSITION_THRESHOLD) && (pivot.getPosition() > position - Constants.CoralPivot.POSITION_THRESHOLD)) {
      counter++;
    } else {
      counter = 0;
    }

    if (counter > Constants.CoralPivot.COUNTER_MAX_VALUE) {
      return true;
    } else {
      return false;
    }
  }
}
