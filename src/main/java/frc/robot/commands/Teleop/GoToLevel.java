// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class GoToLevel extends Command {
  private final double position;
  private Elevator elevator;
  private int counter;

  public GoToLevel(Elevator elevator, double position) {
    this.elevator = elevator;
    this.position = position;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    counter = 0;
    elevator.setPosition(position);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if ((elevator.getPosition() < position + Constants.Elevator.POSITION_THRESHOLD) && (elevator.getPosition() > position - Constants.Elevator.POSITION_THRESHOLD)) {
      counter++;
    } else {
      counter = 0;
    }

    if (counter > Constants.Elevator.COUNTER_MAX_VALUE) {
      return true;
    } else {
      return false;
    }
  }
}
