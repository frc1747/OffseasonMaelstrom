// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaePivot;

public class DropAlgaeIntake extends Command {
  private AlgaePivot algaePivot;
  private int counter;
  public DropAlgaeIntake(AlgaePivot algaePivot) {
    this.algaePivot = algaePivot;
    addRequirements(algaePivot);
  }


  @Override
  public void initialize() {
    algaePivot.setPosition(Constants.AlgaePivot.DROPPED);
    counter = 0;
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if ((algaePivot.getPosition() < Constants.AlgaePivot.DROPPED + Constants.AlgaePivot.POSITION_THRESHOLD) && (algaePivot.getPosition() > Constants.AlgaePivot.DROPPED - Constants.AlgaePivot.POSITION_THRESHOLD)) {
      counter++;
    } else {
      counter = 0;
    }

    if (counter > Constants.AlgaePivot.COUNTER_MAX_VALUE) {
      return true;
    } else {
      return false;
    }
  }
}
