// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaePivot;


public class StowAlgaeIntake extends Command {
  private AlgaePivot algaePivot;

  public StowAlgaeIntake(AlgaePivot algaePivot) {
    this.algaePivot = algaePivot;
    addRequirements(algaePivot);
  }

  @Override
  public void initialize() {
    algaePivot.setPivotPower(-Constants.AlgaePivot.PIVOT_IN_SPEED);
  }

  @Override
  public void execute() {
    if (algaePivot.getPosition() < Constants.AlgaePivot.SLOW_POSITION){
      algaePivot.setPivotPower(-Constants.AlgaePivot.PIVOT_IN_SPEED_SLOW);
      return;
    }
  }

  @Override
  public void end(boolean interrupted) {
    algaePivot.setPivotPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return algaePivot.switchPressed();
  }
}
