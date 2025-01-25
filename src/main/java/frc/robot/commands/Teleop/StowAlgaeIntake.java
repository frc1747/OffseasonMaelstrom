// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaePivot;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StowAlgaeIntake extends Command {
  /** Creates a new StowAlgaeIntake. */
  private AlgaePivot intakePivot;

  public StowAlgaeIntake(AlgaePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakePivot.setPivotPower(-Constants.AlgaePivot.PIVOT_IN_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakePivot.getPosition() < Constants.AlgaePivot.SLOW_POSITION){
      intakePivot.setPivotPower(-Constants.AlgaePivot.PIVOT_IN_SPEED_SLOW);
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivot.setPivotPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakePivot.switchPressed();
  }
}
