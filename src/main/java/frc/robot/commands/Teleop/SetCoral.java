// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoral extends Command {
  private final double position;
  private CoralPivot coralPivot;

  public SetCoral(CoralPivot coralPivot, double position) {
    this.coralPivot = coralPivot;
    this.position = position;
    addRequirements(coralPivot);
  }
    
    // private void addRequirements(CoralPivot coralPivot2) {
    //     // TODO Auto-generated method stub
    //   throw new UnsupportedOperationException("Unimplemented method 'addRequirements'");
    // }
    
      // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    coralPivot.setPosition(position);
  }

  @Override
  public void end(boolean interrupted) {
    coralPivot.setPosition(coralPivot.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
