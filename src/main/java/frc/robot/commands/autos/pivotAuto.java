// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CoralPivot;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class pivotAuto extends Command {
  CoralPivot Coralpivot;
  private Timer timer = new Timer();
  double PivPosition;
  /** Creates a new pivotAuto. */
  public pivotAuto(CoralPivot pivot,double PivPosition) {
   
    this.Coralpivot = pivot;
    this.PivPosition = PivPosition;
    addRequirements(this.Coralpivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Coralpivot.setPosition(PivPosition);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Coralpivot.setPosition(PivPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(4);
  }
}
