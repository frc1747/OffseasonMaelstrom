// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorIntakeCommand extends Command {
  /** Creates a new ElevatorPositiveCommand. */
  private Timer timer = new Timer();
  private Elevator elevator;
  private final double position;
  private CoralPivot pivot;
  private double PivPosition;

  public ElevatorIntakeCommand(Elevator elevator, CoralPivot pivot,double ElvPosition, double PivPosition) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.position = ElvPosition;
    this.PivPosition = PivPosition;
    addRequirements(this.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.elevator.setPosition(this.position);
    pivot.setPosition(PivPosition);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.elevator.setPosition(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2);//Fine tune when bot go brrr
  }
}
