// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CoralPivot;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorAutoCommand extends Command {
  /** Creates a new ElevatorPositiveCommand. */
  private Timer timer = new Timer();
  private Elevator elevator;
  private final double position;
  private PIDController pid;

  public ElevatorAutoCommand(Elevator elevator, double ElvPosition, double PivPosition) {
    this.elevator = elevator;
    this.position = ElvPosition;
    double p = Constants.Elevator.PID_P;
    double i = Constants.Elevator.PID_I;
    double d = Constants.Elevator.PID_D;
    double f = Constants.Elevator.PID_F;
    this.pid = new PIDController(p, i, d);
    addRequirements(this.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     double power = pid.calculate(elevator.getPosition(), this.position);
     this.elevator.setPower(power); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //this.elevator.setPosition(0.0);
   // this.pivot.setPosition(this.pivot.getPosition());
    this.elevator.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //timer.hasElapsed(2);
    boolean stop = true;
    stop = stop && this.elevator.isAtTop() && this.elevator.isAtBottom();

    return stop;
  }
}
