// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.CoralPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoPivot extends Command {
  /** Creates a new AutoPivot. */
  frc.robot.subsystems.CoralPivot pivot;
  double position;
  PIDController pid;

  public AutoPivot(frc.robot.subsystems.CoralPivot pivot , double position) {
    this.pivot = pivot;
    this.position = position;
    double p = Constants.CoralPivot.PID_P;
    double i = Constants.CoralPivot.PID_I;
    double d = Constants.CoralPivot.PID_D;
    double f = Constants.CoralPivot.PID_F;
    this.pid = new PIDController(p, i, d);
    pid.setTolerance(.0001);
    addRequirements(this.pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   double pow = pid.calculate(pivot.getPosition(), position);
   pivot.setPower(pow); 
    //pivot.setPosition(this.PivPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean stop = true;

    stop = stop && !pivot.getLimitSwitch();

    
    return stop;
}
}