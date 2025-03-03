// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends Command {
  private double speed;
  private Elevator elevator;
  private Joystick joystick;

  public MoveElevator(Elevator elevator, double speed, Joystick joystick) {
    this.elevator = elevator;
    this.speed = speed;
    this.joystick = joystick;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevator.setPower(joystick.getRawAxis(1));
  }

  @Override
  public void end(boolean interrupted) {
    //elevator.setPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
