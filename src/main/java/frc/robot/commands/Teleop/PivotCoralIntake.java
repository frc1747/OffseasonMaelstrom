// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivot;

public class PivotCoralIntake extends Command {
  private CoralPivot pivot;
  private double speed;
  private Joystick joystick;

  public PivotCoralIntake(CoralPivot pivot, double speed, Joystick joystick) {
    this.pivot = pivot;
    this.speed = speed;
    this.joystick = joystick;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    pivot.setPivotPower(joystick.getRawAxis(XboxController.Axis.kLeftY.value) / 2);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
