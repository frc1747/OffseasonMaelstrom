// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;

public class IntakeAlgae extends Command {
  private Algae intake;

  public IntakeAlgae(Algae intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakePower(Constants.Algae.ROLLER_SPEED);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intake.setIntakePower(0.0);
  }

  @Override
  public boolean isFinished() {
    return intake.switchPressed();
  }
}
