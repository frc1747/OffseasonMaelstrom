// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Blinkin;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Lights extends Command {
  private LimeLight limelight;
  private Blinkin blinkin;
  public boolean done = false;
  public Lights(LimeLight limelight, Blinkin blinkin) {
    this.limelight = limelight;
    this.blinkin = blinkin;
    addRequirements(limelight, blinkin);
  }

  @Override
  public void initialize() {
    if (limelight.tagInView()) {  //will fix method name when needed
      blinkin.setColor(Constants.Blinkin.GREEN);
    } else {
      blinkin.setColor(Constants.Blinkin.RED);
    }
    done = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
