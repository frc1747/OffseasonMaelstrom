// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {
  private Spark blinkin;

  public Blinkin() {
    blinkin = new Spark(8);
  }

  public void setColor(double color){
    blinkin.set(color);
  }

  @Override
  public void periodic() {
  }
}
