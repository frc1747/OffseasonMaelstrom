// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {
  private Talon coral;
  private DigitalInput limitSwitch;
  /** Creates a new Coral. */
  public Coral() {
    coral = new Talon(Constants.CoralConstants.INTAKE); //ID INTAKE does not exist yet, fix later
    limitSwitch = new DigitalInput(Constants.CoralConstants.CORAL_LIMIT_SWITCH); //ID CORAL_LIMIT_SWITCH does not exist yet, fix later
    //configure brake mode
  }

  public void setIntakePower(double power){
    coral.set(power);
  }

  public boolean switchPressed() {
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
