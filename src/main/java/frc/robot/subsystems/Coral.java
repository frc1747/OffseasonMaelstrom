// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {
  private SparkMax coral;
  private DigitalInput limitSwitch;

  public Coral() {
    coral = new SparkMax(Constants.Coral.INTAKE_ID, MotorType.kBrushless);
    limitSwitch = new DigitalInput(Constants.Coral.CORAL_LIMIT_SWITCH_ID); 
    //configure brake mode
  }

  public void setIntakePower(double power) {
    coral.set(power);
  }

  public boolean switchPressed() {
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    
  }
}
