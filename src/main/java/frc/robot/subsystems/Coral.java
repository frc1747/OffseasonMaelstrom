// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {
  private SparkMax coral;
  private DigitalInput limitSwitch;
  private double pow;

  public Coral() {
    coral = new SparkMax(Constants.Coral.INTAKE_ID, MotorType.kBrushless);
    limitSwitch = new DigitalInput(Constants.Coral.CORAL_LIMIT_SWITCH_ID); 
    //configure brake mode
  }
  public RelativeEncoder getEncoder() {
    return coral.getAlternateEncoder();
  }

  public void setIntakePower(double power) {
    pow = -power;
  }

  public boolean switchPressed() {
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    double mult = 1.0;
    if (switchPressed()) { // will not descend if bottom limit hit
      if (pow > 0) mult = 0.0;
    } 
    coral.set(-pow * mult);
    SmartDashboard.putBoolean("Coral Limit Switch", switchPressed());
  }
}
