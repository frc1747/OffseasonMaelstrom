// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algae extends SubsystemBase {
  private SparkMax intake;
  private DigitalInput limitSwitch;

  public Algae() {
    intake = new SparkMax(Constants.Algae.INTAKE_ID, MotorType.kBrushless);
    //limitSwitch = new DigitalInput(Constants.Algae.LIMIT_SWITCH_ID);
  }

  public void setIntakePower(double power) {
    intake.set(power);
  }

  public boolean switchPressed() {
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Algae Limit Switch", switchPressed());
  }
}
