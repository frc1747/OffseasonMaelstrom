// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algae extends SubsystemBase {
  /** Creates a new Algae. */
  private SparkMax intake;
  private DigitalInput limitSwitch;

  public Algae() {
    intake = new SparkMax(Constants.AlgaeConstants.INTAKE, MotorType.kBrushless); //Mark will add constants
    limitSwitch = new DigitalInput(Constants.AlgaeConstants.ALGAE_LIMIT_SWITCH); //^^^
  }

  public void setIntakePower(double power){
    intake.set(power);
  }

  public boolean switchPressed(){
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
