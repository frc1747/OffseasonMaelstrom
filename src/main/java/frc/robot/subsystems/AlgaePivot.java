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


public class AlgaePivot extends SubsystemBase {
  private SparkMax pivot;
  private DigitalInput limitSwitch;

  public AlgaePivot() {
    pivot = new SparkMax(Constants.AlgaePivot.PIVOT_ID, MotorType.kBrushless);
    limitSwitch = new DigitalInput(Constants.AlgaePivot.LIMIT_SWITCH_ID);
  }

  public void setPivotPower(double power){
    pivot.set(power);
  }
  
  public double getPosition(){
    return pivot.getEncoder().getPosition();
  }

  public void dropIntake(){
    pivot.getEncoder().setPosition(Constants.AlgaePivot.DROPPED); //DROPPED TBD
  }



  public boolean switchPressed(){
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Pivot Encoder", getPosition());
  }
}
