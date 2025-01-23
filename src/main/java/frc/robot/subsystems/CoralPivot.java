// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralPivot extends SubsystemBase {
  private SparkMax pivot;
  /** Creates a new CoralPivot. */
  public CoralPivot() {
    pivot = new SparkMax(Constants.CoralConstants.PIVOT, MotorType.kBrushless);
  }

  public void setPivotPower(double power){
    pivot.set(power);
  }

  public double getPosition(){
    return pivot.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Pivot Encoder", getPosition());
  }
}
