// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax elevatorLeft;
  private SparkMax elevatorRight;
  private DigitalInput limitSwitchTop;
  private DigitalInput limitSwitchBottom;


  public Elevator() {
    elevatorLeft = new SparkMax(Constants.elevatorConstants.elevatorLeft, MotorType.kBrushless);
    elevatorRight = new SparkMax(Constants.elevatorConstants.elevatorRight, MotorType.kBrushless);
    limitSwitchTop = new DigitalInput(Constants.elevatorConstants.limitSwitchTop );
    limitSwitchBottom = new DigitalInput(Constants.elevatorConstants.limitSwitchBottom );
  }
  public boolean getTop(){
    return  limitSwitchTop.get();
  }
  public boolean getBottom(){
    return limitSwitchBottom.get();
  }

  public void setPower(double pow){
    elevatorLeft.set(pow);
    elevatorRight.set(pow);
  }

  @Override
  public void periodic() {
    setPower(0);
    // This method will be called once per scheduler run
  }
}
