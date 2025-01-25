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
    elevatorLeft = new SparkMax(Constants.Elevator.LEFT_ID, MotorType.kBrushless);
    elevatorRight = new SparkMax(Constants.Elevator.RIGHT_ID, MotorType.kBrushless);
    limitSwitchTop = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_TOP);
    limitSwitchBottom = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_BOTTOM);
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
  }
}
