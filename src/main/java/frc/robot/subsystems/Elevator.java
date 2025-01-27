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
  private SparkMax elevator;
  private DigitalInput limitSwitchTop;
  private DigitalInput limitSwitchBottom;
  private DigitalInput levelOneLinebreak;
  private DigitalInput levelTwoLinebreak;
  private DigitalInput levelThreeLinebreak;
  private DigitalInput levelFourLinebreak;

  public Elevator() {
    elevator = new SparkMax(Constants.Elevator.ELEVATOR_ID, MotorType.kBrushless);
    limitSwitchTop = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_TOP_ID);
    limitSwitchBottom = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_BOTTOM_ID);
    levelOneLinebreak = new DigitalInput(Constants.Elevator.LEVEL_ONE_LINEBREAK_ID);
    levelTwoLinebreak = new DigitalInput(Constants.Elevator.LEVEL_TWO_LINEBREAK_ID);
    levelThreeLinebreak = new DigitalInput(Constants.Elevator.LEVEL_THREE_LINEBREAK_ID);
    levelFourLinebreak = new DigitalInput(Constants.Elevator.LEVEL_FOUR_LINEBREAK_ID);
  }
  public boolean isAtTop(){
    return  !limitSwitchTop.get();
  }
  public boolean isAtBottom(){
    return !limitSwitchBottom.get();
  }
  public void setPower(double pow){
    elevator.set(pow);
  }
  public boolean isAtLevelOne(){
    return !levelOneLinebreak.get();
  }
  public boolean isAtLevelTwo(){
    return !levelTwoLinebreak.get();
  }
  public boolean isAtLevelThree(){
    return !levelThreeLinebreak.get();
  }
  public boolean isAtLevelFour(){
    return !levelFourLinebreak.get();
  }

  @Override
  public void periodic() {
  }
}
