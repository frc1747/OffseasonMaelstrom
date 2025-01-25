// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private TalonFX climbingLeft;
  private TalonFX climbingRight;

  public Climber() {
    climbingLeft = new TalonFX(Constants.Climber.LEFT_ID);
    climbingRight = new TalonFX(Constants.Climber.RIGHT_ID);
    climbingLeft.setNeutralMode(NeutralModeValue.Brake);
    climbingRight.setNeutralMode(NeutralModeValue.Brake);
  }
  
  public void setClimberLPower(double power){
    climbingLeft.set(power); //could be wrong direction, fix when we get bot
  }
  public void setCLimberRPower(double power){
    climbingRight.set(-power); //could be wrong direction, fix when we get bot
  }

  public double getLPosition() {
    return climbingLeft.getPosition().getValueAsDouble();
  }

  public double getRPosition() {
    return climbingRight.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber L Position", getLPosition());
    SmartDashboard.putNumber("Climber R Position", getRPosition());
  }
}
