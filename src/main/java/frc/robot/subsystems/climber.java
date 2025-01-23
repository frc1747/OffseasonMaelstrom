// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class climber extends SubsystemBase {
  
  private TalonFX climbingLeft;
  private TalonFX climbingRight;

  /** Creates a new climber. */
  public climber() {
    climbingLeft = new TalonFX(Constants.ClimberConstants.LEFT); //ID LEFT doesnt exist yet, fix later
    climbingRight = new TalonFX(Constants.ClimberConstants.RIGHT); //ID RIGHT doesnt exist yet, fix later
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
