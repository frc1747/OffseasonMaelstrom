// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private TalonFX climbingLeft;
  private TalonFX climbingRight;
  private DigitalInput linebreak;

  public Climber() {
    climbingLeft = new TalonFX(Constants.Climber.LEFT_ID);
    climbingRight = new TalonFX(Constants.Climber.RIGHT_ID);
    climbingLeft.setNeutralMode(NeutralModeValue.Brake);
    climbingRight.setNeutralMode(NeutralModeValue.Brake);
  }
  
  public void setClimberPower(double power){
    climbingLeft.set(power);
    climbingRight.set(-power);
  }

  public double getPosition() {
    return climbingLeft.getPosition().getValueAsDouble();
  }

  public boolean cageInPosition(){
    return !linebreak.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", getPosition());
  }
}
