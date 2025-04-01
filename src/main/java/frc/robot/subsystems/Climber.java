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
  //private TalonFX climbingRight;
  private DigitalInput limitSwitch;
  private double pow;

  public Climber() {
    climbingLeft = new TalonFX(Constants.Climber.LEFT_ID);
    //climbingRight = new TalonFX(Constants.Climber.RIGHT_ID);
    climbingLeft.setNeutralMode(NeutralModeValue.Brake);
    //climbingRight.setNeutralMode(NeutralModeValue.Brake);
    limitSwitch = new DigitalInput(Constants.Climber.LIMIT_SWITCH_ID);
    pow = 0.0;
  }
  
  public void setClimberPower(double pow) {
    this.pow = pow;
    //climbingRight.set(-power);
  }

  public double getPosition() {
    return climbingLeft.getPosition().getValueAsDouble();
  }

  public void setCoast() {
    climbingLeft.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrake() {
    climbingLeft.setNeutralMode(NeutralModeValue.Brake);
  }

   public boolean isAtBottom() {
     return !limitSwitch.get();
   }

  @Override
  public void periodic() {
    double mult = 1.0;
    if (isAtBottom()) { // will not descend if bottom limit hit
      mult = 0.0;
    }
    climbingLeft.set(pow*mult);
    SmartDashboard.putNumber("Climber Position", getPosition());
  }
}
