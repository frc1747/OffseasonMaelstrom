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

public class Elevator extends SubsystemBase {
  private TalonFX elevator;
  private DigitalInput limitSwitchTop;
  private DigitalInput limitSwitchBottom;

  public Elevator() {
    elevator = new TalonFX(Constants.Elevator.ELEVATOR_ID);
    elevator.setNeutralMode(NeutralModeValue.Brake);
    limitSwitchTop = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_TOP_ID);
    limitSwitchBottom = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_BOTTOM_ID);
  }
  public boolean isAtTop() {
    return  !limitSwitchTop.get();
  }
  public boolean isAtBottom() {
    return !limitSwitchBottom.get();
  }
  public void setPower(double pow) {
    elevator.set(pow);
  }

  public void setPosition(double position) {
    elevator.setPosition(position);
  }

  public double getPosition() {
    return elevator.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getPosition());
  }
}
