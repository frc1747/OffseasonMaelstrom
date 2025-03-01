// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonFX elevator;
  private DigitalInput limitSwitchTop;
  private DigitalInput limitSwitchBottom;
  private DutyCycleEncoder encoder;
  private PositionVoltage PositionControl;

  public Elevator() {
    elevator = new TalonFX(Constants.Elevator.ELEVATOR_ID);
    elevator.setNeutralMode(NeutralModeValue.Brake);
    limitSwitchTop = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_TOP_ID);
    limitSwitchBottom = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_BOTTOM_ID);
    encoder = new DutyCycleEncoder(Constants.Elevator.ENCODER_CHANNEL);
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Elevator.PID_P;
    slot0Configs.kI = Constants.Elevator.PID_I;
    slot0Configs.kD = Constants.Elevator.PID_D;
    elevator.getConfigurator().apply(slot0Configs);
    this.PositionControl =  new PositionVoltage(0).withSlot(0);
  }

  public boolean isAtTop() {
    return  !limitSwitchTop.get();
  }

  public boolean isAtBottom() {
    return !limitSwitchBottom.get();
  }

  public void setPower(double pow) {
    elevator.set(-pow);
  }

  public void setPosition(double position) {
    //double distance = (position - encoder.get())*Constants.Elevator.MOTOR_TO_SHAFT_RATIO;
    elevator.setControl(PositionControl.withPosition(position));
  }

  public double getPosition() {
    return encoder.get();
  }

  @Override
  public void periodic() {
    if(isAtTop() || isAtBottom()) setPower(0);
    if(isAtBottom())elevator.setPosition(0);
    SmartDashboard.putNumber("Elevator AbsPosition", getPosition());
    SmartDashboard.putNumber("Elevator RelPosition", elevator.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Bottom Limit Switch", isAtBottom());
    SmartDashboard.putBoolean("Top Limit Switch", isAtTop());
  }
}
