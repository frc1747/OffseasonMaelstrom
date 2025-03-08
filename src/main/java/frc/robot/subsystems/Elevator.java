// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonFX elevator;
  private DigitalInput limitSwitchTop;
  private DigitalInput limitSwitchBottom;
  private Encoder encoder;
  private PositionVoltage PositionControl;
  private double pow;
  private boolean bottomLimit;
  private boolean topLimit;

  public Elevator() {
    elevator = new TalonFX(Constants.Elevator.ELEVATOR_ID);
    elevator.setNeutralMode(NeutralModeValue.Brake);
    limitSwitchTop = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_TOP_ID);
    limitSwitchBottom = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_BOTTOM_ID);
    encoder = new Encoder(Constants.Elevator.ENCODER_A, Constants.Elevator.ENCODER_B, true);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Elevator.PID_P;
    slot0Configs.kI = Constants.Elevator.PID_I;
    slot0Configs.kD = Constants.Elevator.PID_D;
    elevator.getConfigurator().apply(slot0Configs);
    this.PositionControl =  new PositionVoltage(0).withSlot(0);
    
    pow = 0.0;
    bottomLimit = false;
    topLimit = false;
  }

  public boolean isAtTop() {
    return  !limitSwitchTop.get();
  }

  public boolean isAtBottom() {
    return !limitSwitchBottom.get();
  }

  public void setPower(double pow) {
    this.pow = -pow;
  }

  public void setPosition(double position) {
    //double distance = (position - encoder.get())*Constants.Elevator.MOTOR_TO_SHAFT_RATIO;
    elevator.setControl(PositionControl.withPosition(position));// abs encoder
  }

  // encoder position in number of rotations
  public double getPosition() {
    return encoder.get() / 2048.0;
  }

  public double getKrakenPosition() {
    return elevator.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    double mult = 1.0;
    if (getPosition() > 6.5) { // cut power by 80% if encoder above 6.5 rotations
      mult = 0.2;
    }
    if (isAtBottom()) { // will not descend if bottom limit hit
      if (pow < 0) mult = 0.0;
    } 
    if (isAtTop()) { // will not ascend if top limit hit
      if (pow > 0) mult = 0.0;
    }
    elevator.set(-pow * mult);

    if(isAtBottom()) {
      elevator.setPosition(0);
      encoder.reset();
    }
    //System.out.println("Encoder: " + getPosition());
    SmartDashboard.putNumber("Elevator AbsPosition", getPosition());
    SmartDashboard.putNumber("Elevator RelPosition", elevator.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Bottom Limit Switch", isAtBottom());
    SmartDashboard.putBoolean("Top Limit Switch", isAtTop());
  }
}
