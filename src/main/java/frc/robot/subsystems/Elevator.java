// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
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
  private PIDController pid;

  public Elevator() {
    elevator = new TalonFX(Constants.Elevator.ELEVATOR_ID);
    elevator.setNeutralMode(NeutralModeValue.Brake);
    limitSwitchTop = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_TOP_ID);
    limitSwitchBottom = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_BOTTOM_ID);
    encoder = new Encoder(Constants.Elevator.ENCODER_A, Constants.Elevator.ENCODER_B, true);

    double p = Constants.Elevator.PID_P;
    double i = Constants.Elevator.PID_I;
    double d = Constants.Elevator.PID_D;
    double f = Constants.Elevator.PID_F;
    this.pid = new PIDController(p, i, d);
    pid.setTolerance(.0001);
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
    pow = pid.calculate(encoder.get()/2048.0, position);
  }

  // encoder position in number of rotations
  public double getPosition() {
    return encoder.get() / 2048.0;
  }

  public double getKrakenPosition() {
    return elevator.getPosition().getValueAsDouble();
  }

  public void goHome() {
    if (!isAtBottom()) {
      setPower(1);
    }
  }

  @Override
  public void periodic() {
    double mult = 1.0;
    if (getPosition() > Constants.Elevator.TOP_SLOW_POS && elevator.get() < 0) { // cut power by 80% if encoder above TOP_SLOW_POS
      mult = 0.5;
    }
    if (getPosition() < Constants.Elevator.BOTTOM_SLOW_POS && elevator.get() > 0) { // cut power by 80% if encoder below BOTTOM_SLOW_POS
      mult = 0.5;
    }
    if (isAtBottom()) { // will not descend if bottom limit hit
      if (pow < 0) mult = 0.0;
    } 
    if (isAtTop()) { // will not ascend if top limit hit
      if (pow > 0) mult = 0.0;
    }
    elevator.set(-pow * mult);
    //if(pow == 0); elevator.set(Constants.Elevator.HOLD_SPEED);

    if(isAtBottom()) {
      elevator.setPosition(0);
      encoder.reset();
    }
    pid.reset();
    //System.out.println("Encoder: " + getPosition());
    SmartDashboard.putNumber("Elevator AbsPosition", getPosition());
    SmartDashboard.putNumber("Elevator RelPosition", elevator.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Bottom Limit Switch", isAtBottom());
    SmartDashboard.putBoolean("Top Limit Switch", isAtTop());
    SmartDashboard.putNumber("ELevator Height %", 100*(getKrakenPosition()/-239)); // sean wants 100 to 0 D:

  }
}
