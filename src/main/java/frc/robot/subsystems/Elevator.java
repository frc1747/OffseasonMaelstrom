// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax elevator;
  private DigitalInput limitSwitchTop;
  private DigitalInput limitSwitchBottom;
  private SparkClosedLoopController controller;
  private SparkAbsoluteEncoder encoder;

  public Elevator() {
    elevator = new SparkMax(Constants.Elevator.ELEVATOR_ID, MotorType.kBrushless);
    limitSwitchTop = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_TOP_ID);
    limitSwitchBottom = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_BOTTOM_ID);
    controller = elevator.getClosedLoopController();
    encoder = elevator.getAbsoluteEncoder();
    double p = Constants.Elevator.PID_P;
    double i = Constants.Elevator.PID_I;
    double d = Constants.Elevator.PID_D;
    double f = Constants.Elevator.PID_F;

    SparkMaxConfig config = new SparkMaxConfig();
    config
      .closedLoop
        .pidf(
          p,
          i,
          d,
          f
        );
    
    elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    controller.setReference(position, SparkBase.ControlType.kPosition);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Elevator Velocity", getVelocity());
  }
}
