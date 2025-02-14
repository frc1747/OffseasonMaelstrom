// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class AlgaePivot extends SubsystemBase {
  private SparkMax pivot;
  private DigitalInput limitSwitch;
  private SparkClosedLoopController controller;
  private AbsoluteEncoder encoder;

  public AlgaePivot() {
    pivot = new SparkMax(Constants.AlgaePivot.PIVOT_ID, MotorType.kBrushless);
    limitSwitch = new DigitalInput(Constants.AlgaePivot.LIMIT_SWITCH_ID);
    controller = pivot.getClosedLoopController();
    encoder = pivot.getAbsoluteEncoder();
    double p = Constants.AlgaePivot.PID_P;
    double i = Constants.AlgaePivot.PID_I;
    double d = Constants.AlgaePivot.PID_D;
    double f = Constants.AlgaePivot.PID_F;

    SparkMaxConfig config = new SparkMaxConfig();
    config
      .closedLoop
        .pidf(
          p,
          i,
          d,
          f
        );
    
    pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPivotPower(double power) {
    pivot.set(power);
  }

  public void setPosition(double position) {
    controller.setReference(position, SparkBase.ControlType.kPosition);
  }
  
  public double getPosition() {
    return encoder.getPosition();
  }

  public boolean switchPressed() {
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Pivot Encoder", getPosition());
    SmartDashboard.putBoolean("Algae Pivot Limit Switch", switchPressed());
  }
}
