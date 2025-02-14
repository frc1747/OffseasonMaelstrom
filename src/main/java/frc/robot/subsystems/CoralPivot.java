// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralPivot extends SubsystemBase {
  private SparkMax pivot;
  private SparkClosedLoopController controller;
  private SparkAbsoluteEncoder encoder;

  public CoralPivot() {
    pivot = new SparkMax(Constants.CoralPivot.PIVOT_ID, MotorType.kBrushless);
    controller = pivot.getClosedLoopController();
    encoder = pivot.getAbsoluteEncoder();
    double p = Constants.CoralPivot.PID_P;
    double i = Constants.CoralPivot.PID_I;
    double d = Constants.CoralPivot.PID_D;
    double f = Constants.CoralPivot.PID_F;

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

  public double getPosition() {
    return encoder.getPosition();
  }

  public void setPosition(double position) {
    controller.setReference(position, SparkBase.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Pivot Encoder", getPosition());
  }
}
