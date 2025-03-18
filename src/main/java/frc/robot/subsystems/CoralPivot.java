// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralPivot extends SubsystemBase {
  private SparkMax pivot;
  private SparkClosedLoopController controller;
  private DutyCycleEncoder encoder;
  private DigitalInput limitSwitch;
  private DigitalInput limitSwitchTop;
  private double pow;
  private PIDController pid;

  public CoralPivot() {
    pivot = new SparkMax(Constants.CoralPivot.PIVOT_ID, MotorType.kBrushed);
    //limitSwitch = new DigitalInput(Constants.CoralPivot.CORALPIVOT_LIMIT_SWITCH_BOTTOM_ID);
    limitSwitch = new DigitalInput(Constants.CoralPivot.CORALPIVOT_LIMIT_SWITCH_TOP_ID);
    controller = pivot.getClosedLoopController();
    this.encoder = new DutyCycleEncoder(Constants.CoralPivot.ENCODER);
    double p = Constants.CoralPivot.PID_P;
    double i = Constants.CoralPivot.PID_I;
    double d = Constants.CoralPivot.PID_D;
    double f = Constants.CoralPivot.PID_F;
    this.pid = new PIDController(p, i, d);


    // SparkMaxConfig config = new SparkMaxConfig();
    // config
    //   .idleMode(IdleMode.kBrake)
    //   .closedLoop
    //     .pidf(
    //       p,
    //       i,
    //       d,
    //       f
    //     );
    
  //   pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 }
  

  public void setPivotPower(double power) {
    pow = power;
  }

  public double getPosition() {
    return encoder.get();
  }

  public void setPosition(double position) {
    controller.setReference(position, SparkBase.ControlType.kDutyCycle);
    pow = pid.calculate(encoder.get(), position);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Pivot Encoder", getPosition());
    SmartDashboard.putBoolean("top coral", !limitSwitch.get());
   // SmartDashboard.putBoolean("top limt",!limitSwitchTop.get() );
   double mult = 1.0;
    if (!limitSwitch.get()) { // will not descend if bottom limit hit
      if (pow < 0) mult = 0.0;
    } 
   pivot.set(-pow * mult);
  }
}
