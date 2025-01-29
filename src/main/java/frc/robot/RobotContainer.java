// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  public RobotContainer() {
    public static CTREConfigs ctreConfigs = new CTREConfigs();

    //subsystems
    public final Climber climber = new Climber();
    public final Elevator elevator = new Elevator();
    public final Drivetrain drivetrain = new Drivetrain();
    public final CoralPivot coralPivot = new CoralPivot();
    public final Coral coral = new Coral();
    public final Algae algae = new Algae();
    public final AlgaePivot algaePivot = new AlgaePivot();

    //controllers
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    // operator buttons
    public final POVButton operatorDpadUp = new POVButton(operator, 0);
    public final POVButton operatorDpadRight = new POVButton(operator, 90);
    public final POVButton operatorDpadDown = new POVButton(operator, 180);
    public final POVButton operatorDpadLeft = new POVButton(operator, 270);

    // Drive Controls
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // Driver Buttons
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // Alerts
    private final Alert driverDisconnectedAlert = new Alert("Driver controller is disconnected (port " + driver.getPort() + ").", AlertType.WARNING);
    private final Alert operatorDisconnectedAlert = new Alert("Operator controller is disconnected (port " + operator.getPort() + ").", AlertType.WARNING);

    // BooleanSuppliers
    private final BooleanSupplier rightTrigger = () -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > Short.MAX_VALUE - 10;
    private final BooleanSupplier leftTrigger = () -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > Short.MAX_VALUE - 10;
    private final BooleanSupplier rightBumper = () -> operator.getRawAxis(XboxController.Button.kRightBumper.value) == 1;
    private final BooleanSupplier leftBumper = () -> operator.getRawAxis(XboxController.Button.kLeftBumper.value) == 1;
    private final BooleanSupplier toggleManual = () -> operator.getRawAxis(XboxController.Button.kStart.value) == 1; 

    configureBindings();
  }

  private void configureBindings() {
    //text
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
