// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Teleop.TeleopSwerve;
import frc.robot.commands.Teleop.Climb;
import frc.robot.commands.Teleop.AngleCoral;
import frc.robot.commands.Teleop.DropAlgaeIntake;
import frc.robot.commands.Teleop.EjectAlgae;
import frc.robot.commands.Teleop.EjectCoral;
import frc.robot.commands.Teleop.GoToLevel;
import frc.robot.commands.Teleop.IntakeAlgae;
import frc.robot.commands.Teleop.StowAlgaeIntake;
import frc.robot.commands.Teleop.IntakeCoral;
import frc.robot.commands.Teleop.MoveElevator;
import frc.robot.commands.Teleop.PivotCoralIntake;
//import frc.robot.commands.Teleop.ResetGyro;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
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
    //private final POVButton zeroGyro = new POVButton(driver, 180);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // Alerts
    private final Alert driverDisconnectedAlert = new Alert("Driver controller is disconnected (port " + driver.getPort() + ").", AlertType.kWarning);
    private final Alert operatorDisconnectedAlert = new Alert("Operator controller is disconnected (port " + operator.getPort() + ").", AlertType.kWarning);

    // BooleanSuppliers
    private final BooleanSupplier rightTriggerOperator = () -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > Short.MAX_VALUE - 10;
    private final BooleanSupplier leftTriggerOperator = () -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > Short.MAX_VALUE - 10;
    private final BooleanSupplier BooleanElevator = () -> operator.getRawAxis(XboxController.Axis.kLeftY.value) != 0;
    private final BooleanSupplier BooleanCoralIntakePivot = () -> operator.getRawAxis(XboxController.Axis.kRightY.value) != 0;
    // booleanSuppliers driver
    private final BooleanSupplier rightTriggerDriver = () -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > Short.MAX_VALUE - 10;

    
    //DoubleSuppliers
    private final DoubleSupplier manualCoralIntakePivot = () -> operator.getRawAxis(XboxController.Axis.kRightY.value);
    private final DoubleSupplier manualElevator = () -> operator.getRawAxis(XboxController.Axis.kLeftY.value);

  public RobotContainer() {
    
    
    //drivetrain
    drivetrain.setDefaultCommand(
      new TeleopSwerve(
        drivetrain, 
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis), 
        () -> -driver.getRawAxis(rotationAxis), 
        () -> robotCentric.getAsBoolean()
      )
    );
    configureBindings();
  }

  private void configureBindings() {
    //driver commands 
     new JoystickButton(driver, XboxController.Button.kA.value)
      .whileTrue(new EjectAlgae(algae));
     new Trigger(rightTriggerDriver)
      .onTrue(new DropAlgaeIntake(algaePivot))
      .whileTrue(new IntakeAlgae(algae))
      .onFalse(new StowAlgaeIntake(algaePivot));
     

    //operater Coral commands

    new JoystickButton(operator, XboxController.Button.kB.value)
      .whileTrue(new IntakeCoral(coral));  
    new Trigger(rightTriggerOperator)
      .whileTrue( new EjectCoral(coral));   

    new Trigger(leftTriggerOperator)
      .whileTrue(new Climb(climber, Constants.Climber.CLIMB_SPEED));
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
      .whileTrue(new Climb(climber, -Constants.Climber.CLIMB_SPEED));

    // Elevator
    //Manual
    new Trigger(BooleanElevator)
      .whileTrue(new MoveElevator(elevator,manualElevator.getAsDouble()));

    //Presets (elevator & coral pivot)
    new JoystickButton(operator, XboxController.Button.kA.value)
      .onTrue(new GoToLevel(elevator, Constants.Elevator.LOWER_ALGAE_POSITION));
    new JoystickButton(operator, XboxController.Button.kY.value)
      .onTrue(new GoToLevel(elevator, Constants.Elevator.UPPER_ALGAE_POSITION));
    new JoystickButton(operator, XboxController.Button.kX.value)
      .onTrue(new GoToLevel(elevator, Constants.Elevator.CORAL_STATION_POSITION))
      .onTrue(new AngleCoral(coralPivot, Constants.CoralPivot.CORAL_STATION_POSITION));
    
    operatorDpadUp
      .onTrue(new GoToLevel(elevator, Constants.Elevator.LEVEL_FOUR_POSITION))
      .onTrue(new AngleCoral(coralPivot, Constants.CoralPivot.REEF_POSITION));
    
    operatorDpadRight.onTrue(new GoToLevel(elevator, Constants.Elevator.LEVEL_THREE_POSITION))
      .onTrue(new AngleCoral(coralPivot, Constants.CoralPivot.REEF_POSITION));
    
    operatorDpadLeft.onTrue(new GoToLevel(elevator, Constants.Elevator.LEVEL_TWO_POSITION))
      .onTrue(new AngleCoral(coralPivot, Constants.CoralPivot.REEF_POSITION));
    
    operatorDpadDown.onTrue(new GoToLevel(elevator, Constants.Elevator.LEVEL_ONE_POSITION))
      .onTrue(new AngleCoral(coralPivot, Constants.CoralPivot.REEF_POSITION));

    //Coral Pivot
    //Manual
    new Trigger(BooleanCoralIntakePivot)
      .whileTrue(new PivotCoralIntake(coralPivot, manualCoralIntakePivot.getAsDouble()));

    //Reset Gyro
    // zeroGyro
    //   .onTrue(new ResetGyro(drivetrain));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
