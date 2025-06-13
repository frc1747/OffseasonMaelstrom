// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TO DO:
//Check zeroGyro

package frc.robot;

import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Teleop.TeleopSwerve;
import frc.robot.commands.Teleop.Climb;
import frc.robot.commands.GoToPose2d;
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
import frc.robot.subsystems.ButtonBoard;
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
    public final ButtonBoard buttonBoard = new ButtonBoard(new Joystick(2), new Joystick(3));

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

    // booleanSuppliers driver
    private final BooleanSupplier rightTriggerDriver = () -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0;

    
    //DoubleSuppliers
    private final DoubleSupplier manualCoralIntakePivot = () -> operator.getRawAxis(XboxController.Axis.kRightY.value);
    private final DoubleSupplier manualElevator = () -> operator.getRawAxis(XboxController.Axis.kLeftY.value);

  // Limelight Vision and Pose
  private final LimeLight limeLight = new LimeLight("limelight");
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(drivetrain, limeLight);
  public static Field2d estimatedField;

  public RobotContainer() {
    drivetrain.setPoseEstimator(poseEstimator);
    estimatedField = new Field2d();
    SmartDashboard.putData("Estimated Field", estimatedField);
    System.out.println(estimatedField);


    //drivetrain
    drivetrain.setDefaultCommand(
      new TeleopSwerve(
        drivetrain, 
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> driver.getRawAxis(rotationAxis), 
        () -> robotCentric.getAsBoolean(),
        () -> elevator.getPosition()
      )
    );
    elevator.setDefaultCommand(new MoveElevator(elevator, 0.0, operator));
    coralPivot.setDefaultCommand(new PivotCoralIntake(coralPivot, 0.0, operator));
    configureBindings();
  }

  private void configureBindings() {
   // SmartDashboard.putData("Estimated Field", estimatedField);
    //estimatedField.setRobotPose(poseEstimator.getEstimatedPose());
    //driver commands 
    new JoystickButton(driver, XboxController.Button.kA.value)
      .whileTrue(new EjectAlgae(algae));
    new Trigger(() -> (driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0)) 
      .onTrue(new DropAlgaeIntake(algaePivot))
      .whileTrue(new IntakeAlgae(algae))
      .onFalse(new StowAlgaeIntake(algaePivot));

    // Pose Estimation code (Uses Button Board)
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
   
        buttonBoard.Blue1() // AprTag 18
          .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(3.105, 3.995), new Rotation2d(0))));
        buttonBoard.Blue2() // AprTag 19
          .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(3.802, 5.228), new Rotation2d(-60.255))));
        buttonBoard.Blue3() // AprTag 20
          .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(5.195, 5.228), new Rotation2d(-119.197))));
        buttonBoard.Blue4() // AprTag 21
          .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(5.891, 3.997), new Rotation2d(180.000))));
        buttonBoard.Blue5() // AprTag 22
          .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(5.241, 2.814), new Rotation2d(122.829))));
        buttonBoard.Blue6() // AprTag 17
          .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(3.779, 2.802), new Rotation2d(58.782))));
    
      
      // //if (ally.get() == Alliance.Red)  { 
      //   buttonBoard.Blue1() // AprTag 10
      //     .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(11.652, 3.983), new Rotation2d(0))));
      //   buttonBoard.Blue2() // AprTag 9
      //     .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(12.335, 5.278), new Rotation2d(-59.534))));
      //   buttonBoard.Blue3() // AprTag 8
      //     .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(13.798, 5.254), new Rotation2d(-122.829))));
      //   buttonBoard.Blue4() // AprTag 7
      //     .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(14.517, 4.031), new Rotation2d(180.000))));
      //   buttonBoard.Blue5() // AprTag 6
      //     .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(13.822, 2.808), new Rotation2d(123.311))));
      //   buttonBoard.Blue6() // AprTag 11
      //     .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(12.323, 2.760), new Rotation2d(58.782))));
      // }
    }
    //operater Coral commands
  //  estimatedField.setRobotPose(poseEstimator.getEstimatedPose());

    new JoystickButton(operator, XboxController.Button.kB.value)
      .whileTrue(new IntakeCoral(coral));  
    new Trigger(() -> (operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0))
      .whileTrue( new EjectCoral(coral));   

    new Trigger(() -> (operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0))
      .whileTrue(new Climb(climber, Constants.Climber.CLIMB_SPEED));
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
      .whileTrue(new Climb(climber, -Constants.Climber.CLIMB_SPEED));

    new JoystickButton(driver, XboxController.Button.kB.value) 
      .whileTrue(new GoToPose2d(poseEstimator, drivetrain, new Pose2d(new Translation2d(1, 1), new Rotation2d(0, 0))));

    
    // Elevator
    //Manual
    //I dont know which button is kStart and which is kBack. If this is the wrong button we will fix it later
    //new Trigger(() ->  (Math.abs(manualElevator.getAsDouble()) > .05))
    //  .whileTrue(new MoveElevator(elevator, manualElevator.getAsDouble()/2.0));

    //Presets (elevator & coral pivot)
    // new JoystickButton(operator, XboxController.Button.kA.value)
    //   .onTrue(new GoToLevel(elevator, Constants.Elevator.LOWER_ALGAE_POSITION));
    // new JoystickButton(operator, XboxController.Button.kY.value)
    //   .onTrue(new GoToLevel(elevator, Constants.Elevator.UPPER_ALGAE_POSITION));
    // new JoystickButton(operator, XboxController.Button.kX.value)
    //   .onTrue(new GoToLevel(elevator, Constants.Elevator.CORAL_STATION_POSITION))
    // //   .onTrue(new AngleCoral(coralPivot, Constants.CoralPivot.CORAL_STATION_POSITION));
    
    // operatorDpadUp
    //   .onTrue(new GoToLevel(elevator, Constants.Elevator.LEVEL_FOUR_POSITION))
    //   .onTrue(new AngleCoral(coralPivot, Constants.CoralPivot.REEF_POSITION));
    
    // operatorDpadRight.onTrue(new GoToLevel(elevator, Constants.Elevator.LEVEL_THREE_POSITION))
    //   .onTrue(new AngleCoral(coralPivot, Constants.CoralPivot.REEF_POSITION));
    
    // operatorDpadLeft.onTrue(new GoToLevel(elevator, Constants.Elevator.LEVEL_TWO_POSITION))
    //   .onTrue(new AngleCoral(coralPivot, Constants.CoralPivot.REEF_POSITION));
    
    // operatorDpadDown.onTrue(new GoToLevel(elevator, Constants.Elevator.LEVEL_ONE_POSITION))
    //   .onTrue(new AngleCoral(coralPivot, Constants.CoralPivot.REEF_POSITION));

    // //Coral Pivot
    //Manual
    //I dont know which button is kStart and which is kBack. If this is the wrong button we will fix it later
    //new JoystickButton(operator, XboxController.Button.kStart.value)
    //  .whileTrue(new PivotCoralIntake(coralPivot, manualCoralIntakePivot.getAsDouble()));

    //Reset Gyro
    // zeroGyro
    //   .onTrue(new ResetGyro(drivetrain));
  
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

