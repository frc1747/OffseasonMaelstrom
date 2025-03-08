package frc.robot.commands.Teleop;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {    
    private Drivetrain drivetrain;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private DoubleSupplier elevatorPos;
    private BooleanSupplier SpeedButton;

    public TeleopSwerve(Drivetrain drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, DoubleSupplier elevatorPos, BooleanSupplier SpeedButton ) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.SpeedButton = SpeedButton;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.elevatorPos = elevatorPos;
    }

    @Override
    public void execute() {
        // Get Values, Deadband
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controller.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controller.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Controller.STICK_DEADBAND);
        boolean slowDown = SpeedButton.getAsBoolean();

        // Drive , watch this drive 
        if(elevatorPos.getAsDouble() >= Constants.Elevator.MIN_SLOW_POSITION){
            drivetrain.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Drivetrain.MAX_SPEED*(1.05-elevatorPos.getAsDouble()/Constants.Elevator.TOP_POSITION)), 
                rotationVal * Constants.Drivetrain.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
        }
        else if(slowDown){
            drivetrain.drive( 
                new Translation2d(translationVal, strafeVal).times(Constants.Drivetrain.MAX_SPEED*(1.0-elevatorPos.getAsDouble()/Constants.Elevator.TOP_POSITION)).times(Constants.Drivetrain.SlowSpeed),
                rotationVal * Constants.Drivetrain.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
        } 
        else {
            drivetrain.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Drivetrain.MAX_SPEED),
                rotationVal * Constants.Drivetrain.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true
            );
        }
    }
}