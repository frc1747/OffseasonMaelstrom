package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
  public static class Drivetrain {
    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    public static final SensorDirectionValue canCoderDirection = chosenModule.canCoderDirection; // Should the cancoder be inverted based on the swerve module we're using
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)
    public static final int PIGEON_ID = 50;

    // Physical measurements
    public static final double trackWidth = Units.inchesToMeters(22.5);
    public static final double wheelBase = Units.inchesToMeters(22.5);
    public static final double wheelCircumference = chosenModule.wheelCircumference;
    public static final double CENTER_TO_WHEEL = Math.sqrt(Math.pow(wheelBase / 2.0, 2) + Math.pow(trackWidth / 2.0, 2));

    // Module gear ratios based on the swerve module we're using
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    // Motor inverts based on the swerve module we're using
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    // Drive Motor Conversion Factors
    public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    // Swerve Kinematics 
    // No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    // Swerve Current Limiting
    public static final int angleContinuousCurrentLimit = 40;
    public static final int anglePeakCurrentLimit = 100;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 40;
    public static final int drivePeakCurrentLimit = 100;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    // Swerve Voltage Compensation (default)
    public static final double voltageComp = 12.0;

    // Neutral Modes
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    // TODO: Tune these later
    public static final double MAX_SPEED = 4.1;  // Max speed in m/s
    public static final double MAX_ACCEL = 4.1;  // Max acceleration in m/s
    public static final double maxAngularVelocity = 10.0;  // Rad/s

    // TODO: Tune these later
    public static final double DRIVE_KP = 0.05;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    public static final double DRIVE_KS = 0.0; 
    public static final double DRIVE_KV = 0.0;
    public static final double DRIVE_KA = 0.0;

    public static final double ANGLE_KP = chosenModule.angleKP;
    public static final double ANGLE_KI = chosenModule.angleKI;
    public static final double ANGLE_KD = chosenModule.angleKD;
    public static final double ANGLE_KF = chosenModule.angleKF;

    // Module Specific Constants
    // Front Left Module
    public static final class FRONT_LEFT { 
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(294.4); // was 297.9
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Front Right Module
    public static final class FRONT_RIGHT { 
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(336.9); // was 335.8
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    // Back Left Module
    public static final class REAR_LEFT { 
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(91.8); // was 266.4 - 180
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Back Right Module
    public static final class REAR_RIGHT { 
      public static final int driveMotorID = 31; 
      public static final int angleMotorID = 32;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(273.7); // was 95.3 - 180
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class Algae {
    public static final int INTAKE_ID = 41;
    public static final int ALGAE_LIMIT_SWITCH_ID = 42;
  }

  public static final class AlgaePivot {
    public static final int PIVOT_ID = 43;
    public static final int LIMIT_SWITCH_ID = 44;
  }

  public static final class Coral {
    public static final int INTAKE_ID = 45;
    public static final int CORAL_LIMIT_SWITCH_ID = 46;
  }
  
  public static final class CoralPivot {
    public static final int PIVOT_ID = 47;
  }

  public static final class Climber {
    public static final int LEFT_ID = 48;
    public static final int RIGHT_ID = 49;
    public static final int LINEBREAK_ID = 50;
  }

  public static final class Elevator {
    public static final int ELEVATOR_ID = 51;
    public static final int LIMIT_SWITCH_BOTTOM_ID = 52;
    public static final int LIMIT_SWITCH_TOP_ID = 53;
    public static final int LEVEL_ONE_LINEBREAK_ID = 54;
    public static final int LEVEL_TWO_LINEBREAK_ID = 55;
    public static final int LEVEL_THREE_LINEBREAK_ID = 56;
    public static final int LEVEL_FOUR_LINEBREAK_ID = 57;
  }
    
}
