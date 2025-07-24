// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.Optional;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.LimeLightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Utilities;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private Drivetrain drivetrain;
    private LimeLight limeLight;
    private Pose2d currentEstimate;
    private Pose2d desiredPose;
    private Optional<Alliance> ally;

    /** Creates a new PoseEstimatorSubsystem. */
    public PoseEstimatorSubsystem(Drivetrain drivetrain, LimeLight limeLight) {
        this.drivetrain = drivetrain;
        this.limeLight = limeLight;
        currentEstimate = new Pose2d();      // probably needs to be adjested later
        ally = DriverStation.getAlliance();
    }

    public Pose2d getEstimatedPose() {
        return currentEstimate;
    }

    public void setDesiredPose(Pose2d desiredPose) {
        this.desiredPose = desiredPose;
    }

    @Override
    public void periodic() {
        LimeLightHelpers.SetRobotOrientation(limeLight.getName(), drivetrain.getYaw().getDegrees(), 0, 0, 0, 180, 0);
        LimeLightHelpers.PoseEstimate mt2 = LimeLightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limeLight.getName());
        
        boolean rejectVisionUpdate = false;
        if (mt2 == null) {
            return;
        }
        // test to see if the bot spins to quickly and rejects if it is to fast
        // this is done by testing rejectVisionUpdate
        if (Math.abs(drivetrain.gyro.getRate()) > 720) { 
            rejectVisionUpdate = true;
        } else if (mt2.tagCount == 0) {
            rejectVisionUpdate = true;
        } 
        // it's seeing if we need to reject vision
        if (!rejectVisionUpdate) {
            currentEstimate = mt2.pose;
            drivetrain.setPose(currentEstimate);
        } else {
            currentEstimate = drivetrain.getPoseNOLL();
        }
        RobotContainer.estimatedField.setRobotPose(getEstimatedPose());


        // green light if the robot is within 2 cm and 1.5 degrees of desired pose
        // if (desiredPose != null) {
        //     Transform2d difference = desiredPose.minus(getEstimatedPose());
        //     if (Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2)) > 0.02) {
        //         SmartDashboard.putBoolean("In Position", false);
        //     } else if (difference.getRotation().getRadians() > Math.PI/120) {
        //         SmartDashboard.putBoolean("In Position", false);
        //     } else {
        //         SmartDashboard.putBoolean("In Position", true);
        //     }
        // } else {
        //     SmartDashboard.putBoolean("In Position", false);
        // }
    }
}
