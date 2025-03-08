// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.Optional;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.LimeLightHelpers;
import frc.robot.LimeLightHelpers.PoseEstimate;
import frc.robot.RobotContainer;
import frc.robot.Utilities;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private Drivetrain drivetrain;
    private LimeLight frontLimeLight;
    private Pose2d currentEstimate;
        private Object currentTime;
    private LimeLight rearLimeLight;
            
                /** Creates a new PoseEstimatorSubsystem. 
                         * @param rearLimeLight */
                        public PoseEstimatorSubsystem(Drivetrain drivetrain, LimeLight frontlimeLight,LimeLight rearLimeLight) {
                            this.drivetrain = drivetrain;
                            this.frontLimeLight = frontLimeLight;
                            this.rearLimeLight = rearLimeLight;
            currentEstimate = new Pose2d();      // probably needs to be adjested later
        }
    
        public Pose2d getEstimatedPose() {
            return currentEstimate;
        }
    
        @Override
        public void periodic() {
            LimeLightHelpers.SetRobotOrientation(frontLimeLight.getName(), drivetrain.getYaw().getDegrees(), 0, 0, 0, 0, 0);
            LimeLightHelpers.SetRobotOrientation(rearLimeLight.getName(), (drivetrain.getYaw().getDegrees() + 180) % 360, 0, 0, 0, 0, 0); // Adjusted for rear camera

            LimeLightHelpers.PoseEstimate frontPoseEstimate = LimeLightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(frontLimeLight.getName());
            LimeLightHelpers.PoseEstimate rearPoseEstimate = LimeLightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(rearLimeLight.getName());
            
            boolean rejectFrontVisionUpdate = shouldRejectPoseUpdate(frontPoseEstimate, drivetrain);
            boolean rejectRearVisionUpdate = shouldRejectPoseUpdate(rearPoseEstimate, drivetrain);
        
            Pose2d frontPose = frontPoseEstimate != null ? frontPoseEstimate.pose : null;
            Pose2d rearPose = rearPoseEstimate != null ? adjustRearPose(rearPoseEstimate.pose) : null; // Adjust rear pose
            
            if (!rejectFrontVisionUpdate && !rejectRearVisionUpdate) {
                // Both cameras provide valid data, take an average
                currentEstimate = Utilities.average(frontPose, rearPose);
            } else if (!rejectFrontVisionUpdate) {
                // Only front camera is valid
                currentEstimate = Utilities.average(frontPose, drivetrain.getPose());
            } else if (!rejectRearVisionUpdate) {
                // Only rear camera is valid
                currentEstimate = Utilities.average(rearPose, drivetrain.getPose());
            } else {
                // No valid vision input, rely on odometry
                currentEstimate = drivetrain.getPose();
            }

            drivetrain.setPose(currentEstimate);

            RobotContainer.estimatedField.setRobotPose(getEstimatedPose());
        }

        private Pose2d adjustRearPose(Pose2d rearPose) {
            Rotation2d correctedRotation = rearPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
            return new Pose2d(rearPose.getTranslation(), correctedRotation);
        }
    
        private boolean shouldRejectPoseUpdate(PoseEstimate frontPoseEstimate, Drivetrain drivetrain2) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'shouldRejectPoseUpdate'");
        }

        public void updatePoseEstimator() {
            Optional<Pose2d> frontPose = getLimelightPose("limelight-front");
                    if (frontPose.isPresent()) {
                        PoseEstimatorSubsystem.addVisionMeasurement(frontPose.get(), currentTime, VecBuilder.fill(0.7, 07, 0.9));
                    }
            
                    Optional<Pose2d> rearPose = getLimelightPose("limelight-rear");
                    if (rearPose.isPresent()) {
                        PoseEstimatorSubsystem.addVisionMeasurement(rearPose.get(), currentTime, VecBuilder.fill(0.7, 0.7, 0.9));
                }
            }
        
            private static void addVisionMeasurement(Pose2d pose2d, Object currentTime2, Vector<N3> fill) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'addVisionMeasurement'");
        }

            private Optional<Pose2d> getLimelightPose(String string) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'getLimelightPose'");
            }
        
            public void setDesiredPose(Object object) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDesiredPose'");
    }
}
