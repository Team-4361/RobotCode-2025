// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

//need to test ahh vision code

public class AlignToAprilTagPhoton extends Command {
  private PIDController xController, yController, rotController;
  private final PhotonCamera camera;
  private final SwerveSubsystem drivebase;
  private final boolean isRightOffset;
  private Timer targetLostTimer, alignedTimer;
  private int targetID = -1;
  
  // Distance to maintain from tag (1 foot in meters)
  private static final double TARGET_DISTANCE_METERS = 0.3048; // 1 foot
  
  // Lateral offset (1 foot in meters)
  private static final double LATERAL_OFFSET_METERS = 0.3048; // 1 foot
  
  // Camera position relative to robot center (must be calibrated for your robot)
  private final Transform3d cameraToRobot;
  
  /**
   * Creates a new command to align to an AprilTag using PhotonVision
   * 
   * @param cameraName Name of the PhotonVision camera
   * @param drivebase The swerve drive subsystem
   * @param isRightOffset True to position 1 foot to the right, false for 1 foot to the left
   */
  public AlignToAprilTagPhoton(String cameraName, SwerveSubsystem drivebase, boolean isRightOffset) {
    this.camera = new PhotonCamera(cameraName);
    this.drivebase = drivebase;
    this.isRightOffset = isRightOffset;
    
    // Define camera position relative to robot center - update these for your robot
    this.cameraToRobot = new Transform3d(
        new Translation3d(VisionConstants.CAMERA_X_OFFSET, VisionConstants.CAMERA_Y_OFFSET, VisionConstants.CAMERA_Z_OFFSET),
        new Rotation3d(0, 0, 0));
    
    // Configure PID controllers
    xController = new PIDController(VisionConstants.X_APRILTAG_ALIGNMENT_P, 0.0, 0.0);
    yController = new PIDController(VisionConstants.Y_APRILTAG_ALIGNMENT_P, 0.0, 0.0);
    rotController = new PIDController(VisionConstants.ROT_APRILTAG_ALIGNMENT_P, 0.0, 0.0);
    
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    // Reset timers
    targetLostTimer = new Timer();
    alignedTimer = new Timer();
    targetLostTimer.start();
    alignedTimer.start();
    
    // Set controller tolerances
    xController.setTolerance(VisionConstants.X_TOLERANCE_APRILTAG);
    yController.setTolerance(VisionConstants.Y_TOLERANCE_APRILTAG);
    rotController.setTolerance(VisionConstants.ROT_TOLERANCE_APRILTAG);
    
    // Set rotation setpoint to 0 degrees (facing the tag)
    rotController.setSetpoint(0.0);
    
    // Forward position setpoint (1 foot from tag)
    xController.setSetpoint(TARGET_DISTANCE_METERS);
    
    // Lateral position setpoint (1 foot to left or right based on parameter)
    double lateralSetpoint = isRightOffset ? LATERAL_OFFSET_METERS : -LATERAL_OFFSET_METERS;
    yController.setSetpoint(lateralSetpoint);
    
    // Record initial target if visible
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      targetID = result.getBestTarget().getFiducialId();
    }
  }

  @Override
  public void execute() {
    PhotonPipelineResult result = camera.getLatestResult();
    
    // Check if we see a target
    if (result.hasTargets()) {
      PhotonTrackedTarget target = null;
      
      // If we have a specific target ID, find that one
      if (targetID != -1) {
        for (PhotonTrackedTarget t : result.getTargets()) {
          if (t.getFiducialId() == targetID) {
            target = t;
            break;
          }
        }
      } 
      // Otherwise use best target
      else if (result.hasTargets()) {
        target = result.getBestTarget();
        targetID = target.getFiducialId();
      }
      
      if (target != null) {
        // Reset target lost timer since we can see it
        targetLostTimer.reset();
        
        // Get the transform from camera to target
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        
        // Convert to robot-to-target (similar to your Limelight code)
        Transform3d robotToTarget = cameraToTarget.plus(cameraToRobot.inverse());
        
        // Extract the positional data we need
        double x = robotToTarget.getX(); // Forward/backward distance
        double y = robotToTarget.getY(); // Left/right distance
        double rot = Math.toDegrees(robotToTarget.getRotation().getZ()); // Yaw angle
        
        // Log values to SmartDashboard for debugging
        SmartDashboard.putNumber("RobotToTag X", x);
        SmartDashboard.putNumber("RobotToTag Y", y);
        SmartDashboard.putNumber("RobotToTag Rot", rot);
        
        // Calculate PID outputs
        double xSpeed = xController.calculate(x);
        double ySpeed = yController.calculate(y);
        double rotSpeed = rotController.calculate(rot);
        
        // Apply speed limits
        xSpeed = limitMagnitude(xSpeed, VisionConstants.MAX_SPEED_X);
        ySpeed = limitMagnitude(ySpeed, VisionConstants.MAX_SPEED_Y);
        rotSpeed = limitMagnitude(rotSpeed, VisionConstants.MAX_SPEED_ROT);
        
        // Drive the robot using YAGSL - robot-relative like your original code
        drivebase.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, false);
        
        // Check if we're at the setpoint
        if (xController.atSetpoint() && 
            yController.atSetpoint() && 
            rotController.atSetpoint()) {
          // We're aligned, let the timer run
        } else {
          // Not aligned, reset the timer
          alignedTimer.reset();
        }
      } else {
        // Don't see the specific target we want
        drivebase.drive(new Translation2d(), 0, false);
      }
    } else {
      // No targets visible
      drivebase.drive(new Translation2d(), 0, false);
    }
    
    // Update dashboard with timer values
    SmartDashboard.putNumber("Target Lost Timer", targetLostTimer.get());
    SmartDashboard.putNumber("Aligned Timer", alignedTimer.get());
    SmartDashboard.putNumber("Target ID", targetID);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Command finishes if:
    // 1. Target is lost for too long
    // 2. Robot is aligned at setpoint for required time
    return targetLostTimer.hasElapsed(VisionConstants.APRILTAG_ALIGNMENT_TIMEOUT) || 
           alignedTimer.hasElapsed(VisionConstants.APRILTAG_ALIGNMENT_SETTLED_TIME);
  }
  
  /**
   * Limit the magnitude of a value while preserving sign
   */
  private double limitMagnitude(double value, double limit) {
    if (Math.abs(value) > limit) {
      return Math.copySign(limit, value);
    }
    return value;
  }
}