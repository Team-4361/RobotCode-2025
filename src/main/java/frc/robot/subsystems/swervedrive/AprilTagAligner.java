package frc.robot.subsystems.swervedrive;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem; // Ensure this matches your YAGSL swerve subsystem

//useless do not use this just test code 

public class AprilTagAligner 
extends SubsystemBase {
    private static final double TARGET_DISTANCE_METERS = 0.91; // 3 feet in meters
    private final PhotonCamera camera;
    private final SwerveSubsystem swerve;

    private final PIDController xController = new PIDController(0.8, 0, 0);
    private final PIDController yController = new PIDController(0.8, 0, 0);
    private final PIDController rotationController = new PIDController(0.05, 0, 0);

    public AprilTagAligner(String cameraName, SwerveSubsystem swerve) {
        this.camera = new PhotonCamera(cameraName);
        this.swerve = swerve;

        // Set PID tolerances
        xController.setTolerance(0.05);  // Forward/backward tolerance
        yController.setTolerance(0.02);  // Left/right tolerance
        rotationController.setTolerance(2.0); // Rotation tolerance in degrees
    }

    public void alignToAprilTag() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
            return;
        }

        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d tagPose = target.getBestCameraToTarget();
        
        double xOffset = tagPose.getX() - TARGET_DISTANCE_METERS; // Distance error
        double yOffset = tagPose.getY(); // Sideways error
        double rotationError = Math.toRadians(target.getYaw()); // Convert yaw to radians

        double xSpeed = xController.calculate(xOffset, 0); // Forward/back
        double ySpeed = yController.calculate(yOffset, 0); // Strafe
        double rotationSpeed = rotationController.calculate(rotationError, 0); // Rotate

        swerve.driveFieldOriented(new ChassisSpeeds(xSpeed, -ySpeed, rotationSpeed)); 
    }

    public void stop() {
        swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }
}

