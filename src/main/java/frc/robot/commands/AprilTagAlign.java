package frc.robot.commands;

/* WHEN TESTING MAKE SURE THERE I SOMEONE THAT IS READY TO DISABLE(THE KEYBIND IS SET to 12 ON THE LEFT JOYSTICK)
 * 
 */

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose3d; // Import Pose3d

public class AprilTagAlign extends Command {
    private final SwerveSubsystem swerve;
    private final PhotonCamera camera = new PhotonCamera("Camera1"); // Hardcoded camera Name (will add to constands later prob)

    private static final double TARGET_DISTANCE_METERS = 0.91; // 3 feet away from target
    private static final double STRAFE_SPEED = 0.5;
    private static final double ROTATION_SPEED = 0.5;
    private static final double FORWARD_SPEED = 0.5;
    private static final double POSITION_TOLERANCE = 0.05; // 5 cm tolerance
    private static final double ANGLE_TOLERANCE = 2.0; // 2 degrees tolerance

    public AprilTagAlign(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        var result = camera.getLatestResult();

        if (!result.hasTargets()) {
            swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0)); // Stop moving if no tag
            return;
        }

        PhotonTrackedTarget target = result.getBestTarget();

        // Get the pose (position and rotation) of the target relative to the camera
        var tagTransform = target.getBestCameraToTarget(); // This is Transform3d

        // Convert Transform3d to Pose3d manually if necessary
        Pose3d tagPose = new Pose3d(tagTransform.getTranslation(), tagTransform.getRotation());

        // Extract position (x, y, z) and rotation (yaw, pitch) from Pose3d
        double x = tagPose.getX(); // Forward/backward distance
        double y = tagPose.getY(); // Sideways distance (left/right)
        double z = tagPose.getZ(); // Vertical distance (up/down)

        double yaw = target.getYaw(); // Rotation around Z axis
        double pitch = target.getPitch(); // Pitch (up/down) rotation

        double strafeSpeed = 0.0;
        double rotationSpeed = 0.0;
        double forwardSpeed = 0.0;

        // Strafe to center the AprilTag (on X-axis)
        if (Math.abs(yaw) > ANGLE_TOLERANCE) {
            strafeSpeed = Math.signum(yaw) * STRAFE_SPEED;
        }

        // Rotate to be parallel with the AprilTag (on Y-axis and Z-axis)
        if (Math.abs(pitch) > ANGLE_TOLERANCE) {
            rotationSpeed = Math.signum(pitch) * ROTATION_SPEED;
        }

        // Move forward/backward to be exactly 3 feet away (X-axis distance)
        double distanceError = x - TARGET_DISTANCE_METERS; // X-axis for distance error
        if (Math.abs(distanceError) > POSITION_TOLERANCE) {
            forwardSpeed = Math.signum(distanceError) * FORWARD_SPEED;
        }

        // Send movement commands to YAGSL's swerve system
        swerve.driveFieldOriented(new ChassisSpeeds(forwardSpeed, -strafeSpeed, rotationSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0)); // Stop the robot
    }

    @Override
    public boolean isFinished() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) return false;

        PhotonTrackedTarget target = result.getBestTarget();
        var tagTransform = target.getBestCameraToTarget(); // This is Transform3d
        Pose3d tagPose = new Pose3d(tagTransform.getTranslation(), tagTransform.getRotation());

        double x = tagPose.getX(); // Forward/backward distance
        double yaw = target.getYaw();
        double distanceError = Math.abs(x - TARGET_DISTANCE_METERS);

        return Math.abs(yaw) < ANGLE_TOLERANCE && distanceError < POSITION_TOLERANCE;
    }
}