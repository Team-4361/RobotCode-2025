package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = -1;
  private PhotonCamera photonCamera;

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);

    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.photonCamera = new PhotonCamera("YourCameraName"); // Set PhotonVision camera name

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    PhotonPipelineResult result = photonCamera.getLatestResult();
    if (result.hasTargets()) {
        tagID = result.getBestTarget().getFiducialId();
    } else {
        tagID = -1;
    }
  }

  @Override
  public void execute() {
    PhotonPipelineResult result = photonCamera.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();

      if (target.getFiducialId() == tagID) {
        this.dontSeeTagTimer.reset();

        double xPos = target.getBestCameraToTarget().getX();
        double yPos = target.getBestCameraToTarget().getY();
        double rotPos = target.getBestCameraToTarget().getRotation().getZ();

        SmartDashboard.putNumber("x", xPos);
        SmartDashboard.putNumber("y", yPos);
        SmartDashboard.putNumber("rotation", rotPos);

        double xSpeed = xController.calculate(xPos);
        double ySpeed = -yController.calculate(yPos);
        double rotValue = -rotController.calculate(rotPos);

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("RotValue", rotValue);


        drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

        if (!rotController.atSetpoint() ||
            !yController.atSetpoint() ||
            !xController.atSetpoint()) {
          stopTimer.reset();
        }
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}
