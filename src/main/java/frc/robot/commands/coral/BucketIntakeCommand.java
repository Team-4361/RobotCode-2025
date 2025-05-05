package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;

//certified sahas code trust not gonna break neo550 motor cuz i built different


public class BucketIntakeCommand extends Command {
    private final BucketSubsystem coral;
    private PIDController intakePID;


    private static final double kP = 0.04; // Proportional gain
    private static final double kI = 0.0; // Integral gain
    private static final double kD = 0.0; // Derivative gain
    double targetPosition;


  public BucketIntakeCommand(BucketSubsystem subsystem) {
    coral = subsystem;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    
    coral.ResetEncoder();
    targetPosition = -4;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      intakePID = new PIDController(kP, kI, kD);
      double currentPos = coral.GetEncoderPos();
      intakePID.setTolerance(0.2);
      targetPosition = -4;
      
      double pidOutput = intakePID.calculate(currentPos, targetPosition);
      pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
      SmartDashboard.putNumber("PidOutput:", pidOutput);
      SmartDashboard.putNumber("Stuff: ", coral.GetEncoderPos());
      SmartDashboard.putNumber("TargetPos", targetPosition);

      coral.SetMotorSpeed(pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
      coral.stop();
      coral.ResetEncoder();
      targetPosition = 0;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return intakePID.atSetpoint(); // Stop when PID reaches setpointreturn false;
  }
    
}

