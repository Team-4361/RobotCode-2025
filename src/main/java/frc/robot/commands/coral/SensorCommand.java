package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
public class SensorCommand extends Command {
    private final BucketSubsystem coral;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SensorCommand(BucketSubsystem subsystem) {
    coral = subsystem;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    coral.setMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
        coral.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
    
}

