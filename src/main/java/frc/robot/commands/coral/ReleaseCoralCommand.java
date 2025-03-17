package frc.robot.commands.coral;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;

public class ReleaseCoralCommand extends Command {
    private final BucketSubsystem bucket;
    
    public ReleaseCoralCommand(BucketSubsystem bucket) {
        this.bucket = bucket;
        addRequirements(bucket);
    }

    @Override
    public void initialize() {
        bucket.release();
    }

    @Override
    public void execute() {
        bucket.release();
    }

    @Override
    public void end(boolean interrupted) 
    {
        bucket.stop();
        
    }

    @Override
    public boolean isFinished() {
        return false; // Check if it's at the target position
    }


} 
