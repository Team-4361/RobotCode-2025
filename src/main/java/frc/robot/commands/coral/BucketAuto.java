package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;

public class BucketAuto extends Command {
    private final BucketSubsystem bucket;
    
    public BucketAuto(BucketSubsystem bucket) {
        this.bucket = bucket;
        addRequirements(bucket); 
    }

    @Override
    public void initialize() {
        
        
    }

    @Override
    public void execute() {
        if (!bucket.getSensor2()) {
            bucket.release(); //shoots out the coral
        }
        else {
            bucket.stop();
        }

  
    }

    @Override
    public void end(boolean interrupted) 
    {
        bucket.stop();
        // if something interrupts, stops the bucket as an emergency
    }

    @Override
    public boolean isFinished() {
        return bucket.getSensor2();
      // Check if it's at the target position
    }


} 
