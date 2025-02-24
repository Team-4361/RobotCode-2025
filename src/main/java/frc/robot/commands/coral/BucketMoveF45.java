package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
public class BucketMoveF45 extends Command {
private final BucketSubsystem bucket;

    public BucketMoveF45(BucketSubsystem subsystem) {
        this.bucket = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(bucket);
    }

    @Override
    public void initialize()
    {
        bucket.forwardBucket();
    }
    @Override
    public void execute()
    {
        bucket.forwardBucket();
    }

    @Override
    public void end(boolean interrupted)
    {
        bucket.zeroBucket();
    }

    @Override
    public boolean isFinished()
    {
        return false; //idr how to implement this 
    }

}
