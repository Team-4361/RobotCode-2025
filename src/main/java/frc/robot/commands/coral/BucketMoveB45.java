package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
public class BucketMoveB45 extends Command {
private final BucketSubsystem bucket;

    public BucketMoveB45(BucketSubsystem subsystem) {
        this.bucket = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(bucket);
    }

    @Override
    public void initialize()
    {

        bucket.backwardsBucket();
    }
    @Override
    public void execute()
    {
        bucket.backwardsBucket();
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
