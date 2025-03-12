/*package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
public class BucketMoveF45 extends Command {
private final BucketSubsystem coral;
private double currentAngle;

    public BucketMoveF45(BucketSubsystem subsystem) {
        this.coral = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(coral);
    }

    @Override
    public void initialize()
    {

    }
    @Override
    public void execute()
    {
        
        coral.setPower(0.25);
        //coral.backwardsBucket();
    }

    @Override
    public void end(boolean interrupted)
    {
        coral.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

}*/
