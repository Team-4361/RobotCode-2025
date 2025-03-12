package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
public class BucketMoveB45 extends Command {
private final BucketSubsystem coral;
private double currentAngle;
private double speed;

    public BucketMoveB45(BucketSubsystem subsystem, double speed) {
        this.coral = subsystem;
        this.speed = speed;
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
        
        coral.setPower(speed);
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

}
