package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
public class ElevatorUpCommand extends Command {
private final ElevatorSubsystem elevator;
private final BucketSubsystem bucket;

    public ElevatorUpCommand(ElevatorSubsystem subsystem, BucketSubsystem bucket) {
        this.elevator = subsystem;
        this.bucket = bucket;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(elevator);
        addRequirements(bucket);
    }

    @Override
    public void initialize()
    {
        elevator.elevatorMoveUp();
        bucket.setPosition(0.0);

    }
    @Override
    public void execute()
    {
        elevator.elevatorMoveUp();
    }

    @Override
    public void end(boolean interrupted)
    {
        elevator.stopElevator();
    }

    @Override
    public boolean isFinished()
    {
        return false; //idr how to implement this 
    }
}