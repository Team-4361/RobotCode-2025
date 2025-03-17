package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
public class ElevatorUpCommand extends Command {
private final ElevatorSubsystem elevator;

    public ElevatorUpCommand(ElevatorSubsystem subsystem) {
        this.elevator = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(elevator);
    }

    @Override
    public void initialize()
    {
        elevator.elevatorMoveUp();
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