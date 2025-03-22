package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
public class ElevatorDownCommand extends Command {
private final ElevatorSubsystem elevator;

    public ElevatorDownCommand(ElevatorSubsystem subsystem) {
        this.elevator = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(elevator);
    }

    @Override
    public void initialize()
    {
        //Prepares to move the elevator down
        elevator.elevatorMoveDown();    }
    @Override
    public void execute()
    {
        //Moves the elevator down
        elevator.elevatorMoveDown();
    }

    @Override
    public void end(boolean interrupted)
    {
        //Stops the elevator if something is in the way as an emergency
        elevator.stopElevator();
    }

    @Override
    public boolean isFinished()
    {
        return false; //idr how to implement this 
    }

}
