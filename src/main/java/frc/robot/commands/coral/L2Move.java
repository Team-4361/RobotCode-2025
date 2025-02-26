package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Coral;
public class L2Move extends Command {
private final ElevatorSubsystem elevator;
private double currentHeight; 

    public L2Move(ElevatorSubsystem subsystem) {
        this.elevator = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(elevator);
        

    }

    @Override
    public void initialize()
    {
       currentHeight = elevator.getHeight(); 
    }
    @Override
    public void execute()
    {
        if(currentHeight > Constants.Coral.L2_POS)
        {
            elevator.setMotors(-Constants.ElevatorConstants.ELEVATOR_SPEED);
            elevator.setGoal(Constants.Coral.L2_POS);
        }
        else if(currentHeight < Constants.Coral.L2_POS)
        {
            elevator.setMotors(Constants.ElevatorConstants.ELEVATOR_SPEED);
            elevator.setGoal(Constants.Coral.L2_POS);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        elevator.stop();
    }

    @Override
    public boolean isFinished()
    {
       return elevator.atHeight(Constants.Coral.L2_POS, Constants.ElevatorConstants.ELEVATOR_TOLERANCE).getAsBoolean();
    }

}
