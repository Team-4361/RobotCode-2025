package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
public class MoveElevatorPos extends Command {
private final ElevatorSubsystem elevator;
private double currentHeight;
private double tHeight;

    public MoveElevatorPos(ElevatorSubsystem subsystem, double height) {
        this.elevator = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(elevator);
        

    }

    @Override
    public void initialize()
    {
       currentHeight = elevator.getPositionMeters();
    }
    @Override
    public void execute()
    {
        elevator.setElevatorHeight(tHeight);
        /*if(currentHeight > Constants.Coral.L1_POS)
        {
            //elevator.setMotors(-Constants.ElevatorConstants.ELEVATOR_SPEED);
            elevator.setElevatorHeight(Constants.Coral.L1_POS);
        }
        else if(currentHeight < Constants.Coral.L1_POS)
        {
            //elevator.setMotors(Constants.ElevatorConstants.ELEVATOR_SPEED);
            elevator.setElevatorHeight(Constants.Coral.L1_POS);
        }*/
    }

    @Override
    public void end(boolean interrupted)
    {
        elevator.stop();
    }

    @Override
    public boolean isFinished()
    {
       return elevator.aroundHeight(tHeight, Constants.ElevatorConstants.ELEVATOR_TOLERANCE);
    }

}
