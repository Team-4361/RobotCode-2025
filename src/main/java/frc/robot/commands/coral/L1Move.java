package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
public class L1Move extends Command {
private final ElevatorSubsystem elevator;

    public L1Move(ElevatorSubsystem subsystem) {
        this.elevator = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(elevator);
    }

    @Override
    public void initialize()
    {
        elevator.getHeight();
    }
    @Override
    public void execute()
    {
        elevator.setMotors(Constants.ElevatorConstants.ELEVATOR_SPEED);
        elevator.reachGoal(Constants.Coral.L1_POS);
    }

    @Override
    public void end(boolean interrupted)
    {
        elevator.stop();
    }

    @Override
    public boolean isFinished()
    {
        return elevator.atHeight(Constants.Coral.L1_POS, Constants.ElevatorConstants.ELEVATOR_TOLERANCE);
    }

}
